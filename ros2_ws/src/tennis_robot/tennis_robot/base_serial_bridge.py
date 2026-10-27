"""Bridge the final twist_mux command to the P2-64 Mega drive firmware."""

from __future__ import annotations

import fcntl
import json
import math
import time

import rclpy
from geometry_msgs.msg import TransformStamped, TwistStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import String
from tf2_ros import TransformBroadcaster

from tennis_robot.base_serial_protocol import BaseSerialSession


class BaseSerialBridge(Node):
    """Own the Mega port, enforce the command watchdog, and publish odometry."""

    def __init__(self, serial_factory=None):
        super().__init__("base_serial_bridge")

        defaults = (
            ("port", "/dev/ttyACM0"),
            ("baud", 500000),
            ("counts_per_rev", 0.0),
            ("wheel_radius_m", 0.085),
            ("track_m", 0.585),
            ("pwm_ceiling", 1.0),
            ("no_load_speed_rad_s", 122.0 * 2.0 * math.pi / 60.0),
            ("mode", "gyro"),
            ("max_linear_m_s", 0.2),
            ("max_angular_rad_s", 0.4),
            ("command_timeout_s", 0.25),
        )
        for name, default in defaults:
            self.declare_parameter(name, default)

        counts_per_rev = float(self.get_parameter("counts_per_rev").value)
        if counts_per_rev <= 0.0:
            raise RuntimeError("counts_per_rev must be measured/configured before arming")

        # The runner and bridge deliberately share one non-blocking port lock.
        self._lock = open("/tmp/tennis_robot_ttyACM0.lock", "w", encoding="ascii")
        fcntl.flock(self._lock, fcntl.LOCK_EX | fcntl.LOCK_NB)

        if serial_factory is None:
            import serial

            def serial_factory(port, baud):
                return serial.Serial(port, baud, timeout=0.005, exclusive=True)

        port = str(self.get_parameter("port").value)
        baud = int(self.get_parameter("baud").value)
        self._raw_serial = serial_factory(port, baud)
        time.sleep(2.0)

        self._session = BaseSerialSession(self._raw_serial)
        self._session.configure(
            counts_per_rev=counts_per_rev,
            radius_m=float(self.get_parameter("wheel_radius_m").value),
            track_m=float(self.get_parameter("track_m").value),
            pwm_ceiling=float(self.get_parameter("pwm_ceiling").value),
            no_load_speed_rad_s=float(
                self.get_parameter("no_load_speed_rad_s").value
            ),
            mode=str(self.get_parameter("mode").value),
        )

        self._last_command_time = time.monotonic()
        self._moving = False

        self.create_subscription(
            TwistStamped,
            "/drive_test/cmd_vel",
            self._command_callback,
            10,
        )
        self._odom_publisher = self.create_publisher(Odometry, "/odom", 20)
        self._imu_publisher = self.create_publisher(Imu, "/imu/data_raw", 20)
        self._telemetry_publisher = self.create_publisher(
            String,
            "/drive/telemetry",
            20,
        )
        self._tf_broadcaster = TransformBroadcaster(self)

        self.create_timer(0.01, self._poll_serial)
        self.create_timer(0.05, self._watchdog)

    def _command_callback(self, message: TwistStamped) -> None:
        maximum_v = float(self.get_parameter("max_linear_m_s").value)
        maximum_w = float(self.get_parameter("max_angular_rad_s").value)
        linear = max(-maximum_v, min(maximum_v, message.twist.linear.x))
        angular = max(-maximum_w, min(maximum_w, message.twist.angular.z))

        self._session.command(linear, angular)
        self._moving = abs(linear) > 1.0e-6 or abs(angular) > 1.0e-6
        self._last_command_time = time.monotonic()

    def _watchdog(self) -> None:
        timeout = float(self.get_parameter("command_timeout_s").value)
        if self._moving and time.monotonic() - self._last_command_time > timeout:
            self._session.stop()
            self._moving = False
            return
        self._session.send("PING")

    @staticmethod
    def _yaw_quaternion(yaw_rad: float) -> tuple[float, float, float, float]:
        return (0.0, 0.0, math.sin(yaw_rad / 2.0), math.cos(yaw_rad / 2.0))

    def _poll_serial(self) -> None:
        telemetry = self._session.read()
        if telemetry is None:
            return

        stamp = self.get_clock().now().to_msg()
        _, _, quaternion_z, quaternion_w = self._yaw_quaternion(telemetry.yaw_rad)

        odometry = Odometry()
        odometry.header.stamp = stamp
        odometry.header.frame_id = "odom"
        odometry.child_frame_id = "base_footprint"
        odometry.pose.pose.position.x = telemetry.x_m
        odometry.pose.pose.position.y = telemetry.y_m
        odometry.pose.pose.orientation.z = quaternion_z
        odometry.pose.pose.orientation.w = quaternion_w
        mean_wheel_speed = (
            telemetry.w_lf + telemetry.w_lr + telemetry.w_rf + telemetry.w_rr
        ) / 4.0
        odometry.twist.twist.linear.x = mean_wheel_speed * float(
            self.get_parameter("wheel_radius_m").value
        )
        odometry.twist.twist.angular.z = telemetry.gz_rad_s
        self._odom_publisher.publish(odometry)

        imu = Imu()
        imu.header.stamp = stamp
        imu.header.frame_id = "imu_link"
        imu.orientation_covariance[0] = -1.0
        imu.angular_velocity.z = telemetry.gz_rad_s
        self._imu_publisher.publish(imu)

        telemetry_message = String()
        telemetry_message.data = json.dumps(telemetry.__dict__, separators=(",", ":"))
        self._telemetry_publisher.publish(telemetry_message)

        transform = TransformStamped()
        transform.header.stamp = stamp
        transform.header.frame_id = "odom"
        transform.child_frame_id = "base_footprint"
        transform.transform.translation.x = telemetry.x_m
        transform.transform.translation.y = telemetry.y_m
        transform.transform.rotation.z = quaternion_z
        transform.transform.rotation.w = quaternion_w
        self._tf_broadcaster.sendTransform(transform)

    def destroy_node(self):
        try:
            self._session.stop()
            self._session.send("DISARM")
            self._raw_serial.close()
            fcntl.flock(self._lock, fcntl.LOCK_UN)
            self._lock.close()
        finally:
            super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = BaseSerialBridge()
        rclpy.spin(node)
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
