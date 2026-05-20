"""Webots controller that detects tennis balls in the robot camera stream."""

from __future__ import annotations

import math
import os
import time
from typing import TYPE_CHECKING

from collector import (
    BallObservationInput,
    BaseCommand,
    CollectorCommand,
    CollectorState,
    ConceptACollectorBehavior,
    ConceptACommand,
    ConceptAConfig,
)
from control_bus import RobotCommandStore
from controller import Camera, Display, Motor, RangeFinder, Supervisor
from survey import CourtSurveyBehavior, SurveyState
from telemetry import setup_telemetry

RGB_VISION_REQUESTED = os.getenv("USE_RGB_VISION", "").strip().lower() in {"1", "true", "yes", "on"}

try:
    if not RGB_VISION_REQUESTED:
        raise ImportError("RGB/depth vision disabled; set USE_RGB_VISION=true to enable it")

    import cv2
    import numpy as np
    from perception import (
        BallDetection,
        BallObservation,
        CameraMount,
        RobotPose2D,
        detect_largest_ball,
        estimate_ball_observation,
        estimate_depth_ball_observation,
        observation_to_world,
    )

    VISION_ENABLED = True
except ImportError as exc:
    cv2 = None
    np = None
    VISION_ENABLED = False
    VISION_IMPORT_ERROR = exc

    if TYPE_CHECKING:
        from perception import BallDetection, BallObservation
    else:
        BallDetection = object
        BallObservation = object


TIME_STEP_MS = 32
MAX_SPEED_RAD_S = 6.28
WHEEL_RADIUS_M = 0.11
TRACK_WIDTH_M = 0.48
INTAKE_ZONE_X_M = (0.34, 0.62)
INTAKE_HALF_WIDTH_M = 0.11
INTAKE_MAX_HEIGHT_M = 0.12
SUPERVISED_FOV_RAD = 1.05
SUPERVISED_MAX_RANGE_M = 8.0
FRONT_CAMERA_MOUNT = CameraMount(x_m=0.31, y_m=0.0, yaw_rad=0.0) if VISION_ENABLED else None


def _env_float(name: str, default: float) -> float:
    value = os.getenv(name)
    if value is None:
        return default
    try:
        return float(value)
    except ValueError:
        print(f"invalid {name}={value!r}; using {default}")
        return default


class BallDetectorController:
    def __init__(self) -> None:
        self.robot = Supervisor()
        self.camera = self._device("front_camera", Camera)
        self.depth_camera = self._optional_device("front_depth", RangeFinder)
        self.display = self._device("camera_display", Display)
        self.left_motor = self._device("left_wheel_motor", Motor)
        self.right_motor = self._device("right_wheel_motor", Motor)
        self.lift_motor = self._optional_device("lift_wheel_motor", Motor)
        self.telemetry = setup_telemetry("ball-detector-controller")
        self.max_speed_rad_s = _env_float("ROBOT_MAX_WHEEL_SPEED_RAD_S", MAX_SPEED_RAD_S)
        self.behavior = ConceptACollectorBehavior(ConceptAConfig.from_env())
        self.survey_behavior = CourtSurveyBehavior.from_env()
        self.command_store = RobotCommandStore.from_env()
        self.control_mode = "idle"
        self.robot_node = self.robot.getSelf()
        self.collection_confirmed = False
        self.collection_count = 0
        self.last_command: ConceptACommand | None = None
        self.loop_count = 0

        self.camera.enable(TIME_STEP_MS)
        if self.depth_camera is not None:
            self.depth_camera.enable(TIME_STEP_MS)
        self.left_motor.setPosition(math.inf)
        self.right_motor.setPosition(math.inf)
        if self.lift_motor is not None:
            self.lift_motor.setPosition(math.inf)
        self.set_speed(0.0, 0.0)
        if VISION_ENABLED:
            print("ball_detector controller started with RGB/depth vision")
        else:
            print(f"ball_detector controller started in supervised emulator mode: {VISION_IMPORT_ERROR}")

    def _device(self, name: str, expected_type: type):
        device = self.robot.getDevice(name)
        if not isinstance(device, expected_type):
            raise TypeError(f"Device {name!r} is not a {expected_type.__name__}")
        return device

    def _optional_device(self, name: str, expected_type: type):
        device = self.robot.getDevice(name)
        if device is None:
            return None
        if not isinstance(device, expected_type):
            raise TypeError(f"Device {name!r} is not a {expected_type.__name__}")
        return device

    def set_speed(self, left: float, right: float) -> None:
        self.left_motor.setVelocity(max(-self.max_speed_rad_s, min(self.max_speed_rad_s, left)))
        self.right_motor.setVelocity(max(-self.max_speed_rad_s, min(self.max_speed_rad_s, right)))

    def set_base_command(self, linear_speed_m_s: float, angular_speed_rad_s: float) -> None:
        left = (linear_speed_m_s - angular_speed_rad_s * TRACK_WIDTH_M / 2) / WHEEL_RADIUS_M
        right = (linear_speed_m_s + angular_speed_rad_s * TRACK_WIDTH_M / 2) / WHEEL_RADIUS_M
        self.set_speed(left, right)

    def set_collector_command(self, lift_wheel_speed: float) -> None:
        if self.lift_motor is not None:
            self.lift_motor.setVelocity(max(-self.max_speed_rad_s, min(self.max_speed_rad_s, lift_wheel_speed * 4.0)))

    def run(self) -> None:
        while self.robot.step(TIME_STEP_MS) != -1:
            loop_start = time.perf_counter()
            with self.telemetry.start_span("simulation.step"):
                image = self._camera_frame()
                depth_frame = self._depth_frame()
                self.telemetry.add_frame()
                detection = self._detect_largest_ball(image)
                if VISION_ENABLED:
                    observation = self._observation_from_detection(detection, depth_frame)
                else:
                    observation = self._supervised_ball_observation()
                control_command = self.command_store.read()
                if control_command.mode == "survey":
                    command = self._survey_command_for_mode(control_command.mode, depth_frame)
                else:
                    command = self._collector_command_for_mode(control_command.mode, observation)
                self.collection_confirmed = False
                self._draw_debug(image, detection, command)
                self._apply_command(command)
                self.telemetry.add_collector_state(command.state.value)
                self.collection_confirmed = self._simulate_collection(command)
                self.last_command = command
                self.loop_count += 1
                if self.loop_count % 60 == 0:
                    self._print_status(command, observation, depth_frame)
            duration_ms = (time.perf_counter() - loop_start) * 1000
            self.telemetry.record_loop_duration(duration_ms)

    def _collector_command_for_mode(self, mode: str, observation: BallObservationInput) -> ConceptACommand:
        if mode != self.control_mode:
            self.behavior.reset()
            self.survey_behavior.reset()
            self.control_mode = mode
            print(f"control mode changed to {self.control_mode}")

        if mode == "collect":
            return self.behavior.update(
                observation,
                TIME_STEP_MS / 1000,
                collection_confirmed=self.collection_confirmed,
            )

        return ConceptACommand(
            state=CollectorState.IDLE,
            base=BaseCommand(0.0, 0.0),
            collector=CollectorCommand(0.0, False),
        )

    def _survey_command_for_mode(self, mode: str, depth_frame: np.ndarray | None) -> ConceptACommand:
        if mode != self.control_mode:
            self.behavior.reset()
            self.survey_behavior.reset()
            self.control_mode = mode
            print(f"control mode changed to {self.control_mode}")

        x_m, y_m, _z_m = self.robot_node.getPosition()
        survey_command = self.survey_behavior.update(
            x_m,
            y_m,
            self._robot_yaw_rad(),
            self._front_range_m(depth_frame),
            TIME_STEP_MS / 1000,
        )
        if survey_command.state == SurveyState.DONE:
            return ConceptACommand(
                state=CollectorState.IDLE,
                base=BaseCommand(0.0, 0.0),
                collector=CollectorCommand(0.0, False),
            )
        return ConceptACommand(
            state=CollectorState.SURVEY,
            base=survey_command.base,
            collector=CollectorCommand(0.0, False),
        )

    def _camera_frame(self) -> np.ndarray:
        if not VISION_ENABLED:
            return None
        width = self.camera.getWidth()
        height = self.camera.getHeight()
        raw = self.camera.getImage()
        frame = np.frombuffer(raw, np.uint8).reshape((height, width, 4))
        return cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR)

    def _depth_frame(self) -> np.ndarray | None:
        if not VISION_ENABLED or self.depth_camera is None:
            return None
        width = self.depth_camera.getWidth()
        height = self.depth_camera.getHeight()
        return np.array(self.depth_camera.getRangeImage(), dtype=np.float32).reshape((height, width))

    def _detect_largest_ball(self, frame: np.ndarray) -> BallDetection | None:
        if not VISION_ENABLED or frame is None:
            return None
        return detect_largest_ball(frame)

    def _observation_from_detection(
        self,
        detection: BallDetection | None,
        depth_frame: np.ndarray | None,
    ) -> BallObservationInput:
        if detection is None:
            return BallObservationInput(visible=False)

        observation = self._estimate_observation(detection, depth_frame)
        world_observation = observation_to_world(
            observation,
            RobotPose2D(*self._robot_pose_2d()),
            FRONT_CAMERA_MOUNT,
        )
        self.telemetry.add_detection(
            detection.area_px,
            observation.distance_m,
            observation.bearing_rad,
            observation.distance_source,
        )
        return BallObservationInput(
            visible=True,
            bearing_rad=observation.bearing_rad,
            distance_m=observation.distance_m,
            confidence=min(1.0, detection.area_px / 6000),
            robot_x_m=world_observation.robot_x_m,
            robot_y_m=world_observation.robot_y_m,
            world_x_m=world_observation.world_x_m,
            world_y_m=world_observation.world_y_m,
        )

    def _estimate_observation(
        self,
        detection: BallDetection,
        depth_frame: np.ndarray | None,
    ) -> BallObservation:
        if depth_frame is not None:
            depth_observation = estimate_depth_ball_observation(
                detection,
                depth_frame,
                self.camera.getWidth(),
                self.camera.getHeight(),
                self.camera.getFov(),
            )
            if depth_observation is not None:
                return depth_observation

        return estimate_ball_observation(
            detection,
            self.camera.getWidth(),
            self.camera.getFov(),
        )

    def _supervised_ball_observation(self) -> BallObservationInput:
        nearest: tuple[float, float, float, float, float, float] | None = None
        for index in range(100):
            ball = self.robot.getFromDef(f"TENNIS_BALL_{index:02d}")
            if ball is None:
                continue
            ball_position = ball.getPosition()
            x, y, _z = self._world_to_robot_local(ball_position)
            if x <= 0:
                continue
            distance_m = math.hypot(x, y)
            bearing_rad = math.atan2(y, x)
            if abs(bearing_rad) > SUPERVISED_FOV_RAD / 2 or distance_m > SUPERVISED_MAX_RANGE_M:
                continue
            if nearest is None or distance_m < nearest[0]:
                nearest = (distance_m, bearing_rad, x, y, ball_position[0], ball_position[1])

        if nearest is None:
            return BallObservationInput(visible=False)
        distance_m, bearing_rad, robot_x_m, robot_y_m, world_x_m, world_y_m = nearest
        return BallObservationInput(
            visible=True,
            bearing_rad=bearing_rad,
            distance_m=distance_m,
            confidence=1.0,
            robot_x_m=robot_x_m,
            robot_y_m=robot_y_m,
            world_x_m=world_x_m,
            world_y_m=world_y_m,
        )

    def _draw_debug(
        self,
        frame: np.ndarray | None,
        detection: BallDetection | None,
        command: ConceptACommand,
    ) -> None:
        if not VISION_ENABLED or frame is None:
            return
        if detection is not None:
            cv2.rectangle(
                frame,
                (detection.x, detection.y),
                (detection.x + detection.width, detection.y + detection.height),
                (0, 0, 255),
                2,
            )
            cv2.circle(frame, (int(detection.center_x), int(detection.center_y)), 4, (255, 0, 0), -1)

        cv2.putText(
            frame,
            f"collector={command.state.value} balls={self.collection_count}",
            (16, 28),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (255, 255, 255),
            2,
            cv2.LINE_AA,
        )
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        image_ref = self.display.imageNew(rgb.tobytes(), Display.RGB, frame.shape[1], frame.shape[0])
        self.display.imagePaste(image_ref, 0, 0, False)
        self.display.imageDelete(image_ref)

    def _apply_command(self, command: ConceptACommand) -> None:
        self.set_base_command(command.base.linear_speed_m_s, command.base.angular_speed_rad_s)
        self.set_collector_command(command.collector.lift_wheel_speed)

    def _front_range_m(self, depth_frame: np.ndarray | None) -> float | None:
        if depth_frame is None:
            return None
        height, width = depth_frame.shape
        crop = depth_frame[height // 2 - 8 : height // 2 + 8, width // 2 - 8 : width // 2 + 8]
        valid = crop[np.isfinite(crop) & (crop > 0)]
        if valid.size == 0:
            return None
        return float(np.median(valid))

    def _robot_yaw_rad(self) -> float:
        orientation = self.robot_node.getOrientation()
        return math.atan2(orientation[3], orientation[0])

    def _robot_pose_2d(self) -> tuple[float, float, float]:
        x_m, y_m, _z_m = self.robot_node.getPosition()
        return (x_m, y_m, self._robot_yaw_rad())

    def _print_status(
        self,
        command: ConceptACommand,
        observation: BallObservationInput,
        depth_frame: np.ndarray | None,
    ) -> None:
        status = (
            f"mode={self.control_mode} "
            f"collector={command.state.value} "
            f"visible={observation.visible} "
            f"distance={observation.distance_m:.2f}m "
            f"bearing={math.degrees(observation.bearing_rad):.1f}deg "
            f"balls={self.collection_count}"
        )
        if observation.world_x_m is not None and observation.world_y_m is not None:
            status += f" ball_world=({observation.world_x_m:.2f},{observation.world_y_m:.2f})"
        if self.control_mode == "survey":
            x_m, y_m, _z_m = self.robot_node.getPosition()
            target = self.survey_behavior.current_target()
            target_text = "none" if target is None else f"({target[0]:.2f},{target[1]:.2f})"
            front_range = self._front_range_m(depth_frame)
            front_range_text = "none" if front_range is None else f"{front_range:.2f}m"
            status += (
                f" survey_state={self.survey_behavior.state.value} "
                f"waypoint={self.survey_behavior.waypoint_index + 1}/{len(self.survey_behavior.waypoints)} "
                f"pos=({x_m:.2f},{y_m:.2f}) "
                f"target={target_text} "
                f"front_range={front_range_text}"
            )
        print(status)

    def _simulate_collection(self, command: ConceptACommand) -> bool:
        if not command.collector.intake_enabled:
            return False

        for index in range(100):
            ball = self.robot.getFromDef(f"TENNIS_BALL_{index:02d}")
            if ball is None:
                continue
            local_position = self._world_to_robot_local(ball.getPosition())
            if self._in_intake_zone(local_position):
                ball.remove()
                self.collection_count += 1
                self.telemetry.add_collection()
                print(f"collected tennis_ball_{index:02d}; total={self.collection_count}")
                return True
        return False

    def _world_to_robot_local(self, world_position: list[float]) -> tuple[float, float, float]:
        robot_position = self.robot_node.getPosition()
        orientation = self.robot_node.getOrientation()
        dx = world_position[0] - robot_position[0]
        dy = world_position[1] - robot_position[1]
        dz = world_position[2] - robot_position[2]
        return (
            orientation[0] * dx + orientation[3] * dy + orientation[6] * dz,
            orientation[1] * dx + orientation[4] * dy + orientation[7] * dz,
            orientation[2] * dx + orientation[5] * dy + orientation[8] * dz,
        )

    def _in_intake_zone(self, local_position: tuple[float, float, float]) -> bool:
        x, y, z = local_position
        return (
            INTAKE_ZONE_X_M[0] <= x <= INTAKE_ZONE_X_M[1]
            and abs(y) <= INTAKE_HALF_WIDTH_M
            and z <= INTAKE_MAX_HEIGHT_M
        )


if __name__ == "__main__":
    BallDetectorController().run()
