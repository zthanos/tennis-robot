"""Fake-serial tests for the physical drive bridge protocol."""

import unittest

from tennis_robot.base_serial_protocol import BaseSerialSession, parse_telemetry


class FakeSerial:
    def __init__(self):
        self.writes = []
        self.lines = []

    def write(self, value):
        self.writes.append(value)

    def flush(self):
        pass

    def readline(self):
        return self.lines.pop(0) if self.lines else b""

    def close(self):
        pass


class TestProtocol(unittest.TestCase):
    def test_fake_serial_config_and_stop(self):
        fake = FakeSerial()
        session = BaseSerialSession(fake)
        session.configure(700, 0.085, 0.585, 1.0, 12.775810, "gyro")
        session.command(0.2, 0.4)
        session.stop()

        self.assertIn(b"CFG NOLOAD 12.77581\n", fake.writes)
        self.assertIn(b"ARM\n", fake.writes)
        self.assertEqual(fake.writes[-1], b"STOP\n")

    def test_parse(self):
        line = (
            "T,100,1,GYRO,0.2,0.4,0.1,0.2,0.3,0.4,1,2,3,4,"
            "5,6,7,8,0.4,0.1,0.2,0.3,0,10002,4380"
        )
        telemetry = parse_telemetry(line)
        self.assertIsNotNone(telemetry)
        self.assertEqual(telemetry.rr, 4)
        self.assertAlmostEqual(telemetry.y_m, 0.3)
        self.assertEqual(telemetry.max_loop_gap_us, 10002)

    def test_reject_truncated(self):
        self.assertIsNone(parse_telemetry("T,1,2"))


try:
    import rclpy
    from tennis_robot.base_serial_bridge import BaseSerialBridge
except ImportError:
    rclpy = None


@unittest.skipIf(rclpy is None, "ROS 2 environment required")
class TestBridgeNodeWithFakeSerial(unittest.TestCase):
    def test_node_opens_fake_and_disarms_on_destroy(self):
        fake = FakeSerial()
        rclpy.init(args=["--ros-args", "-p", "counts_per_rev:=700.0"])
        node = BaseSerialBridge(serial_factory=lambda _port, _baud: fake)
        node.destroy_node()
        rclpy.shutdown()

        self.assertTrue(
            any(command.startswith(b"CFG NOLOAD 12.77581") for command in fake.writes)
        )
        self.assertIn(b"ARM\n", fake.writes)
        self.assertEqual(fake.writes[-2:], [b"STOP\n", b"DISARM\n"])


if __name__ == "__main__":
    unittest.main()
