"""Webots controller that detects tennis balls in the robot camera stream."""

from __future__ import annotations

import math
import time

import cv2
import numpy as np
from controller import Camera, Display, Motor, Robot
from perception import BallDetection, detect_largest_ball, estimate_ball_observation
from telemetry import setup_telemetry


TIME_STEP_MS = 32
MAX_SPEED_RAD_S = 6.28
SCAN_SPEED_RAD_S = 1.4

TARGET_CENTER_TOLERANCE_PX = 24


class BallDetectorController:
    def __init__(self) -> None:
        self.robot = Robot()
        self.camera = self._device("front_camera", Camera)
        self.display = self._device("camera_display", Display)
        self.left_motor = self._device("left_wheel_motor", Motor)
        self.right_motor = self._device("right_wheel_motor", Motor)
        self.telemetry = setup_telemetry("ball-detector-controller")

        self.camera.enable(TIME_STEP_MS)
        self.left_motor.setPosition(math.inf)
        self.right_motor.setPosition(math.inf)
        self.set_speed(0.0, 0.0)

    def _device(self, name: str, expected_type: type):
        device = self.robot.getDevice(name)
        if not isinstance(device, expected_type):
            raise TypeError(f"Device {name!r} is not a {expected_type.__name__}")
        return device

    def set_speed(self, left: float, right: float) -> None:
        self.left_motor.setVelocity(max(-MAX_SPEED_RAD_S, min(MAX_SPEED_RAD_S, left)))
        self.right_motor.setVelocity(max(-MAX_SPEED_RAD_S, min(MAX_SPEED_RAD_S, right)))

    def run(self) -> None:
        while self.robot.step(TIME_STEP_MS) != -1:
            loop_start = time.perf_counter()
            with self.telemetry.start_span("simulation.step"):
                image = self._camera_frame()
                self.telemetry.add_frame()
                detection = self._detect_largest_ball(image)
                self._draw_debug(image, detection)
                self._drive_from_detection(detection)
            duration_ms = (time.perf_counter() - loop_start) * 1000
            self.telemetry.record_loop_duration(duration_ms)

    def _camera_frame(self) -> np.ndarray:
        width = self.camera.getWidth()
        height = self.camera.getHeight()
        raw = self.camera.getImage()
        frame = np.frombuffer(raw, np.uint8).reshape((height, width, 4))
        return cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR)

    def _detect_largest_ball(self, frame: np.ndarray) -> BallDetection | None:
        return detect_largest_ball(frame)

    def _draw_debug(self, frame: np.ndarray, detection: BallDetection | None) -> None:
        if detection is not None:
            cv2.rectangle(
                frame,
                (detection.x, detection.y),
                (detection.x + detection.width, detection.y + detection.height),
                (0, 0, 255),
                2,
            )
            cv2.circle(frame, (int(detection.center_x), int(detection.center_y)), 4, (255, 0, 0), -1)

        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        image_ref = self.display.imageNew(rgb.tobytes(), Display.RGB, frame.shape[1], frame.shape[0])
        self.display.imagePaste(image_ref, 0, 0, False)
        self.display.imageDelete(image_ref)

    def _drive_from_detection(self, detection: BallDetection | None) -> None:
        if detection is None:
            self.set_speed(-SCAN_SPEED_RAD_S, SCAN_SPEED_RAD_S)
            return

        observation = estimate_ball_observation(
            detection,
            self.camera.getWidth(),
            self.camera.getFov(),
        )
        error_px = detection.center_x - self.camera.getWidth() / 2
        self.telemetry.add_detection(detection.area_px, observation.distance_m, observation.bearing_rad)

        if abs(error_px) < TARGET_CENTER_TOLERANCE_PX:
            self.set_speed(2.0, 2.0)
            return

        turn = 1.5 if error_px > 0 else -1.5
        self.set_speed(turn, -turn)


if __name__ == "__main__":
    BallDetectorController().run()
