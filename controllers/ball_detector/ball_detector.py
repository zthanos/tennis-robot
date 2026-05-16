"""Webots controller that detects tennis balls in the robot camera stream."""

from __future__ import annotations

import math
import time

import cv2
import numpy as np
from controller import Camera, Display, Motor, Robot
from telemetry import setup_telemetry


TIME_STEP_MS = 32
MAX_SPEED_RAD_S = 6.28
SCAN_SPEED_RAD_S = 1.4

MIN_BALL_AREA_PX = 20
TARGET_CENTER_TOLERANCE_PX = 24

# Tennis balls vary from yellow-green to green depending on lighting.
HSV_LOWER = np.array([25, 80, 80], dtype=np.uint8)
HSV_UPPER = np.array([85, 255, 255], dtype=np.uint8)


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
                self._drive_from_detection(image.shape[1], detection)
            duration_ms = (time.perf_counter() - loop_start) * 1000
            self.telemetry.record_loop_duration(duration_ms)

    def _camera_frame(self) -> np.ndarray:
        width = self.camera.getWidth()
        height = self.camera.getHeight()
        raw = self.camera.getImage()
        frame = np.frombuffer(raw, np.uint8).reshape((height, width, 4))
        return cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR)

    def _detect_largest_ball(self, frame: np.ndarray) -> tuple[int, int, int, int] | None:
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, HSV_LOWER, HSV_UPPER)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((7, 7), np.uint8))

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        candidates = [cv2.boundingRect(contour) for contour in contours if cv2.contourArea(contour) > MIN_BALL_AREA_PX]
        if not candidates:
            return None
        return max(candidates, key=lambda rect: rect[2] * rect[3])

    def _draw_debug(self, frame: np.ndarray, detection: tuple[int, int, int, int] | None) -> None:
        if detection is not None:
            x, y, width, height = detection
            cv2.rectangle(frame, (x, y), (x + width, y + height), (0, 0, 255), 2)
            cv2.circle(frame, (x + width // 2, y + height // 2), 4, (255, 0, 0), -1)

        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        image_ref = self.display.imageNew(rgb.tobytes(), Display.RGB, frame.shape[1], frame.shape[0])
        self.display.imagePaste(image_ref, 0, 0, False)
        self.display.imageDelete(image_ref)

    def _drive_from_detection(self, frame_width: int, detection: tuple[int, int, int, int] | None) -> None:
        if detection is None:
            self.set_speed(-SCAN_SPEED_RAD_S, SCAN_SPEED_RAD_S)
            return

        x, _, width, _ = detection
        ball_center_x = x + width / 2
        error_px = ball_center_x - frame_width / 2
        self.telemetry.add_detection(width * detection[3])

        if abs(error_px) < TARGET_CENTER_TOLERANCE_PX:
            self.set_speed(2.0, 2.0)
            return

        turn = 1.5 if error_px > 0 else -1.5
        self.set_speed(turn, -turn)


if __name__ == "__main__":
    BallDetectorController().run()
