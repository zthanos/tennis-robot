"""OpenCV camera overlay and BMP serialization helpers — no Webots dependency."""

from __future__ import annotations

import base64
import math
import struct
from typing import TYPE_CHECKING

try:
    import cv2
    import numpy as np
    _CV2_AVAILABLE = True
except ImportError:
    cv2 = None
    np = None
    _CV2_AVAILABLE = False

if TYPE_CHECKING:
    from collector import BallObservationInput
    from perception import BallDetection
    from survey import CourtSurveyBehavior, SurveyVision


def bgra_bmp_data_url(bgra: bytes, width: int, height: int) -> str:
    pixel_bytes = width * height * 4
    if len(bgra) != pixel_bytes:
        bgra = bgra[:pixel_bytes].ljust(pixel_bytes, b"\x00")
    file_size = 54 + pixel_bytes
    file_header = struct.pack("<2sIHHI", b"BM", file_size, 0, 0, 54)
    dib_header = struct.pack(
        "<IiiHHIIiiII",
        40, width, -height, 1, 32, 0, pixel_bytes, 2835, 2835, 0, 0,
    )
    return "data:image/bmp;base64," + base64.b64encode(file_header + dib_header + bgra).decode("ascii")


def draw_label(frame, text: str, x: int, y: int, color: tuple[int, int, int]) -> None:
    if not _CV2_AVAILABLE:
        return
    font = cv2.FONT_HERSHEY_SIMPLEX
    scale = 0.48
    thickness = 1
    (tw, th), baseline = cv2.getTextSize(text, font, scale, thickness)
    x = max(2, min(x, frame.shape[1] - tw - 4))
    y = max(th + 4, min(y, frame.shape[0] - baseline - 4))
    cv2.rectangle(frame, (x - 2, y - th - 4), (x + tw + 4, y + baseline + 3), (0, 0, 0), -1)
    cv2.putText(frame, text, (x, y), font, scale, color, thickness, cv2.LINE_AA)


def survey_recognized_object(
    survey_behavior: CourtSurveyBehavior,
    last_survey_vision: SurveyVision | None,
) -> dict:
    nav = survey_behavior.telemetry()
    vision_class = None if last_survey_vision is None else last_survey_vision.obstacle_class
    label = vision_class or ("court_line" if nav.get("line_detected") else "unknown")
    if label is None:
        label = "unknown"
    source = nav.get("path_driver") or "none"
    if vision_class:
        source = "oak_visual_classifier"
    line_offset = nav.get("line_offset_m")
    oak_range = None if last_survey_vision is None else last_survey_vision.center_m
    front_lidar = nav.get("front_lidar_range_m")
    distance_m = line_offset if line_offset is not None else (oak_range if oak_range is not None else front_lidar)
    return {
        "label": label,
        "distance_m": distance_m,
        "source": source,
        "confidence": None,
        "roi": "front_camera_survey",
        "boundary": None,
    }


def draw_survey_overlay(
    frame,
    survey_behavior: CourtSurveyBehavior,
    last_survey_vision: SurveyVision | None,
) -> None:
    if not _CV2_AVAILABLE:
        return
    h, w = frame.shape[:2]
    obj = survey_recognized_object(survey_behavior, last_survey_vision)
    label = obj["label"]
    distance_m = obj["distance_m"]
    source = obj["source"]
    color = (80, 220, 255) if label == "net" else (0, 210, 120) if label == "fence" else (180, 180, 180)

    vx0, vx1 = int(w * 0.05), int(w * 0.95)
    vy0, vy1 = int(h * 0.12), int(h * 0.58)
    cv2.rectangle(frame, (vx0, vy0), (vx1, vy1), color, 1)

    dx0, dx1 = int(w * 0.33), int(w * 0.67)
    dy0, dy1 = int(h * 0.56), int(h * 0.92)
    cv2.rectangle(frame, (dx0, dy0), (dx1, dy1), (255, 255, 255), 1)

    distance_text = "dist=none" if distance_m is None else f"dist={distance_m:.2f}m"
    draw_label(frame, f"{label} {distance_text} {source}", vx0 + 4, max(18, vy0 - 8), color)


def draw_recognition_overlay(
    frame,
    detection: BallDetection | None,
    observation: BallObservationInput | None,
    control_mode: str,
    survey_behavior: CourtSurveyBehavior,
    last_survey_vision: SurveyVision | None,
) -> None:
    if not _CV2_AVAILABLE:
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
        ball_distance = None if observation is None or math.isinf(observation.distance_m) else observation.distance_m
        ball_label = "ball" if ball_distance is None else f"ball {ball_distance:.2f}m"
        draw_label(frame, ball_label, detection.x, max(18, detection.y - 8), (0, 0, 255))

    if control_mode == "map_court":
        draw_survey_overlay(frame, survey_behavior, last_survey_vision)


def draw_debug(
    frame,
    detection: BallDetection | None,
    command,
    observation: BallObservationInput | None,
    display,
    collection_count: int,
    control_mode: str,
    survey_behavior: CourtSurveyBehavior,
    last_survey_vision: SurveyVision | None,
) -> None:
    if not _CV2_AVAILABLE or frame is None:
        return
    draw_recognition_overlay(frame, detection, observation, control_mode, survey_behavior, last_survey_vision)
    cv2.putText(
        frame,
        f"collector={command.state.value} balls={collection_count}",
        (16, 28),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.7,
        (255, 255, 255),
        2,
        cv2.LINE_AA,
    )

    display_width = display.getWidth()
    display_height = display.getHeight()
    display_frame = frame
    if frame.shape[1] != display_width or frame.shape[0] != display_height:
        display_frame = cv2.resize(frame, (display_width, display_height), interpolation=cv2.INTER_AREA)

    from controller import Display
    rgb = cv2.cvtColor(display_frame, cv2.COLOR_BGR2RGB)
    image_ref = display.imageNew(rgb.tobytes(), Display.RGB, display_width, display_height)
    display.imagePaste(image_ref, 0, 0, False)
    display.imageDelete(image_ref)
