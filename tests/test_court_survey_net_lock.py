"""P2-99c: the net-lock range is measured from the LiDAR, not from base_link.

Runs the real ``CourtSurveyV2Node._try_lock_net`` on a stand-in object (no
spinning node) with a TF buffer stub that places lidar_link off the base_link
axis, as on the robot (C1 at base_link [-0.316, +0.110]). The old code added
the range to the base_link pose and fails here by exactly -lidar_x.
"""

from __future__ import annotations

import math
from types import SimpleNamespace

import pytest

pytest.importorskip("rclpy")
pytest.importorskip("tf2_ros")

from tennis_robot import court_survey_v2_node as survey  # noqa: E402


class _Tf:
    def __init__(self, frames: dict[str, tuple[float, float, float]]):
        self.frames = frames
        self.calls: list[tuple[str, str]] = []

    def lookup_transform(self, target, source, _time, _timeout=None):
        self.calls.append((target, source))
        x, y, yaw = self.frames[source]
        return SimpleNamespace(transform=SimpleNamespace(
            translation=SimpleNamespace(x=x, y=y, z=0.0),
            rotation=SimpleNamespace(x=0.0, y=0.0, z=math.sin(yaw / 2.0), w=math.cos(yaw / 2.0)),
        ))


def _fake_survey(robot=(5.7, 0.0, 0.0), lidar_offset=(-0.316, 0.110), front_range=2.316):
    rx, ry, yaw = robot
    c, s = math.cos(yaw), math.sin(yaw)
    lx = rx + c * lidar_offset[0] - s * lidar_offset[1]
    ly = ry + s * lidar_offset[0] + c * lidar_offset[1]
    fake = SimpleNamespace(
        _vision={"obstacle_class": "net", "obstacle_source": "neural_court_scene",
                 "obstacle_confidence": 0.9},
        _vision_received_at=100.0,
        _now=lambda: 100.1,
        _front_range_m=front_range,
        _robot_x=rx, _robot_y=ry, _robot_yaw=yaw,
        _scan_frame_id="lidar_link",
        _tf_buffer=_Tf({"lidar_link": (lx, ly, yaw), "base_link": (rx, ry, yaw)}),
        _locked_net=None,
        get_logger=lambda: SimpleNamespace(info=lambda *_: None),
        _record_event=lambda *_a, **_k: None,
    )
    fake._sensor_pose_in_map = lambda: survey.CourtSurveyV2Node._sensor_pose_in_map(fake)
    return fake


def test_net_lock_range_is_measured_from_the_lidar_not_base_link():
    # Net at map x 8.000; base_link at 5.7, so the C1 (x-0.316) reads 2.616.
    fake = _fake_survey(front_range=2.616)
    assert survey.CourtSurveyV2Node._try_lock_net(fake) is True
    lock = fake._locked_net
    assert lock["map_x_m"] == pytest.approx(8.0, abs=1e-3)
    # centre on the robot heading line, not on the off-axis C1 line (y+0.110)
    assert lock["map_y_m"] == pytest.approx(0.0, abs=1e-3)
    assert (lock["sensor_x_m"], lock["sensor_y_m"]) == pytest.approx((5.384, 0.110), abs=1e-3)
    assert ("map", "lidar_link") in fake._tf_buffer.calls


def test_net_lock_follows_the_lidar_when_the_robot_is_turned():
    yaw = math.radians(30.0)
    fake = _fake_survey(robot=(1.0, 2.0, yaw), front_range=2.0)
    assert survey.CourtSurveyV2Node._try_lock_net(fake) is True
    # the C1 is 0.316 m behind base_link along the heading -> base reach 1.684
    assert fake._locked_net["map_x_m"] == pytest.approx(1.0 + 1.684 * math.cos(yaw), abs=1e-3)
    assert fake._locked_net["map_y_m"] == pytest.approx(2.0 + 1.684 * math.sin(yaw), abs=1e-3)


def test_net_lock_waits_for_the_lidar_tf():
    fake = _fake_survey()
    fake._scan_frame_id = ""
    assert survey.CourtSurveyV2Node._try_lock_net(fake) is False
    assert fake._locked_net is None
