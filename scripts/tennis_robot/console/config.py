"""Configuration and small shared helpers for the console package.

ConsoleConfig is the single injected value object that carries filesystem
locations and the ROS environment prelude into the services, so no service
derives paths from ``__file__`` or reads globals.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path


# Sourced before every ros2 CLI invocation so the console works from a plain
# (non-ROS) shell. Kept here so the survey launch and nav-test paths agree.
ROS_PRELUDE = (
    "source /opt/ros/humble/setup.bash; "
    "source /ros2_ws/install/setup.bash; "
)

# Typical webcam horizontal FOV; tune if monocular distance estimates are off.
WEBCAM_FOV_DEG = 60.0

STATIC_CONTENT_TYPES = {
    ".css": "text/css; charset=utf-8",
    ".js": "application/javascript; charset=utf-8",
    ".html": "text/html; charset=utf-8",
}


def text_from_subprocess_output(value: object) -> str:
    """Normalise subprocess stdout (str/bytes/None) to text."""
    if value is None:
        return ""
    if isinstance(value, bytes):
        return value.decode("utf-8", errors="replace")
    return str(value)


@dataclass(frozen=True)
class ConsoleConfig:
    """Injected paths + settings. Built once in the entrypoint."""

    root: Path
    host: str = "127.0.0.1"
    port: int = 8081
    ros_prelude: str = ROS_PRELUDE

    @property
    def runtime_dir(self) -> Path:
        return self.root / "runtime"

    @property
    def court_boundary_path(self) -> Path:
        return self.runtime_dir / "court_boundary.json"

    @property
    def court_survey_live_path(self) -> Path:
        return self.runtime_dir / "court_survey_live.json"

    @property
    def robot_path_path(self) -> Path:
        return self.runtime_dir / "robot_path.json"

    @property
    def survey_log_path(self) -> Path:
        return self.runtime_dir / "court_survey_control_panel.log"

    @property
    def html_path(self) -> Path:
        return self.root / "scripts" / "control_panel.html"

    @property
    def static_dir(self) -> Path:
        return self.root / "scripts" / "control_panel"
