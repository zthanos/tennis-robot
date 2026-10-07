"""Shared import path setup for the offline test suite.

``scripts/tennis_robot/__init__.py`` is a shim package that extends its own
``__path__`` to the ROS sources, so with ``scripts/`` on ``sys.path`` BOTH
``tennis_robot.console`` (console-only) and ``tennis_robot.<ros module>``
resolve.  Individual test modules only insert ``ros2_ws/src/tennis_robot``,
which binds the name ``tennis_robot`` to the ROS package alone — and because
that binding is process-wide, every later ``tennis_robot.console`` import in the
same run fails.

conftest runs before any test module, so we bind the name here: inserting
``scripts/`` is not enough on its own (a test module's own ``sys.path.insert(0,
...)`` still lands ahead of it), so we also import the package eagerly.  The
first successful ``import tennis_robot`` wins for the whole process, and the
shim resolves both halves — which makes the full-suite result match the
per-file result.
"""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))

import tennis_robot  # noqa: E402,F401  (pin the shim, see module docstring)
