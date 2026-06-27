"""Package shim for the ``tennis_robot`` namespace as seen from ``scripts/``.

The console package (``tennis_robot.console``) lives here under ``scripts/`` so
it sits next to the other console-only code, while the ROS modules it depends on
(``tennis_robot.control_bus``, ``tennis_robot.perception``, ``tennis_robot.survey``)
physically live in ``ros2_ws/src/tennis_robot/tennis_robot/``.

Without this file, ``scripts/tennis_robot`` would be a *namespace* package that
shadows the real ROS package whenever ``scripts/`` is first on ``sys.path`` and
ROS is not sourced — so ``import tennis_robot.control_bus`` would fail. We make
this a regular package and extend its search path to include the ROS source
tree, so both halves resolve (and the console boots without sourcing ROS, since
control_bus is pure-Python). When ROS *is* sourced the same submodules resolve
from source; only ``console`` is unique to this location.
"""

import os

__path__.append(
    os.path.abspath(
        os.path.join(
            os.path.dirname(__file__), "..", "..",
            "ros2_ws", "src", "tennis_robot", "tennis_robot",
        )
    )
)
