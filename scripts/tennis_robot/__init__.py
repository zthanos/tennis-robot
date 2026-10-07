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

Extending ``__path__`` only covers *submodules*. The ROS package's own
``__init__`` also defines top-level symbols (``yaw_from_quaternion``), and
whichever ``__init__`` runs first wins for the whole process — so pinning this
shim used to break ``from tennis_robot import yaw_from_quaternion`` inside the
ROS nodes. We therefore re-export that ``__init__``'s public names here, which
lets one process import the console half and the ROS half together.
"""

import importlib.util
import os

_ROS_PACKAGE_DIR = os.path.abspath(
    os.path.join(
        os.path.dirname(__file__), "..", "..",
        "ros2_ws", "src", "tennis_robot", "tennis_robot",
    )
)

__path__.append(_ROS_PACKAGE_DIR)


def _reexport_ros_package_init() -> None:
    """Merge the ROS package ``__init__``'s public names into this namespace."""
    init_path = os.path.join(_ROS_PACKAGE_DIR, "__init__.py")
    if not os.path.isfile(init_path):
        # Installed-only deployment: the ROS half is already the sole __init__.
        return
    name = "tennis_robot._ros_package_init"
    spec = importlib.util.spec_from_file_location(name, init_path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    exported = getattr(module, "__all__", None)
    if exported is None:
        exported = [
            symbol for symbol, value in vars(module).items()
            if not symbol.startswith("_")
            and getattr(value, "__module__", None) == name
        ]
    globals().update({symbol: getattr(module, symbol) for symbol in exported})


_reexport_ros_package_init()
