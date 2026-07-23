"""Court survey entry point — delegates to the v2 LiDAR-occupancy survey node.

The previous dead-reckoning perimeter FSM has been replaced by
``court_survey_v2_node`` (occupancy map → Court Knowledge Model; see
docs/survey/court-survey-v2-spec-el.md). Git history retains the old implementation.

This thin module keeps the installed console entry
``court_survey_mission_node:main`` valid regardless of colcon entry-point
regeneration — it simply re-exports the v2 main().
"""

from __future__ import annotations

try:
    from tennis_robot.court_survey_v2_node import main
except ModuleNotFoundError:  # running from source tree
    from court_survey_v2_node import main


if __name__ == "__main__":
    main()
