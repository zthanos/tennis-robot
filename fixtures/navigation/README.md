# Navigation replay fixtures

These fixtures capture real robot telemetry so navigation changes can be
regression-tested against behavior that was observed in Gazebo.

## Survey fixtures

Run every survey replay fixture:

```powershell
.\.venv-win\Scripts\python.exe scripts\replay_navigation_fixtures.py
```

Run one fixture directly:

```powershell
.\.venv-win\Scripts\python.exe scripts\replay_ros2_lidar_survey.py fixtures\navigation\survey\find_baseline_fence_stuck_2026-06-05.jsonl
```

Check whether a recording contains a complete command trace:

```powershell
.\.venv-win\Scripts\python.exe scripts\replay_survey_cmd_vel.py runtime\survey_replay_latest.jsonl
```

Inside the ROS/Gazebo container, a command trace can be replayed to `/cmd_vel`
with:

```bash
python3 /workspace/scripts/replay_survey_cmd_vel.py /workspace/runtime/survey_replay_latest.jsonl --publish
```

Only recordings made after `cmd_linear_m_s` and `cmd_angular_rad_s` were added
to `survey_replay_record.py` can be replayed this way.

`find_baseline_fence_stuck_2026-06-05.jsonl` documents the run where the survey
confirmed the net, entered the 180-degree baseline leg, then drove into the
fence. The regression check now asserts that a required turn emits no forward
command until the turn is complete, and that queued decision processing does
not emit a fence-reached event in the same tick as an earlier survey event.

`baseline_full_turn_still_find_baseline_2026-06-05.jsonl` captures the next
case: the baseline 180-degree turn completes before the first forward command,
but the replay still remains in `find_baseline`. Use it for the next phase of
the survey state-machine fix.

`long_side_inside_court_timeout_2026-06-05.jsonl` is the first fixture with a
complete command trace. Live telemetry shows the robot turns to the long-side
leg before `left_sideline_to_fence_m` is recorded, then drives the long side at
y around -4.6 to -5.0 instead of staying clearly outside the sideline path.
