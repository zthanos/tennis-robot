# Court Knowledge Model Specification

## Purpose

The purpose of the Court Knowledge Model process is to enable the robot to
autonomously discover, understand, and map a tennis court environment before
any operational activities begin.

The Court Knowledge Model process is responsible only for environment
understanding and knowledge acquisition.

It does not perform ball collection, route optimization, coverage planning, or
task execution.

The output of the Court Knowledge Model process is a complete environmental
representation that can later be consumed by other subsystems.

## Objectives

The Court Knowledge Model process must determine:

- Court dimensions.
- Court boundaries.
- Fence locations.
- Obstacles inside and around the court.
- Free movement corridors.
- Entry and exit points.
- Clearance between court lines and surrounding fences.
- Areas accessible to the robot.
- Areas inaccessible to the robot.

The Court Knowledge Model process creates environmental knowledge only.

## Inputs

### Sensors

#### OAK-D Camera

Used for:

- Tennis court line detection.
- Court boundary identification.
- Visual localization support.
- Verification of detected court geometry.

#### LiDAR

Used for:

- Obstacle detection.
- Fence detection.
- Distance measurements.
- Environment mapping.
- SLAM and localization.

## Preconditions

Before the Court Knowledge Model process begins:

- Robot is placed somewhere inside the tennis court.
- Sensors are operational.
- LiDAR calibration completed.
- OAK-D calibration completed.
- Battery level above minimum operational threshold.

No prior map is assumed.

No knowledge of the environment is available.

## Court Knowledge Strategy

### Step 1 - Court Boundary Detection

The robot uses the OAK-D camera to locate the external court boundary lines.

The objective is to identify:

- Left sideline.
- Right sideline.
- Baseline A.
- Baseline B.

The robot establishes an initial court reference frame.

### Step 2 - Positioning Outside The Court Boundary

To map the complete environment, the robot must position itself outside the
playable area.

The robot navigates so that:

- The external court boundary line remains on its right side.
- The robot travels in the clearance area between the court line and the
  surrounding fence.

This guarantees visibility of:

- Court lines.
- Fence locations.
- Clearance distances.

### Step 3 - Perimeter Traversal

The robot performs a complete perimeter traversal around the court.

The robot continuously:

- Detects court lines using OAK-D.
- Maps surroundings using LiDAR.
- Records obstacles.
- Measures fence distances.
- Updates localization.

The traversal must cover the entire perimeter.

#### Required Map Court Traversal FSM

The Map Court traversal must be deterministic and sensor-driven. It must not use
simulator/world-specific waypoints or pre-recorded court coordinates. The same
behavior must run in simulation and on the physical robot when given equivalent
OAK-D, LiDAR, and localization inputs.

The required traversal order is:

1. `FIND_FIRST_OBSTACLE`
   - Drive forward from the current robot heading until the first obstacle is
     close enough to classify.
   - Do not begin by spinning in place and do not begin from an arbitrary
     wall-following loop.
   - Classify the first obstacle using OAK-D close-range evidence and LiDAR
     front-sector evidence.
   - OAK-D-only net classification must not be accepted immediately at startup;
     it requires either explicit visual classification or a minimum forward
     travel distance so stale/near-field depth cannot start the wrong path.
   - If the first obstacle is the net, continue with the net-first traversal.
   - If the first obstacle is a fence, begin the left-turn perimeter traversal
     from that fence and complete only after returning to the same reference
     point.

2. `APPROACH_NET`
   - Drive toward the detected net until the robot reaches a safe standoff
     distance of 0.10 m from the measured net boundary.
   - Maintain obstacle avoidance using LiDAR and OAK-D depth.

3. `TURN_LEFT_AT_NET`
   - When the robot is near the net, turn left.
   - Record this first net-left-turn location as the loop-completion reference.

4. `FOLLOW_NET_TO_FENCE`
   - Move parallel to the net.
   - Continue until LiDAR/OAK-D indicates the robot is near the surrounding
     fence or a corner constraint.

5. `TURN_LEFT_AT_FENCE`
   - At the fence/corner, turn left.
   - The turn trigger must come from sensor evidence, not from an absolute
     coordinate.

6. `FOLLOW_FENCE_TO_NEXT_FENCE`
   - Follow the fence boundary until the next fence/corner is detected.
   - Record fence geometry, obstacles, clearance, and blocked passages while
     moving.

7. `TURN_LEFT_AT_FENCE`
   - Turn left at the next fence/corner and continue perimeter traversal.

8. `FOLLOW_FENCE_TO_NET`
   - Continue until the net is detected again from the opposite approach.

9. `CROSS_NET_ON_RIGHT_SIDE`
   - Cross around the net through the right-side available gap.
   - Use LiDAR side clearances and OAK-D depth to keep the robot centered in
     the gap and away from the net post/fence.

10. `FOLLOW_SECOND_HALF_PERIMETER`
    - Repeat the same left-turn perimeter pattern for the other half of the
      court: fence -> left turn -> fence -> left turn -> net.

11. `COMPLETE_AT_FIRST_NET_TURN_REFERENCE`
    - Complete only when the robot returns near the first net-left-turn
      reference after traversing both halves.
    - Completion requires loop closure, sufficient traveled distance, valid
      sensor coverage, and successful map validation.

The state transitions must be based on named sensor events:

- `first_obstacle_net`
- `first_obstacle_fence`
- `net_detected`
- `near_net`
- `near_fence`
- `corner_detected`
- `right_side_net_gap_detected`
- `gap_crossed`
- `loop_closed`

The Map Court process must fail with a structured reason if any required sensor
event cannot be detected with sufficient confidence.

### Step 4 - Environment Mapping

During traversal the robot creates:

#### Court Map

Contains:

- Court dimensions.
- Court orientation.
- Court boundaries.

#### Obstacle Map

Contains:

- Fixed obstacles.
- Temporary obstacles.
- Fence structures.

#### Clearance Map

Contains:

- Distance from court boundaries to fences.
- Narrow passages.
- Areas too small for traversal.

#### Accessibility Map

Contains:

- Reachable regions.
- Unreachable regions.
- Traversable paths.

### Step 5 - Validation Pass

After the initial environment mapping pass, the robot validates:

- Court perimeter closure.
- Consistency of dimensions.
- Completeness of obstacle detection.
- Fence continuity.

Any missing area triggers another Court Knowledge Model attempt for the missing
segment.

## Outputs

The Court Knowledge Model process produces a Court Knowledge Model containing:

### Court Geometry

- Length.
- Width.
- Orientation.
- Court boundaries.

### Fence Geometry

- Fence positions.
- Fence distances from court.

### Obstacles

- Obstacle locations.
- Obstacle dimensions.

### Accessibility Data

- Traversable areas.
- Non-traversable areas.
- Entry points.
- Exit points.

### Navigation Data

- Safe movement corridors.
- Boundary-following routes.

## Constraints

The Court Knowledge Model process:

- Must not collect balls.
- Must not perform coverage planning.
- Must not assume a navigation matrix exists.
- Must not require a predefined map.
- Must not depend on Collection logic.

The Court Knowledge Model process is responsible only for environmental
knowledge acquisition.

## Error Conditions

Court Knowledge Model creation fails if:

- Court boundaries cannot be detected.
- Full perimeter traversal cannot be completed.
- Localization confidence falls below acceptable thresholds.
- LiDAR mapping becomes inconsistent.
- Significant areas remain unexplored.
- Fence locations cannot be determined.

A failed Court Knowledge Model attempt must return a failure status and no
operational phase may start.

## Definition Of Done

The Court Knowledge Model process is considered complete only when all of the
following conditions are satisfied.

### Court Geometry

- Court length has been measured.
- Court width has been measured.
- Court boundaries have been identified.
- Court orientation has been established.

### Fence Mapping

- All surrounding fences have been detected.
- Fence locations have been mapped.
- Distance between court boundaries and fences has been measured.

### Obstacle Mapping

- All detected obstacles have been recorded.
- Obstacle locations have been stored in the environment map.

### Accessibility Analysis

- Traversable areas have been identified.
- Non-traversable areas have been identified.
- Entry and exit paths have been discovered.

### Coverage Completeness

- Entire court perimeter has been traversed.
- No unexplored perimeter segments remain.
- Environment map is fully connected.

### Validation

- Court geometry passes consistency checks.
- Fence geometry passes consistency checks.
- Localization confidence is above the minimum threshold.
- Mapping confidence is above the minimum threshold.

### Deliverables

The robot has produced:

- Court Geometry Model.
- Fence Geometry Model.
- Obstacle Map.
- Accessibility Map.
- Traversable Route Information.

### Completion Result

```text
Court Knowledge Model Status = SUCCESS
```

If any of the above conditions are not met:

```text
Court Knowledge Model Status = FAILED
Reason = <Failure Cause>
```

A failed Court Knowledge Model cannot be used by the Collection subsystem.
