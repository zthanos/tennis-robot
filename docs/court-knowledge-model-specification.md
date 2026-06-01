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
