# ΑΡΧΕΙΟ — μη ενεργό half-court scan route overview

> Δεν χρησιμοποιείται ως specification ή οδηγός υλοποίησης για `collect_route`.
> Το ενεργό specification είναι το [Ενεργός οδηγός συνεχούς διαδρομής](../collection-route-rules-el.md).

# Half-Court Scan And Route Overview

Last checked: 2026-05-27

This document defines the first mapping and route-planning approach for the tennis robot collector. The goal is to avoid reactive back-and-forth movement by scanning one half of the court, building a temporary ball map, planning a collection route, and refreshing that map during collection.

The intended first hardware path uses a low-mounted Waveshare/Slamtec RPLIDAR C1 360-degree 2D LiDAR for real-time court/obstacle mapping, with the top-mounted OAK-D S2 as the primary tennis-ball detector:

- Low 360-degree LiDAR sweep for court boundaries, obstacle edges, net/fence/wall clearance, people/bags, route costmaps, and shadow zones.
- Top OAK-D RGB image for tennis-ball detection and color/shape classification.
- OAK-D stereo depth for distance, clusters, and balls in LiDAR shadow zones.
- IMU as part of future pose estimation.
- Wheel encoders, when the mobile base is selected, for odometry.

## Strategy

Split collection into two independent court phases:

1. Collect all reachable balls on one side of the net.
2. Move/reset to the other side.
3. Repeat the same scan, route, refresh, and collect loop.

The first implementation does not require full SLAM. It uses court-bounded local mapping:

```text
LiDAR scan + robot pose -> obstacle/cost map + shadow zones in court coordinates
OAK-D RGB/depth + robot pose -> ball positions in court coordinates
```

The LiDAR does not try to be a tennis-ball detector. A 6.7 cm ball is too small and inconsistent for reliable 2D LiDAR returns. Its job is to keep the planner's map fresh without a slow survey phase, and to tell the OAK-D where the scene is blocked, cluttered, or worth a targeted look.

## High-Level Flow

```mermaid
flowchart TD
    A["Start collection command"] --> B["Select court phase"]
    B --> C["Initial half-court scan"]
    C --> D["Build ball map"]
    D --> E["Plan route for visible/reachable balls"]
    E --> F["Collect next ball"]
    F --> G{"Refresh needed?"}
    G -- "No" --> H{"More planned balls?"}
    G -- "Yes" --> I["Local refresh scan"]
    I --> J["Merge observations into ball map"]
    J --> K["Replan remaining route"]
    K --> H
    H -- "Yes" --> F
    H -- "No" --> L{"Other half pending?"}
    L -- "Yes" --> B
    L -- "No" --> M["Collection complete"]
```

## Perception-To-Map Pipeline

```mermaid
flowchart LR
    LIDAR["Low 360-degree LiDAR scan"] --> COST["Obstacle / clearance costmap"]
    LIDAR --> SHADOW["Shadow zones / occluded sectors"]
    RGB["Top OAK-D RGB frame"] --> DET["Visual ball detector"]
    DEPTH["OAK-D stereo depth"] --> RANGE["Depth crop / median range"]
    DET --> VOBS["Visual/depth observations"]
    RANGE --> VOBS
    SHADOW --> LOOK["Targeted OAK-D look/refresh"]
    LOOK --> DET
    POSE["robot pose x/y/yaw"] --> WORLD["Court coordinate projection"]
    VOBS --> WORLD
    WORLD --> MAP["Ball map"]
    POSE --> COST
    MAP --> ROUTE["Route planner"]
    COST --> ROUTE
```

The simulated code already has the core projection shape in `controllers/ball_detector/perception.py`:

```text
BallDetection -> BallObservation -> robot XY -> world/court XY
```

The physical robot should preserve this contract even if the camera detector changes from HSV thresholding to a neural model later. LiDAR should feed traversability, clearance, and shadow-zone data; OAK-D observations should feed the ball map.

## Ball Map State

```mermaid
stateDiagram-v2
    [*] --> detected
    detected --> planned
    planned --> approaching
    approaching --> collected
    approaching --> missing
    approaching --> blocked
    missing --> detected: refresh scan sees it again
    blocked --> planned: obstacle clears / route changes
    collected --> [*]
```

Minimum fields for the first implementation:

| Field | Purpose |
|---|---|
| `id` | Stable temporary ball identity. |
| `x_m`, `y_m` | Court position estimate. |
| `confidence` | Detection reliability or freshness score. |
| `last_seen_s` | Helps age out stale detections. |
| `state` | `detected`, `planned`, `approaching`, `collected`, `missing`, or `blocked`. |
| `phase` | Court side / half-court ownership. |

## Route Selection

The first route planner can be greedy instead of a perfect TSP solver. It should choose the next reachable ball with the lowest weighted cost:

```text
cost = path_distance
     + turn_cost
     + stale_detection_penalty
     + obstacle_penalty
     + edge_penalty
     + lidar_clearance_penalty
     - confidence_bonus
```

With RPLIDAR C1 installed, `obstacle_penalty`, `edge_penalty`, and `lidar_clearance_penalty` should come from the 2D costmap instead of camera-only heuristics. The route should be recalculated after a refresh scan, after a blocked route, or after a missing ball.

## Sensor Roles

| Sensor | Main question | Algorithm output |
|---|---|---|
| Waveshare/Slamtec RPLIDAR C1 low 360-degree LiDAR | Where can the robot move safely, and what areas are blocked from view? | Occupancy/cost map, wall/net/fence clearance, obstacle inflation, shadow zones. |
| Top OAK-D S2 | Where are the tennis balls? | Ball detections, RGB-D observations, depth, bearing, ball map. |
| Encoders + IMU | Where is the robot now? | Odometry and pose prediction/correction. |

The LiDAR is not the primary tennis-ball detector. A 2D scan can miss balls, merge them into nearby geometry, or shadow everything behind the first object in the scan. The OAK-D stays responsible for ball detection, while the LiDAR keeps the obstacle map live.

## Scan Policy

```mermaid
flowchart TD
    A["Initial scan"] --> B["2-4 scan viewpoints per half"]
    B --> C["Map visible balls"]
    C --> D["Plan route"]
    D --> E["Collect batch"]
    E --> F{"Batch size reached?"}
    F -- "Yes" --> G["Short refresh from current pose"]
    F -- "No" --> H{"Target lost / obstacle?"}
    H -- "Yes" --> G
    H -- "No" --> E
    G --> I["Merge + replan remaining balls"]
    I --> E
```

Recommended starting defaults:

| Parameter | Initial value |
|---|---:|
| Initial scan viewpoints per half | 2-4 |
| Refresh interval | Every 3-5 collected balls |
| Missing-ball retries | 2 |
| Minimum detection confidence | Start permissive, tighten after real camera tests |
| Replan trigger | Refresh scan, blocked path, target missing, safety stop |

## Realistic Ball Distribution

The simulator and Webots ball generator should not use a purely uniform scatter by default. In a physical court, more balls tend to settle near:

- the net, after short/failed returns and balls rolling inward;
- the back court or wall/fence behind the baseline;
- with fewer balls staying in the open middle of the court.

The first biased randomizer uses this approximate mix:

| Zone | Approx. share |
|---|---:|
| Near net | 46% |
| Back court / baseline wall side | 36% |
| Service-line / transition area | 12% |
| Uniform outliers | 6% |

This distribution is still deterministic by seed, but it makes route planning tests more realistic because the planner must handle clustered balls instead of evenly spaced targets.

## First Implementation Scope

For the first implementation in the browser route simulator:

1. Add a two-phase court mode.
2. Generate balls across both halves.
3. Plan phase A and phase B separately.
4. Add explicit phase scan and refresh scan events.
5. Keep telemetry for phases, scans, replans, distance, blocked balls, and collection order.
6. Add a LiDAR obstacle/shadow-zone mode and compare camera-only planning against OAK-D ball detection plus live LiDAR costmaps.

For benchmark validation:

1. Run 100 deterministic random scenarios with 40 balls each.
2. Measure average collection time, distance, collectable rate, replans, and scan events.
3. Estimate miss risk for balls close to net, wall/fence, or obstacles.
4. Compare realistic ball bias against uniform scatter.
5. Compare `rescan_every`, travel speed, safety buffer, and candidate-window settings.
6. Export per-decision candidate rows with `--training-out` for a future next-ball ranking model.

The first useful machine-learning dataset is a learning-to-rank table. Each planning decision writes one row per candidate ball:

```text
robot pose + ball pose + route/risk features -> selected
```

This lets a model learn the planner's current policy first. Later, physical-run outcomes can replace or augment the label with real success/failure and pickup time.

For the Webots/physical controller after that:

1. Publish detected balls in court coordinates.
2. Add a persistent ball map.
3. Feed the current map into a route planner.
4. Convert the selected next ball into the existing `scan -> align -> approach -> capture` state machine.

## Open Decisions

| Decision | Default for now |
|---|---|
| Side order | Start on the left/west half in simulation. |
| Net crossing | Treat halves as independent phases; do not plan through the net. |
| Full SLAM | Defer until court-line correction plus odometry is insufficient. |
| LiDAR | Use the Waveshare/Slamtec RPLIDAR C1 mounted low on the robot for 360-degree obstacle/court mapping and shadow-zone generation, not primary ball detection. |
| Perfect route solver | Defer; use greedy route with penalties and refresh scans. |
