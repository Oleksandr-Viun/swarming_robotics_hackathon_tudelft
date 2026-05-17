# Architecture — TU Delft Swarm Robotics Hackathon 2026, Team D97B27

This document describes the technical design of the winning entry. For the high-level summary see the top-level [`README.md`](../README.md).

---

## 1. Robot roles & topology

| Role     | Count | Hardware                                 | Responsibilities                                                                 |
|----------|-------|------------------------------------------|----------------------------------------------------------------------------------|
| Master   | 1     | MIRTE + 4-DOF gripper arm                | Receive objectives, plan global path on shared map, execute pickup, deliver      |
| Pioneer  | 3     | MIRTE, no gripper                        | Randomized arena traversal with ultrasonic obstacle avoidance, build shared map, detect AprilTag objectives, broadcast both |

Master is `team_id=3, robot_id=0`. Pioneers are `robot_id ∈ {1, 2, 3}`.

All robots run independent ROS 2 nodes locally and communicate **only through the shared server** — there is no direct ROS topic exchange between robots. The shared spatial data acts as the coordination layer; no peer-to-peer task allocation or role swapping is required.

---

## 2. Communication protocol

Implemented by the hackathon-provided `library.communication.Communication` class. Two layers:

### 2.1 Server-pushed callbacks

The server runs the **overhead-camera AprilTag tracker** and pushes global poses to each robot at ~10 Hz:

```python
on_receive_location(x, y, angle, visible, last_seen)
```

`visible=False` when the overhead camera momentarily loses the tag. Every Master navigation loop checks this and stops the robot if true — see §4.3.

### 2.2 Robot-to-robot custom messages

Robots publish discoveries via:

- `on_receive_objective(from_team_id, from_robot_id, tag_id, x, y, angle, visible, last_seen)` — Pioneer found an AprilTag-marked objective; tag IDs in range 21–28 indicate scoreable objectives.
- `on_receive_custom(..., internal_type, bytes)` — custom binary messages:
  - **`internal_type=5`** with `struct.pack("<HH", row, col)` — Pioneer reports a newly-explored free cell. Master's handler calls `nav.planner.set_cell_free(row, col)` to live-update its A* grid (`test_simple_comms.py:36`, `test_hackathon.py:58`).
  - `internal_type=12` with `struct.pack("<BB", a, b)` — generic small message slot.

Binary-packed messages keep server bandwidth minimal compared to JSON, important when three Pioneers stream map updates simultaneously.

> **Snapshot caveat:** the `set_cell_free` method is called from two locations in this snapshot but is not defined in the `AStarPlanner` class file as committed. The competition-day planner had this method; this repo snapshot is missing that addition. The design and intent are unambiguous from the two callsites, the pptx strategy slides, and the Pioneer-side `map.py`; the exact runtime version was lost.

---

## 3. Coordinate frames

The arena coordinate system from the overhead camera is **mirrored on X** relative to the Master's drive frame. Each navigator instance configures `mirror_x` and rewrites incoming poses:

```python
self.pos_x = self.arena_width - x
self.heading = math.pi - angle
```

(`navigate_master.py:74`)

Pioneers use the camera-native frame. Master uses the mirrored frame so that "forward" on the gripper side matches positive X.

---

## 4. Master subsystem

### 4.1 Top-level state machine

`hackathon_2.py:110` — `MainStateMachine`:

```
        ┌──────────────┐
        │ move_to_obj  │ ◀────┐
        └──────┬───────┘      │
               │              │
        ┌──────▼──────┐       │
        │ pickup FSM  │       │
        └──────┬──────┘       │
               │              │
        ┌──────▼──────┐       │
        │ dropoff FSM │       │
        └──────┬──────┘       │
               │              │
        ┌──────▼──────┐       │
        │move_to_start│ ──────┘
        └─────────────┘
```

Each state has an explicit timeout (60–180 s); on expiry the FSM bumps to `move_to_start` rather than risking a stale-state lockup. **Nested FSMs** at the `pickup` and `dropoff` slots are what give the architecture its fault tolerance — a failed grasp retries within the Pick-Up sub-FSM without resetting the whole task cycle.

### 4.2 Global navigation: `GlobalNavigator` (`navigate_master.py`)

- **Map source (design):** static polygon-obstacle map from `arena_map.json` at init, **live-updated** by Pioneer `set_cell_free` calls during the run. The collective Pioneer traversal "carves out" the navigable interior as exploration progresses.
- **Planner:** `AStarPlanner` (see §6) with `robot_radius=0.45 m` — larger than the Pioneers' typical 0.35 m, accounting for the gripper-extended footprint.
- **Localization:** overhead-camera poses via `on_receive_location`. `wait_for_location()` blocks until first visibility.
- **Execution loop (`navigate_to`):**
  1. **Timeout check** — return False if `now - start_time > timeout`
  2. **Visibility check** — if camera lost lock, stop and wait (don't dead-reckon blind across the arena)
  3. **Path pruning lookahead** — scan from the path tail backward; if any later waypoint is within 0.25 m, jump there directly. Keeps the robot moving forward when waypoints are tightly clustered.
  4. **Mecanum holonomic control** — compute relative angle to next waypoint, decompose into `vx = speed·cos(θ)`, `vy = speed·sin(θ)`, drive sideways without rotating the chassis.
  5. **Rotation lock** — separately maintain `heading ≈ 0` via `angular_z = kp_angular · (0 - heading)` clamped to ±0.6 rad/s. Decoupling translation from rotation simplifies the Mecanum control surface significantly.

### 4.3 Camera-lost recovery

The overhead AprilTag tracker drops lock during partial occlusion (Master's own body, other robots crossing). The navigator's response:

```
if not self.visible:
    self.robot.drive(0.0, 0.0, 0.0)   # halt
    time.sleep(0.1)
    continue                          # keep checking
```

No fallback to onboard odometry for global nav — odometry alone drifts too quickly on a Mecanum drive to be trusted for cross-arena movement. The fallback is **stop and wait for re-lock**, which the camera typically achieves in <1 s.

Onboard odometry IS used — but only for short-range close-up movement during pickup (see §5.3), where drift over 0.5–1.0 m is acceptable.

### 4.4 Waypoint navigator (v2)

`waypoint_navigator.py` is the later iteration used by `hackathon_2.py`. Adds **local ultrasonic obstacle avoidance** on top of camera-driven goal-following:

- Three states: normal-pathing, `avoid` (rotate around obstacle), `emergency` (full stop)
- Speed reduction near obstacles even outside `avoid` mode
- Skips the global A* — relies on direct vector-to-goal + local avoidance. Simpler and more robust than A* when the obstacle map is uncertain mid-run.

---

## 5. Pickup subsystem — the win condition

`test_all.py:194` — `PickupStateMachine`. **No other team implemented autonomous pickup.** This was the decisive +0.5 pt/delivery margin.

### 5.1 State diagram

```
   ┌──────┐
   │rotate│ ──────────────────┐  (rotate in place until green seen)
   └───┬──┘                   │
       │ green detected       │ no detection
       ▼                      ▼
   ┌──────┐               (continue rotating)
   │detect│ ─── d > 0.65m ──▶ ┌──────────────────────┐
   └───┬──┘                   │ pos_adjust_not_final │
       │ d ≤ 0.65m            └──────────┬───────────┘
       ▼                                 │
   ┌─────────────┐  ◀──────────────────  ┘ (drive to 0.60m)
   │ pos_adjust  │
   └─────┬───────┘
         │ (drive to 0.25m)
         ▼
       ┌────┐
       │grab│   (open gripper → lower arm → creep 5cm fwd → close gripper → lift to mid-pose)
       └─┬──┘
         │
         ▼
     ┌──────┐
     │verify│ ─── green area ≥ 25% of frame ──▶ ready ✓
     └──┬───┘
        │ green area too small
        ▼
   (back off 0.9 m, retry — max 3 attempts before bailing to rotate)
```

### 5.2 Vision pipeline (`get_dist_and_angle`, `test_all.py:40`)

Objects are green-colored crates. Pipeline:

1. HSV conversion, threshold `H∈[37,90], S∈[41,255], V∈[19,255]` (calibrated on hackathon-day lighting — see calibration triangulation image)
2. Largest contour by area
3. Bounding box → `x_center`, `y_bottom`
4. **Angle estimate:** `(x_center - W/2) / W · H_FOV` — pure pinhole-camera horizontal angle
5. **Distance estimate:** uses known camera height (13 cm) and downward ray angle to floor:
   ```
   ray_angle = (0.5 - y_bottom/H) · V_FOV
   distance  = H_CAMERA / tan(TILT - ray_angle)
   ```
   Returns `None` if the object's bottom edge is above the horizon (invalid — would imply object is above floor or camera is mis-aimed).

Camera calibration constants (`H_CAMERA=13 cm`, `V_FOV=40°`, `H_FOV=62°`) were measured experimentally on-site using a top-down triangulation shot:

![Calibration triangulation](images/calibration_triangulation.jpg)

*Master (left) with three green tree-objectives at measured distances and a grey tape roll as scale reference. The downward camera angle, floor-distance constants, and HSV bounds were fit to make this scene parse correctly.*

### 5.3 Sensor-fusion handoff (`align_objective.py`)

The key technique. Camera-based distance estimates degrade rapidly at close range (object fills the frame, bounding-box edges become unreliable). So:

1. Take **one** camera reading at moderate range (~0.4–0.6 m) — gives `initial_dist`, `initial_yaw`.
2. **Switch to odometry**: project the world-frame target point at `(pos_x + d·cos(yaw), pos_y + d·sin(yaw))`.
3. Drive to the target using **only** odometry, since over <1 m of distance odometry drift is negligible (~cm).

This is the same pattern as **AOCS sensor fusion in spacecraft**: coarse, externally-referenced sensor (star tracker / overhead AprilTag) provides the bootstrap pose, then high-rate dead-reckoning (gyro integration / wheel odometry) takes over for the precision-critical maneuver phase.

### 5.4 Grab + verify

`_grab` (`test_all.py:289`):
1. Open gripper
2. Lower arm to `ARM_PICKUP_POS = [0.0, -1.8, -1.3, 1.0]`
3. Creep **forward 5 cm** under odometry P-control (clamp 0.1 m/s) — last close-in to slip the gripper around the object
4. Close gripper via async `ros2 action send_goal` subprocess — non-blocking to keep ROS spinning
5. Lift to mid-pose `ARM_PICK_MIDDLE_POS_AFTER_CHECK`

`_verify` (`test_all.py:337`):
1. Lift arm to `ARM_CHECK_POS = [0.0, -0.4, -1.5, -2.0]` — points gripper at camera
2. Run green-mask detection on the current frame
3. If green pixel area ≥ `0.25 · 640 · 480` (~25% of frame), object is confirmed in gripper → success
4. Otherwise: back off 0.9 m (3 s × 0.3 m/s), reset arm, retry from `detect` state
5. After 3 failed retries, give up on this objective and return to `rotate` (look for a different one)

The verify step is what makes the system **reliable enough to score consistently** rather than just attempt-and-hope. Most failed grabs are caught and retried within the same approach.

What the Master's HSV-filtered camera sees during the `detect` state:

![Master detector POV](images/master_pov_detection.png)

---

## 6. Path planner — `AStarPlanner` (`path_planner.py`)

- **Grid resolution:** 0.1 m (60×100 cells for a 6×10 m arena)
- **Obstacle source:** polygons from `arena_map.json`. Each cell is occupied if:
  - its center is inside any obstacle polygon (point-in-polygon test), OR
  - it is within `robot_radius` of any polygon vertex (cheap inflation by vertex distance — skips edge-segment distance for speed; conservative since vertex-only inflation slightly overestimates the obstacle footprint, which is the safe error direction)
- **Live updates:** Pioneer reports of free cells (`set_cell_free`, see §2.2) clear occupied flags as exploration proceeds.
- **Search:** standard A*, 8-connected neighbors, Euclidean cost (`√2` for diagonals) and Euclidean heuristic. Priority queue via `heapq`.
- **Output:** list of world-frame waypoints from start to goal, or `None` if unreachable.

Inflation uses `robot_radius=0.45 m` for the Master (vs 0.35 m default) — the gripper-extended footprint is wider than the chassis.

---

## 7. Pioneer mapping — `SimpleMapper` (`map.py`)

Each Pioneer builds a local occupancy grid from LIDAR scans, then broadcasts free-cell discoveries to the Master:

- **Grid:** 6.0 × 7.5 m at 0.05 m resolution (120×150 cells)
- **Update rule:** Bayesian log-odds with `l_occ=+0.7` for hit cells, `l_free=-0.4` for cells along the ray to the hit. Clipped to `[-2.0, +3.5]` to bound certainty and allow recovery from stale info.
- **Ray tracing:** Bresenham from robot grid cell to LIDAR hit cell.
- **Pose source:** overhead camera (same `on_receive_location` callback). Map updates skipped when `visible=False` to avoid corrupting the grid with stale pose.
- **Published as:** `nav_msgs/OccupancyGrid` on `/map` for RViz visualization, **and** as binary cell-update messages to the Master (`internal_type=5`).
- **Emergent coverage:** three Pioneers running randomized traversal in parallel cover the arena meaningfully faster than any single robot — and the resulting map is richer than any one Pioneer's local view.

Example run-end state (from a test run, ~5 min exploration):

![Pioneer-built occupancy grid](images/map_occupancy_grid.jpg)

*Bayesian log-odds map after exploration. White = free, black = unknown/occupied, `R` = Pioneer pose. Notice the structured corridors that emerge — these are the navigable paths the Master plans through.*

![Coverage progress](images/map_explored_coverage.jpg)

*Coverage telemetry from the same run: 64.4% of cells visited (715 / 1110). Rendered in-terminal for live monitoring during runs.*

---

## 8. Design decisions worth calling out

| Decision                                                        | Why                                                                                                          |
|-----------------------------------------------------------------|--------------------------------------------------------------------------------------------------------------|
| Pioneer-fused free-space map, not Pioneer-built SLAM            | Pioneers report only *traversed-safe* cells. Master's A* always has a conservative, monotonically-growing free-space set. No SLAM consistency problems.  |
| Camera-lost ⇒ halt (not dead-reckon)                            | Mecanum odometry drift is too large for cross-arena navigation. Stopping and waiting is reliably <1 s.       |
| Hybrid camera→odometry for pickup approach                      | Camera distance estimate becomes unreliable inside ~0.4 m. Odometry over <1 m is precise enough.             |
| Nested FSMs with explicit retry-and-verify, not open-loop       | Single source of the win condition: failed grabs get caught and retried within the same approach.            |
| Binary-packed comms (`struct.pack`), not JSON                   | Shared server bandwidth budget; three Pioneers stream map updates simultaneously.                            |
| Async gripper command via `subprocess`                          | The blocking gripper API stalled the main ROS loop. `subprocess.Popen` decouples it.                         |
| Vertex-only obstacle inflation in A*                            | Cheap; over-estimates obstacle footprint (safe direction); good enough on a 0.1 m grid.                      |
| Rotation lock decoupled from translation in Mecanum nav         | Lets us write `vx, vy` as a 2D vector problem independent of heading control, halving the controller complexity. |
| Randomized Pioneer traversal, not coordinated coverage          | Zero coordination overhead, emergent coverage works in practice, no failure modes from coordination breaking. |

---

## 9. Engineering story — "the bear and the runners"

Five teams entered. Several built more elaborate cooperative planning systems — distributed task auctions, dynamic role reassignment, joint coverage optimization. They worked in simulation and broke on the MIRTE platform under real-world sensor noise.

Our team made a deliberate call early to **minimize moving parts and minimize coordination**. Pioneers do one thing each (random-walk, log free cells, look for AprilTags). Master does one thing (drive to next reported tag, pick up, deliver). No re-planning, no role swaps, no joint optimization, no peer-to-peer task allocation — Pioneers and Master communicate only by writing to and reading from the shared spatial state.

The autonomous pickup was the only place we spent the complexity budget. That bet paid off: it was the +0.5 pt/delivery multiplier no other team could match, and it ran reliably enough to score consistently across the competition run.

The principle generalizes: **a simple system that runs end-to-end on competition day beats an elaborate one that breaks on first contact with the platform.**
