# hs_topview

Top-view camera based navigation control package.

## Runtime policy

- Use `point_move` as the only `/cmd_vel` publisher in production.
- Do not run `point_move` and `pid_controller` at the same time.
- `pid_controller` is kept only for legacy/simple debugging.

## Launch

```bash
ros2 launch hs_topview point_move.launch.py
```

Optional custom params:

```bash
ros2 launch hs_topview point_move.launch.py \
  params_file:=/path/to/point_move.yaml
```

Waypoint sequence sender launch:

```bash
ros2 launch hs_topview waypoint_sender.launch.py
```

Optional custom waypoint file:

```bash
ros2 launch hs_topview waypoint_sender.launch.py \
  waypoints_file:=/path/to/waypoints.yaml
```

Optional selected waypoint names:

```bash
ros2 launch hs_topview waypoint_sender.launch.py \
  selected_waypoints:=A,C,E
```

Optional A* planning mode:

```bash
ros2 launch hs_topview waypoint_sender.launch.py \
  use_astar:=true \
  start_waypoint:=R1C1 \
  goal_waypoint:=R3C5
```

## Inputs and outputs

- Current pose input: `/amr_pose` (`geometry_msgs/PoseStamped`, frame `map`)
- Goal input:
  - Action: `pinky1/actions/move_to_pid` (`pinky_interfaces/action/MoveToPID`)
  - Topic (optional): `/goal_pose` (`geometry_msgs/PoseStamped`)
- Velocity output: `/cmd_vel` (`geometry_msgs/Twist`)

## Goal modes

- Single goal topic mode: publish one `PoseStamped` to `/goal_pose`.
- Sequential waypoint mode: run `waypoint_sender` and it sends `MoveToPID` goals in order (e.g. `current -> A -> B`).
- A* mode: run `waypoint_sender` with `use_astar:=true`; sender computes `start_waypoint -> goal_waypoint` path on `astar_waypoints` grid and sends goals sequentially.

Single goal example:

```bash
ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped \
"{header: {frame_id: map}, pose: {position: {x: 0.1, y: 0.45, z: 0.0}, orientation: {z: 1.0, w: 0.0}}}"
```

Sequential goal example:

```bash
ros2 run hs_topview waypoint_sender --ros-args \
  -p waypoints_file:=/home/pinky/pinky_topview_ws/src/hs_topview/params/waypoints.yaml
```

Selected waypoint example:

```bash
ros2 run hs_topview waypoint_sender --ros-args \
  -p waypoints_file:=/home/pinky/pinky_topview_ws/src/hs_topview/params/waypoints.yaml \
  -p selected_waypoints:=A,C,E
```

A* mode example:

```bash
ros2 run hs_topview waypoint_sender --ros-args \
  -p waypoints_file:=/home/pinky/pinky_topview_ws/src/hs_topview/params/waypoints.yaml \
  -p use_astar:=true \
  -p start_waypoint:=R1C1 \
  -p goal_waypoint:=R3C5
```

## Waypoint YAML schema

`params/waypoints.yaml` uses this schema:

```yaml
action_name: pinky1/actions/move_to_pid
default_timeout_sec: 60.0
stop_on_failure: true
waypoints:
  - name: A
    frame_id: map
    x: 0.10
    y: 0.45
    z: 0.0
    timeout_sec: 60.0

astar_grid:
  rows: 3
  cols: 5
  naming: row_col
  connectivity: 4
astar_blocked_waypoints: [R2C3]
astar_waypoints:
  - name: R1C1
    frame_id: map
    x: 0.10
    y: 0.10
    z: 0.0
    timeout_sec: 60.0
  - name: R1C2
    frame_id: map
    x: 0.50
    y: 0.10
    z: 0.0
```

Waypoint orientation policy:

- `x,y,z` are required.
- `qx,qy,qz,qw` are optional.
- If orientation is omitted, sender uses default quaternion `(0,0,0,1)`.
- If orientation is provided, all 4 quaternion fields must be provided.
- If only some quaternion fields are provided, sender rejects that waypoint.

Selected waypoint policy:

- `selected_waypoints` is a comma-separated list of names, in execution order.
- If `selected_waypoints` is empty, sender executes all waypoints in YAML order.
- Name matching is exact and case-sensitive.
- If a selected name does not exist in YAML, sender exits with failure.
- If YAML contains duplicated waypoint names, sender exits with failure.
- Repeated names in `selected_waypoints` are allowed (example: `A,B,A`).

A* mode policy:

- Enable with `use_astar: true` and set `start_waypoint`, `goal_waypoint`.
- In A* mode, sender ignores regular `waypoints` and uses only `astar_waypoints`.
- `selected_waypoints` is ignored while A* mode is active.
- `astar_grid.naming` must be `row_col`; astar waypoint names must match `R{row}C{col}`.
- `astar_grid.connectivity` currently supports only `4` (up/down/left/right).
- `astar_blocked_waypoints` is optional and blocks A* nodes by waypoint name.
- Start/goal cannot be blocked.
- If no path exists, sender exits with failure.
- If start equals goal, sender exits successfully without sending goals.

Final align behavior:

- `point_move.yaml` parameter `enable_final_align` controls final in-place yaw alignment.
- `enable_final_align: false` -> stop immediately at position tolerance.
- `enable_final_align: true` -> run final yaw alignment as before.

Failure behavior:

- Action server wait timeout: 10 seconds.
- Invalid quaternion (`norm≈0`), goal reject, timeout, or abort:
  - `stop_on_failure: true` -> stop immediately.
  - `stop_on_failure: false` -> continue to next waypoint.
- A* validation failure or A* no-path result -> stop immediately.
- Empty waypoint list -> exit with warning.

## Key parameters

- `yaw_offset_rad`: camera yaw correction offset
- `pose_timeout_sec`: emergency stop timeout for stale `/amr_pose`
- `pose_frame_id`, `goal_frame_id`: frame validation
- `ignore_goal_topic_during_action`: ignore `/goal_pose` while Action is active
- `use_astar`: enable A* planning in `waypoint_sender`
- `start_waypoint`, `goal_waypoint`: A* start/goal node names
- `astar_grid`: A* grid metadata (`rows`, `cols`, `naming`, `connectivity`)
- `astar_waypoints`: A* node list (must use `R{row}C{col}` names)
- `astar_blocked_waypoints`: blocked nodes for A* planning
