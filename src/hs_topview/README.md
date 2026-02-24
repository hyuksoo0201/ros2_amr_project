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
  selected_waypoints:=A,C,F
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
  -p selected_waypoints:=A,C,F
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
    # Optional orientation:
    # qx: 0.0
    # qy: 0.0
    # qz: 1.0
    # qw: 0.0
    timeout_sec: 60.0
  - name: B
    frame_id: map
    x: 0.30
    y: 0.20
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

Final align behavior:

- `point_move.yaml` parameter `enable_final_align` controls final in-place yaw alignment.
- `enable_final_align: false` -> stop immediately at position tolerance.
- `enable_final_align: true` -> run final yaw alignment as before.

Failure behavior:

- Action server wait timeout: 10 seconds.
- Invalid quaternion (`norm≈0`), goal reject, timeout, or abort:
  - `stop_on_failure: true` -> stop immediately.
  - `stop_on_failure: false` -> continue to next waypoint.
- Empty waypoint list -> exit with warning.

## Key parameters

- `yaw_offset_rad`: camera yaw correction offset
- `pose_timeout_sec`: emergency stop timeout for stale `/amr_pose`
- `pose_frame_id`, `goal_frame_id`: frame validation
- `ignore_goal_topic_during_action`: ignore `/goal_pose` while Action is active
