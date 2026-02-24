# ros2_amr_project

Top-view camera 기반 ROS2 AMR waypoint 주행 프로젝트입니다.
`point_move` 노드가 목표점까지 PID 제어로 주행하고, `waypoint_sender` 노드가 waypoint를 순차 또는 선택 실행합니다.

## 1. Project Structure

```text
pinky_topview_ws/
├─ src/hs_topview/
│  ├─ hs_topview/
│  │  ├─ point_move.py
│  │  └─ waypoint_sender.py
│  ├─ launch/
│  │  ├─ point_move.launch.py
│  │  └─ waypoint_sender.launch.py
│  └─ params/
│     ├─ point_move.yaml
│     └─ waypoints.yaml
└─ src/pinky_interfaces/
   └─ action/MoveToPID.action
```

## 2. Features

- `point_move`
  - `/amr_pose`를 입력으로 받아 `/cmd_vel`을 PID 제어로 생성
  - `MoveToPID` action server + `/goal_pose` topic goal 입력 지원
  - pose timeout/obstacle 시 안전 정지
- `waypoint_sender`
  - YAML waypoint 순차 실행
  - `selected_waypoints:=A,C,E`처럼 이름 기반 선택 실행
  - waypoint별 timeout, 실패 시 중단 여부(`stop_on_failure`) 설정 가능

## 3. Requirements

- Ubuntu + ROS2 Jazzy
- Python 3.12
- ROS2 Python dependencies: `rclpy`, `geometry_msgs`, `std_msgs`, `ament_index_python`, `PyYAML`

## 4. Build

```bash
cd ~/pinky_topview_ws
colcon build --packages-select pinky_interfaces hs_topview
source install/setup.bash
```

## 5. Run

터미널 1:

```bash
source ~/pinky_topview_ws/install/setup.bash
ros2 launch hs_topview point_move.launch.py
```

터미널 2:

```bash
source ~/pinky_topview_ws/install/setup.bash
ros2 launch hs_topview waypoint_sender.launch.py
```

## 6. Usage Examples

전체 waypoint 실행:

```bash
ros2 launch hs_topview waypoint_sender.launch.py
```

선택 waypoint 실행 (`A -> C -> E`):

```bash
ros2 launch hs_topview waypoint_sender.launch.py selected_waypoints:=A,C,E
```

단일 goal(topic) 전송:

```bash
ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped \
"{header: {frame_id: map}, pose: {position: {x: 0.50, y: 0.45, z: 0.0}, orientation: {z: 0.0, w: 1.0}}}"
```

## 7. Runtime Interfaces

- Input pose: `/amr_pose` (`geometry_msgs/PoseStamped`)
- Goal input:
  - Action: `pinky1/actions/move_to_pid` (`pinky_interfaces/action/MoveToPID`)
  - Topic: `/goal_pose` (`geometry_msgs/PoseStamped`, optional)
- Velocity output: `/cmd_vel` (`geometry_msgs/Twist`)
- Obstacle input: `/obstacle_detected` (`std_msgs/Bool`)

`MoveToPID.action`:

```text
# Goal
geometry_msgs/PoseStamped target
float32 timeout_sec
---
# Result
bool success
string message
int32 status
```

## 8. Configuration

`src/hs_topview/params/point_move.yaml`

- `yaw_offset_rad`: top-view yaw 보정값
- `pose_timeout_sec`: pose stale 허용 시간(초)
- `linear_P`, `linear_I`, `linear_D`: 선속도 PID
- `angular_P`, `angular_I`, `angular_D`: 각속도 PID
- `pos_tolerance`: 목표 위치 도달 허용 오차
- `enable_final_align`: 도착 후 최종 yaw 정렬 on/off
- `max_linear_speed`, `max_angular_speed`: 속도 제한

`src/hs_topview/params/waypoints.yaml`

- `action_name`
- `default_timeout_sec`
- `stop_on_failure`
- `waypoints[]`
  - required: `x`, `y`, `z`
  - optional: `name`, `frame_id`, `timeout_sec`, `qx`, `qy`, `qz`, `qw`
  - quaternion은 4개 필드를 모두 주거나 모두 생략해야 함

## 9. Safety/Behavior Notes

- `/amr_pose`가 timeout이면 정지합니다.
- waypoint의 `name`은 고유해야 합니다.
- `selected_waypoints` 이름 매칭은 대소문자 구분합니다.
- `stop_on_failure: true`면 중간 실패 시 즉시 종료합니다.

## 10. Troubleshooting

- `Action server 'pinky1/actions/move_to_pid' not available`
  - `point_move.launch.py` 실행 여부 확인
  - action 이름이 sender/point_move에서 동일한지 확인
- `Ignoring pose frame ... Expected 'map'`
  - `/amr_pose.header.frame_id`가 `map`인지 확인
- `Selected waypoint 'X' not found`
  - `waypoints.yaml`의 `name`과 `selected_waypoints` 문자열이 정확히 일치하는지 확인

## 11. Package Readme

- 상세 패키지 설명: `src/hs_topview/README.md`

## 12. License

TBD

## 13. Author

- GitHub: https://github.com/hyuksoo0201
