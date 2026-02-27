# ros2_amr_project

Top-view camera 기반 ROS2 AMR waypoint 주행 프로젝트입니다.
이 문서는 프로젝트를 빠르게 실행하는 입문 가이드입니다.

## 개요

- `hs_topview`: 주행 제어 노드(`point_move`) + waypoint 전송 노드(`waypoint_sender`)
- `pinky_interfaces`: `MoveToPID` action 인터페이스

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

## 빠른 시작

```bash
cd ~/pinky_topview_ws
colcon build --packages-select pinky_interfaces hs_topview
source install/setup.bash
```

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

## 대표 실행 예시

기본 waypoint 전체 실행:

```bash
ros2 launch hs_topview waypoint_sender.launch.py
```

선택 waypoint 실행:

```bash
ros2 launch hs_topview waypoint_sender.launch.py selected_waypoints:=A,C,E
```

A* 실행:

```bash
ros2 launch hs_topview waypoint_sender.launch.py \
  use_astar:=true \
  start_waypoint:=R1C1 \
  goal_waypoint:=R3C5
```

## 핵심 런타임 인터페이스

- Pose 입력: `/amr_pose` (`geometry_msgs/PoseStamped`)
- Goal 입력(topic): `/goal_pose` (`geometry_msgs/PoseStamped`)
- Goal 입력(action): `pinky1/actions/move_to_pid` (`pinky_interfaces/action/MoveToPID`)
- 속도 출력: `/cmd_vel` (`geometry_msgs/Twist`)
- 장애물 입력: `/obstacle_detected` (`std_msgs/Bool`)

## Troubleshooting

- `Action server 'pinky1/actions/move_to_pid' not available`
  - `ros2 launch hs_topview point_move.launch.py` 실행 상태 확인
  - `point_move.yaml`과 `waypoints.yaml`의 `action_name` 일치 확인
- `Ignoring pose frame ... Expected 'map'`
  - `/amr_pose.header.frame_id`가 `map`인지 확인
- `Selected waypoint 'X' not found`
  - `selected_waypoints` 값과 `waypoints.yaml`의 `name`이 정확히 일치하는지 확인

## 상세 문서

세부 옵션/정책 문서는 [hs_topview 패키지 README](src/hs_topview/README.md)를 참고하세요.
