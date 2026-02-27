# ros2_amr_project

Top-view camera 기반 ROS2 AMR waypoint 주행 프로젝트입니다.
이 문서는 프로젝트를 빠르게 실행하는 입문 가이드입니다.

## 🎬 Demo Videos

<p align="center">
  <a href="https://youtu.be/Qj26KKkLirk">
    <img src="https://img.youtube.com/vi/Qj26KKkLirk/0.jpg" width="30%">
  </a>
  <a href="https://youtu.be/U1CzlIJLetA">
    <img src="https://img.youtube.com/vi/U1CzlIJLetA/0.jpg" width="30%">
  </a>
  <a href="https://youtu.be/stfRpe_FJLw">
    <img src="https://img.youtube.com/vi/stfRpe_FJLw/0.jpg" width="30%">
  </a>
</p>

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

### 주요 파일 위치

- Launch 파일: `src/hs_topview/launch/point_move.launch.py`, `src/hs_topview/launch/waypoint_sender.launch.py`
- 파라미터 파일: `src/hs_topview/params/point_move.yaml`, `src/hs_topview/params/waypoints.yaml`
- 노드 구현: `src/hs_topview/hs_topview/point_move.py`, `src/hs_topview/hs_topview/waypoint_sender.py`

## 대표 실행 예시

터미널 1(외부 topview pose 노드 실행):

```bash
# Required: ros2_topview를 ~/topview_ws로 클론한 경우에 실행합니다.
# git clone https://github.com/hyuksoo0201/ros2_topview.git ~/topview_ws
cd ~/topview_ws
colcon build
source install/setup.bash
ros2 run topview_localization topview_pose_node
```

터미널 2(외부 로봇 bringup 실행):

```bash
# Required: ros2_pinky_pro를 ~/pinky_pro로 클론한 경우에만 실행합니다.
# git clone https://github.com/hyuksoo0201/ros2_pinky_pro.git ~/pinky_pro
cd ~/pinky_pro
colcon build
source install/setup.bash
ros2 launch pinky_bringup bringup_robot.launch.xml
```

터미널 3(이동 노드 실행):

```bash
cd ~/pinky_topview_ws
colcon build
source install/setup.bash
ros2 launch hs_topview point_move.launch.py
```

터미널 4-1(A* 모드 실행):

```bash
cd ~/pinky_topview_ws
source install/setup.bash
ros2 launch hs_topview waypoint_sender.launch.py use_astar:=true \
  start_waypoint:=R5C6 goal_waypoint:=R2C2
```

터미널 4-2(selected_waypoints 모드 실행):

```bash
cd ~/pinky_topview_ws
source install/setup.bash
ros2 launch hs_topview waypoint_sender.launch.py selected_waypoints:=A,C,E
```

## Waypoint Grid (A*) 구성

- A* 모드는 `src/hs_topview/params/waypoints.yaml`의 `astar_grid`와 `astar_waypoints`를 사용합니다.
- 현재 기본 설정의 격자 크기는 `rows: 5`, `cols: 6`이며 총 30개 노드입니다.
- 노드 이름은 `R{row}C{col}` 규칙을 사용합니다. 예: `R1C1`, `R5C6`
- 인접 규칙은 `connectivity: 4`이며 상/하/좌/우 이동만 허용됩니다.
- `use_astar:=true`로 실행하면 `selected_waypoints` 설정은 무시됩니다.


### 차단 노드(astar_blocked_waypoints)

- `astar_blocked_waypoints`에 지정한 노드는 경로 탐색에서 제외됩니다.
- 현재 기본 설정 차단 노드: `R2C3`, `R2C5`, `R4C3`, `R4C5`
- `start_waypoint` 또는 `goal_waypoint`가 차단 노드면 즉시 실패합니다.
- 차단 설정 때문에 경로가 없으면 A* 모드는 실패 종료됩니다.

## 핵심 런타임 인터페이스

- Pose 입력: `/amr_pose` (`geometry_msgs/PoseStamped`)
- Goal 입력(topic): `/goal_pose` (`geometry_msgs/PoseStamped`)
- Goal 입력(action): `pinky1/actions/move_to_pid` (`pinky_interfaces/action/MoveToPID`)
- 속도 출력: `/cmd_vel` (`geometry_msgs/Twist`)
- 장애물 입력: `/obstacle_detected` (`std_msgs/Bool`)

## Troubleshooting

- `/amr_pose`가 들어오지 않아 로봇이 정지함
  - `~/topview_ws`에서 `source install/setup.bash` 후 `ros2 run topview_localization topview_pose_node` 실행
  - 참고 저장소(<https://github.com/hyuksoo0201/ros2_topview/tree/main>) 기준으로 `topview_ws` 구성 확인
- `Action server 'pinky1/actions/move_to_pid' not available`
  - `ros2 launch hs_topview point_move.launch.py` 실행 상태 확인
  - `point_move.yaml`과 `waypoints.yaml`의 `action_name` 일치 확인
- `Ignoring pose frame ... Expected 'map'`
  - `/amr_pose.header.frame_id`가 `map`인지 확인
- `Selected waypoint 'X' not found`
  - `selected_waypoints` 값과 `waypoints.yaml`의 `name`이 정확히 일치하는지 확인

## 상세 문서

패키지 README([src/hs_topview/README.md](src/hs_topview/README.md))는 포인터 문서입니다.
실행/정책/설정의 기준 문서는 이 루트 README입니다.
