# hs_topview

Top-view camera 기반 주행 제어 패키지입니다.
프로젝트 빠른 실행은 루트 문서([../../README.md](../../README.md))를 보고, 이 문서는 운영 옵션/정책을 확인할 때 사용합니다.

## Launch 사용법

`point_move` 실행:

```bash
ros2 launch hs_topview point_move.launch.py
```

`point_move.launch.py` 인자:

- `params_file`: point_move 파라미터 YAML 경로

예시:

```bash
ros2 launch hs_topview point_move.launch.py \
  params_file:=/path/to/point_move.yaml
```

`waypoint_sender` 실행:

```bash
ros2 launch hs_topview waypoint_sender.launch.py
```

`waypoint_sender.launch.py` 인자:

- `waypoints_file`: waypoint YAML 경로
- `selected_waypoints`: 쉼표 구분 이름 목록 (`A,C,E`)
- `use_astar`: A* 모드 on/off (`true`/`false`)
- `start_waypoint`: A* 시작 노드 이름 (`R1C1`)
- `goal_waypoint`: A* 목표 노드 이름 (`R3C5`)

예시:

```bash
ros2 launch hs_topview waypoint_sender.launch.py \
  waypoints_file:=/home/pinky/pinky_topview_ws/src/hs_topview/params/waypoints.yaml \
  selected_waypoints:=A,C,E
```

```bash
ros2 launch hs_topview waypoint_sender.launch.py \
  use_astar:=true \
  start_waypoint:=R1C1 \
  goal_waypoint:=R3C5
```

## Runtime 인터페이스

- Pose 입력: `/amr_pose` (`geometry_msgs/PoseStamped`, frame `map`)
- Goal 입력(topic): `/goal_pose` (`geometry_msgs/PoseStamped`, optional)
- Goal 입력(action): `pinky1/actions/move_to_pid` (`pinky_interfaces/action/MoveToPID`)
- 속도 출력: `/cmd_vel` (`geometry_msgs/Twist`)
- 장애물 입력: `/obstacle_detected` (`std_msgs/Bool`)

## waypoints.yaml 사용법

파일: `src/hs_topview/params/waypoints.yaml`

주요 키:

- `action_name`
- `default_timeout_sec`
- `stop_on_failure`
- `waypoints` (일반 순차 주행)
- `astar_grid`, `astar_blocked_waypoints`, `astar_waypoints` (A* 모드)

regular waypoint 항목 규칙:

- `x`, `y`, `z`는 필수
- `name`, `frame_id`, `timeout_sec`, `qx`, `qy`, `qz`, `qw`는 선택
- quaternion은 4개(`qx,qy,qz,qw`)를 모두 쓰거나 모두 생략해야 함
- quaternion 생략 시 `(0,0,0,1)` 사용

A* 항목 규칙:

- `astar_grid.naming`은 `row_col`만 지원
- `astar_grid.connectivity`는 `4`만 지원
- `astar_waypoints` 이름은 `R{row}C{col}` 형식이어야 함
- `astar_blocked_waypoints`는 선택(차단 노드 목록)

## 실행 정책

- `selected_waypoints`가 비어있으면 `waypoints`를 YAML 순서대로 실행
- `selected_waypoints`가 있으면 해당 이름 순서로 실행(대소문자 구분)
- waypoint 이름 중복이 있으면 실패 처리
- `use_astar:=true`면 regular `waypoints`와 `selected_waypoints`를 무시하고 `astar_waypoints`만 사용
- `stop_on_failure: true`면 실패 즉시 종료, `false`면 다음 waypoint 계속 진행

## 실패/종료 시나리오

- Action 서버 대기 시간 내 미기동: 즉시 실패 종료
- Goal reject 또는 결과 수신 실패: 실패 처리(`stop_on_failure` 정책 적용)
- Goal timeout: 실패 처리(`stop_on_failure` 정책 적용)
- A* 입력 검증 실패(잘못된 grid/name 등): 즉시 실패 종료
- A* no-path: 즉시 실패 종료
- A*에서 start=goal: 성공 종료(이동 없음)
