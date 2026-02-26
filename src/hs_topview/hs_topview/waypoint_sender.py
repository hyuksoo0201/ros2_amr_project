#!/usr/bin/env python3
import heapq
import math
import os
import re
from typing import Any, Dict, List, Optional, Tuple

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from pinky_interfaces.action import MoveToPID
from rclpy.action import ActionClient
from rclpy.node import Node


class WaypointSender(Node):
    def __init__(self):
        super().__init__("waypoint_sender")

        default_waypoints_file = os.path.join(
            get_package_share_directory("hs_topview"),
            "params",
            "waypoints.yaml",
        )
        self.declare_parameter("waypoints_file", default_waypoints_file)
        self.declare_parameter("action_name", "pinky1/actions/move_to_pid")
        self.declare_parameter("server_wait_timeout_sec", 10.0)
        self.declare_parameter("default_timeout_sec", 60.0)
        self.declare_parameter("stop_on_failure", True)
        self.declare_parameter("selected_waypoints", "")
        self.declare_parameter("use_astar", False)
        self.declare_parameter("start_waypoint", "")
        self.declare_parameter("goal_waypoint", "")

        self.waypoints_file = str(self.get_parameter("waypoints_file").value)
        self.action_name = str(self.get_parameter("action_name").value)
        self.server_wait_timeout_sec = float(self.get_parameter("server_wait_timeout_sec").value)
        self.default_timeout_sec = float(self.get_parameter("default_timeout_sec").value)
        self.stop_on_failure = bool(self.get_parameter("stop_on_failure").value)
        self.selected_waypoints = str(self.get_parameter("selected_waypoints").value).strip()
        self.use_astar = bool(self.get_parameter("use_astar").value)
        self.start_waypoint = str(self.get_parameter("start_waypoint").value).strip()
        self.goal_waypoint = str(self.get_parameter("goal_waypoint").value).strip()

        self.client = ActionClient(self, MoveToPID, self.action_name)

    def _load_yaml(self, path: str) -> Optional[Dict[str, Any]]:
        if not os.path.exists(path):
            self.get_logger().error(f"Waypoints file not found: {path}")
            return None

        try:
            with open(path, "r", encoding="utf-8") as f:
                data = yaml.safe_load(f)
        except Exception as e:
            self.get_logger().error(f"Failed to read waypoints file '{path}': {e}")
            return None

        if not isinstance(data, dict):
            self.get_logger().error("Waypoints file root must be a mapping.")
            return None
        return data

    def _apply_file_overrides(self, data: Dict[str, Any]):
        action_name = data.get("action_name")
        if isinstance(action_name, str) and action_name:
            self.action_name = action_name
            self.client = ActionClient(self, MoveToPID, self.action_name)

        default_timeout_sec = data.get("default_timeout_sec")
        if default_timeout_sec is not None:
            try:
                self.default_timeout_sec = float(default_timeout_sec)
            except Exception:
                self.get_logger().warn(
                    f"Ignoring invalid default_timeout_sec: {default_timeout_sec}"
                )

        stop_on_failure = data.get("stop_on_failure")
        if stop_on_failure is not None:
            self.stop_on_failure = bool(stop_on_failure)

    def _make_pose(self, wp: Dict[str, Any], idx: int) -> Optional[PoseStamped]:
        required = ["x", "y", "z"]
        missing = [k for k in required if k not in wp]
        if missing:
            self.get_logger().error(
                f"Waypoint[{idx}] missing fields: {', '.join(missing)}"
            )
            return None

        try:
            x = float(wp["x"])
            y = float(wp["y"])
            z = float(wp["z"])
        except Exception:
            self.get_logger().error(f"Waypoint[{idx}] has non-numeric position fields.")
            return None

        quat_fields = ["qx", "qy", "qz", "qw"]
        quat_present = [k for k in quat_fields if k in wp]

        if len(quat_present) == 0:
            qx, qy, qz, qw = 0.0, 0.0, 0.0, 1.0
        elif len(quat_present) == 4:
            try:
                qx = float(wp["qx"])
                qy = float(wp["qy"])
                qz = float(wp["qz"])
                qw = float(wp["qw"])
            except Exception:
                self.get_logger().error(f"Waypoint[{idx}] has non-numeric quaternion fields.")
                return None
        else:
            self.get_logger().error(
                f"Waypoint[{idx}] quaternion must provide all of qx,qy,qz,qw or none."
            )
            return None

        q_norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
        if q_norm < 1e-6:
            self.get_logger().error(
                f"Waypoint[{idx}] invalid quaternion norm≈0 (qx,qy,qz,qw={qx},{qy},{qz},{qw})."
            )
            return None
        qx /= q_norm
        qy /= q_norm
        qz /= q_norm
        qw /= q_norm

        frame_id = str(wp.get("frame_id", "map"))
        if frame_id == "":
            self.get_logger().error(f"Waypoint[{idx}] frame_id must not be empty.")
            return None

        msg = PoseStamped()
        msg.header.frame_id = frame_id
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = z
        msg.pose.orientation.x = qx
        msg.pose.orientation.y = qy
        msg.pose.orientation.z = qz
        msg.pose.orientation.w = qw
        return msg

    def _send_goal(self, pose: PoseStamped, timeout_sec: float, label: str) -> Optional[MoveToPID.Result]:
        goal = MoveToPID.Goal()
        goal.target = pose
        goal.timeout_sec = float(timeout_sec)

        self.get_logger().info(
            f"Send [{label}] frame={pose.header.frame_id} "
            f"x={pose.pose.position.x:.3f} y={pose.pose.position.y:.3f} timeout={goal.timeout_sec:.1f}s"
        )

        send_future = self.client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()
        if goal_handle is None:
            self.get_logger().error(f"[{label}] no goal response from action server.")
            return None
        if not goal_handle.accepted:
            self.get_logger().error(f"[{label}] goal rejected.")
            return None

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        wrapped_result = result_future.result()
        if wrapped_result is None:
            self.get_logger().error(f"[{label}] failed to receive action result.")
            return None
        return wrapped_result.result

    def _parse_selected_waypoints(self) -> List[str]:
        if not self.selected_waypoints:
            return []
        return [token.strip() for token in self.selected_waypoints.split(",") if token.strip()]

    def _index_waypoints(
        self, waypoints: List[Any]
    ) -> Tuple[Optional[List[Tuple[int, Dict[str, Any], str]]], Optional[Dict[str, Tuple[int, Dict[str, Any]]]], bool]:
        indexed_waypoints: List[Tuple[int, Dict[str, Any], str]] = []
        name_to_wp: Dict[str, Tuple[int, Dict[str, Any]]] = {}
        had_failures = False

        for i, wp in enumerate(waypoints, start=1):
            if not isinstance(wp, dict):
                self.get_logger().error(f"Waypoint[{i}] must be a mapping.")
                had_failures = True
                if self.stop_on_failure:
                    return None, None, had_failures
                continue

            label = str(wp.get("name", f"WP{i}"))
            indexed_waypoints.append((i, wp, label))

            if label in name_to_wp:
                prev_idx, _ = name_to_wp[label]
                self.get_logger().error(
                    f"Duplicate waypoint name '{label}' found at Waypoint[{prev_idx}] and Waypoint[{i}]."
                )
                return None, None, True
            name_to_wp[label] = (i, wp)
        return indexed_waypoints, name_to_wp, had_failures

    def _build_execution_plan(
        self,
        indexed_waypoints: List[Tuple[int, Dict[str, Any], str]],
        name_to_wp: Dict[str, Tuple[int, Dict[str, Any]]],
        selected_names: List[str],
        had_failures: bool,
    ) -> Tuple[Optional[List[Tuple[int, Dict[str, Any], str]]], bool]:
        if not selected_names:
            return indexed_waypoints, had_failures

        selected_plan: List[Tuple[int, Dict[str, Any], str]] = []
        for name in selected_names:
            if name not in name_to_wp:
                self.get_logger().error(
                    f"Selected waypoint '{name}' not found. Available: {', '.join(name_to_wp.keys())}"
                )
                return None, True
            idx, wp = name_to_wp[name]
            selected_plan.append((idx, wp, name))
        return selected_plan, had_failures

    def _parse_grid_cell_name(self, name: str) -> Optional[Tuple[int, int]]:
        match = re.fullmatch(r"R(\d+)C(\d+)", name)
        if match is None:
            return None
        row = int(match.group(1))
        col = int(match.group(2))
        if row < 1 or col < 1:
            return None
        return row, col

    def _load_astar_grid_config(
        self,
        data: Dict[str, Any],
        astar_name_to_wp: Dict[str, Tuple[int, Dict[str, Any]]],
    ) -> Optional[Tuple[int, int, Dict[str, Tuple[int, int]], Dict[Tuple[int, int], str], List[str]]]:
        grid_cfg = data.get("astar_grid")
        if not isinstance(grid_cfg, dict):
            self.get_logger().error("A* mode requires 'astar_grid' mapping in waypoints YAML.")
            return None

        try:
            rows = int(grid_cfg.get("rows"))
            cols = int(grid_cfg.get("cols"))
        except Exception:
            self.get_logger().error("A* mode requires valid integer astar_grid.rows and astar_grid.cols.")
            return None
        if rows <= 0 or cols <= 0:
            self.get_logger().error("astar_grid.rows and astar_grid.cols must be > 0.")
            return None

        naming = str(grid_cfg.get("naming", "row_col")).strip()
        if naming != "row_col":
            self.get_logger().error(
                f"Unsupported astar_grid.naming='{naming}'. Only 'row_col' is supported in this version."
            )
            return None

        connectivity = int(grid_cfg.get("connectivity", 4))
        if connectivity != 4:
            self.get_logger().error(
                f"Unsupported astar_grid.connectivity={connectivity}. Only 4-neighbor connectivity is supported."
            )
            return None

        blocked_raw = data.get("astar_blocked_waypoints", [])
        if blocked_raw is None:
            blocked_raw = []
        if not isinstance(blocked_raw, list):
            self.get_logger().error("'astar_blocked_waypoints' must be a list.")
            return None

        blocked_names: List[str] = []
        for node_name in blocked_raw:
            if not isinstance(node_name, str) or not node_name.strip():
                self.get_logger().error("All astar_blocked_waypoints entries must be non-empty strings.")
                return None
            node_name = node_name.strip()
            if node_name not in astar_name_to_wp:
                self.get_logger().error(f"astar_blocked_waypoint '{node_name}' not found in astar_waypoints.")
                return None
            blocked_names.append(node_name)

        name_to_rc: Dict[str, Tuple[int, int]] = {}
        rc_to_name: Dict[Tuple[int, int], str] = {}
        for name in astar_name_to_wp.keys():
            rc = self._parse_grid_cell_name(name)
            if rc is None:
                self.get_logger().error(
                    f"A* mode requires row/col names. Invalid astar waypoint name '{name}', expected format R{{row}}C{{col}}."
                )
                return None
            row, col = rc
            if row > rows or col > cols:
                self.get_logger().error(
                    f"A* waypoint '{name}' out of declared astar_grid bounds ({rows}x{cols})."
                )
                return None
            if rc in rc_to_name and rc_to_name[rc] != name:
                self.get_logger().error(
                    f"A* grid cell R{row}C{col} is mapped by multiple astar_waypoints: '{rc_to_name[rc]}' and '{name}'."
                )
                return None
            name_to_rc[name] = rc
            rc_to_name[rc] = name

        return rows, cols, name_to_rc, rc_to_name, blocked_names

    def _manhattan(self, a: Tuple[int, int], b: Tuple[int, int]) -> int:
        return abs(a[0] - b[0]) + abs(a[1] - b[1])

    def _run_astar(
        self,
        start_name: str,
        goal_name: str,
        name_to_rc: Dict[str, Tuple[int, int]],
        rc_to_name: Dict[Tuple[int, int], str],
        blocked_names: List[str],
    ) -> Optional[List[str]]:
        blocked = set(blocked_names)
        if start_name in blocked:
            self.get_logger().error(f"start_waypoint '{start_name}' must not be blocked.")
            return None
        if goal_name in blocked:
            self.get_logger().error(f"goal_waypoint '{goal_name}' must not be blocked.")
            return None

        start_rc = name_to_rc[start_name]
        goal_rc = name_to_rc[goal_name]
        deltas = [(-1, 0), (1, 0), (0, -1), (0, 1)]

        open_heap: List[Tuple[int, int, str]] = []
        heapq.heappush(open_heap, (self._manhattan(start_rc, goal_rc), 0, start_name))

        came_from: Dict[str, str] = {}
        g_score: Dict[str, int] = {start_name: 0}
        closed: set[str] = set()

        while open_heap:
            _, current_cost, current_name = heapq.heappop(open_heap)
            if current_name in closed:
                continue
            if current_name == goal_name:
                path = [current_name]
                while current_name in came_from:
                    current_name = came_from[current_name]
                    path.append(current_name)
                path.reverse()
                return path

            closed.add(current_name)
            cur_r, cur_c = name_to_rc[current_name]

            for dr, dc in deltas:
                next_rc = (cur_r + dr, cur_c + dc)
                neighbor_name = rc_to_name.get(next_rc)
                if neighbor_name is None:
                    continue
                if neighbor_name in blocked:
                    continue
                if neighbor_name in closed:
                    continue

                tentative_cost = current_cost + 1
                if tentative_cost >= g_score.get(neighbor_name, 10**9):
                    continue
                g_score[neighbor_name] = tentative_cost
                came_from[neighbor_name] = current_name
                est_total = tentative_cost + self._manhattan(name_to_rc[neighbor_name], goal_rc)
                heapq.heappush(open_heap, (est_total, tentative_cost, neighbor_name))

        return None

    def _build_astar_execution_plan(
        self,
        data: Dict[str, Any],
        astar_name_to_wp: Dict[str, Tuple[int, Dict[str, Any]]],
    ) -> Optional[List[Tuple[int, Dict[str, Any], str]]]:
        if not self.start_waypoint:
            self.get_logger().error("A* mode requires non-empty parameter 'start_waypoint'.")
            return None
        if not self.goal_waypoint:
            self.get_logger().error("A* mode requires non-empty parameter 'goal_waypoint'.")
            return None
        if self.start_waypoint not in astar_name_to_wp:
            self.get_logger().error(f"start_waypoint '{self.start_waypoint}' not found in astar_waypoints.")
            return None
        if self.goal_waypoint not in astar_name_to_wp:
            self.get_logger().error(f"goal_waypoint '{self.goal_waypoint}' not found in astar_waypoints.")
            return None

        grid_cfg = self._load_astar_grid_config(data, astar_name_to_wp)
        if grid_cfg is None:
            return None
        _, _, name_to_rc, rc_to_name, blocked_names = grid_cfg

        path_names = self._run_astar(
            start_name=self.start_waypoint,
            goal_name=self.goal_waypoint,
            name_to_rc=name_to_rc,
            rc_to_name=rc_to_name,
            blocked_names=blocked_names,
        )
        if path_names is None:
            self.get_logger().error(
                f"A* failed to find a path from '{self.start_waypoint}' to '{self.goal_waypoint}'."
            )
            return None

        path_text = " -> ".join(path_names)
        self.get_logger().info(f"A* path:\n\n{path_text}\n")
        if len(path_names) <= 1:
            self.get_logger().info("A* start equals goal. No movement required.")
            return []

        execution_plan: List[Tuple[int, Dict[str, Any], str]] = []
        for name in path_names[1:]:
            idx, wp = astar_name_to_wp[name]
            execution_plan.append((idx, wp, name))
        return execution_plan

    def run(self) -> bool:
        data = self._load_yaml(self.waypoints_file)
        if data is None:
            return False
        self._apply_file_overrides(data)

        had_plan_failures = False
        execution_plan: Optional[List[Tuple[int, Dict[str, Any], str]]] = None
        if self.use_astar:
            if self.selected_waypoints:
                self.get_logger().warn("A* mode enabled: ignoring selected_waypoints parameter.")

            astar_waypoints = data.get("astar_waypoints")
            if not isinstance(astar_waypoints, list):
                self.get_logger().error("A* mode requires 'astar_waypoints' list in waypoints YAML.")
                return False
            if len(astar_waypoints) == 0:
                self.get_logger().error("A* mode requires non-empty 'astar_waypoints'.")
                return False

            _, astar_name_to_wp, had_astar_failures = self._index_waypoints(astar_waypoints)
            if astar_name_to_wp is None:
                return False
            had_plan_failures = had_astar_failures

            execution_plan = self._build_astar_execution_plan(data, astar_name_to_wp)
            if execution_plan is None:
                return False
        else:
            waypoints = data.get("waypoints")
            if not isinstance(waypoints, list):
                self.get_logger().error("'waypoints' must be a list.")
                return False
            if len(waypoints) == 0:
                self.get_logger().warn("Waypoint list is empty. Nothing to do.")
                return False

            indexed_waypoints, name_to_wp, had_plan_failures = self._index_waypoints(waypoints)
            if indexed_waypoints is None or name_to_wp is None:
                return False

            selected_names = self._parse_selected_waypoints()
            execution_plan, had_plan_failures = self._build_execution_plan(
                indexed_waypoints=indexed_waypoints,
                name_to_wp=name_to_wp,
                selected_names=selected_names,
                had_failures=had_plan_failures,
            )
            if execution_plan is None:
                return False
            if selected_names:
                self.get_logger().info(
                    f"Selected waypoint execution order: {', '.join(selected_names)}"
                )

        if len(execution_plan) == 0:
            if self.use_astar:
                return True
            self.get_logger().warn("No valid waypoints to execute.")
            return False

        self.get_logger().info(
            f"Waiting for action server '{self.action_name}' ({self.server_wait_timeout_sec:.1f}s)..."
        )
        if not self.client.wait_for_server(timeout_sec=self.server_wait_timeout_sec):
            self.get_logger().error(
                f"Action server '{self.action_name}' not available within timeout."
            )
            return False

        overall_success = not had_plan_failures

        for i, wp, label in execution_plan:
            pose = self._make_pose(wp, i)
            if pose is None:
                overall_success = False
                if self.stop_on_failure:
                    return False
                continue

            timeout_sec = self.default_timeout_sec
            if "timeout_sec" in wp:
                try:
                    timeout_sec = float(wp["timeout_sec"])
                except Exception:
                    self.get_logger().warn(
                        f"[{label}] invalid timeout_sec '{wp['timeout_sec']}', using default {self.default_timeout_sec:.1f}s"
                    )

            result = self._send_goal(pose, timeout_sec, label)
            if result is None:
                overall_success = False
                if self.stop_on_failure:
                    return False
                continue

            if result.success:
                self.get_logger().info(
                    f"[{label}] reached (status={result.status}, message='{result.message}')"
                )
                continue

            overall_success = False
            self.get_logger().warn(
                f"[{label}] failed (status={result.status}, message='{result.message}')"
            )
            if self.stop_on_failure:
                return False

        if overall_success:
            self.get_logger().info("All waypoints completed successfully.")
        else:
            self.get_logger().warn("Finished with waypoint failures.")
        return overall_success


def main(args=None):
    rclpy.init(args=args)
    node = WaypointSender()
    ok = False
    try:
        ok = node.run()
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            rclpy.shutdown()
        except Exception:
            pass
    raise SystemExit(0 if ok else 1)


if __name__ == "__main__":
    main()
