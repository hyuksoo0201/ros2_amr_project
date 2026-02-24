#!/usr/bin/env python3
import math
import os
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

        self.waypoints_file = str(self.get_parameter("waypoints_file").value)
        self.action_name = str(self.get_parameter("action_name").value)
        self.server_wait_timeout_sec = float(self.get_parameter("server_wait_timeout_sec").value)
        self.default_timeout_sec = float(self.get_parameter("default_timeout_sec").value)
        self.stop_on_failure = bool(self.get_parameter("stop_on_failure").value)
        self.selected_waypoints = str(self.get_parameter("selected_waypoints").value).strip()

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

    def _build_execution_plan(
        self, waypoints: List[Any], selected_names: List[str]
    ) -> Tuple[Optional[List[Tuple[int, Dict[str, Any], str]]], bool]:
        indexed_waypoints: List[Tuple[int, Dict[str, Any], str]] = []
        name_to_wp: Dict[str, Tuple[int, Dict[str, Any]]] = {}
        had_failures = False

        for i, wp in enumerate(waypoints, start=1):
            if not isinstance(wp, dict):
                self.get_logger().error(f"Waypoint[{i}] must be a mapping.")
                had_failures = True
                if self.stop_on_failure:
                    return None, had_failures
                continue

            label = str(wp.get("name", f"WP{i}"))
            indexed_waypoints.append((i, wp, label))

            if label in name_to_wp:
                prev_idx, _ = name_to_wp[label]
                self.get_logger().error(
                    f"Duplicate waypoint name '{label}' found at Waypoint[{prev_idx}] and Waypoint[{i}]."
                )
                return None, True
            name_to_wp[label] = (i, wp)

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

    def run(self) -> bool:
        data = self._load_yaml(self.waypoints_file)
        if data is None:
            return False
        self._apply_file_overrides(data)

        waypoints = data.get("waypoints")
        if not isinstance(waypoints, list):
            self.get_logger().error("'waypoints' must be a list.")
            return False
        if len(waypoints) == 0:
            self.get_logger().warn("Waypoint list is empty. Nothing to do.")
            return False

        selected_names = self._parse_selected_waypoints()
        execution_plan, had_plan_failures = self._build_execution_plan(waypoints, selected_names)
        if execution_plan is None:
            return False
        if len(execution_plan) == 0:
            self.get_logger().warn("No valid waypoints to execute.")
            return False

        if selected_names:
            self.get_logger().info(
                f"Selected waypoint execution order: {', '.join(selected_names)}"
            )

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
