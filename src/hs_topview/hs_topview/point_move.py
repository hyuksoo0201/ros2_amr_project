#!/usr/bin/env python3
import math
import time
from enum import IntEnum
from threading import Lock
from typing import Optional, Tuple

import rclpy
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import Bool, Float64

from pinky_interfaces.action import MoveToPID


def normalize_angle(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def yaw_from_quat(x, y, z, w) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


class PIDController:
    def __init__(self, kp=1.0, ki=0.0, kd=0.0, min_output=-1.0, max_output=1.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.min_output = min_output
        self.max_output = max_output
        self.previous_error = 0.0
        self.integral = 0.0
        self.last_time = None

    def update_gains(self, kp, ki, kd):
        self.kp, self.ki, self.kd = kp, ki, kd

    def reset(self):
        self.previous_error = 0.0
        self.integral = 0.0
        self.last_time = None

    def compute(self, error, current_time=None):
        if current_time is None:
            current_time = time.time()

        if self.last_time is None:
            self.last_time = current_time
            self.previous_error = error
            return 0.0

        dt = current_time - self.last_time
        if dt <= 0.0:
            return 0.0

        p = self.kp * error

        self.integral += error * dt
        max_integral = abs(self.max_output) / (abs(self.ki) + 1e-6)
        self.integral = max(-max_integral, min(max_integral, self.integral))
        i = self.ki * self.integral

        d = self.kd * (error - self.previous_error) / dt

        out = p + i + d
        out = max(self.min_output, min(self.max_output, out))

        self.previous_error = error
        self.last_time = current_time
        return out


class Mode(IntEnum):
    TURN_TO_GOAL = 0
    GO_STRAIGHT = 1
    FINAL_ALIGN = 2


class GoalMoverTopView(Node):
    """Move robot from external top-view pose to target pose using PID."""

    def __init__(self):
        super().__init__("goal_mover_topview_pose")

        self.declare_parameter("cmd_topic", "/cmd_vel")
        self.declare_parameter("pose_topic", "/amr_pose")
        self.declare_parameter("obstacle_topic", "/obstacle_detected")
        self.declare_parameter("enable_goal_pose_sub", True)
        self.declare_parameter("goal_topic", "/goal_pose")
        self.declare_parameter("ignore_goal_topic_during_action", True)

        self.declare_parameter("action_name", "pinky1/actions/move_to_pid")
        self.declare_parameter("pose_frame_id", "map")
        self.declare_parameter("goal_frame_id", "map")
        self.declare_parameter("yaw_offset_rad", 0.0)

        self.declare_parameter("linear_P", 0.5)
        self.declare_parameter("linear_I", 0.0)
        self.declare_parameter("linear_D", 0.0)
        self.declare_parameter("angular_P", 0.2)
        self.declare_parameter("angular_I", 0.0)
        self.declare_parameter("angular_D", 0.05)

        self.declare_parameter("angle_tolerance_deg", 12.0)
        self.declare_parameter("pos_tolerance", 0.03)
        self.declare_parameter("final_yaw_tolerance_deg", 5.0)
        self.declare_parameter("enable_final_align", True)
        self.declare_parameter("min_angular_error_for_min_speed_deg", 3.0)
        self.declare_parameter("straight_to_turn_tolerance_ratio", 1.6)
        self.declare_parameter("straight_heading_deadband_deg", 1.0)

        self.declare_parameter("enable_pid", True)

        self.declare_parameter("max_linear_speed", 0.30)
        self.declare_parameter("max_angular_speed", 1.5)
        self.declare_parameter("straight_max_angular_speed", 0.8)
        self.declare_parameter("straight_angular_scale", 1.0)
        self.declare_parameter("straight_min_linear_scale", 0.4)
        self.declare_parameter("min_linear_speed", 0.06)
        self.declare_parameter("min_angular_speed", 0.10)
        self.declare_parameter("min_speed_distance_threshold", 0.30)

        self.declare_parameter("control_period_sec", 0.02)
        self.declare_parameter("pose_timeout_sec", 0.20)
        self.declare_parameter("warn_throttle_sec", 2.0)

        self._load_params()

        self.linear_pid = PIDController(
            kp=self.linear_P, ki=self.linear_I, kd=self.linear_D,
            min_output=-self.max_linear_speed, max_output=self.max_linear_speed,
        )
        self.angular_pid = PIDController(
            kp=self.angular_P, ki=self.angular_I, kd=self.angular_D,
            min_output=-self.max_angular_speed, max_output=self.max_angular_speed,
        )

        self._state_lock = Lock()
        self._warn_last_times = {}

        self.goal_msg: Optional[PoseStamped] = None
        self.mode: Mode = Mode.TURN_TO_GOAL
        self.obstacle_active = False
        self._reached_flag = False
        self._active_goal_handle = None
        self._action_goal_pending = False

        self._latest_pose: Optional[Tuple[float, float, float]] = None
        self._latest_pose_time: float = 0.0

        self.cmd_pub = self.create_publisher(Twist, self.cmd_topic, 10)
        self.distance_error_pub = self.create_publisher(Float64, "/distance_error", 10)
        self.angle_error_pub = self.create_publisher(Float64, "/angle_error", 10)
        self.final_yaw_error_pub = self.create_publisher(Float64, "/final_yaw_error", 10)

        self.pose_sub = self.create_subscription(PoseStamped, self.pose_topic, self._on_pose, 10)
        self.goal_sub = None
        if self.enable_goal_pose_sub:
            self.goal_sub = self.create_subscription(PoseStamped, self.goal_topic, self._on_goal_pose, 10)

        self.sub_ob = self.create_subscription(Bool, self.obstacle_topic, self._on_obstacle, 10)

        self._as = ActionServer(
            self,
            MoveToPID,
            self.action_name,
            execute_callback=self._execute_cb,
            goal_callback=self._goal_cb,
            cancel_callback=self._cancel_cb,
            callback_group=ReentrantCallbackGroup(),
        )

        self.timer = self.create_timer(self.control_period_sec, self.control_loop)
        goal_topic_name = self.goal_topic if self.enable_goal_pose_sub else "disabled"
        self.get_logger().info(
            f"GoalMoverTopView ready: pose_topic={self.pose_topic} "
            f"goal_topic={goal_topic_name} action={self.action_name}"
        )

    def _load_params(self):
        self.cmd_topic = self.get_parameter("cmd_topic").value
        self.pose_topic = self.get_parameter("pose_topic").value
        self.obstacle_topic = self.get_parameter("obstacle_topic").value
        self.enable_goal_pose_sub = bool(self.get_parameter("enable_goal_pose_sub").value)
        self.goal_topic = self.get_parameter("goal_topic").value
        self.ignore_goal_topic_during_action = bool(self.get_parameter("ignore_goal_topic_during_action").value)

        self.action_name = self.get_parameter("action_name").value
        self.pose_frame_id = self.get_parameter("pose_frame_id").value
        self.goal_frame_id = self.get_parameter("goal_frame_id").value
        self.yaw_offset_rad = float(self.get_parameter("yaw_offset_rad").value)

        self.linear_P = float(self.get_parameter("linear_P").value)
        self.linear_I = float(self.get_parameter("linear_I").value)
        self.linear_D = float(self.get_parameter("linear_D").value)

        self.angular_P = float(self.get_parameter("angular_P").value)
        self.angular_I = float(self.get_parameter("angular_I").value)
        self.angular_D = float(self.get_parameter("angular_D").value)

        angle_tolerance_deg = float(self.get_parameter("angle_tolerance_deg").value)
        self.angle_tolerance = math.radians(angle_tolerance_deg)
        straight_to_turn_tolerance_ratio = float(
            self.get_parameter("straight_to_turn_tolerance_ratio").value
        )
        if straight_to_turn_tolerance_ratio < 1.0:
            self.get_logger().warn(
                "straight_to_turn_tolerance_ratio < 1.0 is not valid. Clamping to 1.0."
            )
            straight_to_turn_tolerance_ratio = 1.0
        self.straight_to_turn_tolerance = self.angle_tolerance * straight_to_turn_tolerance_ratio

        self.final_yaw_tolerance = math.radians(float(self.get_parameter("final_yaw_tolerance_deg").value))
        self.enable_final_align = bool(self.get_parameter("enable_final_align").value)
        self.straight_heading_deadband = math.radians(
            float(self.get_parameter("straight_heading_deadband_deg").value)
        )
        self.min_angular_error_for_min_speed = math.radians(
            float(self.get_parameter("min_angular_error_for_min_speed_deg").value)
        )
        self.pos_tolerance = float(self.get_parameter("pos_tolerance").value)

        self.enable_pid = bool(self.get_parameter("enable_pid").value)

        self.max_linear_speed = float(self.get_parameter("max_linear_speed").value)
        self.max_angular_speed = float(self.get_parameter("max_angular_speed").value)
        self.straight_max_angular_speed = float(self.get_parameter("straight_max_angular_speed").value)
        if self.straight_max_angular_speed <= 0.0:
            self.straight_max_angular_speed = self.max_angular_speed
        self.straight_max_angular_speed = min(self.straight_max_angular_speed, self.max_angular_speed)

        self.straight_angular_scale = float(self.get_parameter("straight_angular_scale").value)
        if self.straight_angular_scale < 0.0:
            self.get_logger().warn("straight_angular_scale < 0.0 is not valid. Clamping to 0.0.")
            self.straight_angular_scale = 0.0

        self.straight_min_linear_scale = float(self.get_parameter("straight_min_linear_scale").value)
        self.straight_min_linear_scale = max(0.0, min(1.0, self.straight_min_linear_scale))

        self.min_linear_speed = float(self.get_parameter("min_linear_speed").value)
        self.min_angular_speed = float(self.get_parameter("min_angular_speed").value)
        self.min_speed_distance_threshold = float(self.get_parameter("min_speed_distance_threshold").value)

        self.control_period_sec = float(self.get_parameter("control_period_sec").value)
        self.pose_timeout_sec = float(self.get_parameter("pose_timeout_sec").value)
        self.warn_throttle_sec = float(self.get_parameter("warn_throttle_sec").value)

    def _warn_throttled(self, key: str, msg: str):
        now = time.time()
        last = self._warn_last_times.get(key, 0.0)
        if now - last >= self.warn_throttle_sec:
            self._warn_last_times[key] = now
            self.get_logger().warn(msg)

    def _is_expected_frame(self, incoming: str, expected: str) -> bool:
        return incoming == expected and incoming != ""

    def _reset_motion_state(self):
        self.mode = Mode.TURN_TO_GOAL
        self._reached_flag = False
        self.linear_pid.reset()
        self.angular_pid.reset()

    def _set_goal(self, goal: PoseStamped):
        self.goal_msg = goal
        self._reset_motion_state()
        source = "action" if self._has_active_action_locked() else "topic"
        self.get_logger().info(
            f"Set goal from {source}: x={goal.pose.position.x:.3f} "
            f"y={goal.pose.position.y:.3f} frame={goal.header.frame_id}"
        )

    def _has_active_action_locked(self) -> bool:
        if self._action_goal_pending:
            return True
        return self._active_goal_handle is not None and self._active_goal_handle.is_active

    def _clear_active_action_locked(self, goal_handle=None):
        if goal_handle is None or self._active_goal_handle is goal_handle:
            self._active_goal_handle = None
        self._action_goal_pending = False

    def _on_pose(self, msg: PoseStamped):
        if not self._is_expected_frame(msg.header.frame_id, self.pose_frame_id):
            self._warn_throttled(
                "pose_frame",
                f"Ignoring pose frame '{msg.header.frame_id}'. Expected '{self.pose_frame_id}'.",
            )
            self._latest_pose = None
            return

        q = msg.pose.orientation
        yaw = yaw_from_quat(q.x, q.y, q.z, q.w)
        yaw = normalize_angle(yaw + self.yaw_offset_rad)

        self._latest_pose = (msg.pose.position.x, msg.pose.position.y, yaw)
        self._latest_pose_time = time.time()

    def _on_goal_pose(self, msg: PoseStamped):
        if not self._is_expected_frame(msg.header.frame_id, self.goal_frame_id):
            self._warn_throttled(
                "goal_frame_topic",
                f"Ignoring goal frame '{msg.header.frame_id}'. Expected '{self.goal_frame_id}'.",
            )
            return

        with self._state_lock:
            if self.ignore_goal_topic_during_action and self._has_active_action_locked():
                self._warn_throttled(
                    "goal_topic_ignored",
                    "Ignoring /goal_pose while Action goal is active.",
                )
                return
            self._set_goal(msg)

    def _pose_is_fresh(self) -> bool:
        return (time.time() - self._latest_pose_time) <= self.pose_timeout_sec

    def _goal_cb(self, goal_request):
        if not self._is_expected_frame(goal_request.target.header.frame_id, self.goal_frame_id):
            self._warn_throttled(
                "goal_frame_action",
                (
                    f"Rejecting Action goal frame '{goal_request.target.header.frame_id}'. "
                    f"Expected '{self.goal_frame_id}'."
                ),
            )
            return GoalResponse.REJECT

        with self._state_lock:
            if self._has_active_action_locked():
                self._warn_throttled("goal_reject_busy", "Rejecting Action goal: another goal is active.")
                return GoalResponse.REJECT
            self._action_goal_pending = True
        return GoalResponse.ACCEPT

    def _cancel_cb(self, goal_handle):
        self.get_logger().warn("MoveToPID cancel requested, stopping.")
        with self._state_lock:
            if self._active_goal_handle is goal_handle:
                self.goal_msg = None
                self._reached_flag = False
                self._clear_active_action_locked(goal_handle)
            elif self._action_goal_pending:
                self._clear_active_action_locked()
        self._publish_stop()
        return CancelResponse.ACCEPT

    def _execute_cb(self, goal_handle):
        req = goal_handle.request
        target: PoseStamped = req.target
        timeout_sec = float(getattr(req, "timeout_sec", 0.0)) if req is not None else 0.0

        with self._state_lock:
            self._active_goal_handle = goal_handle
            self._action_goal_pending = False
            self._set_goal(target)

        t0 = time.time()
        self.get_logger().info(
            f"MoveToPID start: x={target.pose.position.x:.3f} "
            f"y={target.pose.position.y:.3f} timeout={timeout_sec:.1f}s"
        )

        while rclpy.ok():
            if goal_handle.is_cancel_requested:
                self.get_logger().warn("MoveToPID canceled by client.")
                with self._state_lock:
                    self.goal_msg = None
                    self._reached_flag = False
                    self._clear_active_action_locked(goal_handle)
                self._publish_stop()
                goal_handle.canceled()

                res = MoveToPID.Result()
                res.success = False
                res.message = "canceled"
                res.status = 1
                return res

            if self._reached_flag:
                with self._state_lock:
                    self.goal_msg = None
                    self._reached_flag = False
                    self._clear_active_action_locked(goal_handle)
                self._publish_stop()
                goal_handle.succeed()

                res = MoveToPID.Result()
                res.success = True
                res.message = "reached"
                res.status = 0
                return res

            if timeout_sec > 0.0 and (time.time() - t0) > timeout_sec:
                self.get_logger().warn("MoveToPID timeout, stopping.")
                with self._state_lock:
                    self.goal_msg = None
                    self._reached_flag = False
                    self._clear_active_action_locked(goal_handle)
                self._publish_stop()
                goal_handle.abort()

                res = MoveToPID.Result()
                res.success = False
                res.message = "timeout"
                res.status = 2
                return res

            time.sleep(0.02)

        with self._state_lock:
            self.goal_msg = None
            self._reached_flag = False
            self._clear_active_action_locked(goal_handle)
        self._publish_stop()
        goal_handle.abort()

        res = MoveToPID.Result()
        res.success = False
        res.message = "shutdown"
        res.status = 3
        return res

    def _goal_xy_yaw_in_map(self) -> Optional[Tuple[float, float, float]]:
        if self.goal_msg is None:
            return None
        gx = self.goal_msg.pose.position.x
        gy = self.goal_msg.pose.position.y
        gq = self.goal_msg.pose.orientation
        gyaw = yaw_from_quat(gq.x, gq.y, gq.z, gq.w)
        return gx, gy, gyaw

    def _publish_stop(self):
        self.cmd_pub.publish(Twist())

    def _apply_min_speed_with_deadband(self, output: float, minimum: float, error: float, deadband: float) -> float:
        if abs(error) <= deadband:
            return output
        if abs(output) < 1e-9:
            return 0.0
        if abs(output) < minimum:
            return math.copysign(minimum, output)
        return output

    def control_loop(self):
        if self.obstacle_active:
            self._publish_stop()
            return

        if self.goal_msg is None:
            return

        if self._latest_pose is None or (not self._pose_is_fresh()):
            self._warn_throttled("pose_timeout", "Pose data stale. Publishing stop.")
            self._publish_stop()
            return

        if not self._is_expected_frame(self.goal_msg.header.frame_id, self.goal_frame_id):
            self._warn_throttled(
                "goal_frame_runtime",
                f"Goal frame '{self.goal_msg.header.frame_id}' invalid at runtime. Publishing stop.",
            )
            self._publish_stop()
            return

        robot = self._latest_pose
        goal = self._goal_xy_yaw_in_map()
        if goal is None:
            return

        rx, ry, ryaw = robot
        gx, gy, gyaw = goal

        dx = gx - rx
        dy = gy - ry
        dist = math.hypot(dx, dy)
        heading_to_goal = math.atan2(dy, dx)
        heading_err = normalize_angle(heading_to_goal - ryaw)
        final_yaw_err = normalize_angle(gyaw - ryaw)

        dm = Float64(); dm.data = dist
        am = Float64(); am.data = heading_err
        fm = Float64(); fm.data = final_yaw_err
        self.distance_error_pub.publish(dm)
        self.angle_error_pub.publish(am)
        self.final_yaw_error_pub.publish(fm)

        if dist < self.pos_tolerance:
            if not self.enable_final_align:
                self.get_logger().info("Reached goal position. stop (final align disabled).")
                self._reached_flag = True
                self.goal_msg = None
                self.mode = Mode.TURN_TO_GOAL
                self.linear_pid.reset()
                self.angular_pid.reset()
                self._publish_stop()
                return
            self.mode = Mode.FINAL_ALIGN

        cmd = Twist()

        if self.mode == Mode.TURN_TO_GOAL:
            if abs(heading_err) <= self.angle_tolerance:
                self.mode = Mode.GO_STRAIGHT
                self.angular_pid.reset()
                cmd.angular.z = 0.0
                cmd.linear.x = 0.0
            else:
                w = self.angular_pid.compute(heading_err) if self.enable_pid else (self.angular_P * heading_err)
                w = max(-self.max_angular_speed, min(self.max_angular_speed, w))
                w = self._apply_min_speed_with_deadband(
                    w, self.min_angular_speed, heading_err, self.min_angular_error_for_min_speed
                )
                cmd.angular.z = w
                cmd.linear.x = 0.0

        elif self.mode == Mode.GO_STRAIGHT:
            if abs(heading_err) > self.straight_to_turn_tolerance:
                self.mode = Mode.TURN_TO_GOAL
                self.linear_pid.reset()
                self.angular_pid.reset()
                cmd.linear.x = 0.0
                cmd.angular.z = 0.0
            else:
                v = self.linear_pid.compute(dist) if self.enable_pid else (self.linear_P * dist)
                v = max(-self.max_linear_speed, min(self.max_linear_speed, v))
                v = self._apply_min_speed_with_deadband(
                    v, self.min_linear_speed, dist, self.min_speed_distance_threshold
                )

                if abs(heading_err) <= self.straight_heading_deadband:
                    w = 0.0
                else:
                    w = self.angular_pid.compute(heading_err) if self.enable_pid else (self.angular_P * heading_err)
                    w *= self.straight_angular_scale
                    w = max(-self.straight_max_angular_speed, min(self.straight_max_angular_speed, w))
                    w = self._apply_min_speed_with_deadband(
                        w, self.min_angular_speed, heading_err, self.min_angular_error_for_min_speed
                    )

                heading_scale = 1.0
                if self.straight_to_turn_tolerance > 1e-6:
                    heading_scale = 1.0 - min(1.0, abs(heading_err) / self.straight_to_turn_tolerance)
                    heading_scale = max(self.straight_min_linear_scale, heading_scale)

                cmd.linear.x = v * heading_scale
                cmd.angular.z = w

        elif self.mode == Mode.FINAL_ALIGN:
            if abs(final_yaw_err) <= self.final_yaw_tolerance:
                self.get_logger().info("Reached goal pose. stop.")
                self._reached_flag = True
                self.goal_msg = None
                self.mode = Mode.TURN_TO_GOAL
                self.linear_pid.reset()
                self.angular_pid.reset()
                self._publish_stop()
                return
            else:
                w = self.angular_pid.compute(final_yaw_err) if self.enable_pid else (self.angular_P * final_yaw_err)
                w = max(-self.max_angular_speed, min(self.max_angular_speed, w))
                w = self._apply_min_speed_with_deadband(
                    w, self.min_angular_speed, final_yaw_err, self.min_angular_error_for_min_speed
                )
                cmd.angular.z = w
                cmd.linear.x = 0.0

        self.cmd_pub.publish(cmd)

    def _on_obstacle(self, msg: Bool):
        self.obstacle_active = bool(msg.data)
        if self.obstacle_active:
            self._publish_stop()


def main(args=None):
    rclpy.init(args=args)
    node = GoalMoverTopView()

    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        try:
            executor.shutdown()
        except Exception:
            pass
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
