#!/usr/bin/env python3
"""
trajectory_and_controller.py

MoveIt Servo Cartesian PD controller for xArm Lite 6:
- Generates a smooth figure-eight (Lissajous 1:2) trajectory in link_base (XY by default)
- Optionally rotates the trajectory in the XY plane by yaw_deg (e.g., 90°)
- Tracks it using PD in task-space (TwistStamped -> /servo_server/delta_twist_cmds)
- Publishes RViz markers:
    /traj_marker  -> LINE_STRIP trajectory
    /target_marker -> SPHERE target
- Publishes per-cycle metrics:
    /cycle_metrics -> Float64MultiArray [rmse_x, rmse_y, rmse_z, rmse_total, sat_ratio]

Run example:
ros2 run xarm_perturbations trajectory_and_controller --ros-args \
  -p radius:=0.03 -p frequency:=0.07 -p plane:=xy -p hold_z:=true \
  -p yaw_deg:=90.0 \
  -p kp:="[2.5,2.5,2.5]" -p kd:="[0.6,0.6,0.6]" \
  -p max_speed:=0.12 -p deadband:=0.002 -p enable_keyboard:=false
"""

import math
import numpy as np
import csv
import os

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import TwistStamped, Point
from visualization_msgs.msg import Marker
from std_msgs.msg import Float64MultiArray

from tf2_ros import Buffer, TransformListener

import csv
import os


class RobotState:
    RUNNING = 1
    PAUSED = 2
    HOME = 3


class TrajectoryAndController(Node):
    def __init__(self):
        super().__init__("trajectory_and_controller")

        # ---------------------------
        # Parameters (trajectory)
        # ---------------------------
        self.declare_parameter("radius", 0.03)       # meters
        self.declare_parameter("frequency", 0.07)    # Hz
        self.declare_parameter("plane", "xy")        # "xy", "xz", "yz"
        self.declare_parameter("hold_z", True)       # if plane moves z, keep z fixed
        self.declare_parameter("yaw_deg", 0.0)       # rotate inside XY plane (degrees)
        self.declare_parameter("ramp_seconds", 2.0)  # smooth start + ignore metrics during ramp

        self.radius = float(self.get_parameter("radius").value)
        self.frequency = float(self.get_parameter("frequency").value)
        self.plane = str(self.get_parameter("plane").value)
        self.hold_z = bool(self.get_parameter("hold_z").value)
        self.yaw = math.radians(float(self.get_parameter("yaw_deg").value))
        self.ramp_seconds = float(self.get_parameter("ramp_seconds").value)

        # ---------------------------
        # Parameters (controller)
        # ---------------------------
        self.declare_parameter("kp", [2.5, 2.5, 2.5])
        self.declare_parameter("kd", [0.6, 0.6, 0.6])
        self.declare_parameter("max_speed", 0.12)    # m/s (norm limit)
        self.declare_parameter("deadband", 0.002)    # meters
        self.declare_parameter("enable_keyboard", False)

        self.kp = np.array(self.get_parameter("kp").get_parameter_value().double_array_value, dtype=float)
        self.kd = np.array(self.get_parameter("kd").get_parameter_value().double_array_value, dtype=float)
        self.max_speed = float(self.get_parameter("max_speed").value)
        db = float(self.get_parameter("deadband").value)
        self.epsilon = np.array([db, db, db], dtype=float)
        self.enable_keyboard = bool(self.get_parameter("enable_keyboard").value)
        
        
        
        # ---------------------------
        # Parameters (CSV logging)
        # ---------------------------
        self.declare_parameter("save_csv", False)
        self.declare_parameter("csv_filename", "tracking_data.csv")

        self.save_csv = bool(self.get_parameter("save_csv").value)
        self.csv_filename = str(self.get_parameter("csv_filename").value)

        self.csv_file = None
        self.csv_writer = None

        if self.save_csv:
            os.makedirs("data", exist_ok=True)
            self.csv_path = os.path.join("data", self.csv_filename)

            self.csv_file = open(self.csv_path, "w", newline="")
            self.csv_writer = csv.writer(self.csv_file)

            self.csv_writer.writerow([
                "time",
                "x_des", "y_des", "z_des",
                "x_act", "y_act", "z_act",
                "ex", "ey", "ez",
                "vx_cmd", "vy_cmd", "vz_cmd",
                "v_norm",
                "is_saturated"
            ])

            self.get_logger().info(f"📁 Saving CSV to {self.csv_path}")

        # ---------------------------
        # Publishers
        # ---------------------------
        self.servo_pub = self.create_publisher(TwistStamped, "/servo_server/delta_twist_cmds", 10)
        self.traj_marker_pub = self.create_publisher(Marker, "/traj_marker", 10)
        self.target_marker_pub = self.create_publisher(Marker, "/target_marker", 10)
        self.metrics_pub = self.create_publisher(Float64MultiArray, "/cycle_metrics", 10)

        # ---------------------------
        # TF for current EE pose
        # ---------------------------
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # ---------------------------
        # State machine / home / center
        # ---------------------------
        self.robot_state = RobotState.RUNNING
        self.home_position = np.array([0.227, 0.00, 0.468], dtype=float)  # edit as needed
        self.center_offset = np.array([0.00, 0.0, 0.0], dtype=float)
        self.center = None

        # Timing / derivative memory
        self.start_time = self.get_clock().now()
        self.prev_time = self.get_clock().now()
        self.prev_error = np.zeros(3)

        # Cycle tracking
        self.cycle_duration = 1.0 / max(self.frequency, 1e-6)
        self.current_cycle_index = 0

        # Metrics accumulators
        self.cumulative_sq_error = np.zeros(3)
        self.sample_count = 0
        self.sat_count = 0
        self.total_count = 0

        # Throttle
        self.last_info_time = self.get_clock().now()
        self.last_marker_time = self.get_clock().now()

        # Optional keyboard
        if self.enable_keyboard:
            self._start_keyboard()

        # Main loop at 50 Hz
        self.timer = self.create_timer(0.02, self._loop)

        self.get_logger().info(
            "TrajectoryAndController running. "
            "Twist -> /servo_server/delta_twist_cmds | metrics -> /cycle_metrics"
        )
        self.get_logger().info(f"Plane={self.plane} yaw_deg={math.degrees(self.yaw):.1f}")
        
    def _on_param_change(self, params):
        kp_new = None
        kd_new = None

        for p in params:
            if p.name == "kp" and p.type_ == Parameter.Type.DOUBLE_ARRAY:
                kp_new = np.array(p.value, dtype=float)
            if p.name == "kd" and p.type_ == Parameter.Type.DOUBLE_ARRAY:
                kd_new = np.array(p.value, dtype=float)

        if kp_new is not None:
            if kp_new.shape != (3,):
                return SetParametersResult(successful=False, reason="kp must have length 3")
            self.kp = kp_new

        if kd_new is not None:
            if kd_new.shape != (3,):
                return SetParametersResult(successful=False, reason="kd must have length 3")
            self.kd = kd_new

        # Reset deriv/métricas para evitar transiente feo tras cambiar gains
        self.prev_error = np.zeros(3)
        self.prev_time = self.get_clock().now()
        self.cumulative_sq_error[:] = 0.0
        self.sample_count = 0
        self.sat_count = 0
        self.total_count = 0

        self.get_logger().info(f"🔧 Gains updated: kp={self.kp.tolist()} kd={self.kd.tolist()}")
        return SetParametersResult(successful=True)

    # ---------------------------
    # Keyboard (optional)
    # ---------------------------
    def _start_keyboard(self):
        try:
            from pynput import keyboard
        except Exception as e:
            self.get_logger().warn(f"Keyboard disabled (pynput not available): {e}")
            self.enable_keyboard = False
            return

        def on_press(key):
            if hasattr(key, "char") and key.char == "p":
                if self.robot_state == RobotState.RUNNING:
                    self.robot_state = RobotState.PAUSED
                    self._publish_zero()
                    self.get_logger().warn("🔴 Paused.")
                else:
                    self.robot_state = RobotState.RUNNING
                    self.prev_time = self.get_clock().now()
                    self.prev_error = np.zeros(3)
                    self.get_logger().info("🟢 Resumed.")
            if hasattr(key, "char") and key.char == "h":
                self.robot_state = RobotState.HOME
                self.get_logger().info("🔄 Going HOME...")

        self.keyboard_listener = keyboard.Listener(on_press=on_press)
        self.keyboard_listener.start()
        self.get_logger().info("Keyboard enabled: 'p'=pause/resume, 'h'=home")

    # ---------------------------
    # TF read
    # ---------------------------
    def _read_position(self):
        try:
            trans = self.tf_buffer.lookup_transform("link_base", "link_eef", rclpy.time.Time())
            return np.array([
                trans.transform.translation.x,
                trans.transform.translation.y,
                trans.transform.translation.z
            ], dtype=float)
        except Exception as e:
            now = self.get_clock().now()
            if (now - self.last_info_time).nanoseconds > 2e9:
                self.get_logger().warn(f"TF not ready: {e}")
                self.last_info_time = now
            return None

    # ---------------------------
    # Rotation in XY about center
    # ---------------------------
    def _rot_xy(self, dx: float, dy: float) -> tuple[float, float]:
        """Rotate a local (dx,dy) vector by self.yaw in XY."""
        if abs(self.yaw) < 1e-12:
            return dx, dy
        c = math.cos(self.yaw)
        s = math.sin(self.yaw)
        return (c * dx - s * dy, s * dx + c * dy)

    # ---------------------------
    # Trajectory: figure-eight
    # ---------------------------
    def _target(self, t_sec: float) -> np.ndarray:
        cx, cy, cz = self.center
        w = 2.0 * math.pi * self.frequency

        # soft ramp 0..1
        ramp = min(max(t_sec / max(self.ramp_seconds, 1e-6), 0.0), 1.0)

        # local figure-eight (a,b)
        a = ramp * self.radius * math.sin(w * t_sec)
        b = 0.5 * ramp * self.radius * math.sin(2.0 * w * t_sec)

        # apply yaw rotation ONLY in XY plane
        ar, br = self._rot_xy(a, b)

        if self.plane == "xy":
            x, y, z = cx + ar, cy + br, cz

        elif self.plane == "xz":
            # rotate still affects XY only; for XZ we don't rotate by yaw (keep simple)
            x = cx + a
            y = cy
            z = (cz + b) if (not self.hold_z) else cz

        elif self.plane == "yz":
            x = cx
            y = cy + a
            z = (cz + b) if (not self.hold_z) else cz

        else:
            x, y, z = cx + ar, cy + br, cz

        return np.array([x, y, z], dtype=float)

    # ---------------------------
    # RViz markers
    # ---------------------------
    def _publish_markers(self, t_sec: float, target: np.ndarray):
        # throttle to ~10 Hz
        now = self.get_clock().now()
        if (now - self.last_marker_time).nanoseconds < 1e8:
            return
        self.last_marker_time = now

        cx, cy, cz = self.center

        # Trajectory LINE_STRIP
        m = Marker()
        m.header.frame_id = "link_base"
        m.header.stamp = now.to_msg()
        m.ns = "trajectory"
        m.id = 0
        m.type = Marker.LINE_STRIP
        m.action = Marker.ADD
        m.pose.orientation.w = 1.0
        m.scale.x = 0.005
        m.color.r, m.color.g, m.color.b, m.color.a = 0.0, 1.0, 0.0, 1.0

        ramp = min(max(t_sec / max(self.ramp_seconds, 1e-6), 0.0), 1.0)
        N = 200
        for i in range(N + 1):
            phi = 2.0 * math.pi * (i / N)
            a = ramp * self.radius * math.sin(phi)
            b = 0.5 * ramp * self.radius * math.sin(2.0 * phi)

            ar, br = self._rot_xy(a, b)

            p = Point()
            if self.plane == "xy":
                p.x, p.y, p.z = cx + ar, cy + br, cz
            elif self.plane == "xz":
                p.x, p.y = cx + a, cy
                p.z = (cz + b) if (not self.hold_z) else cz
            elif self.plane == "yz":
                p.x, p.y = cx, cy + a
                p.z = (cz + b) if (not self.hold_z) else cz
            else:
                p.x, p.y, p.z = cx + ar, cy + br, cz

            m.points.append(p)

        self.traj_marker_pub.publish(m)

        # Target SPHERE
        s = Marker()
        s.header.frame_id = "link_base"
        s.header.stamp = now.to_msg()
        s.ns = "target"
        s.id = 1
        s.type = Marker.SPHERE
        s.action = Marker.ADD
        s.pose.position.x = float(target[0])
        s.pose.position.y = float(target[1])
        s.pose.position.z = float(target[2])
        s.pose.orientation.w = 1.0
        s.scale.x = s.scale.y = s.scale.z = 0.02
        s.color.r, s.color.g, s.color.b, s.color.a = 1.0, 0.0, 0.0, 1.0
        self.target_marker_pub.publish(s)

    # ---------------------------
    # Publish twist
    # ---------------------------
    def _publish_twist(self, v_xyz: np.ndarray):
        cmd = TwistStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = "link_base"
        cmd.twist.linear.x = float(v_xyz[0])
        cmd.twist.linear.y = float(v_xyz[1])
        cmd.twist.linear.z = float(v_xyz[2])
        cmd.twist.angular.x = 0.0
        cmd.twist.angular.y = 0.0
        cmd.twist.angular.z = 0.0
        self.servo_pub.publish(cmd)

    def _publish_zero(self):
        self._publish_twist(np.zeros(3, dtype=float))

    # ---------------------------
    # PD step + metrics
    # ---------------------------
    def _servo_to(self, target_pos: np.ndarray, t_sec: float):
        current = self._read_position()
        if current is None:
            return

        now = self.get_clock().now()
        dt = (now - self.prev_time).nanoseconds / 1e9
        if dt <= 0.0:
            dt = 1e-6

        error = target_pos - current
        d_error = (error - self.prev_error) / dt

        count_metrics = (t_sec >= self.ramp_seconds)

        # deadband
        active_mask = np.abs(error) > self.epsilon

        # PD (make sure v always exists)
        v = self.kp * error + self.kd * d_error
        v = np.where(active_mask, v, 0.0)

        # saturate by norm
        v_norm = float(np.linalg.norm(v))
        was_saturated = False
        if v_norm > self.max_speed:
            v = v * (self.max_speed / max(v_norm, 1e-9))
            was_saturated = True
            v_norm = float(np.linalg.norm(v))  # recompute after saturation (optional)

        # metrics accumulation (after ramp)
        if count_metrics:
            self.cumulative_sq_error += (error ** 2)
            self.sample_count += 1
            self.total_count += 1
            if was_saturated:
                self.sat_count += 1

        # CSV logging
        if getattr(self, "save_csv", False) and getattr(self, "csv_writer", None) is not None:
            self.csv_writer.writerow([
                float(t_sec),
                float(target_pos[0]), float(target_pos[1]), float(target_pos[2]),
                float(current[0]), float(current[1]), float(current[2]),
                float(error[0]), float(error[1]), float(error[2]),
                float(v[0]), float(v[1]), float(v[2]),
                float(v_norm),
                1.0 if was_saturated else 0.0
            ])
            # flush each ~50 samples to avoid losing data on crash
            if (self.total_count % 50) == 0 and getattr(self, "csv_file", None) is not None:
                self.csv_file.flush()

        self._publish_twist(v)

        self.prev_error = error
        self.prev_time = now

    # ---------------------------
    # Cycle metrics
    # ---------------------------
    def _publish_cycle_metrics_if_needed(self, t_sec: float):
        cycle_index = int(t_sec // self.cycle_duration)
        if cycle_index <= self.current_cycle_index:
            return

        if self.sample_count > 0:
            rmse_xyz = np.sqrt(self.cumulative_sq_error / self.sample_count)
            rmse_total = float(np.linalg.norm(rmse_xyz))
            sat_ratio = float(self.sat_count / max(self.total_count, 1))

            msg = Float64MultiArray()
            msg.data = [
                float(rmse_xyz[0]), float(rmse_xyz[1]), float(rmse_xyz[2]),
                rmse_total,
                sat_ratio
            ]
            self.metrics_pub.publish(msg)

            self.get_logger().info(
                f"🏁 Cycle {self.current_cycle_index} | "
                f"RMSE xyz=({rmse_xyz[0]:.5f},{rmse_xyz[1]:.5f},{rmse_xyz[2]:.5f}) "
                f"total={rmse_total:.5f} sat={sat_ratio:.3f}"
            )

        # reset
        self.cumulative_sq_error[:] = 0.0
        self.sample_count = 0
        self.sat_count = 0
        self.total_count = 0
        self.current_cycle_index = cycle_index

    # ---------------------------
    # Main loop
    # ---------------------------
    def _loop(self):
        # Initialize center once
        if self.center is None:
            p = self._read_position()
            if p is None:
                return
            self.center = p.copy() + self.center_offset

            self.start_time = self.get_clock().now()
            self.prev_time = self.get_clock().now()
            self.prev_error = np.zeros(3)

            self.current_cycle_index = 0
            self.cumulative_sq_error[:] = 0.0
            self.sample_count = 0
            self.sat_count = 0
            self.total_count = 0

            self.get_logger().info(f"✅ Center set to {self.center.round(3)}")
            return

        # Pause
        if self.robot_state == RobotState.PAUSED:
            return

        # HOME mode
        if self.robot_state == RobotState.HOME:
            current = self._read_position()
            if current is None:
                return

            error = self.home_position - current
            dist = float(np.linalg.norm(error))
            if dist < 0.005:
                self._publish_zero()
                self.robot_state = RobotState.RUNNING
                self.center = current.copy() + self.center_offset
                self.start_time = self.get_clock().now()
                self.prev_time = self.get_clock().now()
                self.prev_error = np.zeros(3)
                self.get_logger().info("✅ Home reached. Re-centered and resumed.")
                return

            direction = np.where(np.abs(error) > 1e-4, np.sign(error), 0.0)
            v = direction * 0.08
            v_norm = float(np.linalg.norm(v))
            if v_norm > self.max_speed:
                v = v * (self.max_speed / max(v_norm, 1e-9))
            self._publish_twist(v)
            return

        # RUNNING
        t_sec = (self.get_clock().now() - self.start_time).nanoseconds / 1e9

        self._publish_cycle_metrics_if_needed(t_sec)

        target = self._target(t_sec)
        self._publish_markers(t_sec, target)
        self._servo_to(target, t_sec)
    def destroy_node(self):
        if self.save_csv:
            self.csv_file.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryAndController()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()