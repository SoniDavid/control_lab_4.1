#!/usr/bin/env python3
"""
trajectory_replay.py
====================
ROS2 node that replays the MuJoCo PID trajectory in Gazebo using the
native gz-sim JointPositionController plugin (no gz_ros2_control required).

Pipeline:
  1. Read mujoco_trajectory.csv (from export_trajectory.py).
  2. For every row, publish q_des_<joint> as std_msgs/Float64 to
     /model/so101/joint/<joint>/cmd_pos  (bridged to Gazebo via ros_gz_bridge).
  3. Subscribe to /so101/joint_states (bridged from Gazebo's JointStatePublisher).
  4. Write recorded Gazebo joint positions to gazebo_trajectory.csv.

Topic mapping (via ros_gz_bridge):
  ROS2  /model/so101/joint/shoulder_pan/cmd_pos  → Gazebo  (Float64 → Double)
  Gazebo /world/so101_replay/model/so101/joint_state → ROS2 /so101/joint_states

Usage:
    ros2 run gazebo_mujoco_bridge trajectory_replay \
        --ros-args -p csv_in:=/path/to/mujoco_trajectory.csv \
                   -p csv_out:=/path/to/gazebo_trajectory.csv \
                   -p publish_rate_hz:=500.0 \
                   -p wait_s:=5.0
"""

from __future__ import annotations
import csv
import os
import sys
import time
from threading import Lock

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState


JOINT_NAMES = [
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_roll",
    "gripper",
]

CMD_TOPIC_PREFIX = "/model/so101/joint"   # → /model/so101/joint/<name>/cmd_pos
JS_TOPIC         = "/so101/joint_states"  # bridged from Gazebo


class TrajectoryReplay(Node):

    def __init__(self):
        super().__init__("trajectory_replay")

        # ── Parameters ────────────────────────────────────────────────────
        self.declare_parameter("csv_in",  "")
        self.declare_parameter("csv_out", "")
        self.declare_parameter("publish_rate_hz", 500.0)
        self.declare_parameter("wait_s",  5.0)

        csv_in   = self.get_parameter("csv_in").value
        csv_out  = self.get_parameter("csv_out").value
        self.rate_hz = float(self.get_parameter("publish_rate_hz").value)
        self.wait_s  = float(self.get_parameter("wait_s").value)

        if not csv_in:
            self.get_logger().error("Parameter 'csv_in' is required.")
            sys.exit(1)

        if not csv_out:
            directory = os.path.dirname(csv_in)
            fname = os.path.splitext(os.path.basename(csv_in))[0]
            fname = fname.replace("mujoco", "gazebo")
            csv_out = os.path.join(directory, fname + ".csv")
            if csv_out == csv_in:
                csv_out = os.path.join(directory, fname + "_gazebo.csv")

        self.csv_in  = csv_in
        self.csv_out = csv_out

        # ── Publishers: one per joint ──────────────────────────────────────
        self._cmd_pubs: dict[str, rclpy.publisher.Publisher] = {}
        for jn in JOINT_NAMES:
            topic = f"{CMD_TOPIC_PREFIX}/{jn}/cmd_pos"
            self._cmd_pubs[jn] = self.create_publisher(Float64, topic, 10)

        # ── Joint state subscriber ─────────────────────────────────────────
        self._js_lock   = Lock()
        self._js_rows: list[dict] = []
        self._recording = False

        self.create_subscription(JointState, JS_TOPIC, self._js_cb, 10)

        self.get_logger().info(f"csv_in  : {self.csv_in}")
        self.get_logger().info(f"csv_out : {self.csv_out}")
        self.get_logger().info(f"rate    : {self.rate_hz} Hz")

    # ── Callbacks ──────────────────────────────────────────────────────────

    def _js_cb(self, msg: JointState):
        if not self._recording:
            return
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        row: dict = {"ros_time": stamp}
        name_to_idx = {n: i for i, n in enumerate(msg.name)}
        for jn in JOINT_NAMES:
            idx = name_to_idx.get(jn)
            row[f"q_actual_{jn}"] = (
                float(msg.position[idx]) if idx is not None and idx < len(msg.position)
                else float("nan")
            )
            row[f"qd_actual_{jn}"] = (
                float(msg.velocity[idx]) if idx is not None and idx < len(msg.velocity)
                else float("nan")
            )
        with self._js_lock:
            self._js_rows.append(row)

    # ── Load CSV ───────────────────────────────────────────────────────────

    def _load_csv(self) -> list[dict]:
        rows = []
        with open(self.csv_in, newline="") as f:
            reader = csv.DictReader(f)
            for row in reader:
                rows.append(row)
        if not rows:
            raise RuntimeError("mujoco_trajectory.csv is empty")
        self.get_logger().info(f"Loaded {len(rows)} rows from {self.csv_in}")
        return rows

    # ── Replay ────────────────────────────────────────────────────────────

    def replay(self):
        rows = self._load_csv()

        # Determine which joints are actually in the CSV
        present = [jn for jn in JOINT_NAMES if f"q_des_{jn}" in rows[0]]
        missing = [jn for jn in JOINT_NAMES if jn not in present]
        if missing:
            self.get_logger().warn(f"Joints missing from CSV: {missing}")

        dt = 1.0 / self.rate_hz
        total_time = float(rows[-1]["time"]) - float(rows[0]["time"])

        self.get_logger().info(f"Waiting {self.wait_s:.1f}s for Gazebo to be ready...")
        time.sleep(self.wait_s)

        self._recording = True

        # Build forward + backward sequence (ping-pong)
        rows_reversed = list(reversed(rows))
        passes = [rows, rows_reversed, rows]
        total_steps = sum(len(p) for p in passes)

        self.get_logger().info(
            f"Replaying {total_steps} steps (forward → backward → forward) "
            f"at {self.rate_hz:.0f} Hz..."
        )

        t_wall_start = time.time()
        step = 0
        for pass_idx, pass_rows in enumerate(passes):
            direction = "backward" if pass_idx % 2 == 1 else "forward"
            self.get_logger().info(
                f"  Pass {pass_idx+1}/{len(passes)}: {direction} "
                f"({len(pass_rows)} steps)"
            )
            for row in pass_rows:
                for jn in present:
                    msg = Float64()
                    msg.data = float(row[f"q_des_{jn}"])
                    self._cmd_pubs[jn].publish(msg)

                step += 1
                elapsed = time.time() - t_wall_start
                desired = step * dt
                sleep_t = desired - elapsed
                if sleep_t > 0:
                    time.sleep(sleep_t)

        self._recording = False
        time.sleep(1.0)

        self.get_logger().info(
            f"Replay done. Wall time: {time.time()-t_wall_start:.1f}s"
        )

    # ── Save output ────────────────────────────────────────────────────────

    def save_csv(self):
        with self._js_lock:
            rows = list(self._js_rows)

        if not rows:
            self.get_logger().warn("No /so101/joint_states received – CSV empty.")
            self.get_logger().warn(
                "Check that ros_gz_bridge is running and bridging "
                "/world/so101_replay/model/so101/joint_state "
                "→ /so101/joint_states"
            )
            return

        os.makedirs(os.path.dirname(os.path.abspath(self.csv_out)), exist_ok=True)
        fieldnames = list(rows[0].keys())
        with open(self.csv_out, "w", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=fieldnames, extrasaction="ignore")
            writer.writeheader()
            writer.writerows(rows)

        self.get_logger().info(
            f"Saved {len(rows)} rows → {self.csv_out}"
        )

    # ── Entry point ───────────────────────────────────────────────────────

    def run(self):
        self.replay()
        self.save_csv()
        self.get_logger().info("trajectory_replay finished.")


def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryReplay()

    import threading
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
        spin_thread.join(timeout=2.0)


if __name__ == "__main__":
    main()
