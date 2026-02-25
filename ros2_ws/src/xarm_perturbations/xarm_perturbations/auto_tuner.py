#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import SetParametersResult
from std_msgs.msg import Float64MultiArray

import itertools
import csv
import os
import time


class GridSearchTuner(Node):
    def __init__(self):
        super().__init__('grid_search_tuner')

        # ===== CONFIG =====
        self.controller_node = self.declare_parameter(
            "controller_node", "/trajectory_and_controller"
        ).value
        self.lam_sat = float(self.declare_parameter("lam_sat", 0.10).value)
        self.ignore_first_cycle_after_set = bool(
            self.declare_parameter("ignore_first_cycle", True).value
        )
        self.cycles_per_test = int(self.declare_parameter("cycles_per_test", 1).value)
        self.results_csv = str(self.declare_parameter("results_csv", "data/tuning_results.csv").value)

        # Rango de búsqueda (puedes cambiar desde launch/params)
        self.kp_test_values = list(self.declare_parameter(
            "kp_values", [2.575, 2.6, 2.625, 2.65, 2.675]
        ).value)
        self.kd_test_values = list(self.declare_parameter(
            "kd_values", [0.625, 0.650, 0.675, 0.7, 0.725]
        ).value)

        self.combinations = list(itertools.product(self.kp_test_values, self.kd_test_values))
        self.current_test_idx = 0
        self.results = []

        # ===== ROS =====
        srv_name = f"{self.controller_node}/set_parameters"
        self.param_client = self.create_client(SetParameters, srv_name)
        self.metrics_sub = self.create_subscription(Float64MultiArray, '/cycle_metrics', self.metrics_callback, 10)

        self.awaiting_metrics = False
        self.cycles_to_ignore = 0
        self.cycles_collected = 0
        self.acc_rmse_total = 0.0
        self.acc_sat_ratio = 0.0
        self.acc_rmse_xyz = [0.0, 0.0, 0.0]

        self.get_logger().info(f"Auto-Tuner ready. Waiting for {srv_name} ...")
        if not self.param_client.wait_for_service(timeout_sec=10.0):
            self.get_logger().error("SetParameters service not available. Is trajectory_and_controller running?")
            raise RuntimeError("SetParameters service not available")

        os.makedirs(os.path.dirname(self.results_csv) or ".", exist_ok=True)
        self.csv_file = open(self.results_csv, "w", newline="")
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(["kp", "kd", "rmse_x", "rmse_y", "rmse_z", "rmse_total", "sat_ratio", "score"])

        self.get_logger().info(f"Logging tuner results to: {self.results_csv}")
        self.get_logger().info(f"Starting grid search with {len(self.combinations)} combos...")
        self.set_next_parameters()

    def set_next_parameters(self):
        if self.current_test_idx >= len(self.combinations):
            self.finish_search()
            return

        kp_val, kd_val = self.combinations[self.current_test_idx]
        self.get_logger().info(f"\n--- Testing {self.current_test_idx + 1}/{len(self.combinations)} ---")
        self.get_logger().info(f"Setting Kp={kp_val}, Kd={kd_val}")

        req = SetParameters.Request()
        req.parameters = [
            Parameter('kp', Parameter.Type.DOUBLE_ARRAY, [kp_val, kp_val, kp_val]).to_parameter_msg(),
            Parameter('kd', Parameter.Type.DOUBLE_ARRAY, [kd_val, kd_val, kd_val]).to_parameter_msg(),
        ]

        self.awaiting_metrics = True
        self.cycles_to_ignore = 1 if self.ignore_first_cycle_after_set else 0

        # reset accumulators for this test
        self.cycles_collected = 0
        self.acc_rmse_total = 0.0
        self.acc_sat_ratio = 0.0
        self.acc_rmse_xyz = [0.0, 0.0, 0.0]

        future = self.param_client.call_async(req)
        future.add_done_callback(self._on_param_set_done)

    def _on_param_set_done(self, future):
        try:
            resp = future.result()
        except Exception as e:
            self.get_logger().error(f"SetParameters call failed: {e}")
            self.awaiting_metrics = False
            self.current_test_idx += 1
            self.set_next_parameters()
            return

        ok = all(isinstance(r, SetParametersResult) and r.successful for r in resp.results)
        if not ok:
            reasons = [r.reason for r in resp.results if not r.successful]
            self.get_logger().error(f"SetParameters rejected. Reasons: {reasons}")
            self.awaiting_metrics = False
            self.current_test_idx += 1
            self.set_next_parameters()
            return

        self.get_logger().info("Parameters applied successfully. Waiting for /cycle_metrics ...")

    def metrics_callback(self, msg: Float64MultiArray):
        if not self.awaiting_metrics:
            return
        if len(msg.data) < 5:
            self.get_logger().warn("cycle_metrics expects 5 values: [rmse_x, rmse_y, rmse_z, rmse_total, sat_ratio]")
            return

        if self.cycles_to_ignore > 0:
            self.cycles_to_ignore -= 1
            self.get_logger().info("Ignoring first cycle after parameter change (transient).")
            return

        rmse_x, rmse_y, rmse_z, rmse_total, sat_ratio = msg.data[:5]

        # accumulate over multiple cycles if desired
        self.acc_rmse_xyz[0] += float(rmse_x)
        self.acc_rmse_xyz[1] += float(rmse_y)
        self.acc_rmse_xyz[2] += float(rmse_z)
        self.acc_rmse_total += float(rmse_total)
        self.acc_sat_ratio += float(sat_ratio)
        self.cycles_collected += 1

        if self.cycles_collected < self.cycles_per_test:
            self.get_logger().info(f"Collected {self.cycles_collected}/{self.cycles_per_test} cycles for this combo...")
            return

        # average
        avg_rmse_x = self.acc_rmse_xyz[0] / self.cycles_per_test
        avg_rmse_y = self.acc_rmse_xyz[1] / self.cycles_per_test
        avg_rmse_z = self.acc_rmse_xyz[2] / self.cycles_per_test
        avg_rmse_total = self.acc_rmse_total / self.cycles_per_test
        avg_sat_ratio = self.acc_sat_ratio / self.cycles_per_test

        kp_val, kd_val = self.combinations[self.current_test_idx]
        score = float(avg_rmse_total + self.lam_sat * avg_sat_ratio)

        self.get_logger().info(
            f"Result Kp={kp_val}, Kd={kd_val} | "
            f"RMSE xyz=({avg_rmse_x:.5f},{avg_rmse_y:.5f},{avg_rmse_z:.5f}) "
            f"total={avg_rmse_total:.5f} sat={avg_sat_ratio:.3f} score={score:.5f}"
        )

        row = [kp_val, kd_val, avg_rmse_x, avg_rmse_y, avg_rmse_z, avg_rmse_total, avg_sat_ratio, score]
        self.csv_writer.writerow(row)
        self.csv_file.flush()

        self.results.append({
            'kp': kp_val, 'kd': kd_val,
            'rmse_x': avg_rmse_x, 'rmse_y': avg_rmse_y, 'rmse_z': avg_rmse_z,
            'rmse_total': avg_rmse_total,
            'sat_ratio': avg_sat_ratio,
            'score': score
        })

        self.awaiting_metrics = False
        self.current_test_idx += 1
        self.set_next_parameters()

    def finish_search(self):
        self.get_logger().info("\n=====================================")
        self.get_logger().info("GRID SEARCH COMPLETE. Top results by SCORE (lower is better):")

        if self.csv_file is not None:
            self.csv_file.close()

        if not self.results:
            self.get_logger().warn("No results collected.")
            rclpy.shutdown()
            return

        sorted_results = sorted(self.results, key=lambda x: x['score'])
        top_n = min(10, len(sorted_results))

        for i in range(top_n):
            r = sorted_results[i]
            self.get_logger().info(
                f"{i+1:2d}) score={r['score']:.5f} | rmse_total={r['rmse_total']:.5f} sat={r['sat_ratio']:.3f} "
                f"| Kp={r['kp']} Kd={r['kd']}"
            )

        best = sorted_results[0]
        self.get_logger().info("=====================================")
        self.get_logger().info(
            f"🏆 BEST GAINS: Kp={best['kp']} Kd={best['kd']} "
            f"| score={best['score']:.5f} rmse_total={best['rmse_total']:.5f} sat={best['sat_ratio']:.3f}"
        )
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = GridSearchTuner()
    rclpy.spin(node)


if __name__ == '__main__':
    main()