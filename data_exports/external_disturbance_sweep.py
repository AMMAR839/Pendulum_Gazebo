#!/usr/bin/env python3
import argparse
import csv
import math
import os
import struct
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, Float64MultiArray


BUTTON_A = 1 << 0
BUTTON_B = 1 << 1
BUTTON_X = 1 << 2
BUTTON_Y = 1 << 3

MODE_READY = 3
MODE_SWING_UP = 6
MODE_BALANCE = 7


def make_joystick_packet(seq, buttons):
    body = struct.pack("<BBhhhhH", 0x01, seq & 0xFF, 0, 0, 0, 0, buttons)
    checksum = sum(body) & 0xFFFF
    return b"\xAA\x55" + body + struct.pack("<H", checksum)


def parse_float_list(text):
    values = []
    for item in text.split(","):
        item = item.strip()
        if item:
            values.append(float(item))
    if not values:
        raise argparse.ArgumentTypeError("force list must not be empty")
    return values


def fmt(value, digits=6):
    if value is None or not math.isfinite(value):
        return ""
    return f"{value:.{digits}f}"


class ExternalDisturbanceSweep(Node):
    def __init__(self, args):
        super().__init__("pendulum_external_disturbance_sweep")
        self.args = args
        self.latest_state = None
        self.latest_cart_force = math.nan
        self.latest_hinge_total = math.nan
        self.rows = []
        self.trial_rows = []
        self.summary_rows = []
        self.start_time = time.monotonic()
        self.seq = 0
        self.serial_fd = None
        self.active_trial = None
        self.last_sample_time = 0.0

        self.disturbance_pub = self.create_publisher(
            Float64,
            "/pendulum/external_disturbance_torque_cmd",
            10,
        )
        self.create_subscription(
            Float64MultiArray,
            "/pendulum/sim_state",
            self._state_cb,
            50,
        )
        self.create_subscription(
            Float64,
            "/pendulum/cart_force_cmd",
            self._cart_force_cb,
            50,
        )
        self.create_subscription(
            Float64,
            "/pendulum/hinge_assist_force_cmd",
            self._hinge_total_cb,
            50,
        )

    def _cart_force_cb(self, msg):
        self.latest_cart_force = float(msg.data)

    def _hinge_total_cb(self, msg):
        self.latest_hinge_total = float(msg.data)

    def _state_cb(self, msg):
        if len(msg.data) < 8:
            return
        now = time.monotonic()
        self.latest_state = list(msg.data[:8])

        if self.active_trial is None:
            return
        if now - self.last_sample_time < self.args.sample_period:
            return
        self.last_sample_time = now

        data = self.latest_state
        mode = int(round(data[7]))
        trial_elapsed = now - self.active_trial["started_at"]
        self.rows.append(
            {
                "workspace": self.args.workspace,
                "direction": self.active_trial["direction"],
                "force_n": fmt(self.active_trial["force_n"], 4),
                "torque_nm": fmt(self.active_trial["torque_nm"], 6),
                "elapsed_s": fmt(now - self.start_time, 3),
                "trial_elapsed_s": fmt(trial_elapsed, 3),
                "degree_deg": fmt(data[0], 6),
                "cmX_cm": fmt(data[1], 6),
                "setspeed_cm_s": fmt(data[2], 6),
                "energy_j": fmt(data[3], 9),
                "theta_dot_rad_s": fmt(data[4], 9),
                "theta_rad": fmt(data[5], 9),
                "x_center_cm": fmt(data[6], 6),
                "mode": str(mode),
                "cart_force_cmd_n": fmt(self.latest_cart_force, 9),
                "hinge_total_torque_nm": fmt(self.latest_hinge_total, 9),
                "external_disturbance_torque_nm": fmt(
                    self.active_trial["torque_nm"],
                    9,
                ),
            }
        )

    def spin_for(self, duration_s):
        deadline = time.monotonic() + duration_s
        while time.monotonic() < deadline and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.02)

    def wait_for_state(self, timeout_s):
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.latest_state is not None:
                return True
        return False

    def current_mode(self):
        if self.latest_state is None:
            return None
        return int(round(self.latest_state[7]))

    def current_degree(self):
        if self.latest_state is None:
            return math.nan
        return float(self.latest_state[0])

    def current_theta_dot(self):
        if self.latest_state is None:
            return math.nan
        return float(self.latest_state[4])

    def current_cart_cm(self):
        if self.latest_state is None:
            return math.nan
        return float(self.latest_state[1])

    def publish_disturbance(self, torque_nm):
        msg = Float64()
        msg.data = float(torque_nm)
        self.disturbance_pub.publish(msg)

    def clear_disturbance(self):
        for _ in range(5):
            self.publish_disturbance(0.0)
            self.spin_for(0.02)

    def open_serial(self):
        if self.serial_fd is not None:
            return
        deadline = time.monotonic() + self.args.serial_timeout
        while time.monotonic() < deadline:
            if os.path.exists(self.args.serial):
                self.serial_fd = os.open(self.args.serial, os.O_RDWR | os.O_NOCTTY)
                return
            self.spin_for(0.1)
        raise RuntimeError(f"Serial symlink not found: {self.args.serial}")

    def send_button(self, button):
        os.write(self.serial_fd, make_joystick_packet(self.seq, button))
        self.seq += 1
        self.spin_for(0.12)
        os.write(self.serial_fd, make_joystick_packet(self.seq, 0))
        self.seq += 1
        self.spin_for(0.12)

    def prepare_balance(self):
        self.clear_disturbance()
        if not self.wait_for_state(self.args.state_timeout):
            raise RuntimeError("No /pendulum/sim_state samples received")
        self.open_serial()

        self.send_button(BUTTON_B)
        self.send_button(BUTTON_Y)
        ready_deadline = time.monotonic() + self.args.homing_timeout
        while time.monotonic() < ready_deadline and self.current_mode() != MODE_READY:
            self.spin_for(0.1)

        self.send_button(BUTTON_X)
        swing_started = time.monotonic()
        balance_requested = False
        balance_seen = False

        while time.monotonic() - swing_started < self.args.balance_timeout:
            self.spin_for(0.05)
            mode = self.current_mode()
            degree = abs(self.current_degree())
            since_swing = time.monotonic() - swing_started

            if mode == MODE_BALANCE:
                balance_seen = True
                break

            if (
                not balance_requested
                and since_swing >= self.args.min_swing_before_balance
                and degree <= self.args.manual_balance_deg
            ):
                self.send_button(BUTTON_A)
                balance_requested = True

            if (
                not balance_requested
                and since_swing >= self.args.force_balance_after
            ):
                self.send_button(BUTTON_A)
                balance_requested = True

        if not balance_seen:
            raise RuntimeError("BALANCE mode was not reached before timeout")

        if not self.wait_for_stable_balance():
            raise RuntimeError("BALANCE mode was reached but did not stabilize")

    def wait_for_stable_balance(self):
        stable_since = None
        deadline = time.monotonic() + self.args.stable_timeout
        while time.monotonic() < deadline and rclpy.ok():
            self.spin_for(0.05)
            stable = (
                self.current_mode() == MODE_BALANCE
                and abs(self.current_degree()) <= self.args.stable_angle_deg
                and abs(self.current_theta_dot()) <= self.args.stable_rate_rad_s
            )
            if stable:
                if stable_since is None:
                    stable_since = time.monotonic()
                if time.monotonic() - stable_since >= self.args.stable_time:
                    return True
            else:
                stable_since = None
        return False

    def direction_sign(self, direction):
        if direction == "positive":
            return 1.0
        if direction == "negative":
            return -1.0
        degree = self.current_degree()
        if math.isfinite(degree) and abs(degree) >= self.args.away_deadband_deg:
            return math.copysign(1.0, degree)
        return 1.0

    def failure_reason(self):
        mode = self.current_mode()
        degree = abs(self.current_degree())
        cart_cm = abs(self.current_cart_cm())
        if mode != MODE_BALANCE:
            return f"mode_exited_{mode}"
        if math.isfinite(degree) and degree >= self.args.fail_angle_deg:
            return "angle_limit"
        if math.isfinite(cart_cm) and cart_cm >= self.args.rail_fail_cm:
            return "rail_limit"
        return ""

    def run_trial(self, direction, force_n):
        sign = self.direction_sign(direction)
        torque_nm = sign * force_n * self.args.pendulum_length_m
        self.active_trial = {
            "direction": direction,
            "force_n": force_n,
            "torque_nm": torque_nm,
            "started_at": time.monotonic(),
        }
        self.last_sample_time = 0.0

        max_abs_degree = 0.0
        max_abs_cart_cm = 0.0
        max_abs_theta_dot = 0.0
        fall_reason = ""
        start = time.monotonic()
        deadline = start + self.args.push_duration

        while time.monotonic() < deadline and rclpy.ok():
            self.publish_disturbance(torque_nm)
            self.spin_for(0.02)
            max_abs_degree = max(max_abs_degree, abs(self.current_degree()))
            max_abs_cart_cm = max(max_abs_cart_cm, abs(self.current_cart_cm()))
            max_abs_theta_dot = max(max_abs_theta_dot, abs(self.current_theta_dot()))
            fall_reason = self.failure_reason()
            if fall_reason:
                break

        self.clear_disturbance()
        if not fall_reason:
            recovery_deadline = time.monotonic() + self.args.recovery_observe_time
            while time.monotonic() < recovery_deadline and rclpy.ok():
                self.spin_for(0.05)
                max_abs_degree = max(max_abs_degree, abs(self.current_degree()))
                max_abs_cart_cm = max(max_abs_cart_cm, abs(self.current_cart_cm()))
                max_abs_theta_dot = max(
                    max_abs_theta_dot,
                    abs(self.current_theta_dot()),
                )
                fall_reason = self.failure_reason()
                if fall_reason:
                    fall_reason = f"recovery_{fall_reason}"
                    break

        result = "fall" if fall_reason else "survived"
        self.trial_rows.append(
            {
                "workspace": self.args.workspace,
                "direction": direction,
                "force_n": fmt(force_n, 4),
                "torque_nm": fmt(torque_nm, 6),
                "result": result,
                "fall_reason": fall_reason,
                "max_abs_degree_deg": fmt(max_abs_degree, 6),
                "max_abs_cmX_cm": fmt(max_abs_cart_cm, 6),
                "max_abs_theta_dot_rad_s": fmt(max_abs_theta_dot, 9),
                "final_mode": str(self.current_mode()),
                "push_duration_s": fmt(time.monotonic() - start, 3),
                "sample_count": str(
                    sum(
                        1
                        for row in self.rows
                        if row["workspace"] == self.args.workspace
                        and row["direction"] == direction
                        and abs(float(row["force_n"]) - force_n) < 1e-9
                    )
                ),
            }
        )
        self.active_trial = None
        return result, fall_reason

    def run(self):
        directions = [item.strip() for item in self.args.directions.split(",") if item.strip()]
        for direction in directions:
            self.prepare_balance()
            previous_survived = math.nan
            critical_force = math.nan
            critical_reason = ""

            for force_n in self.args.forces:
                result, reason = self.run_trial(direction, force_n)
                if result == "fall":
                    critical_force = force_n
                    critical_reason = reason
                    break
                previous_survived = force_n
                if not self.wait_for_stable_balance():
                    critical_force = force_n
                    critical_reason = "did_not_recover_to_stable_balance"
                    break

            self.summary_rows.append(
                {
                    "workspace": self.args.workspace,
                    "direction": direction,
                    "critical_force_n": fmt(critical_force, 4),
                    "critical_torque_nm": fmt(
                        critical_force * self.args.pendulum_length_m,
                        6,
                    ),
                    "previous_survived_force_n": fmt(previous_survived, 4),
                    "fall_reason": critical_reason,
                    "tested_forces_n": ";".join(fmt(value, 4) for value in self.args.forces),
                    "pendulum_length_m": fmt(self.args.pendulum_length_m, 3),
                    "fail_angle_deg": fmt(self.args.fail_angle_deg, 3),
                    "rail_fail_cm": fmt(self.args.rail_fail_cm, 3),
                    "push_duration_s": fmt(self.args.push_duration, 3),
                    "recovery_observe_time_s": fmt(
                        self.args.recovery_observe_time,
                        3,
                    ),
                    "notes": "critical_force_is_minimum_tested_force_that_failed",
                }
            )
            self.clear_disturbance()

        if self.serial_fd is not None:
            os.close(self.serial_fd)
            self.serial_fd = None

    def write_outputs(self):
        self._write_csv(
            Path(self.args.samples_output),
            [
                "workspace",
                "direction",
                "force_n",
                "torque_nm",
                "elapsed_s",
                "trial_elapsed_s",
                "degree_deg",
                "cmX_cm",
                "setspeed_cm_s",
                "energy_j",
                "theta_dot_rad_s",
                "theta_rad",
                "x_center_cm",
                "mode",
                "cart_force_cmd_n",
                "hinge_total_torque_nm",
                "external_disturbance_torque_nm",
            ],
            self.rows,
        )
        self._write_csv(
            Path(self.args.trials_output),
            [
                "workspace",
                "direction",
                "force_n",
                "torque_nm",
                "result",
                "fall_reason",
                "max_abs_degree_deg",
                "max_abs_cmX_cm",
                "max_abs_theta_dot_rad_s",
                "final_mode",
                "push_duration_s",
                "sample_count",
            ],
            self.trial_rows,
        )
        self._write_csv(
            Path(self.args.summary_output),
            [
                "workspace",
                "direction",
                "critical_force_n",
                "critical_torque_nm",
                "previous_survived_force_n",
                "fall_reason",
                "tested_forces_n",
                "pendulum_length_m",
                "fail_angle_deg",
                "rail_fail_cm",
                "push_duration_s",
                "recovery_observe_time_s",
                "notes",
            ],
            self.summary_rows,
        )

    def _write_csv(self, output, fieldnames, rows):
        output.parent.mkdir(parents=True, exist_ok=True)
        write_header = not output.exists() or output.stat().st_size == 0
        with output.open("a", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=fieldnames)
            if write_header:
                writer.writeheader()
            writer.writerows(rows)


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--workspace", required=True)
    parser.add_argument("--serial", required=True)
    parser.add_argument(
        "--directions",
        default="positive",
        help="Comma list: positive, negative, or away.",
    )
    parser.add_argument("--pendulum-length-m", type=float, default=0.40)
    parser.add_argument(
        "--forces",
        type=parse_float_list,
        default=parse_float_list(
            "0.10,0.20,0.30,0.40,0.50,0.75,1.00,1.25,1.50,1.75,"
            "2.00,2.50,3.00,3.50,4.00,4.50,5.00,6.00,7.00,8.00,"
            "10.00,12.00,15.00"
        ),
    )
    parser.add_argument("--push-duration", type=float, default=3.0)
    parser.add_argument("--recovery-observe-time", type=float, default=2.0)
    parser.add_argument("--stable-time", type=float, default=1.0)
    parser.add_argument("--stable-timeout", type=float, default=8.0)
    parser.add_argument("--stable-angle-deg", type=float, default=1.0)
    parser.add_argument("--stable-rate-rad-s", type=float, default=0.20)
    parser.add_argument("--fail-angle-deg", type=float, default=18.0)
    parser.add_argument("--rail-fail-cm", type=float, default=38.0)
    parser.add_argument("--away-deadband-deg", type=float, default=0.05)
    parser.add_argument("--balance-timeout", type=float, default=35.0)
    parser.add_argument("--min-swing-before-balance", type=float, default=4.0)
    parser.add_argument("--manual-balance-deg", type=float, default=18.0)
    parser.add_argument("--force-balance-after", type=float, default=18.0)
    parser.add_argument("--state-timeout", type=float, default=20.0)
    parser.add_argument("--serial-timeout", type=float, default=20.0)
    parser.add_argument("--homing-timeout", type=float, default=5.0)
    parser.add_argument("--sample-period", type=float, default=0.05)
    parser.add_argument(
        "--samples-output",
        default="data_exports/external_disturbance_samples_20260509_three_ws.csv",
    )
    parser.add_argument(
        "--trials-output",
        default="data_exports/external_disturbance_trials_20260509_three_ws.csv",
    )
    parser.add_argument(
        "--summary-output",
        default="data_exports/external_disturbance_threshold_summary_20260509_three_ws.csv",
    )
    return parser.parse_args()


def main():
    args = parse_args()
    rclpy.init()
    node = ExternalDisturbanceSweep(args)
    try:
        node.run()
        node.write_outputs()
        for row in node.summary_rows:
            print(
                row["workspace"],
                row["direction"],
                "critical_force_n=" + row["critical_force_n"],
                "previous_survived_force_n=" + row["previous_survived_force_n"],
                "reason=" + row["fall_reason"],
            )
    finally:
        node.clear_disturbance()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
