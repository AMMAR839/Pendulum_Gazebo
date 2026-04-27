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
BUTTON_X = 1 << 2
BUTTON_Y = 1 << 3

MODE_READY = 3
MODE_SWING_UP = 6
MODE_BALANCE = 7


def make_joystick_packet(seq, buttons):
    body = struct.pack("<BBhhhhH", 0x01, seq & 0xFF, 0, 0, 0, 0, buttons)
    checksum = sum(body) & 0xFFFF
    return b"\xAA\x55" + body + struct.pack("<H", checksum)


class SwingBalanceCapture(Node):
    def __init__(self, args):
        super().__init__("pendulum_swing_balance_capture")
        self.args = args
        self.latest_state = None
        self.latest_cart_velocity_cmd = math.nan
        self.latest_cart_force = math.nan
        self.latest_hinge_assist = math.nan
        self.rows = []
        self.start_time = time.monotonic()
        self.last_record_time = 0.0
        self.seq = 0
        self.serial_fd = None
        self.note = "auto_capture"

        self.create_subscription(
            Float64MultiArray,
            "/pendulum/sim_state",
            self._state_cb,
            50,
        )
        self.create_subscription(
            Float64,
            "/pendulum/cart_velocity_cmd",
            self._cart_velocity_cb,
            50,
        )
        self.create_subscription(Float64, "/pendulum/cart_force_cmd", self._force_cb, 50)
        self.create_subscription(
            Float64,
            "/pendulum/hinge_assist_force_cmd",
            self._hinge_cb,
            50,
        )

    def _cart_velocity_cb(self, msg):
        self.latest_cart_velocity_cmd = float(msg.data)

    def _force_cb(self, msg):
        self.latest_cart_force = float(msg.data)

    def _hinge_cb(self, msg):
        self.latest_hinge_assist = float(msg.data)

    def _state_cb(self, msg):
        if len(msg.data) < 8:
            return
        now = time.monotonic()
        data = list(msg.data[:8])
        self.latest_state = data

        if now - self.last_record_time < self.args.sample_period:
            return
        self.last_record_time = now

        mode = int(round(data[7]))
        if mode == MODE_SWING_UP:
            phase = "SWING_UP"
        elif mode == MODE_BALANCE:
            phase = "BALANCE"
        else:
            return

        elapsed = now - self.start_time
        self.rows.append(
            {
                "workspace": self.args.workspace,
                "phase": phase,
                "elapsed_s": f"{elapsed:.3f}",
                "degree_deg": f"{data[0]:.6f}",
                "cmX_cm": f"{data[1]:.6f}",
                "setspeed_cm_s": f"{data[2]:.6f}",
                "energy_j": f"{data[3]:.9f}",
                "theta_dot_rad_s": f"{data[4]:.9f}",
                "theta_rad": f"{data[5]:.9f}",
                "x_center_cm": f"{data[6]:.6f}",
                "mode": mode,
                "cart_velocity_cmd_mps": f"{self.latest_cart_velocity_cmd:.9f}",
                "motor_command_cm_s": f"{self.latest_cart_velocity_cmd * 100.0:.9f}",
                "cart_joint_effort_cmd_n": f"{self.latest_cart_force:.9f}",
                "cart_force_cmd_n": f"{self.latest_cart_force:.9f}",
                "hinge_assist_torque_nm": f"{self.latest_hinge_assist:.9f}",
                "note": self.note,
            }
        )

    def wait_for_state(self, timeout_s):
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.latest_state is not None:
                return True
        return False

    def spin_for(self, duration_s):
        deadline = time.monotonic() + duration_s
        while time.monotonic() < deadline and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.05)

    def current_mode(self):
        if self.latest_state is None:
            return None
        return int(round(self.latest_state[7]))

    def current_degree(self):
        if self.latest_state is None:
            return math.nan
        return float(self.latest_state[0])

    def open_serial(self):
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

    def run_sequence(self):
        if not self.wait_for_state(self.args.state_timeout):
            raise RuntimeError("No /pendulum/sim_state samples received")
        self.open_serial()

        self.send_button(BUTTON_Y)
        ready_deadline = time.monotonic() + self.args.homing_timeout
        while time.monotonic() < ready_deadline and self.current_mode() != MODE_READY:
            self.spin_for(0.1)

        self.send_button(BUTTON_X)
        swing_started = time.monotonic()
        balance_requested = False
        balance_seen = False

        while time.monotonic() - swing_started < self.args.run_time:
            self.spin_for(0.05)
            mode = self.current_mode()
            degree = abs(self.current_degree())
            since_swing = time.monotonic() - swing_started

            if mode == MODE_BALANCE:
                balance_seen = True

            if (
                not balance_seen
                and not balance_requested
                and since_swing >= self.args.min_swing_before_balance
                and degree <= self.args.manual_balance_deg
            ):
                self.note = "manual_A_near_upright"
                self.send_button(BUTTON_A)
                balance_requested = True

            if (
                not balance_seen
                and not balance_requested
                and since_swing >= self.args.force_balance_after
            ):
                self.note = "manual_A_timeout"
                self.send_button(BUTTON_A)
                balance_requested = True

        if self.serial_fd is not None:
            os.close(self.serial_fd)

    def write_csv(self):
        output = Path(self.args.output)
        output.parent.mkdir(parents=True, exist_ok=True)
        fieldnames = [
            "workspace",
            "phase",
            "elapsed_s",
            "degree_deg",
            "cmX_cm",
            "setspeed_cm_s",
            "energy_j",
            "theta_dot_rad_s",
            "theta_rad",
            "x_center_cm",
            "mode",
            "cart_velocity_cmd_mps",
            "motor_command_cm_s",
            "cart_joint_effort_cmd_n",
            "cart_force_cmd_n",
            "hinge_assist_torque_nm",
            "note",
        ]
        write_header = not output.exists() or output.stat().st_size == 0
        with output.open("a", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=fieldnames)
            if write_header:
                writer.writeheader()
            writer.writerows(self.rows)


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--workspace", required=True)
    parser.add_argument("--serial", required=True)
    parser.add_argument("--output", required=True)
    parser.add_argument("--run-time", type=float, default=24.0)
    parser.add_argument("--sample-period", type=float, default=0.05)
    parser.add_argument("--state-timeout", type=float, default=20.0)
    parser.add_argument("--serial-timeout", type=float, default=20.0)
    parser.add_argument("--homing-timeout", type=float, default=5.0)
    parser.add_argument("--min-swing-before-balance", type=float, default=4.0)
    parser.add_argument("--manual-balance-deg", type=float, default=18.0)
    parser.add_argument("--force-balance-after", type=float, default=18.0)
    return parser.parse_args()


def main():
    args = parse_args()
    rclpy.init()
    node = SwingBalanceCapture(args)
    try:
        node.run_sequence()
        node.write_csv()
        print(f"wrote {len(node.rows)} rows to {args.output}")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
