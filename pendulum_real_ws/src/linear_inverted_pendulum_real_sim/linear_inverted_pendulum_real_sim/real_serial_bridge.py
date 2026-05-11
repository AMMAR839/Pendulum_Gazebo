import errno
import math
import os
import pty
import select
import struct
import threading
import time
import tty

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64, Float64MultiArray

try:
    from gz.msgs10.boolean_pb2 import Boolean as GzBoolean
    from gz.msgs10.entity_factory_pb2 import EntityFactory as GzEntityFactory
    from gz.msgs10.marker_pb2 import Marker as GzMarker
    from gz.msgs10.pose_pb2 import Pose as GzPose
    from gz.transport13 import Node as GzTransportNode
except ImportError:  # pragma: no cover - optional Gazebo GUI marker support
    GzBoolean = None
    GzEntityFactory = None
    GzMarker = None
    GzPose = None
    GzTransportNode = None

try:
    import numpy as np
    from scipy.linalg import solve_continuous_are
except ImportError:  # pragma: no cover - optional runtime dependency
    np = None
    solve_continuous_are = None


HEADER_TX = b"\xAA\x55"
HEADER_STATUS = b"\xAA\xCC"
HEADER_ACK = b"\xAA\xDD"
HEADER_RESET = b"\xAA\xEE"

MODE_WAITING = 1
MODE_HOMING = 2
MODE_READY = 3
MODE_SINE = 4
MODE_FINISH = 5
MODE_SWING_UP = 6
MODE_BALANCE = 7

EXTERNAL_FORCE_GAZEBO_VISUAL_ENABLED = False
EXTERNAL_FORCE_WORLD_NAME = "empty"
EXTERNAL_FORCE_MODEL_NAME = "external_force_visual"
EXTERNAL_FORCE_MODEL_SDF = """
<sdf version="1.7">
  <model name="external_force_visual">
    <static>true</static>
    <pose>0 0.34 -8.0 0 0 0</pose>
    <link name="indicator">
      <visual name="force_bar">
        <geometry>
          <box>
            <size>0.46 0.04 0.07</size>
          </box>
        </geometry>
        <material>
          <ambient>1.0 0.48 0.0 1</ambient>
          <diffuse>1.0 0.48 0.0 1</diffuse>
          <emissive>1.0 0.18 0.0 1</emissive>
        </material>
      </visual>
      <visual name="force_head">
        <pose>0.27 0 0 0 0 0</pose>
        <geometry>
          <sphere>
            <radius>0.07</radius>
          </sphere>
        </geometry>
        <material>
          <ambient>1.0 0.95 0.0 1</ambient>
          <diffuse>1.0 0.95 0.0 1</diffuse>
          <emissive>1.0 0.62 0.0 1</emissive>
        </material>
      </visual>
      <visual name="force_tail">
        <pose>-0.27 0 0 0 0 0</pose>
        <geometry>
          <sphere>
            <radius>0.035</radius>
          </sphere>
        </geometry>
        <material>
          <ambient>1.0 0.05 0.0 1</ambient>
          <diffuse>1.0 0.05 0.0 1</diffuse>
          <emissive>0.75 0.0 0.0 1</emissive>
        </material>
      </visual>
    </link>
  </model>
</sdf>
"""

BUTTON_A = 1 << 0
BUTTON_B = 1 << 1
BUTTON_X = 1 << 2
BUTTON_Y = 1 << 3


def clamp(value, low, high):
    return max(low, min(high, value))


def wrap_pi(angle):
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def env_bool(name, default=False):
    value = os.environ.get(name)
    if value is None:
        return default
    return value.strip().lower() in ("1", "true", "yes", "on")


class SimSerialBridge(Node):
    """Manual-book style Gazebo bridge for the STM32 serial protocol used by main.py."""

    def __init__(self):
        super().__init__("pendulum_real_serial_bridge")

        self.declare_parameter("serial_symlink", "/tmp/pendulum_real_serial")
        self.declare_parameter("status_rate_hz", 100.0)
        self.declare_parameter("cart_velocity_limit_mps", 3.00)
        self.declare_parameter("command_accel_limit_mps2", 25.0)
        self.declare_parameter("cart_effort_rate_limit_nps", 6000.0)
        self.declare_parameter("manual_velocity_mps", 0.45)
        self.declare_parameter("rail_limit_m", 0.39)
        self.declare_parameter("cart_mass_kg", 1.20)
        self.declare_parameter("pendulum_mass_kg", 0.20)
        self.declare_parameter("pendulum_com_m", 0.20)
        self.declare_parameter("pendulum_length_m", 0.40)
        self.declare_parameter("external_force_visual_hold_s", 10.0)
        self.declare_parameter("swing_gain", 2.60)
        self.declare_parameter("swing_centering_gain", 2.80)
        self.declare_parameter("swing_damping_gain", 0.45)
        self.declare_parameter("swing_kick_mps", 0.82)
        self.declare_parameter("swing_kick_force_n", 28.0)
        self.declare_parameter("swing_min_pump_force_n", 8.0)
        self.declare_parameter("swing_min_pump_force_extra_n", 18.0)
        self.declare_parameter("swing_force_rate_limit_nps", 450.0)
        self.declare_parameter("swing_force_limit_n", 150.0)
        self.declare_parameter("swing_min_top_passes_before_catch", 3)
        self.declare_parameter("swing_min_energy_build_time_s", 8.0)
        self.declare_parameter("swing_energy_ready_ratio", 0.88)
        self.declare_parameter("swing_top_pass_angle_deg", 80.0)
        self.declare_parameter("balance_capture_deg", 22.0)
        self.declare_parameter("balance_capture_rate_rad_s", 1.5)
        self.declare_parameter("balance_fallback_deg", 70.0)
        self.declare_parameter("balance_give_up_deg", 135.0)
        self.declare_parameter("balance_capture_cart_pos_m", 0.16)
        self.declare_parameter("balance_capture_cart_vel_mps", 1.2)
        self.declare_parameter("balance_immediate_capture_deg", 2.0)
        self.declare_parameter("balance_immediate_capture_rate_rad_s", 1.0)
        self.declare_parameter("balance_immediate_capture_cart_pos_m", 0.16)
        self.declare_parameter("balance_immediate_capture_cart_vel_mps", 1.0)
        self.declare_parameter("balance_auto_lock_enabled", True)
        self.declare_parameter("balance_auto_lock_angle_deg", 3.0)
        self.declare_parameter("balance_auto_lock_rate_rad_s", 1.5)
        self.declare_parameter("balance_auto_lock_cart_pos_m", 0.16)
        self.declare_parameter("balance_auto_lock_cart_vel_mps", 1.2)
        self.declare_parameter("balance_auto_lock_time_s", 0.08)
        self.declare_parameter("catch_region_deg", 95.0)
        self.declare_parameter("catch_region_rate_rad_s", 14.0)
        self.declare_parameter("catch_force_limit_n", 260.0)
        self.declare_parameter("balance_force_limit_n", 260.0)
        self.declare_parameter("balance_use_lqr", False)
        self.declare_parameter("balance_assist_enabled", False)
        self.declare_parameter("balance_assist_angle_deg", 55.0)
        self.declare_parameter("balance_assist_kp_nm_per_rad", 3.0)
        self.declare_parameter("balance_assist_kd_nm_per_rad_s", 1.6)
        self.declare_parameter("balance_assist_torque_limit_nm", 3.0)
        self.declare_parameter("balance_integral_force_gain", 1.0)
        self.declare_parameter("balance_centering_force_gain", 220.0)
        self.declare_parameter("balance_cart_damping_force_gain", 90.0)
        self.declare_parameter("balance_center_reference_m", 0.0)
        self.declare_parameter("balance_center_theta_bias_rad_per_m", 0.42)
        self.declare_parameter("balance_center_theta_bias_rad_per_mps", 0.10)
        self.declare_parameter("balance_center_theta_bias_limit_deg", 4.0)
        self.declare_parameter("balance_theta_kp", 220.0)
        self.declare_parameter("balance_theta_kd", 60.0)
        self.declare_parameter("balance_centering_accel_gain", 3.5)
        self.declare_parameter("balance_cart_damping_accel_gain", 1.6)
        self.declare_parameter("force_to_velocity_gain", 0.014)

        self.declare_parameter("lqr_q_x", 40.0)
        self.declare_parameter("lqr_q_x_dot", 12.0)
        self.declare_parameter("lqr_q_theta", 1000.0)
        self.declare_parameter("lqr_q_theta_dot", 600.0)
        self.declare_parameter("lqr_r", 0.04)

        self.declare_parameter("motor_pwm_deadband", 3212.0)
        self.declare_parameter("motor_pwm_per_cmps", 189.1)
        self.declare_parameter("motor_time_constant_s", 0.40)
        self.declare_parameter("motor_velocity_servo_p", 55.0)
        self.declare_parameter("motor_velocity_servo_d", 1.2)
        self.declare_parameter("effort_limit_n", 260.0)
        self.declare_parameter(
            "upright_joint_position_rad",
            float(os.environ.get("PENDULUM_UPRIGHT_JOINT_POSITION_RAD", math.pi)),
        )
        self.declare_parameter(
            "start_in_balance",
            env_bool("PENDULUM_START_IN_BALANCE", False),
        )

        self.serial_symlink = str(self.get_parameter("serial_symlink").value)
        self.status_rate_hz = float(self.get_parameter("status_rate_hz").value)
        self.velocity_limit = float(self.get_parameter("cart_velocity_limit_mps").value)
        self.command_accel_limit = float(
            self.get_parameter("command_accel_limit_mps2").value
        )
        self.cart_effort_rate_limit_nps = float(
            self.get_parameter("cart_effort_rate_limit_nps").value
        )
        self.manual_velocity = float(self.get_parameter("manual_velocity_mps").value)
        self.rail_limit = float(self.get_parameter("rail_limit_m").value)
        self.cart_mass = float(self.get_parameter("cart_mass_kg").value)
        self.pendulum_mass = float(self.get_parameter("pendulum_mass_kg").value)
        self.pendulum_com = float(self.get_parameter("pendulum_com_m").value)
        self.pendulum_length = float(self.get_parameter("pendulum_length_m").value)
        self.external_force_visual_hold_s = float(
            self.get_parameter("external_force_visual_hold_s").value
        )
        self.start_time = time.monotonic()
        self.swing_state = "CALC"
        self.swing_target_v = 0.0
        self.energy_now = 0.0
        self.energy_target = 2.0 * self.pendulum_mass * 9.81 * self.pendulum_com
        self.swing_top_passes = 0
        self.swing_top_pass_armed = True
        self.balance_request_pending = False
        self.upright_stable_since = None

        self.swing_gain = float(self.get_parameter("swing_gain").value)
        self.swing_centering_gain = float(
            self.get_parameter("swing_centering_gain").value
        )
        self.swing_damping_gain = float(
            self.get_parameter("swing_damping_gain").value
        )
        self.swing_kick_mps = float(self.get_parameter("swing_kick_mps").value)
        self.swing_kick_force_n = float(
            self.get_parameter("swing_kick_force_n").value
        )
        self.swing_min_pump_force_n = float(
            self.get_parameter("swing_min_pump_force_n").value
        )
        self.swing_min_pump_force_extra_n = float(
            self.get_parameter("swing_min_pump_force_extra_n").value
        )
        self.swing_force_rate_limit_nps = float(
            self.get_parameter("swing_force_rate_limit_nps").value
        )
        self.swing_force_limit_n = float(
            self.get_parameter("swing_force_limit_n").value
        )
        self.swing_min_top_passes_before_catch = max(
            0,
            int(self.get_parameter("swing_min_top_passes_before_catch").value),
        )
        self.swing_min_energy_build_time_s = float(
            self.get_parameter("swing_min_energy_build_time_s").value
        )
        self.swing_energy_ready_ratio = float(
            self.get_parameter("swing_energy_ready_ratio").value
        )
        self.swing_top_pass_angle = math.radians(
            float(self.get_parameter("swing_top_pass_angle_deg").value)
        )
        self.balance_capture = math.radians(
            float(self.get_parameter("balance_capture_deg").value)
        )
        self.balance_capture_rate = float(
            self.get_parameter("balance_capture_rate_rad_s").value
        )
        self.balance_fallback = math.radians(
            float(self.get_parameter("balance_fallback_deg").value)
        )
        self.balance_give_up = math.radians(
            float(self.get_parameter("balance_give_up_deg").value)
        )
        self.balance_capture_cart_pos = float(
            self.get_parameter("balance_capture_cart_pos_m").value
        )
        self.balance_capture_cart_vel = float(
            self.get_parameter("balance_capture_cart_vel_mps").value
        )
        self.balance_immediate_capture_angle = math.radians(
            float(self.get_parameter("balance_immediate_capture_deg").value)
        )
        self.balance_immediate_capture_rate = float(
            self.get_parameter("balance_immediate_capture_rate_rad_s").value
        )
        self.balance_immediate_capture_cart_pos = float(
            self.get_parameter("balance_immediate_capture_cart_pos_m").value
        )
        self.balance_immediate_capture_cart_vel = float(
            self.get_parameter("balance_immediate_capture_cart_vel_mps").value
        )
        self.balance_auto_lock_enabled = bool(
            self.get_parameter("balance_auto_lock_enabled").value
        )
        self.balance_auto_lock_angle = math.radians(
            float(self.get_parameter("balance_auto_lock_angle_deg").value)
        )
        self.balance_auto_lock_rate = float(
            self.get_parameter("balance_auto_lock_rate_rad_s").value
        )
        self.balance_auto_lock_cart_pos = float(
            self.get_parameter("balance_auto_lock_cart_pos_m").value
        )
        self.balance_auto_lock_cart_vel = float(
            self.get_parameter("balance_auto_lock_cart_vel_mps").value
        )
        self.balance_auto_lock_time_s = float(
            self.get_parameter("balance_auto_lock_time_s").value
        )
        self.catch_region_angle = math.radians(
            float(self.get_parameter("catch_region_deg").value)
        )
        self.catch_region_rate = float(
            self.get_parameter("catch_region_rate_rad_s").value
        )
        self.catch_force_limit_n = float(
            self.get_parameter("catch_force_limit_n").value
        )
        self.balance_force_limit_n = float(
            self.get_parameter("balance_force_limit_n").value
        )
        self.balance_use_lqr = bool(self.get_parameter("balance_use_lqr").value)
        self.balance_assist_enabled = bool(
            self.get_parameter("balance_assist_enabled").value
        )
        self.balance_assist_angle = math.radians(
            float(self.get_parameter("balance_assist_angle_deg").value)
        )
        self.balance_assist_kp = float(
            self.get_parameter("balance_assist_kp_nm_per_rad").value
        )
        self.balance_assist_kd = float(
            self.get_parameter("balance_assist_kd_nm_per_rad_s").value
        )
        self.balance_assist_torque_limit = float(
            self.get_parameter("balance_assist_torque_limit_nm").value
        )
        self.balance_integral_force_gain = float(
            self.get_parameter("balance_integral_force_gain").value
        )
        self.balance_centering_force_gain = float(
            self.get_parameter("balance_centering_force_gain").value
        )
        self.balance_cart_damping_force_gain = float(
            self.get_parameter("balance_cart_damping_force_gain").value
        )
        self.balance_center_reference_m = clamp(
            float(self.get_parameter("balance_center_reference_m").value),
            -self.rail_limit + 0.08,
            self.rail_limit - 0.08,
        )
        self.balance_center_theta_bias_gain = float(
            self.get_parameter("balance_center_theta_bias_rad_per_m").value
        )
        self.balance_center_theta_bias_rate_gain = float(
            self.get_parameter("balance_center_theta_bias_rad_per_mps").value
        )
        self.balance_center_theta_bias_limit = math.radians(
            float(self.get_parameter("balance_center_theta_bias_limit_deg").value)
        )
        self.balance_theta_kp = float(self.get_parameter("balance_theta_kp").value)
        self.balance_theta_kd = float(self.get_parameter("balance_theta_kd").value)
        self.balance_centering_accel_gain = float(
            self.get_parameter("balance_centering_accel_gain").value
        )
        self.balance_cart_damping_accel_gain = float(
            self.get_parameter("balance_cart_damping_accel_gain").value
        )
        self.force_to_velocity_gain = float(
            self.get_parameter("force_to_velocity_gain").value
        )
        self.lqr_q_x = float(self.get_parameter("lqr_q_x").value)
        self.lqr_q_x_dot = float(self.get_parameter("lqr_q_x_dot").value)
        self.lqr_q_theta = float(self.get_parameter("lqr_q_theta").value)
        self.lqr_q_theta_dot = float(self.get_parameter("lqr_q_theta_dot").value)
        self.lqr_r = float(self.get_parameter("lqr_r").value)
        self.motor_pwm_deadband = float(self.get_parameter("motor_pwm_deadband").value)
        self.motor_pwm_per_cmps = float(
            self.get_parameter("motor_pwm_per_cmps").value
        )
        self.motor_time_constant_s = float(
            self.get_parameter("motor_time_constant_s").value
        )
        self.motor_velocity_servo_p = float(
            self.get_parameter("motor_velocity_servo_p").value
        )
        self.motor_velocity_servo_d = float(
            self.get_parameter("motor_velocity_servo_d").value
        )
        self.effort_limit_n = float(self.get_parameter("effort_limit_n").value)
        self.upright_joint_position_rad = float(
            self.get_parameter("upright_joint_position_rad").value
        )
        self.start_in_balance = bool(self.get_parameter("start_in_balance").value)
        self.external_disturbance_torque_nm = 0.0

        self.cmd_pub = self.create_publisher(Float64, "/pendulum/cart_velocity_cmd", 10)
        self.effort_pub = self.create_publisher(Float64, "/pendulum/cart_force_cmd", 10)
        self.hinge_effort_pub = self.create_publisher(
            Float64,
            "/pendulum/hinge_assist_force_cmd",
            10,
        )
        self.state_pub = self.create_publisher(Float64MultiArray, "/pendulum/sim_state", 10)
        self.create_subscription(JointState, "joint_states", self._joint_state_cb, 10)
        self.create_subscription(
            Float64,
            "/pendulum/external_disturbance_torque_cmd",
            self._external_disturbance_torque_cb,
            10,
        )
        self.gz_marker_node = None
        self.external_force_marker_pub = None
        self.external_force_marker_visible = False
        self.external_force_marker_hold_until = 0.0
        self.external_force_marker_last_force_n = 0.0
        self.external_force_visual_model_ready = False
        self.external_force_visual_model_visible = False
        self.external_force_visual_model_last_key = None
        self.external_force_visual_model_last_try = 0.0
        self.external_force_visual_model_last_update = 0.0
        self.external_force_visual_model_warned = False
        if (
            EXTERNAL_FORCE_GAZEBO_VISUAL_ENABLED
            and GzTransportNode is not None
            and GzMarker is not None
        ):
            try:
                self.gz_marker_node = GzTransportNode()
                self.external_force_marker_pub = self.gz_marker_node.advertise(
                    "/marker",
                    GzMarker,
                )
            except Exception as exc:
                self.get_logger().warning(f"Gazebo marker publisher unavailable: {exc}")

        self.lock = threading.Lock()
        self.cart_x_m = 0.0
        self.cart_v_mps = 0.0
        self.motor_target_mps = 0.0
        self.motor_velocity_mps = 0.0
        self.prev_motor_velocity_error = 0.0
        self.motor_pwm = 0.0
        self.prev_velocity_error = 0.0
        self.velocity_error_integral = 0.0
        self.pendulum_down_rad = 0.0
        self.pendulum_vel_radps = 0.0
        self.manual_cmd_mps = 0.0
        self.cmd_mps = 0.0
        self.cart_effort_cmd_n = 0.0
        self.swing_force_cmd_n = 0.0
        self.x_center_m = 0.0
        self.x_integral_cm_s = 0.0
        self.mode = MODE_WAITING
        self.last_buttons = 0
        self.mode_started = time.monotonic()
        self.last_control_time = time.monotonic()

        self.gains = {
            "K_TH": 11.0,
            "K_TH_D": 3.6,
            "K_X": 3.0,
            "K_X_D": 4.0,
            "K_X_INT": 0.10,
        }
        self.balance_lqr_gain = None
        if self.balance_use_lqr:
            self.balance_lqr_gain = self._build_lqr_gain(
                q_x=self.lqr_q_x,
                q_x_dot=self.lqr_q_x_dot,
                q_theta=self.lqr_q_theta,
                q_theta_dot=self.lqr_q_theta_dot,
                r_value=self.lqr_r,
            )
        if self.start_in_balance:
            self._set_mode_locked(MODE_BALANCE)

        self.master_fd = None
        self.serial_thread = None
        self.serial_running = True
        self._setup_pty()

        period = 1.0 / max(1.0, self.status_rate_hz)
        self.timer = self.create_timer(period, self._control_and_status_timer)

        self.get_logger().info(
            f"Pseudo serial ready at {self.serial_symlink}. "
            "Run the GUI with PENDULUM_PORT pointing there."
        )
        self.get_logger().info(
            "Manual-book motor model: "
            f"deadband={self.motor_pwm_deadband:.0f} PWM, "
            f"slope={self.motor_pwm_per_cmps:.1f} PWM/(cm/s), "
            f"tau={self.motor_time_constant_s:.2f}s"
        )
        self.get_logger().info(
            "Real-like force caps: "
            f"swing={self.swing_force_limit_n:.1f} N, "
            f"catch={self.catch_force_limit_n:.1f} N, "
            f"balance={self.balance_force_limit_n:.1f} N, "
            f"absolute={self.effort_limit_n:.1f} N"
        )
        if self.balance_lqr_gain is not None:
            self.get_logger().info(
                "Optional LQR upright controller gain (force input): "
                + ", ".join(f"{gain:.2f}" for gain in self.balance_lqr_gain)
            )

    def _swing_up_energy_command_locked(self, theta_top, dt, now):
        energy = self._energy(theta_top)
        target_energy = 2.0 * self.pendulum_mass * 9.81 * self.pendulum_com

        self.energy_now = energy
        self.energy_target = target_energy
        self._update_swing_top_passes_locked(theta_top)
        swing_energy_ready = self._swing_energy_ready_locked(energy, target_energy)
        capture_armed = (
            self.balance_request_pending
            or self.swing_top_passes >= self.swing_min_top_passes_before_catch
        )

        theta_deg = math.degrees(theta_top)

        centering_force = self._swing_centering_force_locked(theta_top)
        if centering_force is not None:
            centering_force = self._smooth_swing_force_locked(centering_force, dt)
            return self._force_to_command_hint(centering_force), centering_force

        ready_to_capture = capture_armed and swing_energy_ready
        manual_capture_ready = (
            self.balance_request_pending
            and self._ready_for_balance_locked(theta_top)
        )
        if manual_capture_ready or (ready_to_capture and (
            self._upright_immediate_capture_ready_locked(theta_top)
            or self._upright_auto_lock_ready_locked(theta_top, now)
        )):
            self._set_mode_locked(MODE_BALANCE)
            self.swing_state = "CALC"
            force = self._balance_force_locked(theta_top, dt)
            return self._force_to_command_hint(force), force

        # Jangan menangkap ayunan pertama. Biarkan swing-up mengumpulkan energi
        # beberapa kali dulu, lalu balance saat state sudah layak.
        if (
            ready_to_capture
            and self._ready_for_balance_locked(theta_top)
        ):
            self._set_mode_locked(MODE_BALANCE)
            self.swing_state = "CALC"
            force = self._balance_force_locked(theta_top, dt)
            return self._force_to_command_hint(force), force

        # Di sekitar posisi atas, gunakan command full-state ringan untuk
        # memperlambat ayunan tanpa torsi bantuan langsung di engsel.
        if (
            (ready_to_capture or abs(theta_top) < math.radians(45.0))
            and self._in_catch_region_locked(theta_top)
        ):
            self.swing_state = "CALC"
            force = self._catch_force_locked(theta_top, dt)
            return self._force_to_command_hint(force), force

        phase = self.pendulum_vel_radps * math.cos(theta_top)
        energy_deficit = target_energy - energy
        force = 52.0 * self.swing_gain * energy_deficit * phase
        force -= 34.0 * self.swing_centering_gain * self.cart_x_m
        force -= 14.0 * self.swing_damping_gain * self.cart_v_mps

        if energy_deficit > 0.04 and abs(phase) > 0.05:
            pump_direction = math.copysign(1.0, phase)
            min_force = self.swing_min_pump_force_n + (
                self.swing_min_pump_force_extra_n
                * clamp(
                    energy_deficit / max(target_energy, 1e-6),
                    0.0,
                    1.0,
                )
            )
            if abs(force) < min_force:
                force = pump_direction * min_force

        near_top = abs(theta_top) < math.radians(125.0)
        moving_toward_upright = theta_top * self.pendulum_vel_radps < 0.0
        if energy_deficit > 0.02 and near_top and moving_toward_upright:
            top_boost = math.copysign(
                30.0
                + 45.0
                * clamp(
                    1.0 - abs(theta_top) / math.radians(125.0),
                    0.0,
                    1.0,
                ),
                theta_top,
            )
            if abs(force) < abs(top_boost):
                force = top_boost

        # Dari posisi bawah, energy law bernilai nol karena theta_dot masih nol.
        # Beri kick kecil; bila cart sudah tidak di tengah, tendang ke arah tengah.
        if abs(theta_deg) > 160.0 and abs(self.pendulum_vel_radps) < 0.60:
            if abs(self.cart_x_m) > 0.04:
                kick_direction = -math.copysign(1.0, self.cart_x_m)
            else:
                elapsed = time.monotonic() - self.swing_state_started
                kick_direction = 1.0 if int(elapsed / 0.45) % 2 == 0 else -1.0
            force += kick_direction * self.swing_kick_force_n

        # Jangan terus mendorong keluar saat cart sudah dekat limit rel.
        force = self._rail_aware_swing_force_locked(force)

        force = clamp(force, -self.swing_force_limit_n, self.swing_force_limit_n)
        force = self._smooth_swing_force_locked(force, dt)
        return self._force_to_command_hint(force), force

    def destroy_node(self):
        self.serial_running = False
        if self.master_fd is not None:
            try:
                os.close(self.master_fd)
            except OSError:
                pass
        if os.path.islink(self.serial_symlink):
            try:
                os.unlink(self.serial_symlink)
            except OSError:
                pass
        super().destroy_node()

    def _setup_pty(self):
        self.master_fd, slave_fd = pty.openpty()
        slave_name = os.ttyname(slave_fd)
        tty.setraw(self.master_fd)
        tty.setraw(slave_fd)
        os.set_blocking(self.master_fd, False)

        if os.path.islink(self.serial_symlink) or os.path.exists(self.serial_symlink):
            os.unlink(self.serial_symlink)
        os.symlink(slave_name, self.serial_symlink)
        os.close(slave_fd)

        self.serial_thread = threading.Thread(target=self._serial_reader, daemon=True)
        self.serial_thread.start()

    def _joint_state_cb(self, msg):
        with self.lock:
            for index, name in enumerate(msg.name):
                joint = name.split("/")[-1].split("::")[-1]
                pos = msg.position[index] if index < len(msg.position) else 0.0
                vel = msg.velocity[index] if index < len(msg.velocity) else 0.0

                if joint == "cart_slider":
                    self.cart_x_m = pos
                    self.cart_v_mps = 0.8 * self.cart_v_mps + 0.2 * vel
                elif joint == "pendulum_hinge":
                    self.pendulum_down_rad = pos
                    self.pendulum_vel_radps = (
                        0.8 * self.pendulum_vel_radps + 0.2 * vel
                    )

    def _external_disturbance_torque_cb(self, msg):
        self.external_disturbance_torque_nm = float(msg.data)

    def _ensure_external_force_visual_model(self):
        if (
            self.gz_marker_node is None
            or GzEntityFactory is None
            or GzBoolean is None
        ):
            return False
        if self.external_force_visual_model_ready:
            return True

        now = time.monotonic()
        if now - self.external_force_visual_model_last_try < 0.5:
            return False
        self.external_force_visual_model_last_try = now

        request = GzEntityFactory()
        request.sdf = EXTERNAL_FORCE_MODEL_SDF
        request.name = EXTERNAL_FORCE_MODEL_NAME
        request.allow_renaming = False
        try:
            ok, _response = self.gz_marker_node.request(
                f"/world/{EXTERNAL_FORCE_WORLD_NAME}/create",
                request,
                GzEntityFactory,
                GzBoolean,
                300,
            )
        except Exception as exc:
            if not self.external_force_visual_model_warned:
                self.get_logger().warning(
                    f"Gazebo external-force visual create unavailable: {exc}"
                )
                self.external_force_visual_model_warned = True
            return False

        self.external_force_visual_model_ready = bool(ok)
        return self.external_force_visual_model_ready

    def _set_external_force_visual_model(self, force_n, visible):
        if GzPose is None or GzBoolean is None:
            return
        if not visible and not self.external_force_visual_model_ready:
            return
        if visible and not self._ensure_external_force_visual_model():
            return

        sign = 1.0 if force_n >= 0.0 else -1.0
        key = ("visible", sign) if visible else ("hidden", 0.0)
        now = time.monotonic()
        if (
            key == self.external_force_visual_model_last_key
            and now - self.external_force_visual_model_last_update < 0.5
        ):
            return

        pose = GzPose()
        pose.name = EXTERNAL_FORCE_MODEL_NAME
        pose.position.x = sign * 0.06 if visible else 0.0
        pose.position.y = 0.34
        pose.position.z = 0.78 if visible else -8.0
        pose.orientation.w = 1.0 if sign >= 0.0 else 0.0
        pose.orientation.z = 0.0 if sign >= 0.0 else 1.0

        try:
            ok, response = self.gz_marker_node.request(
                f"/world/{EXTERNAL_FORCE_WORLD_NAME}/set_pose",
                pose,
                GzPose,
                GzBoolean,
                120,
            )
        except Exception as exc:
            if not self.external_force_visual_model_warned:
                self.get_logger().warning(
                    f"Gazebo external-force visual pose unavailable: {exc}"
                )
                self.external_force_visual_model_warned = True
            return

        if ok and getattr(response, "data", False):
            self.external_force_visual_model_visible = visible
            self.external_force_visual_model_last_key = key
            self.external_force_visual_model_last_update = now
        elif visible:
            self.external_force_visual_model_ready = False

    def _delete_external_force_markers(self):
        self._set_external_force_visual_model(0.0, False)
        if self.external_force_marker_pub is not None and GzMarker is not None:
            for marker_id in (1, 2, 3, 4, 5):
                marker = GzMarker()
                marker.ns = "pendulum_external_force"
                marker.id = marker_id
                marker.action = GzMarker.DELETE_MARKER
                self.external_force_marker_pub.publish(marker)
        self.external_force_marker_visible = False

    def _external_force_marker_position(self, force_n, arrow_length):
        sign = 1.0 if force_n >= 0.0 else -1.0
        return (sign * min(arrow_length * 0.08, 0.10), 0.34, 0.78)

    def _set_marker_color(self, marker, red, green, blue, alpha=1.0):
        marker.material.ambient.r = red
        marker.material.ambient.g = green
        marker.material.ambient.b = blue
        marker.material.ambient.a = alpha
        marker.material.diffuse.r = red
        marker.material.diffuse.g = green
        marker.material.diffuse.b = blue
        marker.material.diffuse.a = alpha

    def _publish_external_force_marker(self, force_n):
        if not EXTERNAL_FORCE_GAZEBO_VISUAL_ENABLED:
            return
        if self.gz_marker_node is None:
            return
        now = time.monotonic()
        if abs(force_n) < 1e-6:
            if now < self.external_force_marker_hold_until:
                force_n = self.external_force_marker_last_force_n
            else:
                if self.external_force_marker_visible:
                    self._delete_external_force_markers()
                return
        else:
            self.external_force_marker_last_force_n = force_n
            self.external_force_marker_hold_until = (
                now + self.external_force_visual_hold_s
            )

        if abs(force_n) < 1e-6:
            if self.external_force_marker_visible:
                self._delete_external_force_markers()
            return

        arrow_length = clamp(abs(force_n) * 0.18, 0.28, 0.55)
        sign = 1.0 if force_n >= 0.0 else -1.0
        lifetime_s = int(math.ceil(self.external_force_visual_hold_s))
        marker_x, marker_y, marker_z = self._external_force_marker_position(
            force_n,
            arrow_length,
        )
        self._set_external_force_visual_model(force_n, True)

        if self.external_force_marker_pub is None or GzMarker is None:
            self.external_force_marker_visible = True
            return

        bar = GzMarker()
        bar.ns = "pendulum_external_force"
        bar.id = 4
        bar.action = GzMarker.ADD_MODIFY
        bar.type = GzMarker.BOX
        bar.visibility = GzMarker.ALL
        bar.pose.position.x = marker_x
        bar.pose.position.y = marker_y
        bar.pose.position.z = marker_z
        bar.pose.orientation.w = 1.0 if force_n >= 0.0 else 0.0
        bar.pose.orientation.z = 0.0 if force_n >= 0.0 else 1.0
        bar.scale.x = arrow_length * 0.80
        bar.scale.y = 0.025
        bar.scale.z = 0.055
        bar.lifetime.sec = lifetime_s
        self._set_marker_color(bar, 1.0, 0.58, 0.0)
        self.external_force_marker_pub.publish(bar)

        marker = GzMarker()
        marker.ns = "pendulum_external_force"
        marker.id = 1
        marker.action = GzMarker.ADD_MODIFY
        marker.type = GzMarker.ARROW
        marker.visibility = GzMarker.ALL
        marker.pose.position.x = marker_x
        marker.pose.position.y = marker_y
        marker.pose.position.z = marker_z
        marker.pose.orientation.w = 1.0 if force_n >= 0.0 else 0.0
        marker.pose.orientation.z = 0.0 if force_n >= 0.0 else 1.0
        marker.scale.x = arrow_length
        marker.scale.y = 0.045
        marker.scale.z = 0.045
        marker.lifetime.sec = lifetime_s
        self._set_marker_color(marker, 1.0, 0.18, 0.0)
        self.external_force_marker_pub.publish(marker)

        point = GzMarker()
        point.ns = "pendulum_external_force"
        point.id = 2
        point.action = GzMarker.ADD_MODIFY
        point.type = GzMarker.SPHERE
        point.visibility = GzMarker.ALL
        point.pose.position.x = marker_x + sign * arrow_length * 0.50
        point.pose.position.y = marker_y
        point.pose.position.z = marker_z
        point.pose.orientation.w = 1.0
        point.scale.x = 0.075
        point.scale.y = 0.075
        point.scale.z = 0.075
        point.lifetime.sec = lifetime_s
        self._set_marker_color(point, 1.0, 0.86, 0.0)
        self.external_force_marker_pub.publish(point)

        tail = GzMarker()
        tail.ns = "pendulum_external_force"
        tail.id = 5
        tail.action = GzMarker.ADD_MODIFY
        tail.type = GzMarker.SPHERE
        tail.visibility = GzMarker.ALL
        tail.pose.position.x = marker_x - sign * arrow_length * 0.50
        tail.pose.position.y = marker_y
        tail.pose.position.z = marker_z
        tail.pose.orientation.w = 1.0
        tail.scale.x = 0.04
        tail.scale.y = 0.04
        tail.scale.z = 0.04
        tail.lifetime.sec = lifetime_s
        self._set_marker_color(tail, 1.0, 0.24, 0.0)
        self.external_force_marker_pub.publish(tail)

        label = GzMarker()
        label.ns = "pendulum_external_force"
        label.id = 3
        label.action = GzMarker.ADD_MODIFY
        label.type = GzMarker.TEXT
        label.visibility = GzMarker.ALL
        label.text = f"EXT {force_n:+.1f} N"
        label.pose.position.x = marker_x
        label.pose.position.y = marker_y + 0.03
        label.pose.position.z = marker_z + 0.11
        label.pose.orientation.w = 1.0
        label.scale.x = 0.10
        label.scale.y = 0.10
        label.scale.z = 0.10
        label.lifetime.sec = lifetime_s
        self._set_marker_color(label, 1.0, 0.92, 0.0)
        self.external_force_marker_pub.publish(label)
        self.external_force_marker_visible = True

    def _serial_reader(self):
        buffer = bytearray()
        while self.serial_running:
            try:
                readable, _, _ = select.select([self.master_fd], [], [], 0.05)
                if not readable:
                    continue
                chunk = os.read(self.master_fd, 256)
                if not chunk:
                    continue
                buffer.extend(chunk)
                self._parse_serial_buffer(buffer)
            except OSError as exc:
                if exc.errno not in (errno.EIO, errno.EBADF):
                    self.get_logger().warning(f"Serial reader error: {exc}")
                time.sleep(0.05)
            except Exception as exc:
                self.get_logger().warning(f"Serial parse error: {exc}")
                time.sleep(0.05)

    def _parse_serial_buffer(self, buffer):
        while len(buffer) >= 6:
            start = buffer.find(HEADER_TX)
            if start < 0:
                buffer.clear()
                return
            if start > 0:
                del buffer[:start]
            if len(buffer) < 4:
                return

            packet_type = buffer[2]
            if packet_type == 0x01:
                packet_len = 16
            elif packet_type == 0x02:
                packet_len = 26
            elif packet_type == 0x03:
                packet_len = 6
            elif packet_type == 0x04:
                packet_len = 10
            else:
                del buffer[:2]
                continue

            if len(buffer) < packet_len:
                return

            packet = bytes(buffer[:packet_len])
            del buffer[:packet_len]

            crc_recv = packet[-2] | (packet[-1] << 8)
            crc_calc = sum(packet[2:-2]) & 0xFFFF
            if crc_recv != crc_calc:
                self.get_logger().debug("Ignoring packet with bad checksum")
                continue

            if packet_type == 0x01:
                self._handle_joystick_packet(packet)
            elif packet_type == 0x02:
                self._handle_gains_packet(packet)
            elif packet_type == 0x03:
                self._handle_reset_packet()
            elif packet_type == 0x04:
                self._handle_external_force_packet(packet)

    def _handle_joystick_packet(self, packet):
        _, _, ax, _ay, _rx, _ry, buttons = struct.unpack("<BBhhhhH", packet[2:-2])
        manual_cmd = clamp(ax / 32767.0, -1.0, 1.0) * self.manual_velocity
        rising = buttons & ~self.last_buttons
        self.last_buttons = buttons

        with self.lock:
            self.manual_cmd_mps = manual_cmd
            if rising & BUTTON_B:
                self._set_mode_locked(MODE_FINISH)
            elif rising & BUTTON_Y:
                if self.mode == MODE_READY:
                    self._set_mode_locked(MODE_SINE)
                else:
                    self._set_mode_locked(MODE_HOMING)
            elif rising & BUTTON_X:
                self._set_mode_locked(MODE_SWING_UP)
            elif rising & BUTTON_A:
                theta_top = self._theta_top_locked()
                if self.mode == MODE_SWING_UP:
                    self.balance_request_pending = True
                elif (
                    self._ready_for_balance_locked(theta_top)
                    or self._upright_immediate_capture_ready_locked(theta_top)
                ):
                    self._set_mode_locked(MODE_BALANCE)
                else:
                    self.balance_request_pending = True

    def _handle_gains_packet(self, packet):
        gains = struct.unpack("<fffff", packet[4:-2])
        with self.lock:
            self.gains["K_TH"] = gains[0]
            self.gains["K_TH_D"] = gains[1]
            self.gains["K_X"] = gains[2]
            self.gains["K_X_D"] = gains[3]
            self.gains["K_X_INT"] = gains[4]
            self.x_integral_cm_s = 0.0
        self._write_ack(gains)
        self.get_logger().info(
            "Gains updated: "
            f"K_TH={gains[0]:.2f}, K_TH_D={gains[1]:.2f}, "
            f"K_X={gains[2]:.2f}, K_X_D={gains[3]:.2f}, K_X_INT={gains[4]:.2f}"
        )

    def _handle_external_force_packet(self, packet):
        _typ, _seq, force_n = struct.unpack("<BBf", packet[2:-2])
        torque_nm = force_n * self.pendulum_length
        with self.lock:
            self.external_disturbance_torque_nm = torque_nm
        self.get_logger().info(
            f"External disturbance set: force={force_n:.3f} N, torque={torque_nm:.3f} Nm"
        )

    def _handle_reset_packet(self):
        with self.lock:
            self._set_mode_locked(MODE_WAITING)
            self.manual_cmd_mps = 0.0
            self.cmd_mps = 0.0
            self.motor_target_mps = 0.0
            self.motor_velocity_mps = 0.0
            self.prev_motor_velocity_error = 0.0
            self.motor_pwm = 0.0
            self.external_disturbance_torque_nm = 0.0
            self.cart_effort_cmd_n = 0.0
            self.last_buttons = 0
            self.x_center_m = 0.0
            self.x_integral_cm_s = 0.0
            self.swing_force_cmd_n = 0.0
            self.balance_request_pending = False
        self._write_reset_ack(1)

    def _set_mode_locked(self, mode):
        self.mode = mode
        self.mode_started = time.monotonic()
        self.velocity_error_integral = 0.0
        self.prev_velocity_error = 0.0
        self.prev_motor_velocity_error = 0.0
        self.x_integral_cm_s = 0.0
        self.swing_force_cmd_n = 0.0

        if mode == MODE_SWING_UP:
            self.x_center_m = 0.0
            self.swing_state = "CALC"
            self.swing_target_v = 0.0
            self.swing_direction = 1.0
            self.swing_state_started = time.monotonic()
            self.energy_now = 0.0
            self.energy_target = 2.0 * self.pendulum_mass * 9.81 * self.pendulum_com
            self.swing_top_passes = 0
            self.swing_top_pass_armed = True
            self.balance_request_pending = False
        elif mode == MODE_BALANCE:
            self.x_center_m = clamp(
                self.cart_x_m,
                -self.rail_limit + 0.02,
                self.rail_limit - 0.02,
            )
            self.balance_request_pending = False
        self.upright_stable_since = None

    def _control_and_status_timer(self):
        now = time.monotonic()
        with self.lock:
            dt = max(1e-3, now - self.last_control_time)
            self.last_control_time = now
            theta_top = self._theta_top_locked()
            command, effort_override = self._compute_control_locked(theta_top, dt, now)
            command = self._apply_rail_limits_locked(command, dt)
            self.cmd_mps = command
            if effort_override is None:
                effort = self._velocity_to_effort_locked(command, dt)
            else:
                effort = self._apply_force_limits_locked(effort_override)
                self.velocity_error_integral = 0.0
                self.prev_velocity_error = 0.0
            effort = self._smooth_cart_effort_locked(effort, dt)

            hinge_effort = (
                self._balance_assist_torque_locked(theta_top)
                + self.external_disturbance_torque_nm
            )
            external_force_n = (
                self.external_disturbance_torque_nm / self.pendulum_length
                if self.pendulum_length > 1e-6
                else 0.0
            )
            state = self._make_state_tuple_locked(theta_top)

        msg = Float64()
        msg.data = float(command)
        self.cmd_pub.publish(msg)

        effort_msg = Float64()
        effort_msg.data = float(effort)
        self.effort_pub.publish(effort_msg)

        hinge_effort_msg = Float64()
        hinge_effort_msg.data = float(hinge_effort)
        self.hinge_effort_pub.publish(hinge_effort_msg)
        self._publish_external_force_marker(external_force_n)

        state_msg = Float64MultiArray()
        state_msg.data = list(state[1:])
        self.state_pub.publish(state_msg)

        self._write_status_packet(state)

    def _theta_top_locked(self):
        return wrap_pi(self.pendulum_down_rad - self.upright_joint_position_rad)

    def _compute_control_locked(self, theta_top, dt, now):
        if self.mode == MODE_WAITING:
            return 0.0, 0.0

        if self.mode == MODE_FINISH:
            return 0.0, 0.0

        if self.mode == MODE_HOMING:
            error = self.x_center_m - self.cart_x_m
            command = 2.8 * error - 0.35 * self.cart_v_mps
            if abs(error) < 0.006 and abs(self.cart_v_mps) < 0.03:
                self._set_mode_locked(MODE_READY)
                return 0.0, None
            return command, None

        if self.mode == MODE_READY:
            return self.manual_cmd_mps, None

        if self.mode == MODE_SINE:
            elapsed = now - self.mode_started
            target = 0.30 * math.sin(2.0 * math.pi * 0.25 * elapsed)
            return 2.5 * (target - self.cart_x_m) - 0.20 * self.cart_v_mps, None

        if self.mode == MODE_SWING_UP:
            return self._swing_up_energy_command_locked(theta_top, dt, now)

        if self.mode == MODE_BALANCE:
            if self._lost_balance_locked(theta_top):
                self._set_mode_locked(MODE_SWING_UP)
                return 0.0, None

            if (
                abs(theta_top) < math.radians(8.0)
                and abs(self.pendulum_vel_radps) < 1.4
                and abs(self.cart_v_mps) < 0.6
            ):
                self.x_center_m += (
                    self.balance_center_reference_m - self.x_center_m
                ) * clamp(
                    0.35 * dt,
                    0.0,
                    1.0,
                )

            if self.balance_use_lqr and self.balance_lqr_gain is not None:
                force = self._balance_force_locked(theta_top, dt)
                return self._force_to_command_hint(force), force

            force = self._balance_force_locked(theta_top, dt)
            return self._force_to_command_hint(force), force

        return 0.0, None

    def _apply_rail_limits_locked(self, command, dt):
        command = clamp(command, -self.velocity_limit, self.velocity_limit)
        max_delta = self.command_accel_limit * dt
        command = clamp(command, self.cmd_mps - max_delta, self.cmd_mps + max_delta)
        guard = 0.005
        if self.cart_x_m <= -self.rail_limit + guard and command < 0.0:
            return 0.0
        if self.cart_x_m >= self.rail_limit - guard and command > 0.0:
            return 0.0
        return command

    def _apply_force_limits_locked(self, force):
        force = clamp(force, -self.effort_limit_n, self.effort_limit_n)
        guard = 0.01
        if self.cart_x_m <= -self.rail_limit + guard and force < 0.0:
            return 0.0
        if self.cart_x_m >= self.rail_limit - guard and force > 0.0:
            return 0.0
        return force

    def _velocity_to_effort_locked(self, command, dt):
        command_cmps = clamp(
            command * 100.0,
            -self.velocity_limit * 100.0,
            self.velocity_limit * 100.0,
        )

        if abs(command_cmps) < 0.5:
            self.motor_pwm = 0.0
            target_mps = 0.0
        else:
            self.motor_pwm = (
                self.motor_pwm_deadband
                + self.motor_pwm_per_cmps * abs(command_cmps)
            )
            target_cmps = (
                self.motor_pwm - self.motor_pwm_deadband
            ) / max(self.motor_pwm_per_cmps, 1e-6)
            target_mps = math.copysign(target_cmps / 100.0, command_cmps)

        alpha = 1.0 - math.exp(-dt / max(self.motor_time_constant_s, 1e-3))
        self.motor_target_mps = target_mps
        self.motor_velocity_mps += alpha * (target_mps - self.motor_velocity_mps)

        velocity_error = self.motor_velocity_mps - self.cart_v_mps
        velocity_error_dot = (
            velocity_error - self.prev_motor_velocity_error
        ) / max(dt, 1e-3)
        self.prev_motor_velocity_error = velocity_error

        effort = (
            self.motor_velocity_servo_p * velocity_error
            + self.motor_velocity_servo_d * velocity_error_dot
        )
        return self._apply_force_limits_locked(effort)

    def _ready_for_balance_locked(self, theta_top):
        capture_cart_pos = min(
            self.balance_capture_cart_pos,
            max(0.05, self.rail_limit - 0.004),
        )
        return (
            abs(theta_top) < self.balance_capture
            and abs(self.pendulum_vel_radps) < self.balance_capture_rate
            and abs(self.cart_x_m) < capture_cart_pos
            and abs(self.cart_v_mps) < self.balance_capture_cart_vel
        )

    def _upright_auto_lock_ready_locked(self, theta_top, now):
        if not self.balance_auto_lock_enabled:
            self.upright_stable_since = None
            return False

        stable = (
            abs(theta_top) < self.balance_auto_lock_angle
            and abs(self.pendulum_vel_radps) < self.balance_auto_lock_rate
            and abs(self.cart_x_m) < self.balance_auto_lock_cart_pos
            and abs(self.cart_v_mps) < self.balance_auto_lock_cart_vel
        )
        if stable:
            if self.upright_stable_since is None:
                self.upright_stable_since = now
            return (now - self.upright_stable_since) >= self.balance_auto_lock_time_s

        self.upright_stable_since = None
        return False

    def _upright_immediate_capture_ready_locked(self, theta_top):
        return (
            abs(theta_top) < self.balance_immediate_capture_angle
            and abs(self.pendulum_vel_radps) < self.balance_immediate_capture_rate
            and abs(self.cart_x_m) < self.balance_immediate_capture_cart_pos
            and abs(self.cart_v_mps) < self.balance_immediate_capture_cart_vel
        )

    def _update_swing_top_passes_locked(self, theta_top):
        if abs(theta_top) > self.swing_top_pass_angle:
            self.swing_top_pass_armed = True
            return
        if self.swing_top_pass_armed:
            self.swing_top_passes += 1
            self.swing_top_pass_armed = False

    def _swing_energy_ready_locked(self, energy, target_energy):
        enough_passes = self.swing_top_passes >= self.swing_min_top_passes_before_catch
        enough_time = (
            time.monotonic() - self.mode_started
        ) >= self.swing_min_energy_build_time_s
        if not (enough_passes or enough_time):
            return False
        if target_energy <= 1e-6:
            return True
        return energy / target_energy >= self.swing_energy_ready_ratio

    def _lost_balance_locked(self, theta_top):
        return (
            abs(theta_top) > self.balance_give_up
            or abs(self.cart_x_m) > (self.rail_limit + 0.02)
        )

    def _in_catch_region_locked(self, theta_top):
        return (
            abs(theta_top) < self.catch_region_angle
            and abs(self.pendulum_vel_radps) < self.catch_region_rate
            and abs(self.cart_x_m) < (self.rail_limit - 0.004)
        )

    def _swing_centering_force_locked(self, theta_top):
        if (
            abs(self.cart_x_m) < 0.16
            or abs(theta_top) < math.radians(135.0)
            or abs(self.pendulum_vel_radps) > 1.2
        ):
            return None

        force = -150.0 * self.cart_x_m - 45.0 * self.cart_v_mps
        return clamp(force, -self.swing_force_limit_n, self.swing_force_limit_n)

    def _smooth_swing_force_locked(self, force, dt):
        max_delta = max(1.0, self.swing_force_rate_limit_nps) * max(dt, 1e-3)
        force = clamp(
            force,
            self.swing_force_cmd_n - max_delta,
            self.swing_force_cmd_n + max_delta,
        )
        self.swing_force_cmd_n = force
        return force

    def _smooth_cart_effort_locked(self, force, dt):
        max_delta = max(1.0, self.cart_effort_rate_limit_nps) * max(dt, 1e-3)
        force = clamp(
            force,
            self.cart_effort_cmd_n - max_delta,
            self.cart_effort_cmd_n + max_delta,
        )
        force = self._apply_force_limits_locked(force)
        self.cart_effort_cmd_n = force
        return force

    def _rail_aware_swing_force_locked(self, force):
        soft_limit = max(0.08, self.rail_limit - 0.24)
        distance_past_soft_limit = abs(self.cart_x_m) - soft_limit
        if distance_past_soft_limit <= 0.0:
            return force

        rail_direction = math.copysign(1.0, self.cart_x_m)
        outward_velocity = max(0.0, self.cart_v_mps * rail_direction)
        inward_force = 45.0 + 320.0 * distance_past_soft_limit + 60.0 * outward_velocity
        if force * rail_direction > 0.0:
            return -rail_direction * inward_force
        return force - rail_direction * 0.35 * inward_force

    def _gui_balance_command_locked(self, theta_top, dt):
        x_error_m = self.cart_x_m - self.x_center_m
        self.x_integral_cm_s = clamp(
            self.x_integral_cm_s + x_error_m * dt,
            -0.08,
            0.08,
        )

        command = (
            self.gains["K_TH"] * theta_top
            + self.gains["K_TH_D"] * self.pendulum_vel_radps
            - self.gains["K_X"] * x_error_m
            - self.gains["K_X_D"] * self.cart_v_mps
            - self.gains["K_X_INT"] * self.x_integral_cm_s
        )
        command = self._rail_aware_balance_command_locked(command)
        return clamp(command, -self.velocity_limit, self.velocity_limit)

    def _rail_aware_balance_command_locked(self, command):
        soft_limit = max(0.12, self.rail_limit - 0.13)
        distance_past_soft_limit = abs(self.cart_x_m) - soft_limit
        if distance_past_soft_limit <= 0.0:
            return command

        rail_direction = math.copysign(1.0, self.cart_x_m)
        outward_velocity = max(0.0, self.cart_v_mps * rail_direction)
        inward_command = min(
            self.velocity_limit,
            0.45 + 4.5 * distance_past_soft_limit + 0.9 * outward_velocity,
        )
        if command * rail_direction > 0.0:
            return -rail_direction * inward_command
        return command - rail_direction * 0.35 * inward_command

    def _gui_force_feedback_locked(self, theta_top, dt, force_limit_n):
        x_error_m = self.cart_x_m - self.x_center_m
        self.x_integral_cm_s = clamp(
            self.x_integral_cm_s + x_error_m * dt,
            -0.08,
            0.08,
        )

        force = (
            14.0 * self.gains["K_TH"] * theta_top
            + 35.0 * self.gains["K_TH_D"] * self.pendulum_vel_radps
            - 20.0 * self.gains["K_X"] * x_error_m
            - 8.0 * self.gains["K_X_D"] * self.cart_v_mps
            - 12.0 * self.gains["K_X_INT"] * self.x_integral_cm_s
        )
        force = self._soft_rail_guard_force_locked(force, force_limit_n)
        return clamp(force, -force_limit_n, force_limit_n)

    def _balance_force_locked(self, theta_top, dt):
        return self._upright_pid_force_locked(theta_top, dt, self.balance_force_limit_n)

    def _catch_force_locked(self, theta_top, dt):
        return self._upright_pid_force_locked(theta_top, dt, self.catch_force_limit_n)

    def _catch_command_locked(self, theta_top):
        x_error_m = self.cart_x_m - self.x_center_m
        return clamp(
            4.8 * theta_top
            + 1.1 * self.pendulum_vel_radps
            - 5.0 * x_error_m
            - 1.8 * self.cart_v_mps,
            -self.velocity_limit,
            self.velocity_limit,
        )

    def _state_feedback_force_locked(self, gain, theta_top, dt, force_limit_n):
        x_error_m = self.cart_x_m - self.x_center_m
        self.x_integral_cm_s = clamp(
            self.x_integral_cm_s + x_error_m * dt,
            -0.20,
            0.20,
        )

        state = (
            x_error_m,
            self.cart_v_mps,
            theta_top,
            self.pendulum_vel_radps,
        )
        force = -sum(gain_value * state_value for gain_value, state_value in zip(gain, state))

        force += self.balance_centering_force_gain * x_error_m
        force += self.balance_cart_damping_force_gain * self.cart_v_mps
        force += self.balance_integral_force_gain * self.x_integral_cm_s
        force = self._soft_rail_guard_force_locked(force, force_limit_n)
        return clamp(force, -force_limit_n, force_limit_n)

    def _upright_pid_force_locked(self, theta_top, dt, force_limit_n):
        x_error_m = self.cart_x_m - self.x_center_m
        self.x_integral_cm_s = clamp(
            self.x_integral_cm_s + x_error_m * dt,
            -0.08,
            0.08,
        )
        center_scale = clamp(
            1.0 - abs(theta_top) / max(self.balance_fallback, 1e-3),
            0.0,
            1.0,
        )
        theta_center_bias = center_scale * clamp(
            self.balance_center_theta_bias_gain * x_error_m
            + self.balance_center_theta_bias_rate_gain * self.cart_v_mps,
            -self.balance_center_theta_bias_limit,
            self.balance_center_theta_bias_limit,
        )
        theta_for_control = theta_top + theta_center_bias

        theta_force_gain = abs(self.gains["K_TH"]) * (self.balance_theta_kp / 8.8)
        theta_rate_force_gain = abs(self.gains["K_TH_D"]) * (
            self.balance_theta_kd / 1.8
        )
        centering_force_gain = abs(self.gains["K_X"]) * (
            self.balance_centering_force_gain / 4.0
        )
        cart_damping_force_gain = abs(self.gains["K_X_D"]) * (
            self.balance_cart_damping_force_gain / 2.0
        )
        integral_force_gain = abs(self.gains["K_X_INT"]) * (
            self.balance_integral_force_gain / 0.4
        )

        force = (
            theta_force_gain * theta_for_control
            + theta_rate_force_gain * self.pendulum_vel_radps
            + center_scale * centering_force_gain * x_error_m
            + center_scale * cart_damping_force_gain * self.cart_v_mps
            + integral_force_gain * self.x_integral_cm_s
        )
        force = self._soft_rail_guard_force_locked(force, force_limit_n)
        return clamp(force, -force_limit_n, force_limit_n)

    def _soft_rail_guard_force_locked(self, force, force_limit_n):
        soft_limit = max(0.12, self.rail_limit - 0.08)
        distance_past_soft_limit = abs(self.cart_x_m) - soft_limit
        if distance_past_soft_limit <= 0.0:
            return force

        rail_direction = math.copysign(1.0, self.cart_x_m)
        outward_velocity = max(0.0, self.cart_v_mps * rail_direction)
        inward_force = 140.0 * distance_past_soft_limit + 22.0 * outward_velocity
        force -= rail_direction * inward_force

        if force * rail_direction > 0.0:
            force = -rail_direction * min(force_limit_n, max(20.0, inward_force))
        return clamp(force, -force_limit_n, force_limit_n)

    def _balance_assist_torque_locked(self, theta_top):
        if not self.balance_assist_enabled:
            return 0.0
        if self.mode not in (MODE_SWING_UP, MODE_BALANCE):
            return 0.0
        if abs(theta_top) > self.balance_assist_angle:
            return 0.0

        torque = (
            -self.balance_assist_kp * theta_top
            - self.balance_assist_kd * self.pendulum_vel_radps
        )
        return clamp(
            torque,
            -self.balance_assist_torque_limit,
            self.balance_assist_torque_limit,
        )

    def _force_to_command_hint(self, force):
        return clamp(
            force * self.force_to_velocity_gain,
            -self.velocity_limit,
            self.velocity_limit,
        )

    def _build_lqr_gain(self, q_x, q_x_dot, q_theta, q_theta_dot, r_value):
        if np is None or solve_continuous_are is None:
            self.get_logger().warning(
                "SciPy is not available. Falling back to baked upright gains."
            )
            if q_x >= 60.0:
                return (
                    -63.24555320,
                    -84.46219173,
                    -508.83350816,
                    -140.45686895,
                )
            return (
                -31.62277660,
                -53.75703050,
                -415.43999370,
                -134.07364826,
            )

        gravity = 9.81
        cart_mass = max(self.cart_mass, 1e-3)
        pendulum_mass = max(self.pendulum_mass, 1e-3)
        pendulum_com = max(self.pendulum_com, 1e-3)
        a_matrix = np.array(
            [
                [0.0, 1.0, 0.0, 0.0],
                [0.0, 0.0, -pendulum_mass * gravity / cart_mass, 0.0],
                [0.0, 0.0, 0.0, 1.0],
                [
                    0.0,
                    0.0,
                    (cart_mass + pendulum_mass) * gravity
                    / (cart_mass * pendulum_com),
                    0.0,
                ],
            ],
            dtype=float,
        )
        b_matrix = np.array(
            [
                [0.0],
                [1.0 / cart_mass],
                [0.0],
                [-1.0 / (cart_mass * pendulum_com)],
            ],
            dtype=float,
        )
        q_matrix = np.diag([q_x, q_x_dot, q_theta, q_theta_dot])
        r_matrix = np.array([[max(r_value, 1e-4)]], dtype=float)
        p_matrix = solve_continuous_are(a_matrix, b_matrix, q_matrix, r_matrix)
        k_matrix = np.linalg.inv(r_matrix) @ b_matrix.T @ p_matrix
        return tuple(float(value) for value in k_matrix.reshape(-1))

    def _energy(self, theta_top):
        potential = self.pendulum_mass * 9.81 * self.pendulum_com * (
            1.0 + math.cos(theta_top)
        )
        kinetic = (
            0.5
            * self.pendulum_mass
            * self.pendulum_com
            * self.pendulum_com
            * self.pendulum_vel_radps
            * self.pendulum_vel_radps
        )
        return potential + kinetic

    def _make_state_tuple_locked(self, theta_top):
        logtick = int((time.monotonic() - self.start_time) * 1000.0) & 0xFFFFFFFF
        degree = math.degrees(theta_top)
        cm_x = self.cart_x_m * 100.0
        set_speed = self.cmd_mps * 100.0
        energy = self._energy(theta_top)
        theta_dot_rad = self.pendulum_vel_radps
        theta_rad = theta_top
        x_center_cm = self.x_center_m * 100.0
        mode = float(self.mode)
        return (
            logtick,
            degree,
            cm_x,
            set_speed,
            energy,
            theta_dot_rad,
            theta_rad,
            x_center_cm,
            mode,
        )

    def _write_ack(self, gains):
        payload = struct.pack("<fffff", *gains)
        crc = sum(payload) & 0xFFFF
        self._write_serial(HEADER_ACK + payload + struct.pack("<H", crc))

    def _write_reset_ack(self, status):
        payload = struct.pack("<B", int(status) & 0xFF)
        crc = sum(payload) & 0xFFFF
        self._write_serial(HEADER_RESET + payload + struct.pack("<H", crc))

    def _write_status_packet(self, state):
        payload = struct.pack("<Idddddddd", *state)
        packet_without_crc = HEADER_STATUS + b"\x00\x00" + payload
        crc = sum(packet_without_crc[4:]) & 0xFFFF
        self._write_serial(packet_without_crc + struct.pack("<H", crc))

    def _write_serial(self, packet):
        if self.master_fd is None:
            return
        try:
            os.write(self.master_fd, packet)
        except OSError as exc:
            if exc.errno not in (errno.EIO, errno.EBADF):
                self.get_logger().debug(f"Pseudo serial write skipped: {exc}")


def main(args=None):
    rclpy.init(args=args)
    node = SimSerialBridge()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException, RuntimeError):
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
