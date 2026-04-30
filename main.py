import os
os.environ["SDL_JOYSTICK_ALLOW_BACKGROUND_EVENTS"] = "1"

import sys
import threading
import pygame
import math
from threading import Lock
from serial.tools import list_ports

from lib_stick import init_joystick, joystick_sender
from lib_com import open_serial,scan_ports,read_control_status, send_gains, send_reset, make_packet
from lib_data import DataLogger
from lib_udp import UDPBroadcaster

from lib_gui import PendulumGUI

# ============================================================
# KONFIGURASI
# ============================================================
PORT = os.environ.get("PENDULUM_PORT", "COM4")
BAUD = int(os.environ.get("PENDULUM_BAUD", "115200"))
NO_JOYSTICK = os.environ.get("PENDULUM_NO_JOYSTICK", "0") == "1"
IS_REAL_SIM_PORT = PORT.startswith("/tmp/pendulum_real_serial")
IS_PID_SIM_PORT = PORT.startswith("/tmp/pendulum_pid_serial")
IS_SIM_PORT = (
	PORT.startswith("/tmp/pendulum_sim_serial")
	or IS_REAL_SIM_PORT
	or IS_PID_SIM_PORT
	or os.environ.get("PENDULUM_SIM", "0") == "1"
)
FPS = 50

if IS_REAL_SIM_PORT or IS_PID_SIM_PORT:
	DEFAULT_GAINS = {
		"K_TH": 10.0,
		"K_TH_D": 3.0,
		"K_X": 2.4,
		"K_X_D": 3.4,
		"K_X_INT": 0.08
	}
elif IS_SIM_PORT:
	DEFAULT_GAINS = {
		"K_TH": 8.8,
		"K_TH_D": 1.8,
		"K_X": 4.0,
		"K_X_D": 2.0,
		"K_X_INT": 0.4
	}
else:
	DEFAULT_GAINS = {
		"K_TH": -2.50 * 57.0 * 12.0,
		"K_TH_D": -0.030 * 57.0 * 18.0,
		"K_X": 3.0,
		"K_X_D": -1.6 * 2.0,
		"K_X_INT": 0.0
	}

X_MIN_CM = -40
X_MAX_CM = 40.0
X_CENTER_CM = 0

# ============================================================
# SHARED STATE
# ============================================================
pendulum_state = {
	"cmX": X_CENTER_CM,
	"theta": 0.0,
	"x_center": 40.0,
	"running": False,
	"gains_ack": False,
	"gains_ack_values": None,
	"reset_ack": False
}
state_lock = Lock()

current_gains = DEFAULT_GAINS.copy()
gains_lock = Lock()

# ============================================================
# APP CLASS
# ============================================================
class PendulumMonitor:
	def __init__(self):
		pygame.init()
		info = pygame.display.Info()
		self.screen = pygame.display.set_mode(
			(info.current_w, info.current_h),
			pygame.FULLSCREEN | pygame.SCALED
		)
		self.WINDOW_WIDTH = info.current_w
		self.WINDOW_HEIGHT = info.current_h
		self.PANEL_WIDTH = self.WINDOW_WIDTH / 5.0
		self.MAIN_WIDTH = self.WINDOW_WIDTH - self.PANEL_WIDTH

		pygame.display.set_caption("Pendulum Monitor - Live Tuning + UDP")

		self.clock = pygame.time.Clock()
		self.font_large = pygame.font.SysFont("Arial", 22, bold=True)
		self.font_medium = pygame.font.SysFont("Arial", 18)
		self.font_small = pygame.font.SysFont("Arial", 16)
		self.font_input = pygame.font.SysFont("Consolas", 16)

		self.data_logger = DataLogger(base_dir="logs")
		self.udp_broadcaster = UDPBroadcaster(broadcast_ip="192.168.1.255", port=5000)

		self.gui = PendulumGUI(
			screen=self.screen,
			window_w=self.WINDOW_WIDTH,
			window_h=self.WINDOW_HEIGHT,
			main_w=self.MAIN_WIDTH,
			panel_w=self.PANEL_WIDTH,
			fonts=(self.font_large, self.font_medium, self.font_small, self.font_input),
			x_min_cm=X_MIN_CM,
			x_max_cm=X_MAX_CM,
			state_ref=pendulum_state,
			state_lock=state_lock
		)
		self.gui.set_gains_defaults(DEFAULT_GAINS)

		self.serial = None
		self.joystick = None
		self.thread_rx = None
		self.thread_tx = None
		self.command_seq = 0

		self.running = True
		self.gains_sent = False
		self.gains_ack_time = 0.0

		# graph buffers
		self.t_raw = []
		self.cmX_hist = []
		self.degree_hist = []
		self.degree0_hist = []
		self.setspeed_hist = []
		self.r1_hist = []
		self.theta_dot_hist = []
		self.x_center_hist = []
		self.x_mode_hist = []
		self.max_hist = 1000
		self.ctx = {
				"is_running": 0,
				"gains_sent": self.gains_sent,
				"gains_ack": False,
				"reset_ack": False,
				"cmX": 0.0,
				"theta": 0.0,
			}
		self.mode = 0
		
		# serial port management
		# self.avaible_ports = []
		# self.selected_port = None
		# self.port_dropdown_open = False
		# self.serial_connected = False

		# Auto Detect Saat Startup
		# self.scan_serial_ports(auto_select=True)
	
	def scan_serial_ports(self, auto_select=False):
		port = list_ports.comports()
		#simpan daftar port yang tersedia
		self.avaible_ports = [p.device for p in port]
		#jika auto_select diaktifkan, pilih port pertama yang tersedia
		if auto_select and self.selected_port is None:
			for p in port:
				desc = p.description.lower()
				if "STMicroelectronics" in desc :
					self.selected_port = p.device
					return

	def setup_serial(self):
		try:
			self.serial = open_serial(PORT, BAUD)
			print(f"Serial opened: {PORT} @ {BAUD}")

			if NO_JOYSTICK:
				print("Joystick disabled by PENDULUM_NO_JOYSTICK=1")
			else:
				self.joystick = init_joystick(0)

				self.thread_tx = threading.Thread(
					target=joystick_sender,
					args=(self.joystick, self.serial, FPS),
					daemon=True
				)
				self.thread_tx.start()

			self.thread_rx = threading.Thread(
				target=read_control_status,
				args=(self.serial,),
				kwargs={
					"callback": self.on_control_status,
					"ack_callback": self.on_gains_ack,
					"reset_ack_callback": self.on_reset_ack,
					"debug": False
				},
				daemon=True
			)
			self.thread_rx.start()

			return True
		except Exception as e:
			print(f"Failed to open serial: {e}")
			return False
		
	def start_graph(self):
		self.graph_enabled = True
		self.t_raw.clear()
		self.cmX_hist.clear()
		self.degree_hist.clear()
		self.degree0_hist.clear()
		self.setspeed_hist.clear()
		self.r1_hist.clear()
		self.theta_dot_hist.clear()
		self.x_center_hist.clear()
		self.x_mode_hist.clear()
		

	def stop_graph(self):
		self.graph_enabled = False
		self.t_raw.clear()
		self.cmX_hist.clear()
		self.degree_hist.clear()
		self.degree0_hist.clear()
		self.setspeed_hist.clear()
		self.r1_hist.clear()
		self.theta_dot_hist.clear()
		self.x_center_hist.clear()
		self.x_mode_hist.clear()

	def on_control_status(self, sample_tuple):
		logtick, degree, cmX, setspeed, r1, theta_dot, theta, x_center, mode = sample_tuple
		self.mode = mode
		gv = getattr(self.gui, "graph_view", None)
		with state_lock:
			pendulum_state["cmX"] = cmX
			pendulum_state["theta"] = theta
			pendulum_state["x_center"] = x_center
		
		#if self.ctx["is_running"] == 0:
			#cut = len(self.t_raw)
			#del self.t_raw[:cut]
			#del self.cmX_hist[:cut]
			#del self.degree_hist[:cut]
			#self.t_raw.clear()
			#self.cmX_hist.clear()
			#self.degree_hist.clear()
		# graph history
		degree0=0	
		if(gv.running):
			self.t_raw.append(float(logtick))
			self.cmX_hist.append(float(cmX))
			self.degree_hist.append(float(degree))
			self.setspeed_hist.append(float(setspeed))
			self.r1_hist.append(float(r1))
			self.theta_dot_hist.append(float(theta_dot))
			self.x_center_hist.append(float(x_center))
			self.x_mode_hist.append(float(mode))
			
			degree0=float(degree+180)	
			if(degree0>180):
				degree0=degree0-360
			else:degree0=degree0	
			
			self.degree0_hist.append(float(degree0))
		else:
			self.t_raw.clear()
			self.cmX_hist.clear()
			self.degree_hist.clear()
			self.degree0_hist.clear()
			self.setspeed_hist.clear()
			self.r1_hist.clear()
			self.theta_dot_hist.clear()
			self.x_center_hist.clear()
			self.x_mode_hist.clear()
		if len(self.t_raw) > self.max_hist:
			
			n = len(self.t_raw)
			print(n)
			if n <= self.max_hist:
				return
			cut = n - self.max_hist
			print(cut)
			
			del self.t_raw[:cut]
			del self.cmX_hist[:cut]
			del self.degree_hist[:cut]
			del self.degree0_hist[:cut]
			del self.setspeed_hist[:cut]
			del self.r1_hist[:cut]
			del self.theta_dot_hist[:cut]
			del self.x_center_hist[:cut]
			del self.x_mode_hist[:cut]
			aftercut = len(self.t_raw)
			print(aftercut)
			#self.t_raw = self.t_raw[-self.max_hist:]
			#self.cmX_hist = self.cmX_hist[-self.max_hist:]
			#self.degree_hist = self.degree_hist[-self.max_hist:]
			#self.degree0_hist = self.degree0_hist[-self.max_hist:]	
			
			if gv is not None:
				gv._src_last_n = max(0, gv._src_last_n - cut)
		self.data_logger.handle_sample(sample_tuple)
		self.udp_broadcaster.send_control_status(sample_tuple)

	def on_gains_ack(self, gains_tuple):
		K_TH, K_TH_D, K_X, K_X_D, K_X_INT = gains_tuple

		with state_lock:
			pendulum_state["gains_ack"] = True
			pendulum_state["gains_ack_values"] = gains_tuple

		import time
		self.gains_ack_time = time.time()
		print(f"[ACK] Gains confirmed: K_TH={K_TH:.1f}, K_X={K_X:.2f}")

	def on_reset_ack(self, status):
		with state_lock:
			pendulum_state["reset_ack"] = True
			pendulum_state["running"] = False
		print(f"[ACK] Reset confirmed by STM32 (status={status})")

	def apply_gains(self):
		with state_lock:
			if pendulum_state["running"]:
				print("Cannot apply gains while running! Stop first.")
				return

		with gains_lock:
			current_gains["K_TH"] = self.gui.inputs["K_TH"].get_float(DEFAULT_GAINS["K_TH"])
			current_gains["K_TH_D"] = self.gui.inputs["K_TH_D"].get_float(DEFAULT_GAINS["K_TH_D"])
			current_gains["K_X"] = self.gui.inputs["K_X"].get_float(DEFAULT_GAINS["K_X"])
			current_gains["K_X_D"] = self.gui.inputs["K_X_D"].get_float(DEFAULT_GAINS["K_X_D"])
			current_gains["K_X_INT"] = self.gui.inputs["K_X_INT"].get_float(DEFAULT_GAINS["K_X_INT"])

		if self.serial:
			for attempt in range(3):
				send_gains(
					self.serial,
					current_gains["K_TH"],
					current_gains["K_TH_D"],
					current_gains["K_X"],
					current_gains["K_X_D"],
					current_gains["K_X_INT"],
					seq=attempt
				)
				import time
				time.sleep(0.05)

			self.gains_sent = True
			with state_lock:
				pendulum_state["gains_ack"] = False
			print("Gains sent (3 packets for reliability)")

	def start_system(self):
		if not self.gains_sent:
			print("Please apply gains first!")
			return

		with state_lock:
			pendulum_state["running"] = True

		self.gui.set_running_ui_lock(True)
		print("System STARTED")

	def reset_system(self):
		print("=" * 50)
		print("USER CLICKED RESET BUTTON")
		print("=" * 50)

		if self.serial:
			print("Sending reset packet (type 0x03)...")
			send_reset(self.serial)
			with state_lock:
				pendulum_state["reset_ack"] = False
		else:
			print("ERROR: Serial not connected!")

		with state_lock:
			pendulum_state["running"] = False
			pendulum_state["cmX"] = X_CENTER_CM
			pendulum_state["theta"] = 0.0

		self.gui.set_running_ui_lock(False)

		print("Waiting for STM32 reset ACK (0xAA 0xEE)...")
		print("Expected: STM32 should go to State 1 (Wait for Homing)")

	def toggle_record(self):
		self.data_logger.toggle_recording()
		return self.data_logger.is_recording()

	def toggle_udp(self):
		self.udp_broadcaster.toggle()
		return self.udp_broadcaster.enabled

	def send_button_mask(self, buttons, label):
		if not self.serial:
			print(f"[{label}] Serial not connected")
			return
		with state_lock:
			is_running = pendulum_state["running"]
		if not is_running:
			print(f"[{label}] ignored. Click START first.")
			return
		press_packet = make_packet(self.command_seq, 0, 0, 0, 0, buttons)
		self.command_seq = (self.command_seq + 1) & 0xFF
		release_packet = make_packet(self.command_seq, 0, 0, 0, 0, 0)
		self.command_seq = (self.command_seq + 1) & 0xFF
		self.serial.write(press_packet)
		self.serial.write(release_packet)
		print(f"[{label}] command sent")
	# =========================
	# XBOX CONTROL ACTIONS
	# =========================
	def homing(self):
		print("[XBOX] HOMING")
		self.send_button_mask(1 << 3, "HOMING")
	
	def finish(self):
		print("[XBOX] FINISH")
		self.send_button_mask(1 << 1, "FINISH")

	def balance(self):
		print("[XBOX] BALANCE")
		self.send_button_mask(1 << 0, "BALANCE")

	def swing_up(self):
		print("[XBOX] SWING UP")
		self.send_button_mask(1 << 2, "SWING UP")


	def run(self):
		if not self.setup_serial():
			print("Failed to setup serial connection!")
			return

		while self.running:
			events = pygame.event.get()
			for event in events:
				if event.type == pygame.QUIT:
					self.running = False
				if event.type == pygame.KEYDOWN and event.key == pygame.K_ESCAPE:
					self.running = False

			with state_lock:
				is_running = pendulum_state["running"]
				gains_ack = pendulum_state["gains_ack"]
				reset_ack = pendulum_state["reset_ack"]
				cmX = pendulum_state["cmX"]
				theta = pendulum_state["theta"]

			# auto-clear gains_sent if ack too old (same behavior)
			import time
			if self.gains_sent and gains_ack:
				if time.time() - self.gains_ack_time > 3.0:
					self.gains_sent = False
					with state_lock:
						pendulum_state["gains_ack"] = False

			self.gui.handle_events(
				events=events,
				callbacks={
					"apply_gains": self.apply_gains,
					"start": self.start_system,
					"reset": self.reset_system,
					"toggle_record": self.toggle_record,
					"toggle_udp": self.toggle_udp,
					"start_graph": self.start_graph,
					"stop_graph": self.stop_graph,

					"Y": self.homing,
					"B": self.finish,
					"A": self.balance,
					"X": self.swing_up,
				}
			)

			self.ctx = {
				"is_running": is_running,
				"gains_sent": self.gains_sent,
				"gains_ack": gains_ack,
				"reset_ack": reset_ack,
				"cmX": cmX,
				"theta": theta,
				"mode": self.mode
			}
			#print(self.mode)

			graph_data = {
					"t_raw": self.t_raw,
					"cmX": self.cmX_hist,
					"degree": self.degree_hist,
					"degree0": self.degree0_hist,
					"setspeed": self.setspeed_hist,
					"r1": self.r1_hist,
					"theta_dot": self.theta_dot_hist,
					"x_center": self.x_center_hist
				}
			self.gui.draw(self.ctx, graph_data)

			self.clock.tick(FPS)

		if self.serial:
				try:
					self.serial.close()
				except Exception:
					pass
		self.udp_broadcaster.close()
		pygame.quit()
			# Avoid fatal shutdown errors caused by daemon threads still running.
		os._exit(0)


def main():
	app = PendulumMonitor()
	app.run()


if __name__ == "__main__":
	main()
