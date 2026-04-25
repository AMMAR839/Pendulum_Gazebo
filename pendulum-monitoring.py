import os
from turtle import right
os.environ["SDL_JOYSTICK_ALLOW_BACKGROUND_EVENTS"] = "1"

import sys
import threading
import pygame
import math
from threading import Lock

from lib_stick import init_joystick, joystick_sender
from lib_com import open_serial, read_control_status, send_gains, send_reset
from lib_data import DataLogger
from lib_udp import UDPBroadcaster

# ============================================================
# KONFIGURASI
# ============================================================
PORT = "COM6"
BAUD = 115200
FPS = 50

# Default gains dari balance_controller()
DEFAULT_GAINS = {
    "K_TH": -2.50 * 57.0 * 12.0,     # -1710.0
    "K_TH_D": -0.030 * 57.0 * 18.0,  # -30.78
    "K_X": 3.0,
    "K_X_D": -1.6 * 2.0,              # -3.2
    "K_X_INT": 0.0
}

# Rail limits
X_MIN_CM = -40
X_MAX_CM = 40.0
X_CENTER_CM = 0

# ============================================================
# SHARED STATE
# ============================================================
pendulum_state = {
    "cmX": X_CENTER_CM,
    "theta": 0.0,
    "x_center": 40.0,  # Target X_CENTER from STM32
    "running": False,
    "gains_ack": False,
    "gains_ack_values": None,
    "reset_ack": False
}
state_lock = Lock()

# Gains yang akan dikirim ke STM32
current_gains = DEFAULT_GAINS.copy()
gains_lock = Lock()

# ============================================================
# GUI CONSTANTS
# ============================================================
#WINDOW_WIDTH = 950
#WINDOW_HEIGHT = 600
#PANEL_WIDTH = 280
#MAIN_WIDTH = WINDOW_WIDTH - PANEL_WIDTH

# Colors
COLOR_BG = (30, 30, 40)
COLOR_PANEL = (40, 40, 50)
COLOR_TEXT = (220, 220, 220)
COLOR_STATUS_RUN = (80, 200, 120)
COLOR_STATUS_STOP = (200, 80, 80)
COLOR_REC_ON = (220, 60, 60)
COLOR_REC_OFF = (80, 80, 90)
COLOR_UDP_ON = (60, 180, 220)
COLOR_UDP_OFF = (80, 80, 90)
COLOR_BUTTON = (60, 120, 180)
COLOR_BUTTON_HOVER = (80, 150, 220)
COLOR_BUTTON_DISABLED = (60, 60, 70)
COLOR_RAIL = (180, 180, 180)
COLOR_CART = (100, 180, 255)
COLOR_PENDULUM = (255, 255, 255)
COLOR_MASS = (255, 80, 80)

# ============================================================
# GUI HELPER CLASSES
# ============================================================
class Button:
    def __init__(self, x, y, w, h, text, enabled=True, color_on=None, color_off=None):
        self.rect = pygame.Rect(x, y, w, h)
        self.text = text
        self.enabled = enabled
        self.hovered = False
        self.color_on = color_on
        self.color_off = color_off
    
    def draw(self, screen, font):
        if not self.enabled:
            color = COLOR_BUTTON_DISABLED
        elif self.hovered:
            color = COLOR_BUTTON_HOVER
        else:
            # Check if custom colors for ON/OFF state
            if self.color_on and "ON" in self.text:
                color = self.color_on
            elif self.color_off and "OFF" in self.text:
                color = self.color_off
            else:
                color = COLOR_BUTTON
        
        pygame.draw.rect(screen, color, self.rect, border_radius=5)
        pygame.draw.rect(screen, COLOR_TEXT, self.rect, 2, border_radius=5)
        
        text_surf = font.render(self.text, True, COLOR_TEXT)
        text_rect = text_surf.get_rect(center=self.rect.center)
        screen.blit(text_surf, text_rect)
    
    def update_hover(self, pos):
        self.hovered = self.rect.collidepoint(pos) and self.enabled
    
    def is_clicked(self, pos):
        return self.enabled and self.rect.collidepoint(pos)


class InputField:
    def __init__(self, x, y, w, h, label, default_value, enabled=True):
        self.rect = pygame.Rect(x, y, w, h)
        self.label = label
        self.value = str(default_value)
        self.enabled = enabled
        self.active = False
    
    def draw(self, screen, font_label, font_input):
        # Label
        label_surf = font_label.render(self.label, True, COLOR_TEXT)
        screen.blit(label_surf, (self.rect.x, self.rect.y - 22))
        
        # Input box
        color = (70, 70, 80) if self.enabled else (50, 50, 60)
        if self.active:
            color = (90, 90, 100)
        
        pygame.draw.rect(screen, color, self.rect, border_radius=3)
        border_color = COLOR_BUTTON if self.active else (100, 100, 110)
        pygame.draw.rect(screen, border_color, self.rect, 2, border_radius=3)
        
        # Text
        text_surf = font_input.render(self.value, True, COLOR_TEXT)
        text_rect = text_surf.get_rect(midleft=(self.rect.x + 8, self.rect.centery))
        screen.blit(text_surf, text_rect)
    
    def handle_event(self, event):
        if not self.enabled:
            return False
        
        if event.type == pygame.MOUSEBUTTONDOWN:
            self.active = self.rect.collidepoint(event.pos)
        elif event.type == pygame.KEYDOWN and self.active:
            if event.key == pygame.K_BACKSPACE:
                self.value = self.value[:-1]
            elif event.key == pygame.K_RETURN:
                self.active = False
            elif event.unicode in '0123456789.-':
                if len(self.value) < 15:
                    self.value += event.unicode
            return True
        return False
    
    def get_float(self, default=0.0):
        try:
            return float(self.value)
        except ValueError:
            return default


# ============================================================
# MAIN GUI CLASS
# ============================================================
class PendulumMonitor:
    def __init__(self):
        pygame.init()
        info = pygame.display.Info()
        self.screen = pygame.display.set_mode(
			(info.current_w, info.current_h),
			pygame.FULLSCREEN | pygame.SCALED
		)
        self.WINDOW_WIDTH=info.current_w;
        self.WINDOW_HEIGHT=info.current_h;
        self.PANEL_WIDTH = self.WINDOW_WIDTH/5.0;
        self.MAIN_WIDTH = self.WINDOW_WIDTH - self.PANEL_WIDTH
        #self.screen = pygame.display.set_mode((WINDOW_WIDTH, WINDOW_HEIGHT))
        pygame.display.set_caption("Pendulum Monitor - Live Tuning + UDP")
        
        self.clock = pygame.time.Clock()
        self.font_large = pygame.font.SysFont("Arial", 22, bold=True)
        self.font_medium = pygame.font.SysFont("Arial", 18)
        self.font_small = pygame.font.SysFont("Arial", 16)
        self.font_input = pygame.font.SysFont("Consolas", 16)
        
        # Data logger
        self.data_logger = DataLogger(base_dir="logs")
        
        # UDP broadcaster
        self.udp_broadcaster = UDPBroadcaster(
            broadcast_ip="192.168.1.255",  # Ganti sesuai network
            port=5000
        )
        
        # UI Elements
        self._create_ui_elements()
        
        # Serial & Threads
        self.serial = None
        self.joystick = None
        self.thread_rx = None
        self.thread_tx = None
        
        self.running = True
        self.gains_sent = False
        self.gains_ack_time = 0
    def _create_ui_elements(self):
    	# ===== base design (waktu panel masih fixed) =====
        PANEL_WIDTH_BASE = 280.0	# dulu panel fixed 280
        FIELD_W_BASE = 240
        FIELD_H_BASE = 32
        BIGBTN_H_BASE = 40
        SMALLBTN_W_BASE = 115
        TOPBTN_W_BASE = 120
        TOPBTN_H_BASE = 35

        PANEL_X_GAP_BASE = 10
        Y_BASE = 50
        FIELD_DY_BASE = 70
        BUTTONS_Y_GAP_BASE = 350
        STARTRESET_Y_GAP_BASE = 410
        STARTRESET_X_GAP_BASE = 125

        TOPBTN_Y_BASE = 15
        TOPBTN_GAP_BASE = 10	# jarak antar tombol

        # ===== magnify (panel width sekarang dinamis) =====
        # pastikan ini sudah kamu set di tempat lain:
        # self.PANEL_WIDTH = self.WINDOW_WIDTH / 5.0
        # kalau belum, minimal fallback:
        if not hasattr(self, "PANEL_WIDTH") or self.PANEL_WIDTH <= 0:
            self.PANEL_WIDTH = self.WINDOW_WIDTH / 5.0

        self.magnify = float(self.PANEL_WIDTH) / PANEL_WIDTH_BASE

        def S(x):
            return int(round(x * self.magnify))

        # ===== panel kanan =====
        panel_x = int(round(self.MAIN_WIDTH + S(PANEL_X_GAP_BASE)))
        y = S(Y_BASE)

        field_w = S(FIELD_W_BASE)
        field_h = S(FIELD_H_BASE)

        self.inputs = {
            "K_TH": InputField(panel_x, y, field_w, field_h, "K_TH", DEFAULT_GAINS["K_TH"]),
            "K_TH_D": InputField(panel_x, y + S(FIELD_DY_BASE), field_w, field_h, "K_TH_D", DEFAULT_GAINS["K_TH_D"]),
            "K_X": InputField(panel_x, y + S(2 * FIELD_DY_BASE), field_w, field_h, "K_X", DEFAULT_GAINS["K_X"]),
            "K_X_D": InputField(panel_x, y + S(3 * FIELD_DY_BASE), field_w, field_h, "K_X_D", DEFAULT_GAINS["K_X_D"]),
            "K_X_INT": InputField(panel_x, y + S(4 * FIELD_DY_BASE), field_w, field_h, "K_X_INT", DEFAULT_GAINS["K_X_INT"]),
        }

        # Buttons
        self.btn_apply = Button(panel_x, y + S(BUTTONS_Y_GAP_BASE), field_w, S(BIGBTN_H_BASE), "Apply Gains")
        self.btn_start = Button(panel_x, y + S(STARTRESET_Y_GAP_BASE), S(SMALLBTN_W_BASE), S(BIGBTN_H_BASE), "START")
        self.btn_reset = Button(panel_x + S(STARTRESET_X_GAP_BASE), y + S(STARTRESET_Y_GAP_BASE), S(SMALLBTN_W_BASE), S(BIGBTN_H_BASE), "RESET")

        # ===== tombol atas (anchor ke kanan area main) =====
        # Dulu: self.MAIN_WIDTH - 260 dan -130. Itu artinya: 2 tombol (120) + gap (10) + margin (10)
        topbtn_w = S(TOPBTN_W_BASE)
        topbtn_h = S(TOPBTN_H_BASE)
        top_y = S(TOPBTN_Y_BASE)
        gap = S(TOPBTN_GAP_BASE)
        margin = S(10)

        rec_x = int(round(self.MAIN_WIDTH - margin - topbtn_w))
        udp_x = int(round(rec_x - gap - topbtn_w))

        self.btn_udp = Button(udp_x, top_y, topbtn_w, topbtn_h, "UDP OFF",
            color_on=COLOR_UDP_ON, color_off=COLOR_UDP_OFF)
        self.btn_record = Button(rec_x, top_y, topbtn_w, topbtn_h, "REC OFF",
            color_on=COLOR_REC_ON, color_off=COLOR_REC_OFF)

    def setup_serial(self):
        """Setup serial connection and threads."""
        try:
            self.serial = open_serial(PORT, BAUD)
            print(f"Serial opened: {PORT} @ {BAUD}")
            
            # Joystick
            self.joystick = init_joystick(0)
            
            # Thread TX (joystick)
            self.thread_tx = threading.Thread(
                target=joystick_sender,
                args=(self.joystick, self.serial, FPS),
                daemon=True
            )
            self.thread_tx.start()
            
            # Thread RX (control status + gains ack + reset ack)
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
    
    def on_control_status(self, sample_tuple):
        """Callback untuk data dari STM32."""
        logtick, degree, cmX, setspeed, r1, theta_dot, theta, x_center  = sample_tuple
        
        with state_lock:
            pendulum_state["cmX"] = cmX
            pendulum_state["theta"] = theta
            pendulum_state["x_center"] = x_center
            # r1 bisa berisi X_CENTER jika dikirim dari STM32
            # Untuk sekarang kita estimate dari cmX movement pattern
            # Atau bisa ditambahkan ke reserved[3] di STM32
        
        # Data logging (jika recording)
        self.data_logger.handle_sample(sample_tuple)
        
        # UDP broadcast (jika enabled)
        self.udp_broadcaster.send_control_status(sample_tuple)
    
    def on_gains_ack(self, gains_tuple):
        """Callback untuk ACK gains dari STM32."""
        K_TH, K_TH_D, K_X, K_X_D, K_X_INT = gains_tuple
        
        with state_lock:
            pendulum_state["gains_ack"] = True
            pendulum_state["gains_ack_values"] = gains_tuple
        
        import time
        self.gains_ack_time = time.time()
        
        print(f"[ACK] Gains confirmed: K_TH={K_TH:.1f}, K_X={K_X:.2f}")
    
    def on_reset_ack(self, status):
        """Callback untuk ACK reset dari STM32."""
        with state_lock:
            pendulum_state["reset_ack"] = True
            pendulum_state["running"] = False
        
        print(f"[ACK] Reset confirmed by STM32 (status={status})")
    
    def handle_events(self):
        """Handle pygame events."""
        mouse_pos = pygame.mouse.get_pos()
        
        # Update hover states
        self.btn_apply.update_hover(mouse_pos)
        self.btn_start.update_hover(mouse_pos)
        self.btn_reset.update_hover(mouse_pos)
        self.btn_record.update_hover(mouse_pos)
        self.btn_udp.update_hover(mouse_pos)
        
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                self.running = False
            
            # Handle input fields
            for inp in self.inputs.values():
                inp.handle_event(event)
            
            # Handle clicks
            if event.type == pygame.MOUSEBUTTONDOWN and event.button == 1:
                if self.btn_apply.is_clicked(mouse_pos):
                    self.apply_gains()
                elif self.btn_start.is_clicked(mouse_pos):
                    self.start_system()
                elif self.btn_reset.is_clicked(mouse_pos):
                    self.reset_system()
                elif self.btn_record.is_clicked(mouse_pos):
                    self.data_logger.toggle_recording()
                    is_rec = self.data_logger.is_recording()
                    self.btn_record.text = "REC ON" if is_rec else "REC OFF"
                elif self.btn_udp.is_clicked(mouse_pos):
                    self.udp_broadcaster.toggle()
                    is_udp = self.udp_broadcaster.enabled
                    self.btn_udp.text = "UDP ON" if is_udp else "UDP OFF"
    
    def apply_gains(self):
        """Baca gains dari input dan kirim ke STM32."""
        with state_lock:
            if pendulum_state["running"]:
                print("Cannot apply gains while running! Stop first.")
                return
        
        with gains_lock:
            current_gains["K_TH"] = self.inputs["K_TH"].get_float(DEFAULT_GAINS["K_TH"])
            current_gains["K_TH_D"] = self.inputs["K_TH_D"].get_float(DEFAULT_GAINS["K_TH_D"])
            current_gains["K_X"] = self.inputs["K_X"].get_float(DEFAULT_GAINS["K_X"])
            current_gains["K_X_D"] = self.inputs["K_X_D"].get_float(DEFAULT_GAINS["K_X_D"])
            current_gains["K_X_INT"] = self.inputs["K_X_INT"].get_float(DEFAULT_GAINS["K_X_INT"])
        
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
        """Start monitoring."""
        if not self.gains_sent:
            print("Please apply gains first!")
            return
        
        with state_lock:
            pendulum_state["running"] = True
        
        for inp in self.inputs.values():
            inp.enabled = False
        self.btn_apply.enabled = False
        
        print("System STARTED")
    
    def reset_system(self):
        """Reset system ke idle state."""
        print("="*50)
        print("USER CLICKED RESET BUTTON")
        print("="*50)
        
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
        
        for inp in self.inputs.values():
            inp.enabled = True
        self.btn_apply.enabled = True
        
        print("Waiting for STM32 reset ACK (0xAA 0xEE)...")
        print("Expected: STM32 should go to State 1 (Wait for Homing)")
    
    def draw(self):
        """Draw everything."""
        self.screen.fill(COLOR_BG)
        
        with state_lock:
            is_running = pendulum_state["running"]
        
        status_text = "RUNNING" if is_running else "STOPPED"
        status_color = COLOR_STATUS_RUN if is_running else COLOR_STATUS_STOP
        
        pygame.draw.circle(self.screen, status_color, (30, 30), 12)
        status_surf = self.font_large.render(status_text, True, COLOR_TEXT)
        self.screen.blit(status_surf, (55, 18))
        
        # Record & UDP buttons
        self.btn_record.draw(self.screen, self.font_medium)
        self.btn_udp.draw(self.screen, self.font_medium)
        
        # Pendulum visualization
        self.draw_pendulum()
        
        # Right panel
        pygame.draw.rect(self.screen, COLOR_PANEL, 
                        (self.MAIN_WIDTH, 0, self.PANEL_WIDTH, self.WINDOW_HEIGHT))
        
        for inp in self.inputs.values():
            inp.draw(self.screen, self.font_small, self.font_input)
        
        self.btn_apply.draw(self.screen, self.font_medium)
        self.btn_start.draw(self.screen, self.font_medium)
        self.btn_reset.draw(self.screen, self.font_medium)
        
        with state_lock:
            gains_ack = pendulum_state["gains_ack"]
            reset_ack = pendulum_state["reset_ack"]
        
        import time
        if self.gains_sent and gains_ack:
            if time.time() - self.gains_ack_time > 3.0:
                self.gains_sent = False
                with state_lock:
                    pendulum_state["gains_ack"] = False
        
        if self.gains_sent:
            ack_text = "✓ Gains Applied" if gains_ack else "Waiting Gains ACK..."
            ack_color = COLOR_STATUS_RUN if gains_ack else (200, 150, 50)
            ack_surf = self.font_small.render(ack_text, True, ack_color)
            self.screen.blit(ack_surf, (self.MAIN_WIDTH + 15, 470))
        
        if not is_running and reset_ack:
            reset_text = "✓ System Ready"
            reset_surf = self.font_small.render(reset_text, True, COLOR_STATUS_RUN)
            self.screen.blit(reset_surf, (self.MAIN_WIDTH + 15, 490))
        
        with state_lock:
            cmX = pendulum_state["cmX"]
            theta = pendulum_state["theta"]
        
        theta_deg = math.degrees(theta)
        info_text = f"Position: {cmX:.1f} cm  |  Angle: {theta_deg:.2f}°"
        info_surf = self.font_medium.render(info_text, True, COLOR_TEXT)
        self.screen.blit(info_surf, (20, self.WINDOW_HEIGHT - 35))
        
        pygame.display.flip()
    
    def draw_pendulum(self):
        """Draw pendulum visualization."""
        ground_y = self.WINDOW_HEIGHT*0.5
        left_margin = self.MAIN_WIDTH*0.25
        right_margin = self.MAIN_WIDTH*0.75
        
        size_compare=(right_margin
              -left_margin)/80.0;
        
        pygame.draw.line(self.screen, COLOR_RAIL, 
                        (left_margin, ground_y), 
                        (right_margin, ground_y), 4)
        
        pygame.draw.line(self.screen, COLOR_RAIL, 
                        (left_margin, ground_y - 10), 
                        (left_margin, ground_y + 10), 3)
        pygame.draw.line(self.screen, COLOR_RAIL, 
                        (right_margin, ground_y - 10), 
                        (right_margin, ground_y + 10), 3)
        
        text_min = self.font_small.render(f"{X_MIN_CM:.0f} cm", True, COLOR_TEXT)
        text_max = self.font_small.render(f"{X_MAX_CM:.0f} cm", True, COLOR_TEXT)
        self.screen.blit(text_min, (left_margin - 15, ground_y + 15))
        self.screen.blit(text_max, (right_margin - 15, ground_y + 15))
        
        with state_lock:
            cmX = pendulum_state["cmX"]
            theta = pendulum_state["theta"]
        
        alpha = (cmX - X_MIN_CM) / (X_MAX_CM - X_MIN_CM)
        alpha = max(0.0, min(1.0, alpha))
        cart_x = int(left_margin + alpha * (right_margin - left_margin))
        cart_y = ground_y
        
        cart_w = 80
        cart_h = 30
        cart_rect = pygame.Rect(cart_x - cart_w // 2, 
                                cart_y - cart_h // 2,
                                cart_w, cart_h)
        pygame.draw.rect(self.screen, COLOR_CART, cart_rect, border_radius=6)
        
        pend_length = 38*size_compare
        pend_x = int(cart_x + pend_length * math.sin(theta))
        pend_y = int(cart_y - pend_length * math.cos(theta))
        
        pygame.draw.line(self.screen, COLOR_PENDULUM, 
                        (cart_x, cart_y), 
                        (pend_x, pend_y), 8)
        pygame.draw.circle(self.screen, COLOR_MASS, (pend_x, pend_y), 4)
    
    def run(self):
        """Main loop."""
        if not self.setup_serial():
            print("Failed to setup serial connection!")
            return
        
        while self.running:
            self.handle_events()
            self.draw()
            self.clock.tick(FPS)
        
        if self.serial:
            self.serial.close()
        self.udp_broadcaster.close()
        pygame.quit()
        sys.exit(0)


def main():
    app = PendulumMonitor()
    app.run()


if __name__ == "__main__":
    main()