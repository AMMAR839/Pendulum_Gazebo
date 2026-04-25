import math
import pygame
from threading import Lock

from lib_gui_graph import GraphView

# ============================================================
# GUI CONSTANTS
# ============================================================
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

MODE_2D_SIM = 0
MODE_GRAPH = 1

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
    
# berguna untuk mode 2D dan Graph saling eksklusif	
class ModeButton:
    def __init__(self, active_color):
        self.buttons = []
        self.active_index = 0
        self.active_color = active_color

    def add(self, button):
        self.buttons.append(button)

    def set_active_index(self, index):
        self.active_index = index

    def handle_click(self, mouse_pos):
        for i, btn in enumerate(self.buttons):
            if btn.rect.collidepoint(mouse_pos):
                self.active_index = i
                return i
        return None

    def draw(self, screen, font):
        for i, btn in enumerate(self.buttons):
            if i == self.active_index:
                pygame.draw.rect(
                    screen,
                    self.active_color,
                    btn.rect,
                    border_radius=5
                )
            else:
                pygame.draw.rect(
                    screen,
                    COLOR_BUTTON_DISABLED,
                    btn.rect,
                    border_radius=5
                    
                )
            pygame.draw.rect(screen, COLOR_TEXT, btn.rect, 2, border_radius=5)
            text_surf = font.render(btn.text, True, COLOR_TEXT)
            text_rect = text_surf.get_rect(center=btn.rect.center)
            screen.blit(text_surf, text_rect)
 

class InputField:
    def __init__(self, x, y, w, h, label, default_value, enabled=True):
        self.rect = pygame.Rect(x, y, w, h)
        self.label = label
        self.value = str(default_value)
        self.enabled = enabled
        self.active = False

    def draw(self, screen, font_label, font_input):
        label_surf = font_label.render(self.label, True, COLOR_TEXT)
        screen.blit(label_surf, (self.rect.x, self.rect.y - 22))

        color = (70, 70, 80) if self.enabled else (50, 50, 60)
        if self.active:
            color = (90, 90, 100)

        pygame.draw.rect(screen, color, self.rect, border_radius=3)
        border_color = COLOR_BUTTON if self.active else (100, 100, 110)
        pygame.draw.rect(screen, border_color, self.rect, 2, border_radius=3)

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

class XboxButton:
    def __init__(self, center, radius, label, caption, color):
        self.cx, self.cy = center
        self.radius = radius
        self.label = label
        self.caption = caption
        self.color = color
        self.hovered = False

    def draw(self, screen, font_big, font_small):
        color = tuple(min(255, c + 30) for c in self.color) if self.hovered else self.color
        pygame.draw.circle(screen, color, (self.cx, self.cy), self.radius)
        pygame.draw.circle(screen, COLOR_TEXT, (self.cx, self.cy), self.radius, 2)

        label_surf = font_big.render(self.label, True, COLOR_TEXT)
        label_rect = label_surf.get_rect(center=(self.cx, self.cy)) # DIGANTI
        screen.blit(label_surf, label_rect)

        cap_surf = font_small.render(self.caption, True, COLOR_TEXT)
        cap_rect = cap_surf.get_rect(center=(self.cx, self.cy + self.radius + 14))
        screen.blit(cap_surf, cap_rect)

    def update_hover(self, pos):
        dx = pos[0] - self.cx
        dy = pos[1] - self.cy
        self.hovered = (dx*dx + dy*dy) <= self.radius*self.radius

    def is_clicked(self, pos):
        return self.hovered


class PendulumGUI:
    def __init__(self, screen, window_w, window_h, main_w, panel_w, fonts, x_min_cm, x_max_cm, state_ref, state_lock: Lock):
        self.screen = screen
        self.WINDOW_WIDTH = window_w
        self.WINDOW_HEIGHT = window_h
        self.MAIN_WIDTH = main_w
        self.PANEL_WIDTH = panel_w

        self.font_large, self.font_medium, self.font_small, self.font_input = fonts

        self.X_MIN_CM = x_min_cm
        self.X_MAX_CM = x_max_cm

        self.state = state_ref
        self.state_lock = state_lock

        self.active_mode = MODE_2D_SIM

        self._create_ui_elements()
        self.graph_view = GraphView(self.MAIN_WIDTH, self.WINDOW_HEIGHT, self.font_small, self.font_medium)

    def _create_ui_elements(self):
        # ===== base design (waktu panel masih fixed) =====
        PANEL_WIDTH_BASE = 280.0
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
        TOPBTN_GAP_BASE = 10

        self.magnify = float(self.PANEL_WIDTH) / PANEL_WIDTH_BASE

        def S(x):
            return int(round(x * self.magnify))

        panel_x = int(round(self.MAIN_WIDTH + S(PANEL_X_GAP_BASE)))
        y = S(Y_BASE)

        field_w = S(FIELD_W_BASE)
        field_h = S(FIELD_H_BASE)

        # inputs will be filled from caller (main) via set_gains_defaults()
        self.inputs = {
            "K_TH": InputField(panel_x, y, field_w, field_h, "K_TH", 0.0),
            "K_TH_D": InputField(panel_x, y + S(FIELD_DY_BASE), field_w, field_h, "K_TH_D", 0.0),
            "K_X": InputField(panel_x, y + S(2 * FIELD_DY_BASE), field_w, field_h, "K_X", 0.0),
            "K_X_D": InputField(panel_x, y + S(3 * FIELD_DY_BASE), field_w, field_h, "K_X_D", 0.0),
            "K_X_INT": InputField(panel_x, y + S(4 * FIELD_DY_BASE), field_w, field_h, "K_X_INT", 0.0),
        }

        self.btn_apply = Button(panel_x, y + S(BUTTONS_Y_GAP_BASE), field_w, S(BIGBTN_H_BASE), "Apply Gains")
        self.btn_start = Button(panel_x, y + S(STARTRESET_Y_GAP_BASE), S(SMALLBTN_W_BASE), S(BIGBTN_H_BASE), "START")
        self.btn_reset = Button(panel_x + S(STARTRESET_X_GAP_BASE), y + S(STARTRESET_Y_GAP_BASE), S(SMALLBTN_W_BASE), S(BIGBTN_H_BASE), "RESET")

        topbtn_w = S(TOPBTN_W_BASE)
        topbtn_h = S(TOPBTN_H_BASE)
        top_y = S(TOPBTN_Y_BASE)
        gap = S(TOPBTN_GAP_BASE)
        margin = S(10)

        rec_x = int(round(self.MAIN_WIDTH - margin - topbtn_w))
        udp_x = int(round(rec_x - gap - topbtn_w))

        self.btn_udp = Button(udp_x, top_y, topbtn_w, topbtn_h, "UDP OFF", color_on=COLOR_UDP_ON, color_off=COLOR_UDP_OFF)
        self.btn_record = Button(rec_x, top_y, topbtn_w, topbtn_h, "REC OFF", color_on=COLOR_REC_ON, color_off=COLOR_REC_OFF)

        # mode buttons under status on left
        mode_w = int(round(topbtn_w * 0.9))
        mode_h = topbtn_h
        mode_x = int(round(self.MAIN_WIDTH * 0.02))
        mode_y = int(round(self.WINDOW_HEIGHT * 0.09))

        self.btn_mode_2d = Button(mode_x, mode_y, mode_w, mode_h, "2D SIM")
        self.btn_mode_graph = Button(mode_x, (mode_y + mode_h + int(round(8 * self.magnify))), mode_w, mode_h, "GRAPH")

        self.mode_group = ModeButton(active_color=COLOR_BUTTON)
        self.mode_group.add(self.btn_mode_2d)
        self.mode_group.add(self.btn_mode_graph)

        # default active mode
        self.mode_group.set_active_index(1)
        self.active_mode = MODE_GRAPH

        # Xbox buttons area
        pad_center_x = panel_x + field_w // 2
        xbox_y_offset = S(40)
        pad_center_y = y + S(STARTRESET_Y_GAP_BASE + 120) + xbox_y_offset

        r = S(24)
        gap = S(50)

        self.xbox_buttons = {
            "Y": XboxButton(
                (pad_center_x, pad_center_y - gap),
                r, "Y", "HOMING", (245, 197, 66)
            ),
            "B": XboxButton(
                (pad_center_x + gap, pad_center_y),
                r, "B", "FINISH", (220, 80, 80)
            ),
            "A": XboxButton(
                (pad_center_x, pad_center_y + gap),
                r, "A", "BALANCE", (80, 180, 120)
            ),
            "X": XboxButton(
                (pad_center_x - gap, pad_center_y),
                r, "X", "SWING UP", (80, 150, 220)
            ),
        }

        self.state_label = "IDLE"


    def set_gains_defaults(self, gains_dict):
        for k, v in gains_dict.items():
            if k in self.inputs:
                self.inputs[k].value = str(v)

    def set_record_text(self, is_rec: bool):
        self.btn_record.text = "REC ON" if is_rec else "REC OFF"

    def set_udp_text(self, is_udp: bool):
        self.btn_udp.text = "UDP ON" if is_udp else "UDP OFF"

    def set_running_ui_lock(self, is_running: bool):
        for inp in self.inputs.values():
            inp.enabled = not is_running
        self.btn_apply.enabled = not is_running

    def _update_hover(self, mouse_pos):
        self.btn_apply.update_hover(mouse_pos)
        self.btn_start.update_hover(mouse_pos)
        self.btn_reset.update_hover(mouse_pos)
        self.btn_record.update_hover(mouse_pos)
        self.btn_udp.update_hover(mouse_pos)
        self.btn_mode_2d.update_hover(mouse_pos)
        self.btn_mode_graph.update_hover(mouse_pos)

        for btn in self.xbox_buttons.values():
            btn.update_hover(mouse_pos)

    def handle_events(self, events, callbacks, graph_data_provider=None):
        """
        callbacks: dict with keys:
            apply_gains(), start(), reset(), toggle_record()->bool, toggle_udp()->bool
        graph_data_provider: callable -> dict {t_raw, cmX, degree}
        """
        mouse_pos = pygame.mouse.get_pos()
        self._update_hover(mouse_pos)

        for event in events:
            # input fields
            for inp in self.inputs.values():
                inp.handle_event(event)

            # mode-specific events (dropdowns)
            if self.active_mode == MODE_GRAPH:
                self.graph_view.handle_event(event)

            if event.type == pygame.MOUSEBUTTONDOWN and event.button == 1:
                if self.btn_apply.is_clicked(mouse_pos):
                    callbacks["apply_gains"]()
                elif self.btn_start.is_clicked(mouse_pos):
                    callbacks["start"]()
                elif self.btn_reset.is_clicked(mouse_pos):
                    callbacks["reset"]()
                elif self.btn_record.is_clicked(mouse_pos):
                    is_rec = callbacks["toggle_record"]()
                    self.set_record_text(is_rec)
                elif self.btn_udp.is_clicked(mouse_pos):
                    is_udp = callbacks["toggle_udp"]()
                    self.set_udp_text(is_udp)
                else:
                    idx = self.mode_group.handle_click(mouse_pos)
                    if idx is not None:
                        self.active_mode = idx
                # elif self.btn_mode_2d.is_clicked(mouse_pos):
                # 	self.active_mode = MODE_2D_SIM
                # elif self.btn_mode_graph.is_clicked(mouse_pos):
                # 	self.active_mode = MODE_GRAPH
                for key, btn in self.xbox_buttons.items():
                    if btn.is_clicked(mouse_pos):
                        if key in callbacks:
                            callbacks[key]()  # call the corresponding action

    def draw(self, context, graph_data):
        self.screen.fill(COLOR_BG)

        is_running = context["is_running"]
        status_text = "RUNNING" if is_running else "STOPPED"
        status_color = COLOR_STATUS_RUN if is_running else COLOR_STATUS_STOP

        pygame.draw.circle(self.screen, status_color, (30, 30), 12)
        status_surf = self.font_large.render(status_text, True, COLOR_TEXT)
        self.screen.blit(status_surf, (55, 18))

        # top buttons
        self.btn_record.draw(self.screen, self.font_medium)
        self.btn_udp.draw(self.screen, self.font_medium)

        # mode buttons (mutual active)
        self.mode_group.draw(self.screen, self.font_medium)

        # main area
        if self.active_mode == MODE_2D_SIM:
            self._draw_pendulum()
        else:
            self.graph_view.draw(self.screen, graph_data)

        # right panel
        pygame.draw.rect(self.screen, COLOR_PANEL, (self.MAIN_WIDTH, 0, self.PANEL_WIDTH, self.WINDOW_HEIGHT))

        for inp in self.inputs.values():
            inp.draw(self.screen, self.font_small, self.font_input)

        self.btn_apply.draw(self.screen, self.font_medium)
        self.btn_start.draw(self.screen, self.font_medium)
        self.btn_reset.draw(self.screen, self.font_medium)

        for btn in self.xbox_buttons.values():
            btn.draw(self.screen, self.font_medium, self.font_small)

        label_y = max(btn.cy for btn in self.xbox_buttons.values()) + 80
        label_x = self.MAIN_WIDTH + (self.PANEL_WIDTH // 2)

        label_surf = self.font_medium.render(self.state_label, True, COLOR_TEXT)
        label_rect = label_surf.get_rect(center=(label_x, label_y))
        self.screen.blit(label_surf, label_rect)

        # acks
        if context.get("gains_sent", False):
            ack_text = "✓ Gains Applied" if context.get("gains_ack", False) else "Waiting Gains ACK..."
            ack_color = COLOR_STATUS_RUN if context.get("gains_ack", False) else (200, 150, 50)
            ack_surf = self.font_small.render(ack_text, True, ack_color)
            self.screen.blit(ack_surf, (self.MAIN_WIDTH + 15, int(self.WINDOW_HEIGHT * 0.78)))

        if (not is_running) and context.get("reset_ack", False):
            reset_surf = self.font_small.render("✓ System Ready", True, COLOR_STATUS_RUN)
            self.screen.blit(reset_surf, (self.MAIN_WIDTH + 15, int(self.WINDOW_HEIGHT * 0.81)))

        # bottom info
        cmX = context["cmX"]
        theta = context["theta"]
        theta_deg = math.degrees(theta)
        info_text = f"Position: {cmX:.1f} cm  |  Angle: {theta_deg:.2f}°"
        info_surf = self.font_medium.render(info_text, True, COLOR_TEXT)
        self.screen.blit(info_surf, (20, self.WINDOW_HEIGHT - 35))
        mode = context.get("mode", 0)
        if mode == 1:
            self.state_label = "WAITING"
        elif mode == 2:
            self.state_label = "HOMING"
        elif mode == 201:
            self.state_label = "TO CENTER"
        elif mode == 3:
            self.state_label = "READY"
        elif mode == 4:
            self.state_label = "SINUS"
        elif mode == 5:
            self.state_label = "FINISH"
        elif mode == 6:
            self.state_label = "SWING UP"
        elif mode == 7:
            self.state_label = "BALANCING"
        else:
            self.state_label = "UNDEFINED"


        pygame.display.flip()

    def _draw_pendulum(self):
        ground_y = self.WINDOW_HEIGHT * 0.5
        left_margin = self.MAIN_WIDTH * 0.25
        right_margin = self.MAIN_WIDTH * 0.75

        size_compare = (right_margin - left_margin) / 80.0

        pygame.draw.line(self.screen, COLOR_RAIL, (left_margin, ground_y), (right_margin, ground_y), 4)

        pygame.draw.line(self.screen, COLOR_RAIL, (left_margin, ground_y - 10), (left_margin, ground_y + 10), 3)
        pygame.draw.line(self.screen, COLOR_RAIL, (right_margin, ground_y - 10), (right_margin, ground_y + 10), 3)

        text_min = self.font_small.render(f"{self.X_MIN_CM:.0f} cm", True, COLOR_TEXT)
        text_max = self.font_small.render(f"{self.X_MAX_CM:.0f} cm", True, COLOR_TEXT)
        self.screen.blit(text_min, (left_margin - 15, ground_y + 15))
        self.screen.blit(text_max, (right_margin - 15, ground_y + 15))

        with self.state_lock:
            cmX = self.state["cmX"]
            theta = self.state["theta"]

        alpha = (cmX - self.X_MIN_CM) / (self.X_MAX_CM - self.X_MIN_CM)
        alpha = max(0.0, min(1.0, alpha))
        cart_x = int(left_margin + alpha * (right_margin - left_margin))
        cart_y = ground_y

        cart_w = 80
        cart_h = 30
        cart_rect = pygame.Rect(cart_x - cart_w // 2, cart_y - cart_h // 2, cart_w, cart_h)
        pygame.draw.rect(self.screen, COLOR_CART, cart_rect, border_radius=6)

        pend_length = 38 * size_compare
        pend_x = int(cart_x + pend_length * math.sin(theta))
        pend_y = int(cart_y - pend_length * math.cos(theta))

        pygame.draw.line(self.screen, COLOR_PENDULUM, (cart_x, cart_y), (pend_x, pend_y), 8)
        pygame.draw.circle(self.screen, COLOR_MASS, (pend_x, pend_y), 4)
