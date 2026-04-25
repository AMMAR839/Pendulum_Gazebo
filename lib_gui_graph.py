import math
import pygame

COLOR_TEXT = (220, 220, 220)
COLOR_BGBOX = (25, 25, 35)
COLOR_BORDER = (200, 60, 60)
COLOR_BORDER_Y = (220, 220, 60)
COLOR_LINE = (180, 180, 180)

COLOR_BTN_ON = (70, 130, 170)
COLOR_BTN_OFF = (55, 55, 65)
COLOR_BTN_STOP = (75, 85, 95)


def _safe_median_diff(xs):
	if xs is None or len(xs) < 3:
		return None
	d = [xs[i] - xs[i - 1] for i in range(1, len(xs))]
	d = [v for v in d if v is not None]
	if not d:
		return None
	return sorted(d)[len(d) // 2]


def _to_seconds(t_raw):
	# Heuristic:
	# - if median diff > 5 => likely milliseconds
	# - else treat as seconds already
	if t_raw is None or len(t_raw) == 0:
		return []
	md = _safe_median_diff(t_raw)
	if md is None:
		return [0.0 for _ in t_raw]
	scale = 0.001 if md > 5 else 1.0
	t0 = t_raw[0]
	return [(v - t0) * scale for v in t_raw]


def _second_derivative(y, t):
	# Robust against mismatched lengths or very short buffers.
	n = min(len(y), len(t))
	if n < 5:
		return [0.0 for _ in range(n)]
	dd = [0.0] * n
	for i in range(1, n - 1):
		dt1 = t[i] - t[i - 1]
		dt2 = t[i + 1] - t[i]
		if dt1 <= 0 or dt2 <= 0:
			continue
		dy1 = (y[i] - y[i - 1]) / dt1
		dy2 = (y[i + 1] - y[i]) / dt2
		dt = (dt1 + dt2) * 0.5
		dd[i] = (dy2 - dy1) / dt
		if(dd[i]>90 or dd[i]<-90):
			dd[i]=dd[i-1]
		dd[i]=(dd[i]+dd[i-1])/2.0
	dd[0] = dd[1]
	dd[-1] = dd[-2]
	return dd


def _linreg(x, y):
    	# y = a*x + b
	n = len(x)
	if n < 2:
		return 0.0, 0.0

	# force double precision & stable accumulation
	x = [float(v) for v in x]
	y = [float(v) for v in y]

	sx  = math.fsum(x)
	sy  = math.fsum(y)
	sxx = math.fsum(v * v for v in x)
	sxy = math.fsum(x[i] * y[i] for i in range(n))

	den = n * sxx - sx * sx
	if abs(den) < 1e-12:
		return 0.0, 0.0

	a = (n * sxy - sx * sy) / den
	b = (sy - a * sx) / n
	return a, b


class Dropdown:
	def __init__(self, rect: pygame.Rect, options, default_key):
		self.rect = rect
		self.options = list(options)
		self.key = default_key if default_key in self.options else (self.options[0] if self.options else "")
		self.open = False
		self.item_h = rect.height

	def handle_event(self, event):
		if event.type != pygame.MOUSEBUTTONDOWN or event.button != 1:
			return False
		pos = event.pos
		if self.rect.collidepoint(pos):
			self.open = not self.open
			return True
		if not self.open:
			return False
		for i, opt in enumerate(self.options):
			r = pygame.Rect(self.rect.x, self.rect.y + (i + 1) * self.item_h, self.rect.w, self.item_h)
			if r.collidepoint(pos):
				self.key = opt
				self.open = False
				return True
		self.open = False
		return False

	def draw(self, screen, font):
		pygame.draw.rect(screen, (70, 70, 80), self.rect, border_radius=3)
		pygame.draw.rect(screen, (110, 110, 120), self.rect, 2, border_radius=3)
		txt = font.render(self.key, True, COLOR_TEXT)
		screen.blit(txt, (self.rect.x + 8, self.rect.y + 6))
		pygame.draw.polygon(screen, COLOR_TEXT, [
			(self.rect.right - 18, self.rect.y + 10),
			(self.rect.right - 8, self.rect.y + 10),
			(self.rect.right - 13, self.rect.y + 18),
		])
		if self.open:
			for i, opt in enumerate(self.options):
				r = pygame.Rect(self.rect.x, self.rect.y + (i + 1) * self.item_h, self.rect.w, self.item_h)
				pygame.draw.rect(screen, (55, 55, 65), r)
				pygame.draw.rect(screen, (110, 110, 120), r, 1)
				screen.blit(font.render(opt, True, COLOR_TEXT), (r.x + 8, r.y + 6))


class _SmallButton:
	def __init__(self, rect: pygame.Rect, text: str):
		self.rect = rect
		self.text = text
		self.enabled = True
		self.hovered = False

	def update_hover(self, pos):
		self.hovered = self.rect.collidepoint(pos) and self.enabled

	def is_clicked(self, pos):
		return self.enabled and self.rect.collidepoint(pos)

	def draw(self, screen, font, active=False, low_sat=False):
		if not self.enabled:
			col = COLOR_BTN_OFF
		elif low_sat:
			col = COLOR_BTN_STOP
		elif active:
			col = COLOR_BTN_ON
		elif self.hovered:
			col = (80, 150, 190)
		else:
			col = COLOR_BTN_OFF
		pygame.draw.rect(screen, col, self.rect, border_radius=5)
		pygame.draw.rect(screen, COLOR_TEXT, self.rect, 2, border_radius=5)
		t = font.render(self.text, True, COLOR_TEXT)
		r = t.get_rect(center=self.rect.center)
		screen.blit(t, r)


class GraphView:
	def __init__(self, main_width: int, window_height: int, font_small, font_medium):
		self.main_width = int(main_width)
		self.window_height = int(window_height)
		self.font_small = font_small
		self.font_medium = font_medium

		# layout based on blocks
		# Untuk memindahkan main
		left_x = int(self.main_width * 0.15)
		left_w = int(self.main_width * 0.55)
		gap_y = int(self.window_height * 0.04)

		g1_y = int(self.window_height * 0.18)
		g_h = int(self.window_height * 0.32)
		g2_y = g1_y + g_h + gap_y 

		right_x = left_x + left_w + int(self.main_width * 0.02)
		reg_size = int(self.window_height * 0.32)
		reg_y = g1_y

		res_h = int(self.window_height * 0.32)
		res_y = g2_y


		self.rect_g1 = pygame.Rect(left_x, g1_y, left_w, g_h)
		self.rect_g2 = pygame.Rect(left_x, g2_y, left_w, g_h)
		self.rect_reg = pygame.Rect(right_x, reg_y, reg_size, reg_size)
		self.rect_res = pygame.Rect(right_x, res_y, reg_size, res_h)

		# header heights (avoid text collisions)
		self.header_h_ts = 42
		self.header_h_reg = 78

		dd_w = int(left_w * 0.22)
		dd_h = 28
		signals = ["cmX", "degree","degree0", "setspeed", "r1", "theta_dot", "x_center"]
		self.dd1 = Dropdown(pygame.Rect(self.rect_g1.right - dd_w - 10, self.rect_g1.y + 8, dd_w, dd_h), signals, "cmX")
		self.dd2 = Dropdown(pygame.Rect(self.rect_g2.right - dd_w - 10, self.rect_g2.y + 8, dd_w, dd_h), signals, "degree0")

		reg_dd_w = int(reg_size * 0.58)
		self.dd_reg_x = Dropdown(pygame.Rect(self.rect_reg.x + 10, self.rect_reg.y + 8, reg_dd_w, dd_h), ["sin(degree)"], "sin(degree)")
		self.dd_reg_y = Dropdown(pygame.Rect(self.rect_reg.x + 10, self.rect_reg.y + 8 + dd_h + 8, reg_dd_w, dd_h), ["accel(degree)"], "accel(degree)")

		# start/stop/reset
		btn_y = int(self.window_height * 0.10)
		btn_w = 130
		btn_h = 36
		self.btn_start = _SmallButton(pygame.Rect(left_x, btn_y, btn_w, btn_h), "START GRAPH")
		self.btn_reset = _SmallButton(pygame.Rect(left_x + btn_w + 12, btn_y, 110, btn_h), "RESET")

		self.running = False
		self._src_last_n = 0
		self.buf_t_raw = []
		self.buf_cmX = []
		self.buf_deg = []
		self.buf_deg0 = []
		self.buf_setspeed = []
		self.buf_r1 = []
		self.buf_theta_dot = []
		self.buf_x_center = []
		# sliding window (opsi A): keep last N points for display
		self.max_points = 3000

		self.last_a = 0.0
		self.last_b = 0.0

	def reset(self):
		self.running = False
		self._src_last_n = 0
		del self.buf_t_raw[:len(self.buf_t_raw)]
		del self.buf_cmX[:len(self.buf_cmX)]
		del self.buf_deg[:len(self.buf_deg)]
		del self.buf_deg0[:len(self.buf_deg0)]
		self.buf_t_raw.clear()
		self.buf_cmX.clear()
		self.buf_deg.clear()
		self.buf_deg0.clear()
		self.buf_setspeed.clear()
		self.buf_r1.clear()
		self.buf_theta_dot.clear()
		self.buf_x_center.clear()
		self.last_a = 0.0
		self.last_b = 0.0

	def _start_from_now(self, data):
		# Start plotting from current tail (so next samples start at t=0 in the view)
		t_raw = data.get("t_raw", [])
		cmX = data.get("cmX", [])
		deg = data.get("degree", [])
		deg0 = data.get("degree0", [])
		n = min(len(t_raw), len(cmX), len(deg), len(deg0))

		# clear view buffers
		self.buf_t_raw.clear()
		self.buf_cmX.clear()
		self.buf_deg.clear()
		self.buf_deg0.clear()

		# set source cursor to current tail
		self._src_last_n = n

		# reset regression
		self.last_a = 0.0
		self.last_b = 0.0

	def handle_event(self, event, data=None):
		mouse_pos = pygame.mouse.get_pos() if hasattr(pygame, "mouse") else (0, 0)
		self.btn_start.update_hover(mouse_pos)
		self.btn_reset.update_hover(mouse_pos)

		changed = False
		changed |= self.dd1.handle_event(event)
		changed |= self.dd2.handle_event(event)
		changed |= self.dd_reg_x.handle_event(event)
		changed |= self.dd_reg_y.handle_event(event)

		if event.type == pygame.MOUSEBUTTONDOWN and event.button == 1:
			pos = event.pos
			if self.btn_start.is_clicked(pos):
				if not self.running:
					self.reset()
					if data is not None:
						self._start_from_now(data)
					self.running = True
					self.btn_start.text = "STOP"  # switch label
					
				else:
					self.running = False
					# STOP: clear buffers so the graph becomes empty
					#self.reset()
					self.btn_start.text = "START GRAPH"
				return True
			if self.btn_reset.is_clicked(pos):
				self.reset()
				self.btn_start.text = "START GRAPH"
				return True
		return changed

	def _draw_box(self, screen, rect, title, border_color):
		pygame.draw.rect(screen, COLOR_BGBOX, rect)
		pygame.draw.rect(screen, border_color, rect, 3)
		if title:
			screen.blit(self.font_small.render(title, True, border_color), (rect.x + 10, rect.y + 8))

	def _plot_timeseries(self, screen, rect, t, y, label, fixed_range=None):
		if len(t) < 2:
			return

		# plot area: reserve header
		pad_l = 34
		pad_r = 16
		pad_b = 24
		pad_t = self.header_h_ts

		x0 = rect.x + pad_l
		y0 = rect.y + pad_t
		w = rect.w - pad_l - pad_r
		h = rect.h - pad_t - pad_b
		if w <= 5 or h <= 5:
			return

		# axes
		pygame.draw.line(screen, COLOR_LINE, (x0, y0 + h), (x0 + w, y0 + h), 1)
		pygame.draw.line(screen, COLOR_LINE, (x0, y0), (x0, y0 + h), 1)

		# subtitle in header (no collision with title)
		screen.blit(self.font_small.render(label, True, COLOR_TEXT), (rect.x + 10, rect.y + 26))

		tmin, tmax = t[0], t[-1]
		if fixed_range is not None:
			ymin, ymax = fixed_range
		else:
			ymin, ymax = min(y), max(y)
			if abs(ymax - ymin) < 1e-12:
				ymax = ymin + 1.0

		py_max = y0          # paling atas grafik
		py_min = y0 + h      # paling bawah grafik

		# render teks
		txt_max = self.font_small.render(f"{ymax:.2f}", True, COLOR_TEXT)
		txt_min = self.font_small.render(f"{ymin:.2f}", True, COLOR_TEXT)

		# gambar di sumbu Y kiri (seperti MATLAB)
		screen.blit(
			txt_max,
			(rect.x - txt_max.get_width() - 6, py_max - 8)
		)
		screen.blit(
			txt_min,
			(rect.x - txt_min.get_width() - 6, py_min - 8)
		)
		
		if ymin < 0 < ymax:
			y_zero = y0 + h - int((-ymin) / (ymax - ymin) * h)
			pygame.draw.line(screen, (120, 120, 120), (x0, y_zero), (x0 + w, y_zero), 1)

		pts = []
		for i in range(len(t)):
			xt = (t[i] - tmin) / (tmax - tmin) if tmax != tmin else 0.0
			yt = (y[i] - ymin) / (ymax - ymin)
			px = x0 + int(xt * w)
			py = y0 + h - int(yt * h)
			pts.append((px, py))
		if len(pts) >= 2:
			pygame.draw.lines(screen, (120, 200, 255), False, pts, 2)

	def _plot_regression(self, screen, rect, x, y, a, b):
		if len(x) < 2:
			return

		pad_l = 34
		pad_r = 16
		pad_b = 24
		pad_t = self.header_h_reg

		x0 = rect.x + pad_l
		y0 = rect.y + pad_t
		w = rect.w - pad_l - pad_r
		h = rect.h - pad_t - pad_b
		if w <= 5 or h <= 5:
			return

		xmin, xmax = min(x), max(x)
		ymin, ymax = min(y), max(y)
		if abs(xmax - xmin) < 1e-12:
			xmax = xmin + 1.0
		if abs(ymax - ymin) < 1e-12:
			ymax = ymin + 1.0

		pygame.draw.line(screen, COLOR_LINE, (x0, y0 + h), (x0 + w, y0 + h), 1)
		pygame.draw.line(screen, COLOR_LINE, (x0, y0), (x0, y0 + h), 1)

		# scatter downsample
		step = max(1, len(x) // 400)
		for i in range(0, len(x), step):
			xt = (x[i] - xmin) / (xmax - xmin)
			yt = (y[i] - ymin) / (ymax - ymin)
			px = x0 + int(xt * w)
			py = y0 + h - int(yt * h)
			screen.set_at((px, py), (255, 180, 180))

		# fit line
		y1 = a * xmin + b
		y2 = a * xmax + b
		p1 = (x0, y0 + h - int(((y1 - ymin) / (ymax - ymin)) * h))
		p2 = (x0 + w, y0 + h - int(((y2 - ymin) / (ymax - ymin)) * h))
		pygame.draw.line(screen, (120, 255, 120), p1, p2, 2)

	
	def _trim_buffers(self):
		# Keep only last max_points samples (lightweight realtime)
		if self.max_points is None:
			return
		n = len(self.buf_t_raw)
		if n <= self.max_points:
			return
		cut = n - self.max_points
		del self.buf_t_raw[:cut]
		del self.buf_cmX[:cut]
		del self.buf_deg[:cut]
		del self.buf_deg0[:cut]
		del self.buf_setspeed[:cut]
		del self.buf_r1[:cut]
		del self.buf_theta_dot[:cut]
		del self.buf_x_center[:cut]


	def _capture_if_running(self, data):
		if not self.running:
			return
		t_raw = data.get("t_raw", [])
		cmX = data.get("cmX", [])
		deg = data.get("degree", [])
		deg0 = data.get("degree0", [])
		setspeed = data.get("setspeed", [])
		r1 = data.get("r1", [])
		theta_dot = data.get("theta_dot", [])
		x_center = data.get("x_center", [])

		n = min(
			len(t_raw), len(cmX), len(deg), len(deg0),
			len(setspeed), len(r1),
			len(theta_dot), len(x_center)
		)

		if n <= self._src_last_n:
			return
		new_slice = slice(self._src_last_n, n)
		self.buf_t_raw.extend(t_raw[new_slice])
		self.buf_cmX.extend(cmX[new_slice])
		self.buf_deg.extend(deg[new_slice])
		self.buf_deg0.extend(deg0[new_slice])
		self.buf_setspeed.extend(setspeed[new_slice])
		self.buf_r1.extend(r1[new_slice])
		self.buf_theta_dot.extend(theta_dot[new_slice])
		self.buf_x_center.extend(x_center[new_slice])
		self._src_last_n = n

	# helper to pick signal buffer and label based on dropdown key
	def _pick_signal(self, key):
		if key == "cmX":
			return self.buf_cmX, "cmX vs time"
		if key == "degree":
			return self.buf_deg, "degree vs time"
		if key == "degree0":
			return self.buf_deg0, "degree0 vs time"
		if key == "setspeed":
			return self.buf_setspeed, "setspeed vs time"
		if key == "theta_dot":
			return self.buf_theta_dot, "theta_dot vs time"
		if key == "x_center":
			return self.buf_x_center, "x_center vs time"
		if key == "r1":
			return self.buf_r1, "r1 (state) vs time"
		return [], "unknown"


	def draw(self, screen, data):
		# data expected: { "t_raw":[], "cmX":[], "degree":[] }
		if data is None:
			data = {}
        
		# update capture based on running state
		self._capture_if_running(data)
		self._trim_buffers()
		# button draw state
		self.btn_start.text = "STOP" if self.running else "START GRAPH"
		self.btn_start.draw(screen, self.font_medium, active=self.running, low_sat=self.running)
		self.btn_reset.draw(screen, self.font_medium, active=False, low_sat=False)

		# draw blocks
		self._draw_box(screen, self.rect_g1, "GRAFIK 1", COLOR_BORDER)
		self._draw_box(screen, self.rect_g2, "GRAFIK 2", COLOR_BORDER)
		self._draw_box(screen, self.rect_reg, "REGRESI", COLOR_BORDER)
		self._draw_box(screen, self.rect_res, "HASIL REGRESI", COLOR_BORDER_Y)

		# dropdowns (placed in header zones)
		self.dd1.draw(screen, self.font_small)
		self.dd2.draw(screen, self.font_small)
		self.dd_reg_x.draw(screen, self.font_small)
		self.dd_reg_y.draw(screen, self.font_small)

		# if not running and buffer empty => show nothing
		t = _to_seconds(self.buf_t_raw)
		cmX = self.buf_cmX
		deg = self.buf_deg
		deg0 = self.buf_deg0
		
		y1, l1 = self._pick_signal(self.dd1.key)
		y2, l2 = self._pick_signal(self.dd2.key)


		n1 = min(len(t), len(y1))
		n2 = min(len(t), len(y2))
		if n1 >= 2:
			self._plot_timeseries(screen, self.rect_g1, t[:n1], y1[:n1], l1)
		if n2 >= 2:
			self._plot_timeseries(screen, self.rect_g2, t[:n2], y2[:n2], l2)

		# regression: x=sin(degree), y=accel(degree)
		if len(t) >= 5 and len(deg0) >= 5:
			n0 = min(len(t), len(deg0))
			deg_rad = [math.radians(v) for v in deg0[:n0]]
			t_use = t[:n0]
			x = [math.sin(v) for v in deg_rad]
			dd = _second_derivative(deg_rad, t_use)
			n = min(len(x), len(dd))
			if n >= 5:
				i0 = max(2, n // 50)
				i1 = n - i0
				xr = x[:n][i0:i1]
				yr = dd[:n][i0:i1]
				a, b = _linreg(xr, yr)
				self.last_a, self.last_b = a, b
				self._plot_regression(screen, self.rect_reg, xr, yr, a, b)

		# regression text
		line1 = f"y = a*x + b"
		line2 = f"a={self.last_a:.4f}  b={self.last_b:.4f}"

		# physics interpretation
		g = 9.781
		a = self.last_a
		b = self.last_b

		# avoid divide-by-zero / nonsense sign
		if abs(a) > 1e-9:
			# physical pendulum: uniform rod, pivot at one end
			L_est = -3.0 * g / (2.0 * a)
			# equivalent simple pendulum length (I/(m r))
			leq_est = -g / a
			line3 = f"l_eq  ≈ L = -3g/(2a)"
			line4 = f"L_rod ≈ {L_est:.3f} m   (uniform rod, pivot end)"
		else:
			line3 = "l_eq : n/a (a too small)"
			line4 = "L_rod: n/a (a too small)"

		# optional: show bias meaning
		line5 = f"bias b ≈ {b:.4f} rad/s^2 (offset)"

		screen.blit(self.font_small.render(line1, True, COLOR_TEXT), (self.rect_res.x + 10, self.rect_res.y + 42))
		screen.blit(self.font_small.render(line2, True, COLOR_TEXT), (self.rect_res.x + 10, self.rect_res.y + 64))
		screen.blit(self.font_small.render(line3, True, COLOR_TEXT), (self.rect_res.x + 10, self.rect_res.y + 86))
		screen.blit(self.font_small.render(line4, True, COLOR_TEXT), (self.rect_res.x + 10, self.rect_res.y + 108))
		screen.blit(self.font_small.render(line5, True, COLOR_TEXT), (self.rect_res.x + 10, self.rect_res.y + 130))
