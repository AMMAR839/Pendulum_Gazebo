import pygame
import time
from lib_com import make_packet


def init_joystick(index: int = 0):
    pygame.joystick.init()
    if pygame.joystick.get_count() == 0:
        raise RuntimeError("No joystick detected.")
    js = pygame.joystick.Joystick(index)
    js.init()
    print("Joystick connected:", js.get_name())
    return js


def scale_axis(v: float) -> int:
    # pygame axis -1..+1 -> int16
    if v > 1.0:
        v = 1.0
    if v < -1.0:
        v = -1.0
    return int(v * 32767)


def get_buttons_mask(js) -> int:
    m = 0
    for i in range(js.get_numbuttons()):
        if js.get_button(i):
            m |= (1 << i)  # tombol i -> bit i
    return m & 0xFFFF


def joystick_sender(js, ser, fps=50):
    """Thread pengirim joystick → STM32, 50 Hz."""
    period = 1.0 / fps
    seq = 0
    while True:
        pygame.event.pump()  # supaya state joystick update
        ax = scale_axis(js.get_axis(0))
        ay = scale_axis(js.get_axis(1))
        rx = scale_axis(js.get_axis(2) if js.get_numaxes() > 2 else 0.0)
        ry = scale_axis(js.get_axis(3) if js.get_numaxes() > 3 else 0.0)
        buttons = get_buttons_mask(js)

        pkt = make_packet(seq, ax, ay, rx, ry, buttons)
        ser.write(pkt)
        seq = (seq + 1) & 0xFF

        time.sleep(period)
