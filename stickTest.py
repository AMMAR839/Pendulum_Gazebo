# stick_serial.py
import os
os.environ["SDL_JOYSTICK_ALLOW_BACKGROUND_EVENTS"] = "1"
os.environ["SDL_HINT_JOYSTICK_ALLOW_BACKGROUND_EVENTS"] = "1"
import pygame, sys, struct, time
import serial  # pip install pyserial

PORT = "COM6"          # ganti sesuai port kamu (Linux: "/dev/ttyACM0")
BAUD = 115200
FPS  = 50              # 50 Hz kirim
import threading
import struct
def read_control_status(ser):
    HEADER = b'\xAA\xCC'
    hdr_len = 4
    payload_len = 52
    total_len = hdr_len + payload_len + 2

    buffer = bytearray()
    fmt = "<Idddddd"   # uint32 + 6x double (degree, cmX, setspeed, reserved[3])

    while True:
        chunk = ser.read(128)
        if not chunk:
            time.sleep(0.01)
            continue
        buffer.extend(chunk)

        while len(buffer) >= total_len:
            idx = buffer.find(HEADER)
            if idx < 0:
                buffer.clear()
                break
            if idx > 0:
                del buffer[:idx]
            if len(buffer) < total_len:
                break
            print(buffer)
            pkt = buffer[:total_len]
            del buffer[:total_len]

            crc_recv = pkt[-2] | (pkt[-1] << 8)
            crc_calc = sum(pkt[4:-2]) & 0xFFFF
            if crc_recv != crc_calc:
                print("CRC mismatch")
                continue

            data = pkt[4:-2]
            unpacked = struct.unpack(fmt, data)
            logtick = unpacked[0]
            degree, cmX, setspeed = unpacked[1:4]

            print(f"[STM32] tick={logtick:8d} deg={degree:8.3f} cmX={cmX:8.3f} set={setspeed:8.3f}")

def joystick_sender(js, ser, fps=50):
    import time
    seq = 0
    period = 1.0 / fps
    while True:
        pygame.event.pump()  # tetap panggil agar SDL update state
        ax = scale_axis(js.get_axis(0))
        ay = scale_axis(js.get_axis(1))
        rx = scale_axis(js.get_axis(2) if js.get_numaxes() > 2 else 0.0)
        ry = scale_axis(js.get_axis(3) if js.get_numaxes() > 3 else 0.0)
        buttons = get_buttons_mask(js)
        pkt = make_packet(seq, ax, ay, rx, ry, buttons)
        ser.write(pkt)
        seq = (seq + 1) & 0xFF
        time.sleep(period)


def scale_axis(v: float) -> int:
    # pygame axis -1..+1 -> int16
    if v >  1.0: v =  1.0
    if v < -1.0: v = -1.0
    return int(v * 32767)

def get_buttons_mask(js) -> int:
    m = 0
    for i in range(js.get_numbuttons()):
        if js.get_button(i):
            m |= (1 << i)       # tombol i -> bit i
    return m & 0xFFFF

def make_packet(seq, ax, ay, rx, ry, buttons):
    header = b'\xAA\x55'
    typ = 0x01
    body = struct.pack('<BBhhhhH', typ, seq, ax, ay, rx, ry, buttons)
    s = sum(body) % 65535
    chksum = struct.pack('<H', s)
    return header + body + chksum

def main():
    # === SERIAL ===
    ser = serial.Serial(PORT, BAUD, timeout=0)  # non-blocking
    print("Serial opened:", ser.port, BAUD)

    # === PYGAME ===
    pygame.init()
    width, height = 600, 400
    screen = pygame.display.set_mode((width, height))
    pygame.display.set_caption("PS Controller → Serial")
    font = pygame.font.SysFont("Arial", 18)
    clock = pygame.time.Clock()

    pygame.joystick.init()
    if pygame.joystick.get_count() == 0:
        print("No joystick detected.")
        sys.exit(1)
    js = pygame.joystick.Joystick(0)
    js.init()
    print("Joystick connected:", js.get_name())
    # setelah joystick init & serial open:
    t = threading.Thread(target=joystick_sender, args=(js, ser, FPS), daemon=True)
    t.start()
    seq = 0
    t2 = threading.Thread(target=read_control_status, args=(ser,), daemon=True)
    t2.start()
    while True:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                ser.close()
                pygame.quit()
                sys.exit(0)

        pygame.event.pump()

        # ambil axis
        ax = scale_axis(js.get_axis(0))
        ay = scale_axis(js.get_axis(1))
        try:
            rx = scale_axis(js.get_axis(2))
            ry = scale_axis(js.get_axis(3))
        except IndexError:
            rx = 0; ry = 0

        buttons = get_buttons_mask(js)

        #pkt = make_packet(seq, ax, ay, rx, ry, buttons)
        #ser.write(pkt)
        seq = (seq + 1) & 0xFF

        # --- UI ringkas ---
        screen.fill((30, 30, 30))
        screen.blit(font.render(f"AX: {ax:6d}  AY: {ay:6d}  RX: {rx:6d}  RY: {ry:6d}", True, (255,255,255)), (20, 20))
        screen.blit(font.render(f"Buttons: 0x{buttons:04X}", True, (200,200,255)), (20, 45))
        screen.blit(font.render(f"Seq: {seq}", True, (200,255,200)), (20, 70))
        pygame.display.flip()

        clock.tick(FPS)

if __name__ == "__main__":
    main()
