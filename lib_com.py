import serial
import struct
import time
from serial.tools import list_ports



def open_serial(port: str, baud: int, timeout: float = 0.0):
    """Buka port serial."""
    return serial.Serial(port, baud, timeout=timeout)

def scan_ports():
    ports = []
    stm_port = None
    for p in list_ports.comports():
        ports.append(p.device)
        
        desc = (p.description or "").lower()
        manu = (p.manufacturer or "").lower()
        
        if (
            "stmicroelectronics" in desc
            or "stmicroelectronics" in manu
            or p.vid == 0x0483
        ):
            stm_port = p.device

    return ports, stm_port


def make_packet(seq, ax, ay, rx, ry, buttons):
    """Buat paket joystick → STM32."""
    header = b'\xAA\x55'
    typ = 0x01
    body = struct.pack('<BBhhhhH', typ, seq, ax, ay, rx, ry, buttons)
    s = sum(body) % 65535
    chksum = struct.pack('<H', s)
    return header + body + chksum


def make_gains_packet(seq, K_TH, K_TH_D, K_X, K_X_D, K_X_INT):
    """
    Buat paket set gains → STM32.
    
    Format:
    - Header: 0xAA 0x55
    - Type: 0x02
    - Seq: 1 byte
    - Payload: 5 floats (20 bytes)
    - CRC: 2 bytes
    """
    header = b'\xAA\x55'
    typ = 0x02
    
    # Pack: type(1) + seq(1) + 5 floats(20)
    body = struct.pack('<BBfffff', 
                       typ, seq,
                       K_TH, K_TH_D, K_X, K_X_D, K_X_INT)
    
    # CRC = sum(body) % 65536
    s = sum(body) % 65536
    chksum = struct.pack('<H', s)
    
    return header + body + chksum


def send_gains(ser, K_TH, K_TH_D, K_X, K_X_D, K_X_INT, seq=0):
    """Kirim gains ke STM32 via serial."""
    packet = make_gains_packet(seq, K_TH, K_TH_D, K_X, K_X_D, K_X_INT)
    ser.write(packet)
    print(f"[TX] Gains sent: K_TH={K_TH:.2f}, K_TH_D={K_TH_D:.4f}, "
          f"K_X={K_X:.2f}, K_X_D={K_X:.2f}, K_X_INT={K_X_INT:.2f}")


def make_reset_packet(seq=0):
    """
    Buat paket reset → STM32.
    
    Format:
    - Header: 0xAA 0x55
    - Type: 0x03
    - Seq: 1 byte
    - Payload: NONE (0 bytes)
    - CRC: 2 bytes
    """
    header = b'\xAA\x55'
    typ = 0x03
    
    # Pack: type(1) + seq(1), no payload
    body = struct.pack('<BB', typ, seq)
    
    # CRC = sum(body) % 65536
    s = sum(body) % 65536
    chksum = struct.pack('<H', s)
    
    return header + body + chksum


def send_reset(ser, seq=0):
    """Kirim reset command ke STM32 via serial."""
    packet = make_reset_packet(seq)
    
    # Debug: print packet bytes
    print(f"[TX] Reset packet ({len(packet)} bytes): {packet.hex()}")
    
    ser.write(packet)
    print("[TX] Reset command sent to STM32")


def read_control_status(ser, callback=None, ack_callback=None, reset_ack_callback=None, debug: bool = False):
    """
    Thread pembaca data dari STM32.
    
    Mendukung 3 jenis packet:
    1. Control Status (0xAA 0xCC) - existing
    2. Gains ACK (0xAA 0xDD) - gains confirmation
    3. Reset ACK (0xAA 0xEE) - reset confirmation (NEW!)
    
    Format control_status: <Idddddd>
    uint32 logtick + 6x double (degree, cmX, setspeed, reserved[3])
    
    Format gains_ack: 5x float (K_TH, K_TH_D, K_X, K_X_D, K_X_INT)
    
    Format reset_ack: 1 byte status
    """
    HEADER_STATUS = b'\xAA\xCC'
    HEADER_ACK = b'\xAA\xDD'
    HEADER_RESET = b'\xAA\xEE'
    
    status_total_len = 4 + 68 + 2  # header + payload + crc = 58
    ack_total_len = 2 + 20 + 2     # header + payload + crc = 24
    reset_total_len = 2 + 1 + 2    # header + payload + crc = 5
    
    fmt_status = "<Idddddddd"   # uint32 + 7x double
    fmt_ack = "<fffff"        # 5x float

    buffer = bytearray()

    while True:
        chunk = ser.read(128)
        if not chunk:
            time.sleep(0.01)
            continue
        buffer.extend(chunk)
        #print(f"[RX] Received {len(chunk)} bytes, buffer size: {len(buffer)} bytes")

        # Process buffer
        while len(buffer) >= 4:  # minimal header length
            # Try all headers
            idx_status = buffer.find(HEADER_STATUS)
            idx_ack = buffer.find(HEADER_ACK)
            idx_reset = buffer.find(HEADER_RESET)
            
            # Pilih header yang muncul duluan
            first_idx = min([i for i in [idx_status, idx_ack, idx_reset] if i >= 0], default=-1)
            
            if first_idx < 0:
                buffer.clear()
                break
            
            # Process Control Status
            if first_idx == idx_status:
                if idx_status > 0:
                    del buffer[:idx_status]
                
                if len(buffer) < status_total_len:
                    break
                
                pkt = buffer[:status_total_len]
                del buffer[:status_total_len]
                
                # Validate CRC
                crc_recv = pkt[-2] | (pkt[-1] << 8)
                crc_calc = sum(pkt[4:-2]) & 0xFFFF
                if crc_recv != crc_calc:
                    if debug:
                        print("CRC mismatch (control_status)")
                    continue
                
                # Parse data
                data = pkt[4:-2]
                
                #print (pkt)
                #print (len(data))
                unpacked = struct.unpack(fmt_status, data)
                logtick = unpacked[0]
                degree, cmX, setspeed = unpacked[1:4]
                r1, r2, r3, r4, r5 = unpacked[4:9] #?
                #print (f"r5={r5}")
                
                if debug:
                    print(f"[RX] tick={logtick:8d} deg={degree:8.3f} "
                          f"cmX={cmX:8.3f} set={setspeed:8.3f}")
                
                if callback is not None:
                    callback((logtick, degree, cmX, setspeed, r1, r2, r3, r4, r5))
            
            # Process Gains ACK
            elif first_idx == idx_ack:
                print("ACK detected")
                if idx_ack > 0:
                    del buffer[:idx_ack]
                
                if len(buffer) < ack_total_len:
                    break
                
                pkt = buffer[:ack_total_len]
                del buffer[:ack_total_len]
                
                # Validate CRC
                crc_recv = pkt[-2] | (pkt[-1] << 8)
                crc_calc = sum(pkt[2:-2]) & 0xFFFF
                if crc_recv != crc_calc:
                    if debug:
                        print("CRC mismatch (gains_ack)")
                    continue
                
                # Parse gains
                data = pkt[2:-2]
                gains = struct.unpack(fmt_ack, data)
                K_TH, K_TH_D, K_X, K_X_D, K_X_INT = gains
                
                if debug:
                    print(f"[RX] Gains ACK: K_TH={K_TH:.2f}, K_TH_D={K_TH_D:.4f}, "
                          f"K_X={K_X:.2f}, K_X_D={K_X:.2f}, K_X_INT={K_X_INT:.2f}")
                
                if ack_callback is not None:
                    ack_callback(gains)
            
            # Process Reset ACK (NEW!)
            elif first_idx == idx_reset:
                if idx_reset > 0:
                    del buffer[:idx_reset]
                
                if len(buffer) < reset_total_len:
                    break
                
                pkt = buffer[:reset_total_len]
                del buffer[:reset_total_len]
                
                # Validate CRC
                crc_recv = pkt[-2] | (pkt[-1] << 8)
                crc_calc = sum(pkt[2:-2]) & 0xFFFF
                if crc_recv != crc_calc:
                    if debug:
                        print("CRC mismatch (reset_ack)")
                    continue
                
                # Parse status
                status = pkt[2]
                
                print(f"[RX] *** RESET ACK RECEIVED *** status={status}")
                
                if reset_ack_callback is not None:
                    reset_ack_callback(status)
            
            else:
                # No valid header found, clear garbage
                buffer.clear()
                break