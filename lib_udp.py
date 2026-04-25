"""
lib_udp.py - UDP Broadcaster untuk Pendulum Data

Menerima data dari STM32 via serial, lalu broadcast via UDP.
"""

import socket
import struct
import threading
from typing import Optional, Tuple


class UDPBroadcaster:
    """
    UDP broadcaster untuk pendulum control status.
    
    Mengirim data dalam format binary yang sama dengan serial protocol:
    - uint32: logtick
    - 6x double: degree, cmX, setspeed, reserved[0-2]
    Total: 4 + 48 = 52 bytes
    """
    
    def __init__(self, broadcast_ip: str = "192.168.1.255", port: int = 4000):
        """
        Initialize UDP broadcaster.
        
        Args:
            broadcast_ip: IP broadcast address (e.g., "192.168.1.255")
            port: UDP port number
        """
        self.broadcast_ip = broadcast_ip
        self.port = port
        self.enabled = False
        
        # Create UDP socket
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        
        # Stats
        self.packet_count = 0
        self.last_send_time = 0
        
        print(f"[UDP] Broadcaster initialized: {broadcast_ip}:{port}")
    
    def enable(self):
        """Enable UDP broadcasting."""
        self.enabled = True
        print("[UDP] Broadcasting ENABLED")
    
    def disable(self):
        """Disable UDP broadcasting."""
        self.enabled = False
        print("[UDP] Broadcasting DISABLED")
    
    def toggle(self):
        """Toggle broadcasting on/off."""
        self.enabled = not self.enabled
        status = "ENABLED" if self.enabled else "DISABLED"
        print(f"[UDP] Broadcasting {status}")
    
    def send_control_status(self, data_tuple: Tuple):
        """
        Send control status via UDP.
        
        Args:
            data_tuple: (logtick, degree, cmX, setspeed, r1, r2, r3)
        """
        if not self.enabled:
            return
        
        try:
            if len(data_tuple) < 7:
                return

            # Keep the UDP stream compatible with trialUDPuser.py:
            # logtick + degree + cmX + setspeed + three reserved values.
            logtick, degree, cmX, setspeed, r0, r1, r2 = data_tuple[:7]
            
            # Pack data dalam format binary (little-endian)
            # Format: <I (uint32) + 6d (6x double)
            packet = struct.pack('<Idddddd',
                                logtick, 
                                degree, 
                                cmX, 
                                setspeed, 
                                r0, r1, r2)
            
            # Broadcast ke network
            self.sock.sendto(packet, (self.broadcast_ip, self.port))
            
            self.packet_count += 1
            
            # Print stats setiap 100 packets (setiap 2 detik @ 50Hz)
            if self.packet_count % 100 == 0:
                print(f"[UDP] Sent {self.packet_count} packets to {self.broadcast_ip}:{self.port}")
        
        except Exception as e:
            print(f"[UDP] Send error: {e}")
    
    def close(self):
        """Close UDP socket."""
        self.sock.close()
        print("[UDP] Socket closed")
    
    def get_stats(self) -> dict:
        """Get broadcaster statistics."""
        return {
            "enabled": self.enabled,
            "packet_count": self.packet_count,
            "broadcast_ip": self.broadcast_ip,
            "port": self.port
        }
