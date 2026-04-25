"""
udp_receiver.py - UDP Receiver dengan Auto-Save ke CSV

Menerima data pendulum via UDP broadcast dan otomatis save ke CSV.
"""

import socket
import struct
import csv
import os
import time
from datetime import datetime


class UDPReceiver:
    """
    UDP receiver untuk pendulum control status.
    Automatically saves received data to CSV file.
    """
    
    def __init__(self, port: int = 4000, log_dir: str = "udp_logs"):
        """
        Initialize UDP receiver.
        
        Args:
            port: UDP port to listen on
            log_dir: Directory untuk save CSV files
        """
        self.port = port
        self.log_dir = log_dir
        
        # Create log directory
        os.makedirs(self.log_dir, exist_ok=True)
        
        # Create UDP socket
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(('', self.port))
        
        # Stats
        self.packet_count = 0
        self.error_count = 0
        self.start_time = time.time()
        
        # CSV file
        self.csv_file = None
        self.csv_writer = None
        self._open_csv_file()
        
        print("="*60)
        print("UDP RECEIVER - Pendulum Data Logger")
        print("="*60)
        print(f"Listening on: 0.0.0.0:{self.port}")
        print(f"Log directory: {os.path.abspath(self.log_dir)}")
        print(f"CSV file: {self.csv_filename}")
        print("="*60)
        print("Press Ctrl+C to stop\n")
    
    def _open_csv_file(self):
        """Create and open new CSV file with timestamp."""
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.csv_filename = f"udp_log_{timestamp}.csv"
        csv_path = os.path.join(self.log_dir, self.csv_filename)
        
        self.csv_file = open(csv_path, 'w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        
        # Write header
        self.csv_writer.writerow([
            "timestamp",
            "logtick",
            "degree",
            "cmX",
            "setspeed",
            "reserved1",
            "reserved2",
            "reserved3"
        ])
        
        self.csv_file.flush()
    
    def _parse_packet(self, data):
        """
        Parse binary packet.
        
        Format: <Idddddd (uint32 + 6x double = 52 bytes)
        
        Returns:
            tuple or None if parse error
        """
        if len(data) != 52:
            return None
        
        try:
            # Unpack: uint32 + 6 doubles
            unpacked = struct.unpack('<Idddddd', data)
            logtick = unpacked[0]
            degree = unpacked[1]
            cmX = unpacked[2]
            setspeed = unpacked[3]
            r1, r2, r3 = unpacked[4:7]
            
            return (logtick, degree, cmX, setspeed, r1, r2, r3)
        
        except struct.error:
            return None
    
    def _log_to_csv(self, data_tuple):
        """Write data to CSV file."""
        timestamp = time.time()
        row = [timestamp] + list(data_tuple)
        self.csv_writer.writerow(row)
        
        # Flush every 50 packets (~1 second @ 50Hz)
        if self.packet_count % 50 == 0:
            self.csv_file.flush()
    
    def _print_stats(self):
        """Print statistics."""
        elapsed = time.time() - self.start_time
        rate = self.packet_count / elapsed if elapsed > 0 else 0
        
        print(f"\r[Stats] Packets: {self.packet_count:6d} | "
              f"Errors: {self.error_count:4d} | "
              f"Rate: {rate:5.1f} Hz | "
              f"Time: {elapsed:6.1f}s", end='', flush=True)
    
    def run(self):
        """Main receive loop."""
        try:
            while True:
                # Receive packet
                data, addr = self.sock.recvfrom(1024)
                
                # Parse packet
                parsed = self._parse_packet(data)
                
                if parsed is not None:
                    logtick, degree, cmX, setspeed, r1, r2, r3 = parsed
                    
                    # Log to CSV
                    self._log_to_csv(parsed)
                    
                    self.packet_count += 1
                    
                    # Print stats every 10 packets
                    if self.packet_count % 10 == 0:
                        self._print_stats()
                    
                    # Print detail setiap 100 packets
                    if self.packet_count % 100 == 0:
                        print(f"\n[Data] tick={logtick:8d} deg={degree:7.2f} "
                              f"cmX={cmX:6.1f} speed={setspeed:7.0f}")
                
                else:
                    self.error_count += 1
                    if self.error_count % 10 == 0:
                        print(f"\n[WARNING] Parse error (count: {self.error_count})")
        
        except KeyboardInterrupt:
            print("\n\n" + "="*60)
            print("STOPPING...")
            print("="*60)
            self._print_final_stats()
            self._cleanup()
    
    def _print_final_stats(self):
        """Print final statistics."""
        elapsed = time.time() - self.start_time
        rate = self.packet_count / elapsed if elapsed > 0 else 0
        
        print(f"\nFinal Statistics:")
        print(f"  Total packets: {self.packet_count}")
        print(f"  Parse errors:  {self.error_count}")
        print(f"  Duration:      {elapsed:.1f} seconds")
        print(f"  Average rate:  {rate:.1f} Hz")
        print(f"\nData saved to: {os.path.abspath(os.path.join(self.log_dir, self.csv_filename))}")
    
    def _cleanup(self):
        """Close files and socket."""
        if self.csv_file:
            self.csv_file.flush()
            self.csv_file.close()
            print("CSV file closed.")
        
        self.sock.close()
        print("Socket closed.")
        print("\nDone!")


# ============================================================
# MAIN
# ============================================================

def main():
    # Configuration
    UDP_PORT = 5000
    LOG_DIR = "udp_logs"
    
    # Create receiver
    receiver = UDPReceiver(port=UDP_PORT, log_dir=LOG_DIR)
    
    # Start receiving
    receiver.run()


if __name__ == "__main__":
    main()
