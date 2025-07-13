#!/usr/bin/env python3
"""
Serial Monitor for Haptic Glove
Test communication with Arduino firmware
"""

import serial
import time
import struct
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

class GloveMonitor:
    def __init__(self, port='COM3', baud=115200):
        self.serial = serial.Serial(port, baud, timeout=0.1)
        self.data_history = []
        
    def read_data_packet(self):
        """Read binary data packet from Arduino"""
        if self.serial.in_waiting >= 32:  # Size of GloveDataPacket
            data = self.serial.read(32)
            # Parse binary packet
            timestamp, qw, qx, qy, qz, ax, ay, az, f0, f1, f2, f3, f4, status, checksum = \
                struct.unpack('<Ifffffff5fBB', data)
            return {
                'timestamp': timestamp,
                'quaternion': [qw, qx, qy, qz],
                'acceleration': [ax, ay, az],
                'fingers': [f0, f1, f2, f3, f4],
                'status': status
            }
        return None
        
    def send_haptic_command(self, intensity):
        """Send haptic feedback command"""
        command = f"HAPTIC:{intensity}\n"
        self.serial.write(command.encode())
        
    def test_communication(self):
        """Test basic communication"""
        print("Testing communication...")
        self.serial.write(b"STATUS\n")
        time.sleep(0.1)
        response = self.serial.readline().decode().strip()
        print(f"Status response: {response}")
        
if __name__ == "__main__":
    monitor = GloveMonitor()
    monitor.test_communication()
    
    # Real-time data monitoring
    print("Monitoring data (Ctrl+C to stop)...")
    try:
        while True:
            packet = monitor.read_data_packet()
            if packet:
                print(f"Fingers: {packet['fingers']}")
                print(f"Quaternion: {packet['quaternion']}")
            time.sleep(0.01)
    except KeyboardInterrupt:
        print("Monitoring stopped")