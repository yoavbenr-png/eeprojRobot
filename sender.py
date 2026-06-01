"""
sender.py — Computer-side script to send coordinates to XGO Mini robot.

This script runs on the computer that processes drone footage and
tracks trash/robot positions.

Features:
    - Send trash target coordinates to robot
    - Send disposal location coordinates to robot
    - Listen for status messages from robot
    - Simulate continuous coordinate updates

Usage:
    # Single target send
    python sender.py --trash 0.5 0.0 0.0
    
    # Send disposal location
    python sender.py --disposal 0.0 0.5 0.0
    
    # Continuous streaming (simulates live drone feed)
    python sender.py --stream
    
    # Listen for robot status
    python sender.py --listen
"""

import argparse
import json
import socket
import threading
import time

# Robot and computer network configuration
ROBOT_IP = '172.20.10.3'     # IP of the XGO Mini robot
ROBOT_PORT = 5000             # Port robot listens on for coordinates


class RobotCommunicator:
    """Handles bidirectional communication with the robot."""
    
    def __init__(self):
        self.status_listener = None
    
    def send_trash_coords(self, x: float, y: float, z: float = 0.0):
        """Send trash target coordinates to robot."""
        try:
            data = {
                "x": x,
                "y": y,
                "z": z
            }
            
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            message = json.dumps(data).encode('utf-8')
            sock.sendto(message, (ROBOT_IP, ROBOT_PORT))
            sock.close()
            
            print(f"[Sent] Coordinates → robot: ({x:.3f}, {y:.3f}, {z:.3f})")
            return True
            
        except Exception as e:
            print(f"[Error] Failed to send coordinates: {e}")
            return False
    
    def send_disposal_coords(self, x: float, y: float, z: float = 0.0):
        """Send disposal location coordinates to robot (when robot is HOLDING)."""
        try:
            data = {
                "x": x,
                "y": y,
                "z": z
            }
            
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            message = json.dumps(data).encode('utf-8')
            sock.sendto(message, (ROBOT_IP, ROBOT_PORT))
            sock.close()
            
            print(f"[Sent] Disposal location → robot: ({x:.3f}, {y:.3f}, {z:.3f})")
            print(f"       (Robot will interpret as disposal when HOLDING)")
            return True
            
        except Exception as e:
            print(f"[Error] Failed to send coordinates: {e}")
            return False

    
    def simulate_streaming(self, duration: float = 60.0, update_interval: float = 2.0):
        """
        Simulate continuous coordinate streaming from drone.
        
        In real implementation, this would read from your drone's
        vision processing system.
        
        Args:
            duration: Total simulation duration in seconds
            update_interval: Seconds between coordinate updates
        """
        print(f"[Streaming] Simulating live coordinate updates for {duration}s")
        print(f"[Streaming] Update interval: {update_interval}s")
        
        start_time = time.time()
        iteration = 0
        
        # Simulate trash moving slightly (e.g., wind, robot correction)
        base_x = 0.5
        base_y = 0.0
        
        while time.time() - start_time < duration:
            # Simulate small movements in trash position
            noise_x = 0.02 * (iteration % 5 - 2)  # ±0.04m variation
            noise_y = 0.01 * ((iteration % 3) - 1)  # ±0.01m variation
            
            x = base_x + noise_x
            y = base_y + noise_y
            
            self.send_trash_coords(x, y, 0.0)
            
            iteration += 1
            time.sleep(update_interval)
        
        print(f"[Streaming] Simulation complete")


def main():
    parser = argparse.ArgumentParser(
        description='Send coordinates to XGO Mini robot'
    )
    
    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument('--trash', nargs=3, type=float, metavar=('X', 'Y', 'Z'),
                      help='Send trash target coordinates (x y z)')
    group.add_argument('--disposal', nargs=3, type=float, metavar=('X', 'Y', 'Z'),
                      help='Send disposal location coordinates (x y z)')
    group.add_argument('--stream', action='store_true',
                      help='Simulate continuous coordinate streaming')
    group.add_argument('--listen', action='store_true',
                      help='Listen for robot status messages')
    
    parser.add_argument('--duration', type=float, default=60.0,
                       help='Streaming duration in seconds (default: 60)')
    parser.add_argument('--interval', type=float, default=2.0,
                       help='Streaming update interval in seconds (default: 2)')
    
    args = parser.parse_args()
    
    comm = RobotCommunicator()
    
    if args.trash:
        x, y, z = args.trash
        comm.send_trash_coords(x, y, z)
    
    elif args.disposal:
        x, y, z = args.disposal
        comm.send_disposal_coords(x, y, z)
    
    elif args.stream:
        # Start status listener in background
        comm.start_status_listener()
        # Start streaming
        comm.simulate_streaming(args.duration, args.interval)
        # Keep alive to receive status messages
        print("\n[Waiting] for robot status messages (Ctrl+C to exit)...")
        try:
            while True:
                time.sleep(1)
        except KeyboardInterrupt:
            print("\n[Exiting]")
    
    elif args.listen:
        comm.start_status_listener()
        print("\n[Waiting] for robot status messages (Ctrl+C to exit)...")
        try:
            while True:
                time.sleep(1)
        except KeyboardInterrupt:
            print("\n[Exiting]")


if __name__ == '__main__':
    # If run without arguments, show example usage
    import sys
    if len(sys.argv) == 1:
        print("=" * 70)
        print("XGO Mini Robot - Computer Communication Script")
        print("=" * 70)
        print("\nExample Usage:")
        print("-" * 70)
        print("  # Send trash at (0.5, 0.0, 0.0)")
        print("  python sender.py --trash 0.5 0.0 0.0")
        print()
        print("  # Send disposal location at (0.0, 0.5, 0.0)")
        print("  python sender.py --disposal 0.0 0.5 0.0")
        print()
        print("  # Simulate live streaming (updates every 2s for 60s)")
        print("  python sender.py --stream")
        print()
        print("  # Just listen for robot status")
        print("  python sender.py --listen")
        print()
        print("  # Custom streaming (10s duration, 1s interval)")
        print("  python sender.py --stream --duration 10 --interval 1")
        print("=" * 70)
        print()
        
    main()  