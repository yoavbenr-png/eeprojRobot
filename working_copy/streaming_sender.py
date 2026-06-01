"""
streaming_sender.py — Send continuous position updates to the robot.

This script demonstrates how to stream position corrections to the robot
from an external tracking system (e.g., overhead camera, motion capture).

Usage:
    1. Start the robot: python control.py
    2. Send a target: python sender.py  (or use send_target() below)
    3. Start streaming: python streaming_sender.py

The robot will use the streamed positions to correct accumulated 
dead-reckoning error during navigation.
"""

import socket
import json
import time
import math

# Robot IP and port
ROBOT_IP = '10.182.211.35'  # Change to your robot's IP
PORT = 5000

# Simulated tracking parameters (replace with your real tracking system)
STREAM_RATE_HZ = 1.0  # Send position updates every second
SIMULATE_ROBOT_MOVEMENT = True  # For testing: simulate robot moving toward target


def send_target(x: float, y: float, z: float = 0.0):
    """Send a navigation target to the robot."""
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
        data = {"type": "target", "x": x, "y": y, "z": z}
        message = json.dumps(data).encode('utf-8')
        s.sendto(message, (ROBOT_IP, PORT))
        print(f"[Target] Sent: {data}")


def send_position_update(x: float, y: float, yaw: float = None):
    """
    Send a position update to the robot.
    
    Args:
        x: Robot's x position in metres (from your tracking system)
        y: Robot's y position in metres (from your tracking system)
        yaw: Robot's heading in degrees (optional, from your tracking system)
    """
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
        data = {"type": "position", "x": x, "y": y}
        if yaw is not None:
            data["yaw"] = yaw
            
        message = json.dumps(data).encode('utf-8')
        s.sendto(message, (ROBOT_IP, PORT))
        print(f"[Position] Sent: x={x:.3f}, y={y:.3f}" + 
              (f", yaw={yaw:.1f}°" if yaw is not None else ""))


def simulate_tracking_system(target_x: float, target_y: float, duration_sec: float = 30):
    """
    Simulate an external tracking system that monitors robot position.
    
    In reality, replace this with your actual tracking system:
    - Overhead camera with object detection
    - Motion capture system (OptiTrack, Vicon)
    - Visual odometry from external camera
    - UWB positioning system
    - etc.
    """
    print("\n" + "="*70)
    print("SIMULATED TRACKING SYSTEM")
    print("="*70)
    print(f"Target: ({target_x:.3f}, {target_y:.3f})")
    print(f"Stream rate: {STREAM_RATE_HZ:.1f} Hz")
    print(f"Duration: {duration_sec:.0f} seconds")
    print("\nReplace this simulation with your real tracking system!")
    print("="*70 + "\n")
    
    # Simulate robot starting at origin
    robot_x = 0.0
    robot_y = 0.0
    robot_yaw = 0.0  # degrees
    
    start_time = time.time()
    iteration = 0
    
    try:
        while time.time() - start_time < duration_sec:
            iteration += 1
            
            if SIMULATE_ROBOT_MOVEMENT:
                # Simulate robot moving toward target at ~0.1 m/s
                dx = target_x - robot_x
                dy = target_y - robot_y
                distance = math.hypot(dx, dy)
                
                if distance > 0.01:  # Not at target yet
                    # Simulate movement
                    speed = 0.10 / STREAM_RATE_HZ  # 0.1 m/s
                    robot_x += (dx / distance) * speed
                    robot_y += (dy / distance) * speed
                    robot_yaw = math.degrees(math.atan2(dy, dx))
                    
                    # Add small noise to simulate tracking system error
                    noise_x = (hash((iteration, 'x')) % 100 - 50) / 5000.0  # ±1cm
                    noise_y = (hash((iteration, 'y')) % 100 - 50) / 5000.0  # ±1cm
                    
                    tracked_x = robot_x + noise_x
                    tracked_y = robot_y + noise_y
                else:
                    tracked_x = robot_x
                    tracked_y = robot_y
            else:
                # No movement simulation - send static position
                tracked_x = robot_x
                tracked_y = robot_y
            
            # Send position update to robot
            send_position_update(tracked_x, tracked_y, robot_yaw)
            
            # Wait for next update
            time.sleep(1.0 / STREAM_RATE_HZ)
            
    except KeyboardInterrupt:
        print("\n[Streaming] Stopped by user")


def stream_from_real_tracking():
    """
    Template for integrating with a real tracking system.
    
    Replace the simulated tracking with your actual system here.
    """
    print("\n" + "="*70)
    print("REAL TRACKING SYSTEM INTEGRATION")
    print("="*70)
    print("Implement your tracking system here:")
    print("  - Read robot position from your camera/sensor")
    print("  - Call send_position_update(x, y, yaw) with the tracked position")
    print("  - Repeat at your desired rate (recommended: 1-10 Hz)")
    print("="*70 + "\n")
    
    # Example integration skeleton:
    # import your_tracking_system
    # tracker = your_tracking_system.RobotTracker()
    # 
    # while True:
    #     position = tracker.get_robot_position()  # Your tracking API
    #     send_position_update(
    #         x=position.x,
    #         y=position.y,
    #         yaw=position.heading
    #     )
    #     time.sleep(1.0 / STREAM_RATE_HZ)


# ──────────────────────────────────────────────────────────────────────
# Usage Examples
# ──────────────────────────────────────────────────────────────────────

if __name__ == '__main__':
    print("""
╔══════════════════════════════════════════════════════════════════╗
║           XGO MINI - POSITION STREAMING SENDER                   ║
╚══════════════════════════════════════════════════════════════════╝

This script sends continuous position updates to the robot to reduce
dead-reckoning error during navigation.

USAGE MODES:
  1. Simulated tracking (for testing)
  2. Real tracking system integration

""")
    
    mode = input("Choose mode:\n  1 = Simulated tracking\n  2 = Real tracking\n\nMode: ").strip()
    
    if mode == '1':
        # Send initial target
        print("\n[Setup] Sending target to robot...")
        send_target(x=0.5, y=0.0, z=0.0)
        time.sleep(1.0)
        
        # Start simulated streaming
        simulate_tracking_system(
            target_x=0.5,
            target_y=0.0,
            duration_sec=60  # Stream for 60 seconds
        )
        
    elif mode == '2':
        print("\n[Info] Real tracking mode selected.")
        print("[Info] Edit stream_from_real_tracking() to integrate your system.\n")
        stream_from_real_tracking()
        
    else:
        print("Invalid mode. Exiting.")