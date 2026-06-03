"""
control.py — main entry point for the XGO-Mini trash collection task.
(Watchdog satisfied: Continuous 10Hz pinging with explicit turnleft/turnright routing)
"""

import json
import math
import time
import numpy as np
from xgolib import XGO
import urllib

from config import *
from network import SharedMemory, NetworkLayer
from grasp import GraspController


class Controller:
    """
    Main navigation FSM for trash collection task using RELATIVE coordinates.
    """

    IDLE              = 'IDLE'
    NAVIGATE_EXTERNAL = 'NAVIGATE_EXTERNAL'
    NAVIGATE_CAMERA   = 'NAVIGATE_CAMERA'
    GRASP             = 'GRASP'
    HOLDING           = 'HOLDING'
    NAVIGATE_DISPOSAL = 'NAVIGATE_DISPOSAL'
    DISPOSE           = 'DISPOSE'

    def __init__(self, shared_mem: SharedMemory):
        self.memory = shared_mem
        self.dog    = XGO(port='/dev/ttyAMA0', version='mini')
        self.grasp  = GraspController(self.dog)

        # Store the current target's relative distances
        self._target_dx: float = 0.0  
        self._target_dy: float = 0.0  
        self._target_dz: float = 0.0  

        self._state: str = self.IDLE
        self._loop_counter: int = 0

        self._log = open("robot_position_log.txt", "w")
        self._log.write("timestamp,dist_to_target,angle_to_target,state\n")

    def _steer(self, turn_cmd: int):
        """Routes a signed turn speed to the correct XGO hardware commands."""
        if turn_cmd > 0:
            self.dog.turnleft(turn_cmd)
        elif turn_cmd < 0:
            self.dog.turnright(abs(turn_cmd))
        else:
            # Safest way to stop turning 
            self.dog.turnleft(0)

    def _log_pos(self):
        """Logs the current distance and angle relative to the active target."""
        ts = time.time()
        dist = math.hypot(self._target_dx, self._target_dy)
        angle = math.degrees(math.atan2(self._target_dy, self._target_dx))
        self._log.write(f"{ts:.3f},{dist:.4f},{angle:.2f},{self._state}\n")
        self._log.flush()

    def _stop(self):
        """Safely halts movement with buffer protection."""
        self.dog.move_x(0)
        time.sleep(0.01)
        self._steer(0)

    def _walk_step(self, dx: float, dy: float) -> bool:
        """
        Calculates distance and angle from relative coordinates.
        Turns if angle error is high, otherwise walks forward smoothly.
        """
        distance = math.hypot(dx, dy)    

        angle_to_target = math.degrees(math.atan2(dy, dx))

        # --- THE FAILSAFE FIX: Ensure angle is small AND target is physically in front (dx > 0) ---
        if abs(angle_to_target) > STOP_AND_TURN_DEG or dx <= 0:
            # ALIGNMENT PHASE: Stop walking, spin in place
            turn_cmd = int(np.clip(
                (angle_to_target / STOP_AND_TURN_DEG) * 5, 
                -MAX_TURN_CMD, MAX_TURN_CMD
            ))
            
            min_turn = 12
            if 0 < turn_cmd < min_turn: turn_cmd = min_turn
            elif 0 > turn_cmd > -min_turn: turn_cmd = -min_turn

            print(f"[FSM] STOP+TURN  err={angle_to_target:+.1f}°  cmd={turn_cmd}")
            self.dog.move_x(0)
            time.sleep(0.01)
            self._steer(turn_cmd)
            
        else:
            # WALKING PHASE: Facing target, move forward
            
            # The Deadband Smoothing + Failsafe
            if abs(angle_to_target) < 5.0 and dx > 0:
                turn_cmd = 0  # Walk perfectly straight!
            else:
                turn_cmd = int(np.clip(
                    (angle_to_target / STOP_AND_TURN_DEG) * MAX_TURN_CMD,
                    -MAX_TURN_CMD, MAX_TURN_CMD,
                ))
            
            print(f"[FSM] WALK  dist={distance:.3f} m  err={angle_to_target:+.1f}°  cmd={turn_cmd}")
            self.dog.move_x(FORWARD_SPEED)
            time.sleep(0.01)
            self._steer(turn_cmd)

        return False    

    def _fetch_live_target(self) -> bool:
        """
        Fetches the target from memory. 
        Returns True if a valid target exists, False if valid=0 or lost.
        """
        target = self.memory.get_target()
        if target is None:
            return False
        
        self._target_dx = target['x']
        self._target_dy = target['y']
        self._target_dz = target.get('z', 0.0)
        return True



    def _fetch_live_disposal(self) -> bool:
        """
        Fetches the target from memory. 
        Returns True if a valid target exists, False if valid=0 or lost.
        """
        disposal = self.memory.get_disposal()
        if disposal is None:
            return False
        
        self._target_dx = disposal['x']
        self._target_dy = disposal['y']
        self._target_dz = disposal.get('z', 0.0)
        return True


    def _get_garbage_coords_from_server(self):
        cmd = None
        try:
            req = urllib.request.Request(REST_API_GARBAGE)
            with urllib.request.urlopen(req, timeout=REST_API_TIMEOUT) as resp:
                cmd = json.loads(resp.read().decode('utf-8'))
            
            if 'x' in cmd and 'y' in cmd and cmd.get('valid') == 1:
                self._target_dx = cmd['x']
                self._target_dy = cmd['y']
                pass
            else:
                self._stop()
                print("[FSM] No valid target (valid=0) — staying IDLE")
            return (cmd.get('valid') == 1)
        except Exception: 
            print("Can't connect to server.")
            exit(1)
            
                

    def _get_basket_coords_from_server(self):
        cmd = None
        try:
            req = urllib.request.Request(REST_API_BASKET)
            with urllib.request.urlopen(req, timeout=REST_API_TIMEOUT) as resp:
                cmd = json.loads(resp.read().decode('utf-8'))
            
            if 'x' in cmd and 'y' in cmd and cmd.get('valid') == 1:
                self._target_dx = cmd['x']
                self._target_dy = cmd['y']
                pass
            else:
                self._stop()
                print("[FSM] No valid target (valid=0) — staying IDLE")
            return (cmd.get('valid') == 1)
        except Exception: 
            print("Can't connect to server.")
            exit(1)

    def run(self):
        print("[System] Standing up...")
        self.dog.action(2)
        time.sleep(2.0)  
        
        # --- ADD THESE TWO LINES ---
        print("[System] Initializing IMU for visual scanning...")
        self.dog.imu(1)
        # ---------------------------
        
        print(f"[System] Ready. Waiting for valid relative target coordinates...")

        try:
            while True:
                loop_start = time.time()
                self._loop_counter += 1

                if self._state == self.IDLE:
                    time.sleep(LOOP_DT * 10)
                    valid = self._get_garbage_coords_from_server()
                    if valid:
                        print(f"[FSM] IDLE → NAVIGATE_EXTERNAL")
                        print(f"[FSM] Initial relative target: ({self._target_dx:.3f}, {self._target_dy:.3f})")
                        self._state = self.NAVIGATE_EXTERNAL
                elif self._state == self.NAVIGATE_EXTERNAL:
                    # SAFETY CHECK: Stop instantly if valid=0
                    if not self._get_garbage_coords_from_server():
                        self._stop()
                        if self._loop_counter % 20 == 0:
                            print("[FSM] Target lost (valid=0). Halting movement...")
                        continue
                    time.sleep(LOOP_DT)

                    distance = math.hypot(self._target_dx, self._target_dy)
                    
                    if distance < CAMERA_RANGE and self._target_dx >= 0:
                        print(f"[FSM] Distance {distance:.3f}m — stopping to check camera")
                        self._stop()
                        self.dog.translation('z', BODY_HEIGHT_CROUCH)
                        time.sleep(1.0)
                        self.dog.attitude('p', BODY_PITCH_GRASP)
                        time.sleep(0.8)

                        if self.grasp.vision.quick_detect():
                            print(f"[FSM] Object visible — NAVIGATE_EXTERNAL -> NAVIGATE_CAMERA")
                            self._state = self.NAVIGATE_CAMERA
                            continue
                        else:
                            print("[FSM] Not visible in quick check - going to GRASP full scan")
                            self._state = self.GRASP
                            continue
                    
                    arrived = self._walk_step(self._target_dx, self._target_dy)
                    
                    if arrived:
                        print(f"[FSM] Stopped before trash target via external navigation")
                        self._state = self.GRASP
                    else:
                        self._log_pos()
                        time.sleep(max(0.0, LOOP_DT - (time.time() - loop_start)))
                        continue

                elif self._state == self.NAVIGATE_CAMERA:
                    frame_data = self.grasp.vision.quick_detect_with_position()

                    if frame_data is None:
                        self._cam_lost_count = getattr(self, '_cam_lost_count', 0) + 1
                        self._stop()
                        if self._cam_lost_count >= CAM_LOST_RETRIES:
                            print(f"[FSM] Lost object — falling through to GRASP for full scan")
                            self._cam_lost_count = 0
                            self.grasp.vision.close_nav_camera()
                            self._state = self.GRASP
                        time.sleep(max(0.0, LOOP_DT - (time.time() - loop_start)))
                        continue

                    self._cam_lost_count = 0
                    cx, cy = frame_data

                    h_err   = (CAMERA_WIDTH / 2.0 - cx) / (CAMERA_WIDTH / 2.0)
                    cy_frac =  cy / CAMERA_HEIGHT

                    if cy_frac >= CAM_STOP_CY_FRAC:
                        self._stop()
                        print(f"[FSM] Object in grasp zone (cy_frac={cy_frac:.2f}) → GRASP")
                        self.grasp.vision.close_nav_camera()
                        self._state = self.GRASP
                        continue

                    turn_cmd = int(np.clip(
                        h_err * VISUAL_TURN_SPEED_MAX,
                        -VISUAL_TURN_SPEED_MAX, VISUAL_TURN_SPEED_MAX
                    ))

                    if abs(h_err) < VISUAL_SERVO_TURN_ONLY_H:
                        self.dog.move_x(CAM_APPROACH_SPEED)
                        time.sleep(0.01)
                        self._steer(turn_cmd)
                    else:
                        min_turn = 12
                        if 0 < turn_cmd < min_turn: turn_cmd = min_turn
                        elif 0 > turn_cmd > -min_turn: turn_cmd = -min_turn
                        self.dog.move_x(0)
                        time.sleep(0.01)
                        self._steer(turn_cmd)

                    self._log_pos()
                    time.sleep(max(0.0, LOOP_DT - (time.time() - loop_start)))
                    continue

                elif self._state == self.GRASP:
                    print(f"[FSM] Beginning grasp sequence")
                    success = self.grasp.execute(self._target_dz)

                    if success:
                        print(f"[FSM] GRASP → HOLDING")
                        self._state = self.HOLDING
                    else:
                        print(f"[FSM] GRASP failed (object not found in scan)")
                        print(f"[FSM] GRASP → IDLE")
                        self._state = self.IDLE

                    self.memory.clear_target()
                    time.sleep(LOOP_DT)
                    continue

                elif self._state == self.HOLDING:

                    cmd = None
                    try:
                        time.sleep(LOOP_DT * 10)
                        req = urllib.request.Request(REST_API_BASKET)
                        with urllib.request.urlopen(req, timeout=REST_API_TIMEOUT) as resp:
                            cmd = json.loads(resp.read().decode('utf-8'))
                        
                        # THE FIX: Check that x/y exist AND valid is exactly 1
                        if 'x' in cmd and 'y' in cmd and cmd.get('valid') == 1:
                            self._target_dx = cmd['x']
                            self._target_dy = cmd['y']
                        else:
                            self._stop()
                            print("[FSM] Holding trash, waiting for valid relative disposal location...")
                        
                            
                    except Exception: 
                        print("Can't connect to server.")
                        exit(1)
                    
                    
                    print(f"[FSM] Disposal location received: ({self._target_dx:.3f}, {self._target_dy:.3f})")
                    print(f"[FSM] HOLDING → NAVIGATE_DISPOSAL")
                    self._state = self.NAVIGATE_DISPOSAL

                elif self._state == self.NAVIGATE_DISPOSAL:

                    valid = self._get_basket_coords_from_server()
                    if valid:
                        print(f"[FSM] relative target: ({self._target_dx:.3f}, {self._target_dy:.3f})")
                    else:
                        self._stop()
                        if self._loop_counter % 20 == 0:
                            print("[FSM] Target lost (valid=0). Halting movement...")
                        time.sleep(LOOP_DT * 5)
                        continue

                    distance = math.hypot(self._target_dx, self._target_dy)

                    if(distance > BASKET_THRESHOLD):
                        self._walk_step(self._target_dx, self._target_dy)
                    else:
                        print(f"[FSM] Arrived at disposal location")
                        self._state = self.DISPOSE
                        self._log_pos()
                        time.sleep(LOOP_DT)
                        continue

                elif self._state == self.DISPOSE:
                    print(f"[FSM] Disposing trash...")
                    self._stop()
                    time.sleep(1.0)
                    self.dog.claw(CLAW_OPEN)
                    time.sleep(1.0)
                    self.dog.arm(ARM_HOME_X , ARM_HOME_Z)
                    time.sleep(1.0)
                    self.memory.clear_target()  
                    self._state = self.IDLE
                    time.sleep(LOOP_DT)
                    continue

                time.sleep(max(0.0, LOOP_DT - (time.time() - loop_start)))

        except KeyboardInterrupt:
            print("\n[System] Interrupted — shutting down")

        finally:
            self._stop()
            self._log.close()
            print("[System] Stopped")


if __name__ == '__main__':
    mem = SharedMemory()
    controller = Controller(mem)
    
    def get_current_state():
        return controller._state

    NetworkLayer(mem, state_callback=get_current_state).start()
    controller.run()