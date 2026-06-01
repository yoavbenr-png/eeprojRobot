"""
control.py — main entry point for the XGO-Mini trash collection task.

FSM states:
    IDLE              — waiting for a trash target
    NAVIGATE_EXTERNAL — following external coordinates, polling for updates
    NAVIGATE_CAMERA   — object in camera range, using vision for navigation
    GRASP             — crouch, visual scan, arm sequence
    HOLDING           — holding trash, sent completion message, waiting for disposal location
    NAVIGATE_DISPOSAL — moving to disposal location
    DISPOSE           — release trash at disposal location
    WALK_HOME         — navigate back to (0, 0) after failed grasp or disposal complete

Coordinate convention:
    Robot starts at world (0, 0) facing world +x.
    x / y in metres.  z = object height in metres (0 = ground level).
"""

import math
import time

import numpy as np
from xgolib import XGO

from config import (
    FORWARD_SPEED, FORWARD_SPEED_MS,
    CAMERA_RANGE, GRASP_THRESHOLD, STOP_BEFORE_TARGET_M,
    STOP_AND_TURN_DEG, ALIGN_THRESHOLD_DEG,
    MAX_TURN_CMD, TURN_VYAW, LOOP_DT,
    COORD_POLL_INTERVAL,
    ARM_HOME_X, ARM_HOME_Z, CLAW_OPEN,
    BODY_HEIGHT_NORMAL, BODY_HEIGHT_CROUCH, BODY_PITCH_GRASP,
    CAM_APPROACH_SPEED, CAM_APPROACH_SPEED_MS, CAM_STOP_CY_FRAC, CAM_LOST_RETRIES,
    CAMERA_WIDTH, CAMERA_HEIGHT,
    VISUAL_TURN_SPEED_MAX, VISUAL_SERVO_TURN_ONLY_H,
)
from network import SharedMemory, NetworkLayer, MessageSender
from grasp import GraspController


class Controller:
    """
    Main navigation FSM for trash collection task.
    """

    # FSM States
    IDLE              = 'IDLE'
    NAVIGATE_EXTERNAL = 'NAVIGATE_EXTERNAL'
    NAVIGATE_CAMERA   = 'NAVIGATE_CAMERA'
    GRASP             = 'GRASP'
    HOLDING           = 'HOLDING'
    NAVIGATE_DISPOSAL = 'NAVIGATE_DISPOSAL'
    DISPOSE           = 'DISPOSE'
    WALK_HOME         = 'WALK_HOME'

    def __init__(self, shared_mem: SharedMemory):
        self.memory = shared_mem
        self.dog    = XGO(port='/dev/ttyAMA0', version='mini')
        self.grasp  = GraspController(self.dog)

        self.x_est: float = 0.0
        self.y_est: float = 0.0
        self.init_yaw: float = 0.0

        self._tx: float = 0.0  
        self._ty: float = 0.0  
        self._tz: float = 0.0  
        self._dx: float = 0.0  
        self._dy: float = 0.0  
        self._dz: float = 0.0  

        self._state: str = self.IDLE
        self._last_coord_poll: float = 0.0
        self._loop_counter: int = 0

        self._log = open("robot_position_log.txt", "w")
        self._log.write("timestamp,x_est,y_est,yaw_world_deg,state\n")

    def _abs_yaw_deg(self) -> float:
        try:
            return float(self.dog.read_yaw())
        except Exception:
            return self.init_yaw

    def _world_yaw_deg(self) -> float:
        raw = self._abs_yaw_deg() - self.init_yaw
        return (raw + 180.0) % 360.0 - 180.0

    def _world_yaw_rad(self) -> float:
        return math.radians(self._world_yaw_deg())

    def _log_pos(self):
        ts  = time.time()
        yaw = self._world_yaw_deg()
        self._log.write(
            f"{ts:.3f},{self.x_est:.4f},{self.y_est:.4f},{yaw:.2f},{self._state}\n"
        )
        self._log.flush()

    def _desired_heading_deg(self, tx: float, ty: float) -> float:
        dx = tx - self.x_est
        dy = ty - self.y_est
        return math.degrees(math.atan2(dy, dx))

    def _heading_error_deg(self, desired: float) -> float:
        err = desired - self._world_yaw_deg()
        return (err + 180.0) % 360.0 - 180.0

    def _stop(self):
        self.dog.move_x(0)
        self.dog.turn(0)

    def _distance_to(self, tx: float, ty: float) -> float:
        dx = tx - self.x_est
        dy = ty - self.y_est
        return math.hypot(dx, dy)

    def _walk_step(self, tx: float, ty: float, threshold: float) -> bool:
        distance = self._distance_to(tx, ty)

        if distance < threshold:
            self._stop()
            return True     

        desired       = self._desired_heading_deg(tx, ty)
        heading_error = self._heading_error_deg(desired)

        if abs(heading_error) > STOP_AND_TURN_DEG:
            self._stop()
            print(f"[FSM] STOP+TURN  err={heading_error:+.1f}°  target={desired:.1f}°")
            self.dog.turn_to(heading_error, vyaw=TURN_VYAW, emax=ALIGN_THRESHOLD_DEG)
        else:
            turn_cmd = int(np.clip(
                heading_error / STOP_AND_TURN_DEG * MAX_TURN_CMD,
                -MAX_TURN_CMD, MAX_TURN_CMD,
            ))
            self.dog.move_x(FORWARD_SPEED)
            self.dog.turn(turn_cmd)

            step_m = FORWARD_SPEED_MS * LOOP_DT
            h_rad  = self._world_yaw_rad()
            self.x_est += step_m * math.cos(h_rad)
            self.y_est += step_m * math.sin(h_rad)

            print(f"[FSM] WALK  dist={distance:.3f} m  "
                  f"pos=({self.x_est:.3f}, {self.y_est:.3f})  "
                  f"hdg={self._world_yaw_deg():.1f}°  "
                  f"err={heading_error:+.1f}°  turn={turn_cmd:+d}")

        return False    

    def _poll_coordinates(self):
        current_time = time.time()
        
        if current_time - self._last_coord_poll < COORD_POLL_INTERVAL:
            return
        
        self._last_coord_poll = current_time
        target = self.memory.get_target()
        if target is None:
            return
        
        new_x = target['x']
        new_y = target['y']
        new_z = target.get('z', 0.0)
        
        if (abs(new_x - self._tx) > 0.01 or abs(new_y - self._ty) > 0.01 or abs(new_z - self._tz) > 0.01):
            print(f"[FSM] Trash coordinates updated: ({self._tx:.3f}, {self._ty:.3f}) → ({new_x:.3f}, {new_y:.3f})")
            self._tx = new_x
            self._ty = new_y
            self._tz = new_z

    def run(self):
        print("[System] Standing up...")
        self.dog.action(2)
        time.sleep(2.0)  

        print("[System] Enabling IMU...")
        self.dog.imu(1)
        time.sleep(1.0)  
        
        print("[System] Calibrating initial heading...")
        yaw_samples = []
        for i in range(5):
            yaw_samples.append(self._abs_yaw_deg())
            time.sleep(0.2)
        
        self.init_yaw = sum(yaw_samples) / len(yaw_samples)
        self.x_est = 0.0
        self.y_est = 0.0
        
        current_world_yaw = self._world_yaw_deg()
        print(f"[System] Position: (0.0, 0.0)")
        print(f"[System] Heading: {current_world_yaw:.1f}° (should be near 0°)")
        
        if abs(current_world_yaw) > 5.0:
            print(f"[System] WARNING: Robot heading is {current_world_yaw:.1f}°, not 0°!")
            print(f"[System] Re-adjusting reference to force heading = 0°...")
            self.init_yaw = self._abs_yaw_deg()
            print(f"[System] Heading reset to 0°")
        
        print(f"[System] Ready. Waiting for trash target coordinates...")

        try:
            while True:
                loop_start = time.time()
                self._loop_counter += 1

                if self._state == self.IDLE:
                    target = self.memory.get_target()
                    if target is None:
                        self._stop()
                        self._log_pos()
                        time.sleep(LOOP_DT)
                        continue

                    self._tx = target['x']
                    self._ty = target['y']
                    self._tz = target.get('z', 0.0)
                    self._last_coord_poll = time.time()
                    
                    print(f"[FSM] IDLE → NAVIGATE_EXTERNAL")
                    print(f"[FSM] Initial trash target: ({self._tx:.3f}, {self._ty:.3f}, {self._tz:.3f})")
                    self._state = self.NAVIGATE_EXTERNAL

                elif self._state == self.NAVIGATE_EXTERNAL:
                    self._poll_coordinates()
                    distance = self._distance_to(self._tx, self._ty)

                    if distance <= STOP_BEFORE_TARGET_M:
                        print(f"[FSM] Stopping {distance:.3f}m from target → GRASP full scan")
                        self._stop()
                        self._state = self.GRASP
                        continue
                    
                    if distance < CAMERA_RANGE:
                        print(f"[FSM] Distance {distance:.3f}m — stopping to check camera")
                        self._stop()
                        self.dog.translation('z', BODY_HEIGHT_CROUCH)
                        time.sleep(1.0)
                        self.dog.attitude('p', BODY_PITCH_GRASP)
                        time.sleep(0.8)

                        if self.grasp.vision.quick_detect():
                            print(f"[FSM] Object visible — NAVIGATE_EXTERNAL → NAVIGATE_CAMERA")
                            self.grasp.vision.open_nav_camera()
                            self._state = self.NAVIGATE_CAMERA
                            continue
                        else:
                            print("[FSM] Not visible in quick check — going to GRASP full scan")
                            self._state = self.GRASP
                            continue
                    
                    arrived = self._walk_step(self._tx, self._ty, STOP_BEFORE_TARGET_M)
                    
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
                        print(f"[FSM] Camera: object not seen ({self._cam_lost_count}/{CAM_LOST_RETRIES})")
                        if self._cam_lost_count >= CAM_LOST_RETRIES:
                            print(f"[FSM] Lost object — falling through to GRASP for full scan")
                            self._cam_lost_count = 0
                            self.grasp.vision.close_nav_camera()
                            self._state = self.GRASP
                        self._log_pos()
                        time.sleep(max(0.0, LOOP_DT - (time.time() - loop_start)))
                        continue

                    self._cam_lost_count = 0
                    cx, cy = frame_data

                    h_err   = (CAMERA_WIDTH / 2.0 - cx) / (CAMERA_WIDTH / 2.0)
                    cy_frac =  cy / CAMERA_HEIGHT

                    print(f"[FSM] Camera approach: cx={cx} cy={cy} h_err={h_err:+.2f} cy_frac={cy_frac:.2f}")

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
                        self.dog.turn(turn_cmd)
                        step_m = CAM_APPROACH_SPEED_MS * LOOP_DT
                        h_rad  = self._world_yaw_rad()
                        self.x_est += step_m * math.cos(h_rad)
                        self.y_est += step_m * math.sin(h_rad)
                    else:
                        self._stop()
                        self.dog.turn(turn_cmd)

                    self._log_pos()
                    time.sleep(max(0.0, LOOP_DT - (time.time() - loop_start)))
                    continue

                elif self._state == self.GRASP:
                    print(f"[FSM] Beginning grasp sequence at pos=({self.x_est:.3f}, {self.y_est:.3f})")
                    success = self.grasp.execute(self._tz)
                    self._log_pos()

                    if success:
                        print(f"[FSM] GRASP → HOLDING")
                        print(f"[FSM] Trash successfully collected!")
                        MessageSender.send_status(
                            status="trash_collected",
                            position={"x": self.x_est, "y": self.y_est},
                            message="Trash Collected, ready for Disposal location"
                        )
                        self._state = self.HOLDING
                    else:
                        print(f"[FSM] GRASP failed (object not found in scan)")
                        print(f"[FSM] GRASP → WALK_HOME")
                        self._state = self.WALK_HOME

                    self.memory.clear_target()
                    time.sleep(LOOP_DT)
                    continue

                elif self._state == self.HOLDING:
                    new_coords = self.memory.get_target()
                    if new_coords is None:
                        self._stop()
                        if self._loop_counter % 100 == 0:  
                            print(f"[FSM] Holding trash, waiting for disposal location...")
                        self._log_pos()
                        time.sleep(LOOP_DT)
                        continue
                    
                    self._dx = new_coords['x']
                    self._dy = new_coords['y']
                    self._dz = new_coords.get('z', 0.0)
                    
                    print(f"[FSM] Disposal location received: ({self._dx:.3f}, {self._dy:.3f}, {self._dz:.3f})")
                    print(f"[FSM] HOLDING → NAVIGATE_DISPOSAL")
                    
                    self.memory.clear_target()
                    self._state = self.NAVIGATE_DISPOSAL

                elif self._state == self.NAVIGATE_DISPOSAL:
                    arrived = self._walk_step(self._dx, self._dy, GRASP_THRESHOLD)
                    if arrived:
                        print(f"[FSM] Arrived at disposal location")
                        self._state = self.DISPOSE
                    else:
                        self._log_pos()
                        time.sleep(max(0.0, LOOP_DT - (time.time() - loop_start)))
                        continue

                elif self._state == self.DISPOSE:
                    print(f"[FSM] Disposing trash...")
                    print(f"[Dispose] Opening claw")
                    self.dog.claw(CLAW_OPEN)
                    time.sleep(1.0)
                    print(f"[Dispose] Retracting arm")
                    self.dog.arm(ARM_HOME_X, ARM_HOME_Z)
                    time.sleep(1.0)
                    print(f"[Dispose] Trash disposed successfully ✓")
                    
                    MessageSender.send_status(
                        status="disposal_complete",
                        position={"x": self.x_est, "y": self.y_est},
                        message="Trash disposed, returning to base"
                    )
                    self.memory.clear_disposal()
                    self._state = self.WALK_HOME
                    time.sleep(LOOP_DT)
                    continue

                elif self._state == self.WALK_HOME:
                    arrived = self._walk_step(0.0, 0.0, GRASP_THRESHOLD)
                    if arrived:
                        self._stop()
                        self.x_est = 0.0
                        self.y_est = 0.0
                        print(f"[FSM] Returned to origin (0, 0)")
                        self._state = self.IDLE
                    else:
                        self._log_pos()
                        time.sleep(max(0.0, LOOP_DT - (time.time() - loop_start)))
                        continue

                time.sleep(max(0.0, LOOP_DT - (time.time() - loop_start)))

        except KeyboardInterrupt:
            print("\n[System] Interrupted — shutting down")

        finally:
            self._stop()
            self.dog.imu(0)
            self._log.close()
            print("[System] Stopped")


if __name__ == '__main__':
    mem = SharedMemory()
    NetworkLayer(mem).start()
    Controller(mem).run()

    