"""
vision.py — camera-based object detection and pre-grasp alignment.
"""

import time
from typing import Optional

import cv2
import numpy as np

from config import (
    CAMERA_INDEX, CAMERA_WIDTH, CAMERA_HEIGHT,
    CAMERA_HFOV_DEG,
    OBJ_HSV_LOWER, OBJ_HSV_UPPER, OBJ_MIN_AREA,
    VISUAL_SCAN_STEP_DEG, VISUAL_SCAN_MAX_DEG,
    VISUAL_SCAN_LOOK_FRAMES, VISUAL_SCAN_FRAME_SLEEP,
    VISUAL_SCAN_PASSES, VISUAL_REACQUIRE_ANGLE_DEG, VISUAL_REACQUIRE_TRIES,
    VISUAL_BACKUP_STEPS, VISUAL_BACKUP_SPEED, VISUAL_BACKUP_TIME,
    VISUAL_MIN_GRASP_AREA, VISUAL_SIZE_APPROACH_STEPS, VISUAL_SIZE_STEP_SPEED,
    VISUAL_SIZE_STEP_TIME, VISUAL_SIZE_MAX_APPROACHES,
    VISUAL_H_TOLERANCE, VISUAL_CY_TARGET_FRAC, VISUAL_CY_TOLERANCE,
    VISUAL_SERVO_TURN_ONLY_H, VISUAL_TURN_SPEED_MAX, VISUAL_APPROACH_SPEED,
    VISUAL_SERVO_ITER, VISUAL_SERVO_DT,
    QUICK_DETECT_FRAMES, QUICK_DETECT_SLEEP,
)


class VisionController:

    def __init__(self, dog):
        self.dog = dog
        self._nav_cap = None   
        self._last_detection = None  

    # ------------------------------------------------------------------ helpers

    def _stop(self):
        self.dog.move_x(0)
        self.dog.turn(0)

    def _raw_yaw(self) -> float:
        try:
            return float(self.dog.read_yaw())
        except Exception:
            return 0.0

    def _turn_to_yaw(self, target: float):
        for _ in range(60):
            err = (target - self._raw_yaw() + 180.0) % 360.0 - 180.0
            if abs(err) < 3.0:
                break
            speed = int(np.clip(err * 0.8, -18.0, 18.0))
            self.dog.turn(speed)
            time.sleep(0.10)
        self._stop()
        time.sleep(0.15)

    def _warmup(self, cap, duration=1.5):
        print(f"[Vision] Warming up camera sensor for {duration}s...")
        t_end = time.time() + duration
        while time.time() < t_end:
            cap.grab()

    # ------------------------------------------------------------------ detection

    def _detect(self, frame) -> Optional[tuple]:
        h     = frame.shape[0]
        roi   = frame[h // 4:, :]
        y_off = h // 4

        hsv  = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv,
                           np.array(OBJ_HSV_LOWER),
                           np.array(OBJ_HSV_UPPER))

        kernel = np.ones((5, 5), np.uint8)
        mask   = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  kernel)
        mask   = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        contours, _ = cv2.findContours(mask,
                                       cv2.RETR_EXTERNAL,
                                       cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return None

        largest = max(contours, key=cv2.contourArea)
        area    = cv2.contourArea(largest)
        if area < OBJ_MIN_AREA:
            return None

        M = cv2.moments(largest)
        if M['m00'] == 0:
            return None

        cx = int(M['m10'] / M['m00'])
        cy = int(M['m01'] / M['m00']) + y_off
        return cx, cy, area

    def _grab(self, cap) -> Optional[tuple]:
        ret, frame = cap.read()
        if not ret:
            return None
        return self._detect(frame)

    def _grab_fresh(self, cap) -> Optional[tuple]:
        for _ in range(4):      
            cap.grab()
        ret, frame = cap.retrieve()
        if not ret:
            return None
        return self._detect(frame)

    def _look(self, cap) -> Optional[tuple]:
        detections = []
        for _ in range(VISUAL_SCAN_LOOK_FRAMES):
            det = self._grab_fresh(cap)
            if det is not None:
                detections.append(det)
            time.sleep(VISUAL_SCAN_FRAME_SLEEP)

        if len(detections) < 2:
            return None

        detections.sort(key=lambda d: (d[1], d[2]), reverse=True)
        self._last_detection = detections[0]
        return detections[0]

    # ------------------------------------------------------------------ quick detection

    def quick_detect(self) -> bool:
        cap = cv2.VideoCapture(CAMERA_INDEX)
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH,  CAMERA_WIDTH)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, CAMERA_HEIGHT)

        if not cap.isOpened():
            return False

        self._warmup(cap, duration=1.0)

        detected = False
        try:
            for _ in range(QUICK_DETECT_FRAMES):
                det = self._grab(cap)
                if det is not None:
                    detected = True
                    break
                time.sleep(QUICK_DETECT_SLEEP)
        finally:
            cap.release()

        return detected

    def open_nav_camera(self):
        if self._nav_cap is not None:
            return
        self._nav_cap = cv2.VideoCapture(CAMERA_INDEX)
        self._nav_cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        self._nav_cap.set(cv2.CAP_PROP_FRAME_WIDTH,  CAMERA_WIDTH)
        self._nav_cap.set(cv2.CAP_PROP_FRAME_HEIGHT, CAMERA_HEIGHT)
        
        self._warmup(self._nav_cap, duration=1.5)
        print("[Vision] Nav camera ready")

    def close_nav_camera(self):
        if self._nav_cap is not None:
            self._nav_cap.release()
            self._nav_cap = None

    def quick_detect_with_position(self) -> Optional[tuple]:
        if self._nav_cap is None or not self._nav_cap.isOpened():
            return None
        det = self._grab_fresh(self._nav_cap)
        if det is None:
            return None
        self._last_detection = det
        cx, cy, _ = det
        return (cx, cy)

    # ------------------------------------------------------------------ Phase 1: SCAN

    def _sweep(self, cap, start_yaw: float, direction: int) -> bool:
        steps = VISUAL_SCAN_MAX_DEG // VISUAL_SCAN_STEP_DEG
        for i in range(1, steps + 1):
            angle    = i * VISUAL_SCAN_STEP_DEG
            dest_yaw = start_yaw + direction * angle

            label = 'left' if direction > 0 else 'right'
            print(f"[Vision] Sweep {label}  "
                  f"step {i}/{steps}  angle={direction * angle:+d}°")
            self._turn_to_yaw(dest_yaw)

            det = self._look(cap)
            if det is not None:
                print(f"[Vision] Object found at {direction * angle:+d}° ({label}) ✓")
                return True
        return False

    def _scan(self, cap) -> bool:
        start_yaw = self._raw_yaw()

        for scan_pass in range(1, VISUAL_SCAN_PASSES + 1):
            print(f"[Vision] Scan pass {scan_pass}/{VISUAL_SCAN_PASSES}: straight ahead")
            self._turn_to_yaw(start_yaw)

            if self._look(cap) is not None:
                print(f"[Vision] Object found straight ahead on pass {scan_pass} ✓")
                return True

            # ── NEW: BACKUP MANEUVER TO CLEAR BLIND SPOT ──
            if scan_pass == 1:
                print(f"[Vision] Target not seen straight ahead. Backing up {VISUAL_BACKUP_STEPS} steps to clear blind spot...")
                self.dog.turn(0)
                for b_step in range(1, VISUAL_BACKUP_STEPS + 1):
                    self.dog.move_x(VISUAL_BACKUP_SPEED)
                    time.sleep(VISUAL_BACKUP_TIME)
                    self.dog.move_x(0)
                    time.sleep(0.20)
                
                # Check straight ahead one more time after backing up
                if self._look(cap) is not None:
                    print("[Vision] Object found straight ahead after backing up ✓")
                    return True
            # ──────────────────────────────────────────────

            directions = (-1, +1) if scan_pass % 2 == 1 else (+1, -1)
            for direction in directions:
                label = 'left' if direction > 0 else 'right'
                print(f"[Vision] Scan pass {scan_pass}: sweeping {label}")
                if self._sweep(cap, start_yaw, direction=direction):
                    return True

                print(f"[Vision] Scan pass {scan_pass}: returning to centre")
                self._turn_to_yaw(start_yaw)
                time.sleep(0.20)

        print("[Vision] Object not found after all scan passes")
        self._turn_to_yaw(start_yaw)
        return False

    def _reacquire_nearby(self, cap) -> bool:
        base_yaw = self._raw_yaw()
        offsets = [0]
        for k in range(1, VISUAL_REACQUIRE_TRIES + 1):
            offsets.extend([+k * VISUAL_REACQUIRE_ANGLE_DEG,
                            -k * VISUAL_REACQUIRE_ANGLE_DEG])

        for offset in offsets:
            print(f"[Vision] Reacquire: checking offset {offset:+.0f}°")
            self._turn_to_yaw(base_yaw + offset)
            if self._look(cap) is not None:
                print(f"[Vision] Reacquired object at offset {offset:+.0f}° ✓")
                return True

        self._turn_to_yaw(base_yaw)
        return False

    def _object_big_enough_for_pickup(self) -> bool:
        if self._last_detection is None:
            return False

        cx, cy, area = self._last_detection
        ok = area >= VISUAL_MIN_GRASP_AREA
        print(f"[Vision] Size gate: area={area:.0f} px²  "
              f"required>={VISUAL_MIN_GRASP_AREA} px²  "
              f"cx={cx} cy={cy}  {'✓ CLOSE ENOUGH' if ok else 'too small / too far'}")
        return ok

    def _approach_two_steps_for_size(self):
        print(f"[Vision] Object is too small — moving closer "
              f"{VISUAL_SIZE_APPROACH_STEPS} short steps")
        self.dog.turn(0)
        for step in range(1, VISUAL_SIZE_APPROACH_STEPS + 1):
            self.dog.move_x(VISUAL_SIZE_STEP_SPEED)
            time.sleep(VISUAL_SIZE_STEP_TIME)
            self.dog.move_x(0)
            time.sleep(0.20)
        self._stop()
        time.sleep(0.35)

    # ------------------------------------------------------------------ Phase 2: SERVO

    def _servo(self, cap) -> bool:
        cy_target = VISUAL_CY_TARGET_FRAC * CAMERA_HEIGHT
        frame_cx  = CAMERA_WIDTH / 2.0

        for step in range(VISUAL_SERVO_ITER):
            det = self._grab_fresh(cap)

            if det is None:
                self._stop()
                time.sleep(0.15)
                det = self._grab_fresh(cap)
                if det is None:
                    print("[Vision] Servo: object temporarily lost — trying local reacquire")
                    if self._reacquire_nearby(cap):
                        continue

                    print("[Vision] Servo: local reacquire failed — doing full scan again")
                    if self._scan(cap):
                        continue

                    return False

            self._last_detection = det
            cx, cy, area = det
            h_err  = (frame_cx - cx)  / frame_cx
            cy_err = (cy - cy_target) / CAMERA_HEIGHT
            
            h_aligned = abs(h_err) <= VISUAL_H_TOLERANCE
            dist_aligned = (cy_err >= -VISUAL_CY_TOLERANCE)

            converged = h_aligned and dist_aligned

            if converged:
                self._stop()
                print(f"[Vision] Servo {step:02d}  cx={cx}  cy={cy:.0f}  "
                      f"h_err={h_err:+.3f}  cy_err={cy_err:+.3f}  ✓ CONVERGED")
                return True

            if abs(h_err) > VISUAL_SERVO_TURN_ONLY_H:
                turn_cmd = int(np.clip(h_err * VISUAL_TURN_SPEED_MAX, -VISUAL_TURN_SPEED_MAX, VISUAL_TURN_SPEED_MAX))
                min_turn = 8  
                if 0 < turn_cmd < min_turn: turn_cmd = min_turn
                elif 0 > turn_cmd > -min_turn: turn_cmd = -min_turn

                print(f"[Vision] Servo {step:02d} [SLOW-TURN] cx={cx} cy={cy:.0f} h_err={h_err:+.3f} turn={turn_cmd:+d}")
                self.dog.move_x(0)
                self.dog.turn(turn_cmd)
                time.sleep(VISUAL_SERVO_DT)
                continue

            turn_cmd = int(np.clip(h_err * VISUAL_TURN_SPEED_MAX, -VISUAL_TURN_SPEED_MAX, VISUAL_TURN_SPEED_MAX))
            if abs(h_err) > VISUAL_H_TOLERANCE:
                min_turn = 8
                if 0 < turn_cmd < min_turn: turn_cmd = min_turn
                elif 0 > turn_cmd > -min_turn: turn_cmd = -min_turn

            fwd_cmd = (int(-np.sign(cy_err) * VISUAL_APPROACH_SPEED) if abs(cy_err) > VISUAL_CY_TOLERANCE else 0)
            fwd_cmd = max(0, fwd_cmd) # Prevent moving backward

            print(f"[Vision] Servo {step:02d} [COMBINED] cx={cx} cy={cy:.0f} h_err={h_err:+.3f} cy_err={cy_err:+.3f} turn={turn_cmd:+d} fwd={fwd_cmd:+d}")
            self.dog.move_x(fwd_cmd)
            self.dog.turn(turn_cmd)
            time.sleep(VISUAL_SERVO_DT)

        self._stop()
        return True

    # ------------------------------------------------------------------ public

    def scan_and_align(self) -> bool:
        cap = cv2.VideoCapture(CAMERA_INDEX)
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)         
        cap.set(cv2.CAP_PROP_FRAME_WIDTH,  CAMERA_WIDTH)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, CAMERA_HEIGHT)

        if not cap.isOpened():
            return False

        print("[Vision] === scan_and_align start ===")
        self._warmup(cap, duration=1.5)
        found = False

        try:
            for size_attempt in range(VISUAL_SIZE_MAX_APPROACHES + 1):
                if not self._scan(cap):
                    return False        

                found = self._servo(cap)
                if not found:
                    continue

                if self._object_big_enough_for_pickup():
                    break

                if size_attempt >= VISUAL_SIZE_MAX_APPROACHES:
                    found = False
                    break

                self._approach_two_steps_for_size()
        finally:
            cap.release()
            self._stop()

        print("[Vision] === scan_and_align complete ===")
        return found