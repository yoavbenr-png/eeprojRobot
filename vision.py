"""
vision.py — camera-based object detection and pre-grasp alignment.
"""

import time
from typing import Optional
import math
import cv2
import numpy as np

from config import *


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

    def _turn_to_yaw(self, target: float , offset=0):
        for _ in range(60):
            err = (target - self._raw_yaw() + 180.0) % 360.0 - 180.0
            if abs(err) < 3.0:
                break
            speed = int(np.clip(err * 0.8, -18.0, 18.0))
            self.dog.turn(speed)
            if(offset == 10 or offset == -10):
                self.dog.move_x(-4)
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
        
        # ── FIX: AGGRESSIVE HORIZON CROP ──
        # Ignore the top 60% of the camera frame (walls/background)
        # Only look at the bottom 40% (the floor)
        crop_line = int(h * 0.60)
        roi   = frame[crop_line:, :]
        y_off = crop_line
        # ──────────────────────────────────

        hsv  = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        
        # Initialize an entirely black mask of the same dimensions
        combined_mask = np.zeros(hsv.shape[:2], dtype=np.uint8)

        # Loop through our target colors and combine them into one master mask
        for color in TARGET_COLORS:
            lower = np.array(color["lower"])
            upper = np.array(color["upper"])
            mask = cv2.inRange(hsv, lower, upper)
            # Add this color's mask to the combined mask
            combined_mask = cv2.bitwise_or(combined_mask, mask)

        # Apply morphology to clean up noise on the combined mask
        kernel = np.ones((5, 5), np.uint8)
        mask   = cv2.morphologyEx(combined_mask, cv2.MORPH_OPEN,  kernel)

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

    def _analyze_existing_frame(self, cap) -> Optional[tuple]:
        # Takes an exiting image from the camera buffer, then analyze it with _detect()
        ret, frame = cap.read()
        if not ret:
            return None
        return self._detect(frame)

    def _analyze_new_frame(self, cap) -> Optional[tuple]:
        # Takes a new image from the camera, then analyze it with _detect()
        for _ in range(4):      
            cap.grab()
        ret, frame = cap.retrieve()
        if not ret:
            return None
        return self._detect(frame)

    def _look(self, cap) -> Optional[tuple]:
        detections = []
        for _ in range(VISUAL_SCAN_LOOK_FRAMES):
            det = self._analyze_new_frame(cap)
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
        self.open_nav_camera()
        detected = False
        try:
            for _ in range(QUICK_DETECT_FRAMES):
                det = self._analyze_existing_frame(self._nav_cap)
                if det is not None:
                    detected = True
                    break
                time.sleep(QUICK_DETECT_SLEEP)
        except Exception as e:
            print(f"[Vision] Error occurred: {e}")

        return detected

    def open_nav_camera(self):
        # Opens the navigation camera, then returns if successful.
        if self._nav_cap is None:
            self._nav_cap = cv2.VideoCapture(CAMERA_INDEX)
        elif self._nav_cap.isOpened():
            return False
        
        self._nav_cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        self._nav_cap.set(cv2.CAP_PROP_FRAME_WIDTH,  CAMERA_WIDTH)
        self._nav_cap.set(cv2.CAP_PROP_FRAME_HEIGHT, CAMERA_HEIGHT)
        
        self._warmup(self._nav_cap, duration=1.0)
        print("[Vision] Nav camera ready")
        return True

    def close_nav_camera(self):
        if self._nav_cap is not None:
            self._nav_cap.release()
            self._nav_cap = None

    def quick_detect_with_position(self) -> Optional[tuple]:
        
        if self._nav_cap is None or not self._nav_cap.isOpened():
            print("I'm stuck here(3)")
            return None
        det = self._analyze_new_frame(self._nav_cap)
        print("I'm stuck here(4)")
        if det is None:
            print("I'm stuck here(5)")
            return None
        self._last_detection = det
        cx, cy, _ = det
        return (cx, cy)

    # ------------------------------------------------------------------ Phase 1: SCAN

    def _reacquire_nearby(self, cap) -> bool:
        base_yaw = self._raw_yaw()
        offsets = [0]
        for k in range(1, VISUAL_REACQUIRE_TRIES + 1):
            offsets.extend([+k * VISUAL_REACQUIRE_ANGLE_DEG,
                            -k * VISUAL_REACQUIRE_ANGLE_DEG])

        for offset in offsets:
            print(f"[Vision] Reacquire: checking offset {offset:+.0f}°")
            self._turn_to_yaw(base_yaw + offset, offset)
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

    def clamp_small_value(self, val, min_threshold=1.0):
        # Check if the magnitude is strictly between 0 and the threshold
        if 0 < abs(val) < min_threshold:
            return math.copysign(min_threshold, val)
        return val

    # ------------------------------------------------------------------ Phase 2: SERVO

    def _servo(self, cap) -> bool:
        cy_target = VISUAL_CY_TARGET_FRAC * CAMERA_HEIGHT

        # If it grabs to the right, use a POSITIVE number to shift the body left (e.g., 15)
        # If it grabs to the left, use a NEGATIVE number to shift the body right (e.g., -15)
        parallax_offset = 15
        frame_cx  = (CAMERA_WIDTH / 2.0) + parallax_offset
        # ──────────────────────────

        for step in range(VISUAL_SERVO_ITER):
            det = self._analyze_new_frame(cap)

            if det is None:
                self._stop()
                det = self._analyze_new_frame(cap)
                if det is None:
                    print("[Vision] Servo: object temporarily lost — trying local reacquire")
                    if self._reacquire_nearby(cap):
                        continue
                    return False

            self._last_detection = det
            cx, cy, _ = det
            h_err = (frame_cx - cx)  / frame_cx
            cy_frac = cy / CAMERA_HEIGHT
            cy_err = (cy - cy_target) / CAMERA_HEIGHT
            
            h_aligned = abs(h_err) <= VISUAL_H_TOLERANCE

            if h_aligned:
                self._stop()
                print(f"[Vision] Servo {step:02d}  cx={cx}  cy={cy:.0f}  "
                      f"cy_frac={cy_frac:.3f} > {VISUAL_CY_GRASP_MAX_FRAC:.2f}  "
                      f"h_err={h_err:+.3f}  ✓ H_ALIGNED")
                return True

            turn_cmd = int(np.clip(h_err * VISUAL_TURN_SPEED_MAX, -VISUAL_TURN_SPEED_MAX, VISUAL_TURN_SPEED_MAX))

            if abs(h_err) > VISUAL_SERVO_TURN_ONLY_H:
                print(f"[Vision] Servo {step:02d} [SLOW-TURN] cx={cx} cy={cy:.0f} h_err={h_err:+.3f}")
            else:
                # Only kill the turn command if we are safely inside our target alignment tolerance
                if h_aligned:
                    turn_cmd = 0

            # Move forward only while the object is still above the grasp band.
            # Once cy_frac reaches VISUAL_CY_GRASP_MIN_FRAC, the next loop will initiate grasp.
            fwd_cmd = VISUAL_APPROACH_SPEED if cy_frac < VISUAL_CY_GRASP_MIN_FRAC else 0
            fwd_cmd = self.clamp_small_value(fwd_cmd, MIN_FWD_THRESHOLD)
            turn_cmd = self.clamp_small_value(turn_cmd, MIN_TURN_THRESHOLD)
            print(f"[Vision] Servo {step:02d} [COMBINED] cx={cx} cy={cy:.0f} cy_frac={cy_frac:.3f} h_err={h_err:+.3f} cy_err={cy_err:+.3f} turn={turn_cmd} fwd={fwd_cmd}")
            if(fwd_cmd != 0):
                self.dog.move_x(fwd_cmd)
                print(f"[Vision] Moving forward at speed {fwd_cmd}")
                time.sleep(0.2)
            if(turn_cmd != 0):
                self.dog.turn(turn_cmd)
                print(f"[Vision] Turning at speed {turn_cmd}")
                time.sleep(0.2)
            time.sleep(VISUAL_SERVO_DT)

        self._stop()
        return True

    # ------------------------------------------------------------------ public

    def scan_and_align(self) -> bool:
        self.open_nav_camera()    
        print("[Vision] === scan_and_align start ===")
        found = False

        try:
            for size_attempt in range(VISUAL_SIZE_MAX_APPROACHES + 1):    
                found = self._servo(self._nav_cap)
                if not found:
                    print("[Vision] Target completely lost. Aborting alignment loop.")
                    break # Break early; don't waste time repeating full scans

                if self._object_big_enough_for_pickup():
                    break

                if size_attempt >= VISUAL_SIZE_MAX_APPROACHES:
                    found = False
                    break

                self._approach_two_steps_for_size()
        finally:
            self._stop()

        print(f"[Vision] === scan_and_align complete, found: {found} ===")
        return found