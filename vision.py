"""
vision.py — camera-based object detection and pre-grasp alignment.

VisionController.scan_and_align() runs two sequential phases:

  Phase 1 — SCAN  (incremental sweep)
    Checks straight ahead first.  Then steps RIGHT in VISUAL_SCAN_STEP_DEG
    increments up to VISUAL_SCAN_MAX_DEG, checking VISUAL_SCAN_LOOK_FRAMES
    frames at each angle.  If still not found, returns to 0° and repeats
    the same sweep to the LEFT.  The robot stays at the angle where the
    object is first detected so no information is lost.

    Stepping at every angle (instead of jumping to ±45° in one go)
    guarantees that objects at intermediate angles, e.g. 20°, are caught.

  Phase 2 — SERVO  (unified heading + distance control)
    As soon as the object is detected — regardless of which scan angle
    triggered it — a single closed-loop controller runs every
    VISUAL_SERVO_DT seconds and applies BOTH corrections simultaneously:

        turn  ∝  h_err  (cx error → rotate toward object)
        fwd   ∝ −cy_err (cy error → drive forward/back to target distance)

    This is correct for any starting condition: if the robot detected the
    object mid-scan at 30° and 40 cm too far, the servo will rotate it
    to face the object AND drive it forward at the same time.  Converges
    when both |h_err| ≤ VISUAL_H_TOLERANCE and |cy_err| ≤ VISUAL_CY_TOLERANCE.

Sign conventions (empirically verified):
    turn(+N)  = CCW = physically LEFT;  IMU yaw increases
    turn(−N)  = CW  = physically RIGHT; IMU yaw decreases
    camera is horizontally mirrored:
        h_err > 0  (cx > frame_cx) → object is physically to the LEFT
        h_err < 0  (cx < frame_cx) → object is physically to the RIGHT
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
        self._nav_cap = None   # persistent camera handle for NAVIGATE_CAMERA
        self._last_detection = None  # latest (cx, cy, area) from scan/look/servo

    # ------------------------------------------------------------------ helpers

    def _stop(self):
        self.dog.move_x(0)
        self.dog.turn(0)

    def _raw_yaw(self) -> float:
        """Raw IMU yaw in degrees.  Returns 0.0 on read failure."""
        try:
            return float(self.dog.read_yaw())
        except Exception:
            return 0.0

    def _turn_to_yaw(self, target: float):
        """
        Rotate until the raw IMU yaw is within 3° of target.

        Sign convention (verified from navigation logs):
            turn(+N) = counterclockwise (LEFT) → IMU yaw increases
            turn(−N) = clockwise (RIGHT)        → IMU yaw decreases

        Therefore:
            turn LEFT  N° → target = current_yaw + N  (err>0 → speed>0 → CCW ✓)
            turn RIGHT N° → target = current_yaw − N  (err<0 → speed<0 → CW  ✓)
        """
        for _ in range(60):
            err = (target - self._raw_yaw() + 180.0) % 360.0 - 180.0
            if abs(err) < 3.0:
                break
            speed = int(np.clip(err * 0.8, -18.0, 18.0))
            self.dog.turn(speed)
            time.sleep(0.10)
        self._stop()
        time.sleep(0.15)    # brief settle after turn completes

    # ------------------------------------------------------------------ detection

    def _detect(self, frame) -> Optional[tuple]:
        """
        Colour-mask detection on the bottom half of the frame.
        A ground-level object always appears in the lower portion when
        the body is crouched and pitched forward.

        Returns (cx, cy, area) in full-frame pixel coordinates, or None.
        """
        h     = frame.shape[0]
       # roi   = frame[h // 2:, :]
        roi = frame[h // 4:, :]
        #y_off = h // 2
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
        """Capture one frame and return detection, or None."""
        ret, frame = cap.read()
        if not ret:
            return None
        return self._detect(frame)

    def _grab_fresh(self, cap) -> Optional[tuple]:
        """
        Flush the Pi camera's internal frame buffer before reading.

        VideoCapture on the Pi buffers several frames.  After a turn the
        buffer still contains pre-rotation frames, so the first few cap.read()
        calls will show the *old* view.  Calling cap.grab() repeatedly drains
        those stale frames; the final cap.retrieve() gives the live view.
        """
        for _ in range(4):      # drain up to 4 buffered frames
            cap.grab()
        ret, frame = cap.retrieve()
        if not ret:
            return None
        return self._detect(frame)

    def _look(self, cap) -> Optional[tuple]:
        """
        Take several frames and return a stable detection.
        Requires multiple detections, then returns the largest/lowest one.
        """
        detections = []

        for _ in range(VISUAL_SCAN_LOOK_FRAMES):
            det = self._grab_fresh(cap)
            if det is not None:
                detections.append(det)
            time.sleep(VISUAL_SCAN_FRAME_SLEEP)

        if len(detections) < 2:
            return None

        # Prefer object that is lower in frame, then larger area
        detections.sort(key=lambda d: (d[1], d[2]), reverse=True)
        self._last_detection = detections[0]
        return detections[0]

    # ------------------------------------------------------------------ quick detection

    def quick_detect(self) -> bool:
        """
        Quick camera check to see if object is visible WITHOUT body adjustment.
        Used during NAVIGATE_EXTERNAL to determine if robot is within camera range.
        
        Returns True if object detected, False otherwise.
        Does NOT move the robot or adjust body position.
        """
        cap = cv2.VideoCapture(CAMERA_INDEX)
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH,  CAMERA_WIDTH)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, CAMERA_HEIGHT)

        if not cap.isOpened():
            return False

        detected = False
        try:
            # Check multiple frames to reduce false negatives
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
        """
        Open and warm up the camera once when entering NAVIGATE_CAMERA.
        Keeps it open across many quick_detect_with_position() calls so the
        Pi camera buffer doesn't reset between loop iterations.
        """
        if self._nav_cap is not None:
            return
        self._nav_cap = cv2.VideoCapture(CAMERA_INDEX)
        self._nav_cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        self._nav_cap.set(cv2.CAP_PROP_FRAME_WIDTH,  CAMERA_WIDTH)
        self._nav_cap.set(cv2.CAP_PROP_FRAME_HEIGHT, CAMERA_HEIGHT)
        # Drain first few stale/dark frames so the buffer is live
        print("[Vision] Warming up nav camera...")
        for _ in range(8):
            self._nav_cap.grab()
        print("[Vision] Nav camera ready")

    def close_nav_camera(self):
        """Close the persistent camera handle when leaving NAVIGATE_CAMERA."""
        if self._nav_cap is not None:
            self._nav_cap.release()
            self._nav_cap = None

    def quick_detect_with_position(self) -> Optional[tuple]:
        """
        Returns (cx, cy) centroid of detected object, or None.
        Uses the persistent camera opened by open_nav_camera() so the
        Pi buffer stays warm across loop iterations.
        """
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
        """
        Sweep incrementally from start_yaw in one direction.

        direction: −1 = clockwise (physically RIGHT, yaw decreases)
                   +1 = counterclockwise (physically LEFT, yaw increases)

        Steps through every VISUAL_SCAN_STEP_DEG angle up to
        VISUAL_SCAN_MAX_DEG, calling _look() at each position.
        Returns True and stays at the detection angle if found.
        Returns False (without changing heading) if not found anywhere.
        """
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
        """
        Robust multi-pass scan.

        The old version did one right sweep and one left sweep. That is fast,
        but it can miss the object if the robot stopped slightly short/long,
        the object is near the edge, the camera buffer is stale, or one angle
        happens to get a bad frame.

        This version:
            1. checks straight ahead,
            2. sweeps right and left,
            3. repeats the scan for VISUAL_SCAN_PASSES,
            4. alternates scan order each pass,
            5. returns to centre only after a full failed pass.

        The robot still stays at the heading where the object is detected.
        """
        start_yaw = self._raw_yaw()

        for scan_pass in range(1, VISUAL_SCAN_PASSES + 1):
            print(f"[Vision] Scan pass {scan_pass}/{VISUAL_SCAN_PASSES}: straight ahead")
            self._turn_to_yaw(start_yaw)

            if self._look(cap) is not None:
                print(f"[Vision] Object found straight ahead on pass {scan_pass} ✓")
                return True

            # Alternate direction order so we do not always spend the early
            # high-confidence frames on the same side.
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
        """
        Short local search used when the servo loses the object.

        This avoids a common failure mode: the object is found during scan,
        then disappears for a couple of frames during alignment, and the whole
        grasp fails immediately. Instead, try centre/right/left around the
        current heading before giving up.
        """
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
        """
        Return True only if the latest detected blob is large enough in the frame.

        The detection area grows as the robot gets closer. This prevents the
        arm sequence from running when the object was found, but is still too
        far away for a reliable pickup.
        """
        if self._last_detection is None:
            print("[Vision] Size gate: no detection stored yet")
            return False

        cx, cy, area = self._last_detection
        ok = area >= VISUAL_MIN_GRASP_AREA
        print(f"[Vision] Size gate: area={area:.0f} px²  "
              f"required>={VISUAL_MIN_GRASP_AREA} px²  "
              f"cx={cx} cy={cy}  {'✓ CLOSE ENOUGH' if ok else 'too small / too far'}")
        return ok

    def _approach_two_steps_for_size(self):
        """
        Move forward a small fixed amount when the object was detected but is
        still too small in the frame. This is deliberately open-loop and short:
        walk a couple of bursts, stop, then scan again from the new position.
        """
        print(f"[Vision] Object is too small — moving closer "
              f"{VISUAL_SIZE_APPROACH_STEPS} short steps")
        self.dog.turn(0)
        for step in range(1, VISUAL_SIZE_APPROACH_STEPS + 1):
            print(f"[Vision] Size approach step {step}/{VISUAL_SIZE_APPROACH_STEPS}")
            self.dog.move_x(VISUAL_SIZE_STEP_SPEED)
            time.sleep(VISUAL_SIZE_STEP_TIME)
            self.dog.move_x(0)
            time.sleep(0.20)
        self._stop()
        time.sleep(0.35)

    # ------------------------------------------------------------------ Phase 2: SERVO

    def _servo(self, cap) -> bool:
        """
        Two-mode closed-loop servo that corrects heading then distance.

        TURN-ONLY mode  (|h_err| > VISUAL_SERVO_TURN_ONLY_H)
        ───────────────
        The object is far from frame centre.  Moving forward while looking
        sideways at a large angle causes the object to slide out of the
        frame before the turn can compensate.

        Instead of issuing a raw turn() command and re-reading the (stale)
        camera buffer, we use the HFOV to estimate the required yaw change:

            delta_deg = h_err × (CAMERA_HFOV_DEG / 2)

        and call _turn_to_yaw() which uses IMU feedback and is immune to
        camera-buffer lag.  A fresh frame is then grabbed before the next
        iteration so the controller always sees the post-turn view.

        COMBINED mode   (|h_err| ≤ VISUAL_SERVO_TURN_ONLY_H)
        ─────────────
        The robot is roughly facing the object.  Both corrections are
        applied simultaneously at a gentler forward speed:

            turn_cmd = proportional to h_err
            fwd_cmd  = −sign(cy_err) × VISUAL_APPROACH_SPEED

        Converges when both |h_err| ≤ VISUAL_H_TOLERANCE and
        |cy_err| ≤ VISUAL_CY_TOLERANCE.
        """
        cy_target = VISUAL_CY_TARGET_FRAC * CAMERA_HEIGHT
        frame_cx  = CAMERA_WIDTH / 2.0

        print(f"[Vision] Servo: cy_target={cy_target:.0f} px  "
              f"turn-only threshold h_err>{VISUAL_SERVO_TURN_ONLY_H:.2f}")

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

                    print("[Vision] Servo: object lost after recovery attempts")
                    return False

            self._last_detection = det
            cx, cy, area = det
            h_err  = (cx - frame_cx)  / frame_cx
            cy_err = (cy - cy_target) / CAMERA_HEIGHT

            converged = (abs(h_err)  <= VISUAL_H_TOLERANCE and
                         abs(cy_err) <= VISUAL_CY_TOLERANCE)

            if converged:
                self._stop()
                print(f"[Vision] Servo {step:02d}  cx={cx}  cy={cy:.0f}  "
                      f"h_err={h_err:+.3f}  cy_err={cy_err:+.3f}  ✓ CONVERGED")
                print("[Vision] At ideal grasp position ✓")
                return True

            if abs(h_err) > VISUAL_SERVO_TURN_ONLY_H:
                turn_cmd = int(np.clip(
                    h_err * VISUAL_TURN_SPEED_MAX,
                    -VISUAL_TURN_SPEED_MAX,
                    VISUAL_TURN_SPEED_MAX,
                ))

                print(f"[Vision] Servo {step:02d} [SLOW-TURN] "
                    f"cx={cx} cy={cy:.0f} h_err={h_err:+.3f} "
                    f"turn={turn_cmd:+d}")

                self.dog.move_x(0)
                self.dog.turn(turn_cmd)
                time.sleep(VISUAL_SERVO_DT)
                continue

            # ── COMBINED: facing object — correct distance and heading together ──
            turn_cmd = int(np.clip(
                h_err * VISUAL_TURN_SPEED_MAX,
                -VISUAL_TURN_SPEED_MAX, VISUAL_TURN_SPEED_MAX,
            ))
            fwd_cmd = (int(-np.sign(cy_err) * VISUAL_APPROACH_SPEED)
                       if abs(cy_err) > VISUAL_CY_TOLERANCE else 0)

            print(f"[Vision] Servo {step:02d} [COMBINED]  "
                  f"cx={cx}  cy={cy:.0f}  "
                  f"h_err={h_err:+.3f}  cy_err={cy_err:+.3f}  "
                  f"turn={turn_cmd:+d}  fwd={fwd_cmd:+d}")

            # During GRASP, do not walk while crouched.
            # Only turn slightly to center the object.
            self.dog.move_x(0)
            self.dog.turn(turn_cmd)
            time.sleep(VISUAL_SERVO_DT)

        self._stop()
        print("[Vision] Servo: max iterations — proceeding with best-effort position")
        return True

    # ------------------------------------------------------------------ public

    def scan_and_align(self) -> bool:
        """
        Full pipeline: open camera → incremental scan → unified servo.

        The body must already be crouched and pitched (done by GraspController)
        so the camera is angled toward the ground before this is called.

        Returns True  if the object was found and the robot is positioned
                       for grasping (even if servo didn't fully converge).
        Returns False if the object was not detected at any scan angle
                       — GraspController signals WALK_HOME to the FSM.
        """
        cap = cv2.VideoCapture(CAMERA_INDEX)
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)         # minimise buffer lag on Pi
        cap.set(cv2.CAP_PROP_FRAME_WIDTH,  CAMERA_WIDTH)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, CAMERA_HEIGHT)

        if not cap.isOpened():
            print("[Vision] WARNING: camera failed to open — skipping alignment")
            return False

        print("[Vision] === scan_and_align start ===")
        found = False

        try:
            # Phase 1 — locate object with incremental sweep.
            # After each successful scan/alignment, check the detected blob size.
            # If the object still looks too small, move closer by two short steps
            # and scan again before allowing the arm pickup sequence.
            for size_attempt in range(VISUAL_SIZE_MAX_APPROACHES + 1):
                print(f"[Vision] Size-gated scan attempt "
                      f"{size_attempt + 1}/{VISUAL_SIZE_MAX_APPROACHES + 1}")

                if not self._scan(cap):
                    return False        # triggers WALK_HOME

                found = self._servo(cap)
                if not found:
                    print("[Vision] Servo failed — doing full search again")
                    continue

                if self._object_big_enough_for_pickup():
                    break

                if size_attempt >= VISUAL_SIZE_MAX_APPROACHES:
                    print("[Vision] Object detected but still too small after "
                          "all approach attempts — not picking up")
                    found = False
                    break

                self._approach_two_steps_for_size()

        finally:
            cap.release()
            self._stop()

        print("[Vision] === scan_and_align complete ===")
        return found