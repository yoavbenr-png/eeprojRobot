"""
grasp.py — full pick-up sequence for the XGO-Mini.
"""

import time
import numpy as np

from config import *
from vision import VisionController


class GraspController:

    def __init__(self, dog):
        self.dog    = dog
        self.vision = VisionController(dog)

    def execute(self, tz: float = 0.0) -> bool:
        arm_z = int(np.clip(tz * ARM_Z_SCALE + ARM_Z_OFFSET, -95, 155))

        print("[Grasp] 1/9  Crouching")
        self.dog.translation('z', BODY_HEIGHT_CROUCH)
        time.sleep(1.2)

        print(f"[Grasp] 2/9  Pitching forward {BODY_PITCH_GRASP}°")
        self.dog.attitude('p', BODY_PITCH_GRASP)
        time.sleep(1.0)

        print("[Grasp] 3/9  Visual scan and alignment")
        found = self.vision.scan_and_align()

        if not found:
            print("[Grasp] Object not found — standing up and retrying")
            self.dog.attitude('p', 0)
            time.sleep(0.8)
            self.dog.translation('z', BODY_HEIGHT_NORMAL)
            time.sleep(1.2)
            return False    

        # ── NEW: Single final forward step ──
        print("[Grasp] 3.5/9 Taking 1 final step forward")
        self.dog.move_x(FINAL_STEP_SPEED)
        time.sleep(FINAL_STEP_TIME)
        self.dog.move_x(0)
        time.sleep(0.5)  # Let the chassis settle completely
        # ────────────────────────────────────

        print("[Grasp] 4/9  Opening claw")
        self.dog.claw(CLAW_OPEN)
        time.sleep(0.5)

        print(f"[Grasp] 5/9  Extending arm  x={ARM_REACH_X} mm  z={arm_z} mm")
        self.dog.arm(ARM_REACH_X, arm_z)
        time.sleep(1.5)

        print("[Grasp] 6/9  Closing claw")
        self.dog.claw(CLAW_CLOSED)
        time.sleep(2.0)

        print("[Grasp] 7/9  Retracting arm")
        self.dog.arm(ARM_HOME_X, ARM_HOME_Z)
        time.sleep(1.0)

        print("[Grasp] 8/9  Levelling body")
        self.dog.attitude('p', 0)
        time.sleep(0.8)

        print("[Grasp] 9/9  Standing up")
        self.dog.translation('z', BODY_HEIGHT_NORMAL)
        time.sleep(1.2)

        print("[Grasp] Complete ✓")
        return True