"""
grasp.py — full pick-up sequence for the XGO-Mini.

GraspController.execute(tz) drives the robot through:
    1. Crouch    — lower body so camera angles toward ground
    2. Pitch     — tilt nose forward, dropping arm mount closer to floor
    3. Align     — VisionController.scan_and_align() locates the object,
                   centres on it, and drives toward it
    4. Open      — open claw
    5. Extend    — push arm forward and down to object height
    6. Close     — grip object
    7. Retract   — pull arm back (object held)
    8. Level     — reset body pitch before standing
    9. Stand     — return to normal walking height

Returns True  if the object was found and the grasp was attempted.
Returns False if scan_and_align() could not find the object at all —
        the caller (Controller FSM) will then walk the robot back to (0,0).
"""

import time

import numpy as np

from config import (
    ARM_REACH_X, ARM_Z_SCALE, ARM_Z_OFFSET, ARM_HOME_X, ARM_HOME_Z,
    CLAW_OPEN, CLAW_CLOSED,
    BODY_HEIGHT_NORMAL, BODY_HEIGHT_CROUCH, BODY_PITCH_GRASP,
)
from vision import VisionController


class GraspController:

    def __init__(self, dog):
        self.dog    = dog
        self.vision = VisionController(dog)

    def execute(self, tz: float = 0.0) -> bool:
        """
        Pick up an object whose top surface is at height tz metres above
        the ground (tz = 0.0 for a flat object lying on the floor).

        Returns True if a grasp was attempted (object was found),
                False if the object was never detected (caller walks home).
        """
        arm_z = int(np.clip(tz * ARM_Z_SCALE + ARM_Z_OFFSET, -95, 155))

        # 1. Crouch — MUST happen before vision so camera points at the ground
        print("[Grasp] 1/9  Crouching")
        self.dog.translation('z', BODY_HEIGHT_CROUCH)
        time.sleep(1.2)

        # 2. Pitch forward — lowers arm mount and tilts camera downward
        print(f"[Grasp] 2/9  Pitching forward {BODY_PITCH_GRASP}°")
        self.dog.attitude('p', BODY_PITCH_GRASP)
        time.sleep(1.0)

        # 3. Visual scan and alignment — returns False if object not found
        print("[Grasp] 3/9  Visual scan and alignment")
        found = self.vision.scan_and_align()

        if not found:
            # Stand the robot back up before the caller walks it home
            print("[Grasp] Object not found — standing up and signalling walk-home")
            self.dog.attitude('p', 0)
            time.sleep(0.8)
            self.dog.translation('z', BODY_HEIGHT_NORMAL)
            time.sleep(1.2)
            return False    # ← FSM will transition to WALK_HOME

        # 4. Open claw before moving arm toward object
        print("[Grasp] 4/9  Opening claw")
        self.dog.claw(CLAW_OPEN)
        time.sleep(0.5)

        # 5. Extend arm to object
        print(f"[Grasp] 5/9  Extending arm  x={ARM_REACH_X} mm  z={arm_z} mm")
        self.dog.arm(ARM_REACH_X, arm_z)
        time.sleep(1.5)

        # 6. Close claw with maximum force
        print("[Grasp] 6/9  Closing claw")
        self.dog.claw(CLAW_CLOSED)
        time.sleep(1.0)

        # 7. Retract arm — object is now held
        print("[Grasp] 7/9  Retracting arm")
        self.dog.arm(ARM_HOME_X, ARM_HOME_Z)
        time.sleep(1.0)

        # 8. Level body before standing to avoid lurching
        print("[Grasp] 8/9  Levelling body")
        self.dog.attitude('p', 0)
        time.sleep(0.8)

        # 9. Stand back up to normal height
        print("[Grasp] 9/9  Standing up")
        self.dog.translation('z', BODY_HEIGHT_NORMAL)
        time.sleep(1.2)

        print("[Grasp] Complete ✓")
        return True