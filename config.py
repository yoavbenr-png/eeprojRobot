"""
config.py — all tunable constants for the XGO-Mini pick-up task.
Edit this file to tune behaviour without touching logic code.
"""

# ── Network ───────────────────────────────────────────────────────────
LISTEN_PORT = 5000          # UDP port to receive target coordinates on
COMPUTER_IP = '172.20.10.2' # IP address of the computer (sender)
COMPUTER_PORT = 5001         # Port to send completion messages to

# Coordinate source mode: 'udp' | 'file' | 'rest'
COORD_SOURCE = 'file'        # ← change to 'rest' for REST API, 'file' for local testing
COORD_FILE_PATH = '/tmp/trash_coords.json'
COORD_POLL_INTERVAL = 2.0    # seconds between polls (used by 'file' and 'rest' modes)

# ── REST API (used when COORD_SOURCE = 'rest') ────────────────────────
REST_API_URL     = 'http://172.20.10.2:8080/coordinates'  # ← set to your PC's endpoint
REST_API_TIMEOUT = 10.0  # seconds before giving up on a slow response
REST_API_KEY     = ''    # optional Bearer token — leave empty if no auth needed

# ── Navigation ────────────────────────────────────────────────────────
FORWARD_SPEED       = 18    # move_x() units during walking (range −25…25)
FORWARD_SPEED_MS    = 0.5 / 5.0 * 3/2  # real speed (m/s) — CALIBRATE: measure distance in 5s / 5
CAMERA_RANGE        = 0.3  # metres: start checking camera before the coordinate stop margin
GRASP_THRESHOLD     = 0.10  # metres: fallback stop if camera never triggers
STOP_AND_TURN_DEG   = 30.0  # degrees: heading error above this → full stop-and-turn
STOP_BEFORE_TARGET_M = 0.227 # Stop before trash co-ordinate
ALIGN_THRESHOLD_DEG =  8.0  # degrees: emax for turn_to()
MAX_TURN_CMD        = 25    # max turn() units for in-walk heading correction
TURN_VYAW           = 60    # deg/s angular speed passed to turn_to()
LOOP_DT             = 0.10  # seconds per control-loop iteration

# ── Camera approach (NAVIGATE_CAMERA state) ───────────────────────────
CAM_APPROACH_SPEED    = 8     # move_x() units during camera approach (slow)
CAM_APPROACH_SPEED_MS = 0.04  # estimated real speed at CAM_APPROACH_SPEED (m/s)
CAM_STOP_CY_FRAC      = 0.70  # visual stop-before distance once object is detected.
#                               This replaces coordinate-based stopping after visual lock.
#                               lower = stops farther away, higher = walks closer (max ~0.92).
#                               Tune this to match STOP_BEFORE_TARGET_M on the real robot.
CAM_LOST_RETRIES      = 14     # consecutive "not seen" frames before falling through to GRASP

# ── Arm ───────────────────────────────────────────────────────────────
#   dog.arm(arm_x, arm_z)
#       arm_x ∈ [−80, 155] mm   positive = forward
#       arm_z ∈ [−95, 155] mm   positive = up, negative = down
#   dog.claw(pos)  0 = open, 255 = fully closed
ARM_REACH_X  =  80      # mm: how far forward to extend arm while grasping
ARM_Z_SCALE  = 800.0    # mm of arm travel per metre of object height
ARM_Z_OFFSET = -200     # mm: offset so tz=0 (ground) maps to max downward reach
ARM_HOME_X   =   0      # mm: retracted position
ARM_HOME_Z   =   0      # mm: retracted position
CLAW_OPEN    =   0
CLAW_CLOSED  = 255      # maximum grip force

# ── Body pose (used during grasp) ─────────────────────────────────────
#   dog.translation('z', mm)  body height ∈ [75, 115] mm above ground
BODY_HEIGHT_NORMAL = 105    # mm: normal walking height
BODY_HEIGHT_CROUCH =  75    # mm: lowest position for ground-level pickup
BODY_PITCH_GRASP   =  10    # degrees: forward pitch applied during grasp/vision
#                             tilts camera down toward ground and lowers arm mount

# ── Camera ────────────────────────────────────────────────────────────
CAMERA_INDEX  = 0       # cv2 VideoCapture device index
CAMERA_WIDTH  = 320     # capture resolution (lower = faster on Pi)
CAMERA_HEIGHT = 240

# HSV colour range for the target object.
# Tune with:  hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV); print(hsv[h//2, w//2])
# Hue reference: red≈0/170  orange≈10  yellow≈25  green≈55  blue≈110


OBJ_HSV_LOWER = (20, 120, 25)
OBJ_HSV_UPPER = (130, 255, 255)

OBJ_MIN_AREA = 200      # px²: blobs smaller than this are discarded as noise

# ── Vision — Phase 1: SCAN ────────────────────────────────────────────
# Incremental sweep: checks straight ahead first, then steps right in
# VISUAL_SCAN_STEP_DEG increments up to VISUAL_SCAN_MAX_DEG — checking
# at every position so intermediate angles (e.g. 20°) are not missed.
# Then returns to 0° and repeats the same sweep to the left.
# The robot stays at the angle where the object is first found so the
# servo phase can immediately correct from that position.
#
CAMERA_HFOV_DEG         = 62.2 # Pi Camera V2 horizontal field-of-view in degrees.
#                                Used by the servo TURN-ONLY mode to estimate the
#                                rotation needed to face the object from h_err.
VISUAL_SCAN_STEP_DEG    =  8   # degrees per incremental step (9 × 5 = 45° exactly)
VISUAL_SCAN_MAX_DEG     = 96   # maximum sweep angle each side
VISUAL_SCAN_LOOK_FRAMES =  14   # number of frames grabbed at each scan angle
VISUAL_SCAN_FRAME_SLEEP = 0.10 # seconds between frames during a look
VISUAL_SCAN_PASSES      =  2   # repeat full right/left search before giving up
VISUAL_REACQUIRE_ANGLE_DEG = 10 # local recovery step if servo briefly loses object
VISUAL_REACQUIRE_TRIES  =  3   # checks ±10, ±20, ±30 degrees around current heading

# ── Vision — Size gate before pickup ─────────────────────────────────
# After a scan finds the object, require the detected blob to be large
# enough before allowing the arm pickup sequence. If it is still too small,
# the robot walks forward a short fixed burst, scans again, and checks size again.
# TUNE: watch the printed area=???? when the robot is at the right pickup distance.
VISUAL_MIN_GRASP_AREA       = 2500  # px²: object must look at least this big to pick up
VISUAL_SIZE_APPROACH_STEPS  = 2     # number of short forward bursts when object is too small
VISUAL_SIZE_STEP_SPEED      = 8     # move_x() units for each short approach burst
VISUAL_SIZE_STEP_TIME       = 0.35  # seconds per burst; increase for bigger "steps"
VISUAL_SIZE_MAX_APPROACHES  = 2     # max approach+rescan cycles before giving up

# ── Vision — Phase 2: SERVO ───────────────────────────────────────────
# Once the object is detected (at any scan angle), a closed-loop servo
# corrects heading (cx) and distance (cy).
#
# The servo runs in two modes depending on how far off-centre the object
# is horizontally:
#
#   TURN-ONLY mode  (|h_err| > VISUAL_SERVO_TURN_ONLY_H)
#     The object is far from frame centre — the robot first rotates to
#     face it WITHOUT moving forward.  Moving forward while looking
#     sideways at a large angle causes the object to slide out of view.
#     A proportional turn command (higher gain) is applied.
#
#   COMBINED mode   (|h_err| ≤ VISUAL_SERVO_TURN_ONLY_H)
#     The robot is roughly facing the object.  Both turn (gentle) and
#     forward/back are applied simultaneously each iteration.
#
# TUNING: run once, note the printed "cy=???" value when the arm just
# reaches the block, then set:
#   VISUAL_CY_TARGET_FRAC = <that cy value> / CAMERA_HEIGHT
#
VISUAL_H_TOLERANCE       = 0.15  # convergence: |cx_err| as fraction of half-frame width
VISUAL_CY_TARGET_FRAC    = 0.82  # target cy as fraction of full frame height
VISUAL_CY_TOLERANCE      = 0.08  # convergence: ±5 % of frame height ≈ ±12 px at 240 p
VISUAL_SERVO_TURN_ONLY_H = 0.45  # h_err above this → turn only, no forward movement
VISUAL_TURN_SPEED_MAX    = 20    # max turn() units (proportional, scales with h_err)
VISUAL_APPROACH_SPEED    = 5     # move_x() units for forward/back correction
VISUAL_SERVO_ITER        = 60    # max servo loop iterations before giving up
VISUAL_SERVO_DT          = 0.15  # seconds per servo loop iteration

# ── Vision — Quick Detection (for camera range check) ─────────────────
QUICK_DETECT_FRAMES = 6          # Number of frames to check for quick detection
QUICK_DETECT_SLEEP  = 0.1        # Seconds between quick detection frames