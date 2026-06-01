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
FORWARD_SPEED_MS    = 0.9 / 5.0   # real speed (m/s) — CALIBRATE: measure distance in 5s / 5

CAMERA_RANGE        = 0.25 # metres: start checking camera 25cm away
STOP_BEFORE_TARGET_M = 0.24 # metres: Stop blind coordinate navigation 24cm away

GRASP_THRESHOLD     = 0.10  # metres: fallback stop if camera never triggers

# REVERTED: Back to 30.0 so the robot steers diagonally while walking
STOP_AND_TURN_DEG   = 30.0  
ALIGN_THRESHOLD_DEG =  8.0  

MAX_TURN_CMD        = 25    # max turn() units for in-walk heading correction
TURN_VYAW           = 60    # deg/s angular speed passed to turn_to()
LOOP_DT             = 0.10  # seconds per control-loop iteration

# ── Camera approach (NAVIGATE_CAMERA state) ───────────────────────────
CAM_APPROACH_SPEED    = 14    # move_x() units during camera approach
CAM_APPROACH_SPEED_MS = 0.04  # estimated real speed at CAM_APPROACH_SPEED (m/s)

CAM_STOP_CY_FRAC      = 0.82  
CAM_LOST_RETRIES      = 25    

# ── Arm ───────────────────────────────────────────────────────────────
#   dog.arm(arm_x, arm_z)
#       arm_x ∈ [−80, 155] mm   positive = forward
#       arm_z ∈ [−95, 155] mm   positive = up, negative = down
#   dog.claw(pos)  0 = open, 255 = fully closed
ARM_REACH_X  =  85      # mm: retracted reach
ARM_Z_SCALE  = 800.0    # mm of arm travel per metre of object height
ARM_Z_OFFSET = -200     # mm: offset so tz=0 (ground) maps to max downward reach
ARM_HOME_X   =   0      # mm: retracted position
ARM_HOME_Z   =   0      # mm: retracted position
CLAW_OPEN    =   0
CLAW_CLOSED  = 255      # maximum grip force

# ── Blind Creep ───────────────────────────────────────────────────────
#GRASP_CREEP_SPEED = 12   
#GRASP_CREEP_TIME  = 1.2  

# ── Final Forward Step (After Camera Alignment) ───────────────────────
FINAL_STEP_SPEED = 14    # Speed of the single forward step
FINAL_STEP_TIME  = 0.3   # Duration of the step in seconds (adjust for longer/shorter step)

# ── Body pose (used during grasp) ─────────────────────────────────────
#   dog.translation('z', mm)  body height ∈ [75, 115] mm above ground
BODY_HEIGHT_NORMAL = 105    # mm: normal walking height
BODY_HEIGHT_CROUCH =  75    # mm: lowest position for ground-level pickup
BODY_PITCH_GRASP   =  10    # degrees: forward pitch applied during grasp/vision

# ── Camera ────────────────────────────────────────────────────────────
CAMERA_INDEX  = 0       # cv2 VideoCapture device index
CAMERA_WIDTH  = 320     # capture resolution (lower = faster on Pi)
CAMERA_HEIGHT = 240

# ======================================================================
# COLOR VISION CALIBRATION
# ======================================================================
OBJ_HSV_LOWER = (110, 50, 40)   
OBJ_HSV_UPPER = (165, 255, 255) 

OBJ_MIN_AREA = 100      

# ── Vision — Phase 1: SCAN ────────────────────────────────────────────
CAMERA_HFOV_DEG         = 62.2 
VISUAL_SCAN_STEP_DEG    =  4   
VISUAL_SCAN_MAX_DEG     = 96   
VISUAL_SCAN_LOOK_FRAMES =  14  
VISUAL_SCAN_FRAME_SLEEP = 0.10 
VISUAL_SCAN_PASSES      =  2   
VISUAL_REACQUIRE_ANGLE_DEG = 10 
VISUAL_REACQUIRE_TRIES  =  3   

VISUAL_BACKUP_STEPS     = 5      # Take 4 steps back
VISUAL_BACKUP_SPEED     = -14    # Negative number = reverse gear
VISUAL_BACKUP_TIME      = 0.35    # Seconds per backward step

# ── Vision — Size gate before pickup ─────────────────────────────────
VISUAL_MIN_GRASP_AREA       = 500   
VISUAL_SIZE_APPROACH_STEPS  = 2     
VISUAL_SIZE_STEP_SPEED      = 14    
VISUAL_SIZE_STEP_TIME       = 0.15  
VISUAL_SIZE_MAX_APPROACHES  = 2     

# ── Vision — Phase 2: SERVO ───────────────────────────────────────────
VISUAL_H_TOLERANCE       = 0.20  
VISUAL_CY_TARGET_FRAC    = 0.92  
VISUAL_CY_TOLERANCE      = 0.02  

VISUAL_SERVO_TURN_ONLY_H = 0.45  
VISUAL_TURN_SPEED_MAX    = 15    
VISUAL_APPROACH_SPEED    = 12    

VISUAL_SERVO_ITER        = 60    
VISUAL_SERVO_DT          = 0.08  

QUICK_DETECT_FRAMES = 6          
QUICK_DETECT_SLEEP  = 0.1