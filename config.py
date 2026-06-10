"""
config.py — all tunable constants for the XGO-Mini pick-up task.
"""

# ── Network ───────────────────────────────────────────────────────────
LISTEN_PORT = 4999          
COMPUTER_IP = '10.196.208.26'        

COORD_SOURCE = 'rest'        
COORD_FILE_PATH = '/tmp/trash_coords.json'
COORD_POLL_INTERVAL = 0.1    

# ── Fixed Basket (Used ONLY when COORD_SOURCE = 'file') ───────────────
FIXED_BASKET_X = 0.0
FIXED_BASKET_Y = 0.33

# ── REST API ──────────────────────────────────────────────────────────
REST_API_GARBAGE     = f'http://{COMPUTER_IP}:{LISTEN_PORT}/api/garbage' 
REST_API_BASKET     = f'http://{COMPUTER_IP}:{LISTEN_PORT}/api/basket'
REST_API_BATTERY    = f'http://{COMPUTER_IP}:{LISTEN_PORT}/api/set_battery'  
REST_API_TIMEOUT = 10.0  
REST_API_KEY     = ''    

# ── Navigation ────────────────────────────────────────────────────────
FORWARD_SPEED       = 10   
FORWARD_SPEED_MS    = 0.18  

CAMERA_RANGE        = 0.40
# Stop this far before a locked trash target, then switch to camera mode.
TARGET_LOCK_STOP_DISTANCE_M = 0.35
GRASP_THRESHOLD     = 0.25  

# The distance threshold for considering the dog the basket
BASKET_THRESHOLD    = 0.42  


# Allow WALK_MAX_HEADING_ERR deg of wobble while walking. But if it stops to pivot, force 3 deg accuracy.
WALK_MAX_HEADING_ERR = 12.0   
ALIGN_TARGET_DEG     = 3.0  

MIN_FWD_THRESHOLD = 7.0
MIN_TURN_THRESHOLD = 7.0
# --- Turning and Alignment Constants ---
STOP_AND_TURN_DEG = 15.0      # Stop walking and turn if the error is larger than this
ALIGN_THRESHOLD_DEG = 2.0    # Margin of error for the dog.turn_to() command
MAX_TURN_CMD = 10            # Maximum speed for standard turning
TURN_VYAW = 5               # Turning speed for proportional visual turning  

#MAX_TURN_CMD        = 25    
#TURN_VYAW           = 60    
LOOP_DT             = 0.15  

# ── Camera approach ───────────────────────────────────────────────────
CAM_APPROACH_SPEED    = 8    
#CAM_APPROACH_SPEED_MS = 0.04  
CAM_STOP_CY_FRAC      = 0.82  
CAM_LOST_RETRIES      = 25    

# ── Arm ───────────────────────────────────────────────────────────────
ARM_REACH_X  =  88   
ARM_Z_SCALE  = 800.0  #Scaling multiplier for translating real-world Z depth to arm Z commands   
ARM_Z_OFFSET = -200   #Base offset (in mm) applied to the arm's Z height calculation.  
ARM_HOME_X   =   30      
ARM_HOME_Z   =   50    
ARM_FORWARD_Z   =  0
ARM_FORWARD_X   =  130
CLAW_OPEN    =   0
CLAW_CLOSED  = 255     

# ── Final Forward Step ────────────────────────────────────────────────
FINAL_STEP_SPEED = 12   
FINAL_STEP_TIME  = 0.4   

# ── Body pose ─────────────────────────────────────────────────────────
BODY_HEIGHT_NORMAL = 105    
BODY_HEIGHT_CROUCH =  75    
BODY_PITCH_GRASP   =  10    

# ── Camera ────────────────────────────────────────────────────────────
CAMERA_INDEX  = 0       
CAMERA_WIDTH  = 320     
CAMERA_HEIGHT = 240

# Replace the two OBJ_HSV lines with this combined list:
TARGET_COLORS = [
    # Purple/Violet (Your original cube)
    {"lower": (110, 50, 40), "upper": (165, 255, 255)},
    # Red (Requires two ranges because it wraps across the 0/180 line)
    {"lower": (0, 100, 100), "upper": (10, 255, 255)},
    {"lower": (160, 100, 100), "upper": (179, 255, 255)},
    # Green
    {"lower": (40, 50, 50), "upper": (85, 255, 255)},
    # Blue (Standard/Dark)
    {"lower": (100, 150, 50), "upper": (130, 255, 255)},
    # Light Blue / Cyan
    {"lower": (85, 50, 50), "upper": (105, 255, 255)},
    # Orange
    {"lower": (11, 120, 120), "upper": (25, 255, 255)}
]

OBJ_MIN_AREA = 100     # The absolute minimum contour area (in pixels) to consider a detection valid.

# ── Vision — Phase 1: SCAN ────────────────────────────────────────────
CAMERA_HFOV_DEG         = 62.2 
VISUAL_SCAN_STEP_DEG    =  4   
VISUAL_SCAN_MAX_DEG     = 24   
VISUAL_SCAN_LOOK_FRAMES =  14  
VISUAL_SCAN_FRAME_SLEEP = 0.10 
VISUAL_SCAN_PASSES      =  0   #The robot will attempt 1 full scanning sweeps before giving up
VISUAL_REACQUIRE_ANGLE_DEG = 10 
VISUAL_REACQUIRE_TRIES  =  1  #Number of times to attempt local reacquisition. 

VISUAL_BACKUP_STEPS     = 5      
VISUAL_BACKUP_SPEED     = -14    
VISUAL_BACKUP_TIME      = 0.35    

# ── Vision — Size gate before pickup ─────────────────────────────────
VISUAL_MIN_GRASP_AREA       = 100   #Target must be at least 100 pixels in area to be considered close enough to grasp
VISUAL_SIZE_APPROACH_STEPS  = 2     #Number of steps to take forward if the object is visible but too small.
VISUAL_SIZE_STEP_SPEED      = 14    
VISUAL_SIZE_STEP_TIME       = 0.15  
VISUAL_SIZE_MAX_APPROACHES  = 2  #Maximum number of times the robot will attempt a size-approach before giving up.   

# ── Vision — Phase 2: SERVO ───────────────────────────────────────────
VISUAL_H_TOLERANCE       = 0.03  
VISUAL_CY_TARGET_FRAC    = 0.91
VISUAL_CY_GRASP_MIN_FRAC = 0.90
VISUAL_CY_GRASP_MAX_FRAC = 0.93

VISUAL_CY_TOLERANCE      = 0.02  

VISUAL_SERVO_TURN_ONLY_H = 0.30 #If horizontal error exceeds 45%, the robot will stop moving forward and only turn.  
VISUAL_TURN_SPEED_MAX    = 5    
VISUAL_APPROACH_SPEED    = 8    

VISUAL_SERVO_ITER        = 20    
VISUAL_SERVO_DT          = LOOP_DT  

QUICK_DETECT_FRAMES = 4          
QUICK_DETECT_SLEEP  = 0.1 #Pause time (0.1s) between frame captures during a look.
