# ─── Serial ───
SERIAL_PORT = '/dev/ttyAMA0'
SERIAL_BAUD = 115200

# ─── Camera ───
CAMERA_RES = (640, 480)
CAMERA_FPS = 30

# ─── Vision ───
MIN_CONTOUR_AREA = 500
MAX_CONTOUR_AREA = 50000

# ─── HSV Color Ranges (H: 0-180, S: 0-255, V: 0-255) ───
# Red wraps around hue=0, so two ranges needed
HSV_RED_LOWER_1 = (0,   100,  80)
HSV_RED_UPPER_1 = (10,  255, 255)
HSV_RED_LOWER_2 = (170, 100,  80)
HSV_RED_UPPER_2 = (180, 255, 255)

HSV_GREEN_LOWER   = (35,  100,  80)
HSV_GREEN_UPPER   = (85,  255, 255)

HSV_MAGENTA_LOWER = (140, 100,  80)
HSV_MAGENTA_UPPER = (170, 255, 255)

HSV_ORANGE_LOWER  = (10,  150, 150)
HSV_ORANGE_UPPER  = (25,  255, 255)

HSV_BLUE_LOWER    = (100, 150, 100)
HSV_BLUE_UPPER    = (130, 255, 255)

# ─── PD Gains — Wall Following ───
KP_STRAIGHT = 0.5    # proportional gain for straight sections
KD_STRAIGHT = 0.1    # derivative gain for straight sections
KP_CORNER   = 1.2    # stronger proportional during corners
KD_CORNER   = 0.3

# ─── Speeds (RPM sent to STM32) ───
SPEED_STRAIGHT = 300
SPEED_CORNER   = 150

# ─── ToF Thresholds (mm) ───
CORNER_THRESHOLD_MM = 800   # front sensor: start corner handling below this
CENTER_OFFSET       = 0     # 0 = centered between walls; positive = bias right
WALL_DANGER_MM      = 80    # emergency hard correction if closer than this

# ─── Pillar Avoidance ───
AVOIDANCE_OFFSET  = 15.0    # degrees of steering to dodge a pillar
BLEND_DISTANCE    = 1000.0  # mm: full avoidance blend when pillar is this close

# ─── Steering Limits ───
MAX_STEER_ANGLE = 30  # degrees, Ackermann mechanical limit

# ─── Lap / Section Counting ───
# 4 corners × 2 (corner + straight) = 8 sections per lap
SECTIONS_PER_LAP      = 8
GYRO_CORNER_THRESHOLD = 30.0   # deg/s angular rate that counts as a corner
GYRO_CORNER_MIN_MS    = 200    # ms gyro must stay above threshold to confirm corner

# ─── Parking ───
PARKING_OVERSHOOT_MM    = 200
PARKING_REVERSE_ANGLE   = -25   # degrees
PARKING_CLEARANCE_MIN   = 20    # mm: abort parking if walls this close

# ─── Safety ───
WATCHDOG_TIMEOUT_MS = 500
BATTERY_LOW_MV      = 6800   # 2S LiPo minimum voltage

# ─── Loop Rates ───
MAIN_LOOP_HZ   = 30
SERIAL_TX_HZ   = 50
SERIAL_RX_HZ   = 100
