from micropython import const


# Sensor Pins
S_PIR_MOTION_PIN = const(16)
S_RADAR_MOTION_PIN_RX = const(17)
S_RADAR_MOTION_PIN_TX = const(18)
S_RADAR_MOTION_ID_UART = const(1)
S_PHOTO_CELL_PIN = const(4)

# Interface Pins
IF_REBOOT_PIN = 21
IF_BUZZER_PIN = 2
IF_LED_PANEL_PIN = 1

# System State Pins
SYS_BATTERY_VOLTAGE_PIN = const(14)

# Photocell normalized thresholds
V_PHOTO_CELL_NIGHT = const(0.1)
V_PHOTO_CELL_TWILIGHT = const(0.25)
V_PHOTO_CELL_DAWN_DUSK = const(0.4)
V_PHOTO_CELL_DAYLIGHT = const(0.7)
V_PHOTO_CELL_BRIGHT = const(0.9)

GATEWAY_NODE_ID = const(33544)


# ---------------------------------------------------------------------------
# Threat Assessment
# ---------------------------------------------------------------------------

# --- LIGHT thresholds (0.0 – 1.0) ---
LIGHT_DAY = 0.7
LIGHT_DUSK = 0.25

# --- TIME WEIGHTS ---
# Weight < 1.0 suppresses score  → harder to alarm (daytime, many false positives)
# Weight = 1.0 full sensitivity  → dusk, prime intrusion window
# Weight ~ 0.8 slight suppress   → deep night, more animal/wind false positives
WEIGHT_DAY = 0.4  # was 1.0 — heavy suppress, expect lots of daytime movement
WEIGHT_DUSK = 1.0  # was 3.0 — full sensitivity, prime intrusion window
WEIGHT_NIGHT = 0.8  # was 2.0 — slightly suppressed, more wildlife noise at night

# --- SINGLE ALARM THRESHOLD ---
# Score must exceed this (after weight) to trigger.
# With conf=0.88 all bonuses firing: raw=12.28 * 1.0 (dusk) = 12.28 → alarm
# With conf=0.88 at day:             raw=12.28 * 0.4        =  4.91 → no alarm
# With conf=0.3  at dusk:            raw≈3.8  * 1.0         =  3.8  → no alarm (below PRE)
THRESH = 8.0

# PRE-ALARM at 60% of threshold, ALARM at 100%, URGENT at 150%
# (detect.py uses threshold * 0.5 / 1.0 / 1.5 so these map cleanly)

# Keep per-phase names as aliases so detect.py doesn't need changes
THRESH_DAY = THRESH
THRESH_DUSK = THRESH
THRESH_NIGHT = THRESH

# --- PIR ---
PIR_BASE_SCORE = 1.5

# --- RADAR CONFIDENCE (0.0 – 1.0) ---
CONF_SCALE = 6.0
CONF_PEAK_BONUS = 2.0
PEAK_THRESHOLD = 0.75

# --- BEHAVIOR BONUSES ---
BONUS_CONTINUOUS = 2.0
BONUS_STABLE = 1.5

# --- VARIANCE / STABILITY ---
STABLE_VARIANCE = 0.02
NOISE_CUT_OUT_THRESHOLD = 0.25

# --- TIMING ---
RADAR_WINDOW = 8000  # ms
CONTINUOUS_TIME = 4000  # ms
