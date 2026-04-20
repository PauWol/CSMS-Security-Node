from micropython import const


# Sensor Pins
S_PIR_MOTION_PIN = const(4)
S_RADAR_MOTION_PIN_RX = const(16)
S_RADAR_MOTION_PIN_TX = const(17)
S_RADAR_MOTION_ID_UART = const(2)
S_PHOTO_CELL_PIN = const(34)

# Interface Pins
IF_REBOOT_PIN = const(27)
IF_BUZZER_PIN = const(26)
IF_LED_PANEL_PIN = const(33)

# System State Pins
SYS_BATTERY_VOLTAGE_PIN = const(13)

# Photocell normalized thresholds

V_PHOTO_CELL_NIGHT = const(0.1)
V_PHOTO_CELL_TWILIGHT = const(0.25)
V_PHOTO_CELL_DAWN_DUSK = const(0.4)
V_PHOTO_CELL_DAYLIGHT = const(0.7)
V_PHOTO_CELL_BRIGHT = const(0.9)


GATEWAY_NODE_ID = const(0)
