# Offboard control command message

# control mode flags
uint8 MODE_PASS_THROUGH = 0
uint8 MODE_ROLLRATE_PITCHRATE_YAWRATE_THROTTLE = 1
uint8 MODE_ROLL_PITCH_YAWRATE_THROTTLE = 2
uint8 MODE_ROLL_PITCH_YAWRATE_ALTITUDE = 3
uint8 MODE_XPOS_YPOS_YAW_ALTITUDE = 4
uint8 MODE_XVEL_YVEL_YAWRATE_ALTITUDE = 5
uint8 MODE_XACC_YACC_YAWRATE_AZ = 6

# ignore field bitmasks
uint8 IGNORE_NONE = 0
uint8 IGNORE_X = 1
uint8 IGNORE_Y = 2
uint8 IGNORE_Z = 4
uint8 IGNORE_F = 8

Header header
uint8 mode # offboard control mode for interpreting value fields
uint8 ignore # bitmask for ignore specific setpoint values
float32 x
float32 y
float32 z
float32 F
