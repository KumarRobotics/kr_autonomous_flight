# Flight controller status message

Header header

bool armed         # True if armed
bool failsafe      # True if in failsafe
bool rc_override   # True if RC is in control
bool offboard      # True if offboard control is active
int8 control_mode  # Onboard control mode
int8 error_code    # Onboard error code
int16 num_errors   # Number of errors
int16 loop_time_us # Loop time in microseconds
