Header header # Estimated ROS time at moment of measurement
uint8 fix # fix type, as defined in the UBX protocol, enums defined below
time time # GPS time at moment of measurement
float64[3] position # m, ECEF frame
float64 horizontal_accuracy # m
float64 vertical_accuracy # m
float64[3] velocity # m/s, ECEF frame
float64 speed_accuracy # m/s

uint8 FIX_TYPE_NO_FIX = 0 
uint8 FIX_TYPE_DEAD_RECKONING = 1
uint8 FIX_TYPE_2D = 2
uint8 FIX_TYPE_3D = 3 
uint8 FIX_TYPE_GPS_AND_DEAD_RECKONING = 4
uint8 FIX_TYPE_TIME_ONLY = 5
