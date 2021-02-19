Header header
float64[] dts
float64[] coefficents
int8 mode # 0 -- new traj, 1 -- proc traj, 2 -- new stop, 3 -- proc stop
int32 type   # 0 standard polynomial basis
int32 degree   # number degree of polynomial
int32 segments # number of segments in trajectory
int32 dimension # number of dimensions (2 or 3)
#NOTE: coefficents.size() = (degree+1)*segments*dimension
