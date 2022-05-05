std_msgs/Header header

# Resolution 
float32 resolution

# Origin is the origin of map stored as float
geometry_msgs/Point origin
# Dim is the dimension of the map stored as int
geometry_msgs/Point dim

# Subscript (xi, yi, zi) indices probability data[xi + dim.x * yi + zi * dim.x * dim.y]
int8[] data

# Parameters for the occupancy condition of the current map
# Be careful of overflow (should always be within -128 and 128 range)
int8 val_occ = 100
int8 val_even = 50
int8 val_unknown = -1
int8 val_free = 0
# Add val_add to the voxel whenever a point lies in it
int8 val_add = 20
int8 val_default = 0