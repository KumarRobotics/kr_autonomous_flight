# Implemented Basis Choices
uint8 STANDARD=0
uint8 LEGENDRE=1
uint8 BEZIER=2 # Same as BSpline and Bernstein modulo marginalization
uint8 ENDPOINT=3
uint8 CHEBYSHEV=4
# Future Basis Choices
uint8 SPECIAL=5
uint8 FUN=6

# There should be 1 more coeffs than the degree
int32 degree
float32 dt
uint8 basis
float32[] coeffs
int32 start_index
int32 end_index