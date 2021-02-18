# Auxilliary Override Command Message

# Command Types
uint8 AUX_COMMAND_DISABLED = 0
uint8 AUX_COMMAND_SERVO = 1
uint8 AUX_COMMAND_MOTOR = 2

Header header
uint8[14] type_array
float32[14] values

