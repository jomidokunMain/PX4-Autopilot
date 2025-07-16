uint64 timestamp			# time since system start (microseconds)

int64 encoder_position   # The wheel position, in encoder counts since boot. Positive is forward rotation, negative is reverse rotation
int32 speed              # Speed of each wheel, in encoder counts per second. Positive is forward, negative is reverse
uint32 pulses_per_rev    # Number of pulses per revolution for each wheel

# uint64 timestamp			# time since system start (microseconds)

# # uint64 timestamp
# # int64[2] encoder_position
# # int32[2] speed
# # uint32[2] pulses_per_rev
