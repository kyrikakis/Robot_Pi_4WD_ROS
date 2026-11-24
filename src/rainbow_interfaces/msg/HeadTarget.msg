# Where the head should point, for continuous tracking (e.g. following a face):
# the latest message wins and the head turns towards it smoothly, blending from
# its current motion. Angles in radians (pan + = left, tilt + = down), clamped
# to the head's limits. speed: rad/s, 0 = the default.
float64 pan
float64 tilt
float64 speed
