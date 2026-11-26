# A head move for the ESP firmware (topic servo_goal, firmware/main/servos.c).
# Each servo moves towards its angle with limited speed and acceleration (a
# trapezoid, braking to stop on the target), stepped once per 20 ms servo frame.
# A new goal blends in from the current motion. The sender plans the move (both
# servos arriving together is its choice of speeds and accelerations).
#
# Angles are servo degrees, as on servo_s1 / servo_s2 (S1 pan [-90, 90], S2 tilt
# [-90, 20]), fractions allowed. Speeds in deg/s and accelerations in deg/s^2 are
# magnitudes; a speed or acceleration <= 0 jumps straight to the angle.
float32 s1_deg
float32 s2_deg
float32 s1_speed
float32 s2_speed
float32 s1_accel
float32 s2_accel
