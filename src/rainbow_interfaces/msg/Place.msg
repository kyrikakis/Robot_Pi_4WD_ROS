# A named place on the current map (rainbow_brain's places file).
string name
# Pose in the map frame: metres, yaw in radians (0 = +x, positive = left)
float64 x
float64 y
float64 yaw
string room
string description
# Who added it: owner (by voice or service), foxglove (clicked), robot (its guess)
string source
# False for the robot's own guesses until the owner confirms them
bool confirmed
