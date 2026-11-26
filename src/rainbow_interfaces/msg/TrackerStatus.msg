# The head tracker's state (rainbow_brain tracker node), on every change and every second
string target      # what it was asked to follow ("" when not tracking)
bool active
bool seen          # the target is in view now
bool lost          # not seen for lost_after s (or never since the start)
string label       # the detection followed, e.g. "face: Stefanos"
