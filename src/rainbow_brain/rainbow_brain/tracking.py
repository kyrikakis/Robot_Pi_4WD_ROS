"""Tracking logic for the tracker node (no ROS, so it can be tested on its own).

Detections come from any detector publishing vision_msgs/Detection2DArray. Each
result's class_id is a label, optionally followed by a name: the Hailo face
recognition sends "face: <name>" or "face: unknown"; an object detector would
send e.g. "sports ball". A target matches a detection when it equals the label
or the name ("face" = anyone's face, "stefanos" = that person's face).
"""

import math

# What people (and the model) say, and the label the detectors use
ALIASES = {
    'me': 'face', 'someone': 'face', 'anyone': 'face', 'somebody': 'face',
    'anybody': 'face', 'people': 'face', 'person': 'face', 'faces': 'face',
    'ball': 'sports ball',
}


def normalize(text):
    words = str(text).lower().replace('_', ' ').split()
    while words and words[0] in ('the', 'a', 'an', 'my', 'that', 'this'):
        words = words[1:]
    text = ' '.join(words)
    return ALIASES.get(text, text)


def labels(class_id):
    """{'face', 'stefanos'} from 'face: Stefanos'; 'unknown' is not a name."""
    label, _, name = str(class_id).partition(':')
    out = {normalize(label)}
    name = normalize(name)
    if name and name != 'unknown':
        out.add(name)
    out.discard('')
    return out


def matches(target, class_id):
    return normalize(target) in labels(class_id)


def pick(candidates, previous=None):
    """Choose one detection to follow.

    candidates: [(cx, cy, w, h)] in image units (pixels or normalized). With a
    previous centre, the nearest one (keep following the same thing); otherwise
    the largest (the closest).
    """
    if not candidates:
        return None
    if previous is not None:
        px, py = previous
        return min(candidates, key=lambda c: math.hypot(c[0] - px, c[1] - py))
    return max(candidates, key=lambda c: c[2] * c[3])


def pixel_to_angles(cx, cy, width, height, hfov_deg, vfov_deg):
    """Angles (deg) from the image centre to a point, in head terms.

    cx, cy, width, height in the same units: pixels, or 0-1 with width = height = 1.

    Pan positive = left (the pixel is left of the centre), tilt positive = down
    (the pixel is below the centre), like the head's joints.
    """
    fx = (width / 2.0) / math.tan(math.radians(hfov_deg) / 2.0)
    fy = (height / 2.0) / math.tan(math.radians(vfov_deg) / 2.0)
    pan = -math.degrees(math.atan((cx - width / 2.0) / fx))
    tilt = math.degrees(math.atan((cy - height / 2.0) / fy))
    return pan, tilt


def head_target(head_pan, head_tilt, off_pan, off_tilt, gain=0.8, deadband=1.5):
    """Where to point the head (deg), or None if the target is close enough to centre.

    head_pan/tilt: the head's pose when the frame was taken (not now: the head
    may have moved since, and adding the offset to the current pose would
    overshoot). gain < 1 damps the steps (detections are noisy).
    """
    if abs(off_pan) < deadband and abs(off_tilt) < deadband:
        return None
    return head_pan + gain * off_pan, head_tilt + gain * off_tilt
