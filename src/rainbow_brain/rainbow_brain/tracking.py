"""Tracking logic for the tracker node (no ROS, so it can be tested on its own).

Detections come from any detector publishing vision_msgs/Detection2DArray. Each
result's class_id is a label, optionally followed by a name: the Hailo face
recognition sends "face: <name>" or "face: unknown"; its object detector sends
COCO's class names, e.g. "sports ball". A target matches a detection when it
equals the label or the name ("face" = anyone's face, "stefanos" = that person's face).
"""

import math

# What people (and the model) say, and the label the detectors use
ALIASES = {
    'me': 'face', 'someone': 'face', 'anyone': 'face', 'somebody': 'face',
    'anybody': 'face', 'people': 'face', 'person': 'face', 'faces': 'face',
    # The Hailo's object detector uses COCO's class names
    'ball': 'sports ball', 'football': 'sports ball', 'phone': 'cell phone',
    'mobile': 'cell phone', 'mobile phone': 'cell phone', 'teddy': 'teddy bear',
    'plant': 'potted plant', 'sofa': 'couch', 'television': 'tv',
}


# No match for the target: follow this instead ("face": a person's body when no
# face is visible, e.g. turned away or too far). Not for names: a body box does
# not say who it is.
FALLBACK = {'face': 'person'}
# Where the head is in a person box: this fraction of its height from the top
HEAD_FROM_TOP = 0.12


def clean(text):
    words = str(text).lower().replace('_', ' ').split()
    while words and words[0] in ('the', 'a', 'an', 'my', 'that', 'this'):
        words = words[1:]
    return ' '.join(words)


def normalize(text):
    """What someone asked to follow, as a detector label (aliases applied)."""
    text = clean(text)
    return ALIASES.get(text, text)


def labels(class_id):
    """{'face', 'stefanos'} from 'face: Stefanos'; 'unknown' is not a name.

    No aliases here: the detector's own 'person' (a whole body) is not a face.
    """
    label, _, name = str(class_id).partition(':')
    out = {clean(label)}
    name = clean(name)
    if name and name != 'unknown':
        out.add(name)
    out.discard('')
    return out


def matches(target, class_id):
    """Whether a detection is what someone asked to follow (aliases applied)."""
    return normalize(target) in labels(class_id)


def has_label(label, class_id):
    """Whether a detection has exactly this detector label (no aliases)."""
    return clean(label) in labels(class_id)


def aim_point(class_id, cx, cy, w, h):
    """The point to aim at in a box: its centre, or the head for a person's body."""
    if 'person' in labels(class_id):
        return cx, cy - h / 2.0 + HEAD_FROM_TOP * h
    return cx, cy


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


def head_target(head_pan, head_tilt, off_pan, off_tilt, gain=0.8, zone_pan=8.0,
                zone_tilt=6.0):
    """Where to point the head (deg): (pan, tilt), None for an axis to leave alone.

    None altogether while the target is inside the quiet zone in the middle of
    the view (offsets under zone_pan / zone_tilt): the head does not move all
    the time. Each axis moves only when its offset leaves the zone.
    head_pan/tilt: the head's pose when the frame was taken (not now: the head
    may have moved since, and adding the offset to the current pose would
    overshoot). gain < 1 damps the steps (detections are noisy).
    """
    pan = head_pan + gain * off_pan if abs(off_pan) >= zone_pan else None
    tilt = head_tilt + gain * off_tilt if abs(off_tilt) >= zone_tilt else None
    if pan is None and tilt is None:
        return None
    return pan, tilt
