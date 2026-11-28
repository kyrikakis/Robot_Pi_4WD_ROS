"""Pure helpers for the web UI: grids to PNG, poses, scans, paths, velocities.

No ROS imports here, so the tests run without a ROS install.
"""

import math
import struct

import numpy as np

try:
    import cv2
except ImportError:      # the tests can run without OpenCV; encode_png needs it
    cv2 = None


def yaw_from_quaternion(x, y, z, w):
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def quaternion_from_yaw(yaw):
    """(x, y, z, w) of a rotation about +z."""
    return 0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)


def compose(a, b):
    """Pose a then pose b, each (x, y, yaw) in 2D: b expressed in a's frame."""
    ax, ay, ayaw = a
    bx, by, byaw = b
    c, s = math.cos(ayaw), math.sin(ayaw)
    return ax + c * bx - s * by, ay + s * bx + c * by, ayaw + byaw


def map_rgba(data, width, height):
    """Turn an OccupancyGrid into an RGBA image, row 0 at the top (the grid's last row).

    Unknown cells are transparent, free cells light, occupied ones dark.
    """
    grid = np.asarray(data, dtype=np.int16).reshape(height, width)[::-1]
    known = grid >= 0
    occ = np.clip(grid, 0, 100).astype(np.float32) / 100.0
    # free 0 -> 232, occupied 100 -> 24 (BGR grey, same on every channel)
    grey = (232.0 - occ * 208.0).astype(np.uint8)
    img = np.zeros((height, width, 4), dtype=np.uint8)
    img[..., 0] = grey
    img[..., 1] = grey
    img[..., 2] = grey
    img[..., 3] = np.where(known, 255, 0).astype(np.uint8)
    return img


def costmap_rgba(data, width, height):
    """Turn a costmap OccupancyGrid (0-100, -1 unknown) into a translucent RGBA image.

    Free space is transparent; cost grows from faint blue to purple; the inscribed
    (99) and lethal (100) cells are bright red and magenta.
    """
    grid = np.asarray(data, dtype=np.int16).reshape(height, width)[::-1]
    t = np.clip(grid, 0, 100).astype(np.float32) / 100.0
    img = np.zeros((height, width, 4), dtype=np.uint8)   # BGRA
    img[..., 0] = (255 * (1.0 - 0.3 * t)).astype(np.uint8)
    img[..., 1] = (60 * (1.0 - t)).astype(np.uint8)
    img[..., 2] = (90 + 140 * t).astype(np.uint8)
    img[..., 3] = np.where(grid > 0, 40 + 150 * t, 0).astype(np.uint8)
    lethal = grid >= 99
    img[lethal] = (60, 0, 255, 210)
    img[grid == 100] = (255, 0, 255, 220)
    return img


def encode_png(img):
    ok, buf = cv2.imencode('.png', img, [cv2.IMWRITE_PNG_COMPRESSION, 3])
    if not ok:
        raise ValueError('PNG encoding failed')
    return buf.tobytes()


def compact_scan(ranges, range_min, range_max, max_points=360):
    """Ranges as centimetres (0 = no return), at most max_points of them.

    Returns (step, values): every step-th range was kept.
    """
    n = len(ranges)
    if n == 0:
        return 1, []
    step = max(1, int(math.ceil(n / max_points)))
    out = []
    for r in ranges[::step]:
        if math.isfinite(r) and range_min <= r <= range_max:
            out.append(int(round(r * 100.0)))
        else:
            out.append(0)
    return step, out


def thin_path(points, spacing=0.05):
    """Keep path points at least spacing apart (and always the last one)."""
    if not points:
        return []
    out = [points[0]]
    for p in points[1:-1]:
        if math.hypot(p[0] - out[-1][0], p[1] - out[-1][1]) >= spacing - 1e-9:
            out.append(p)
    if len(points) > 1:
        out.append(points[-1])
    return [(round(x, 3), round(y, 3)) for x, y in out]


def clamp(value, low, high):
    return max(low, min(high, value))


def drive_command(x, z, scale, max_linear, max_reverse, max_angular):
    """Joystick (x forward, z left turn, each -1..1) to (linear, angular) velocities.

    scale (0-1) is the UI's speed setting. Reverse has its own, lower limit:
    nothing sees low obstacles behind the robot.
    """
    x = clamp(float(x), -1.0, 1.0)
    z = clamp(float(z), -1.0, 1.0)
    scale = clamp(float(scale), 0.0, 1.0)
    linear = x * (max_linear if x >= 0.0 else max_reverse) * scale
    angular = z * max_angular * max(scale, 0.5)
    return linear, angular


def jpeg_from_cdr(buf):
    """Return the data field of a serialized sensor_msgs/CompressedImage (CDR).

    Taking the topic raw skips rclpy's conversion of every frame (~200 KB at
    15 fps) into Python objects.
    """
    little = buf[1] == 1
    u32 = struct.Struct('<I' if little else '>I').unpack_from
    off = 4 + 8                          # encapsulation header, stamp
    for _ in range(2):                   # frame_id, format: strings
        n = u32(buf, off)[0]
        off += 4 + n
        off += (-(off - 4)) % 4          # CDR aligns from the end of the header
    n = u32(buf, off)[0]
    return bytes(buf[off + 4:off + 4 + n])


AUD = b'\x00\x00\x00\x01\x09'   # H.264 access unit delimiter (x264 aud=1)


def split_access_units(buf):
    """Split an H.264 byte stream at its delimiters: (complete access units, rest).

    Each access unit (one frame) starts with AUD; the last one is complete only
    once the next one starts, so it stays in the rest.
    """
    start = buf.find(AUD)
    if start < 0:
        return [], buf
    units = []
    while True:
        nxt = buf.find(AUD, start + len(AUD))
        if nxt < 0:
            return units, buf[start:]
        units.append(buf[start:nxt])
        start = nxt


def is_keyframe(unit):
    """Whether an H.264 access unit holds an IDR slice (a page can start there)."""
    i = unit.find(b'\x00\x00\x01')
    while 0 <= i < len(unit) - 3:
        if unit[i + 3] & 0x1f == 5:
            return True
        i = unit.find(b'\x00\x00\x01', i + 3)
    return False
