"""The firmware's servo motion (main/servo_motion.c) against the head node's model.

The head node plans each move and runs rainbow_drivers.head_motion to know where
the head is (/joint_states); the ESP runs the same move in C (float). Both must
give the same trajectory. Needs a C compiler (cc); run with pytest from the repo
root on the Mac or the Pi host:

    python3 -m pytest firmware/test
"""

import os
import subprocess
import sys

import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(HERE))
sys.path.insert(0, os.path.join(ROOT, 'src', 'rainbow_drivers'))

from rainbow_drivers.head_motion import Axis, HeadMotion  # noqa: E402

DT = 0.02
TOLERANCE_DEG = 0.01   # float on the ESP vs double in Python


@pytest.fixture(scope='module')
def driver(tmp_path_factory):
    exe = tmp_path_factory.mktemp('servo') / 'driver'
    main = os.path.join(ROOT, 'firmware', 'main')
    subprocess.run(['cc', '-O2', '-I', main, os.path.join(HERE, 'servo_motion_driver.c'),
                    os.path.join(main, 'servo_motion.c'), '-lm', '-o', str(exe)], check=True)
    return str(exe)


def run_c(driver, script):
    out = subprocess.run([driver], input=script, capture_output=True, text=True, check=True)
    return [float(x) for x in out.stdout.split()]


def run_py(low, high, commands):
    axis = Axis(low, high)
    speed = accel = 0.0
    positions = []
    for cmd in commands:
        if cmd[0] == 'set':
            _, target, speed, accel = cmd
            axis.target = HeadMotion.clamp(target, low, high)   # as HeadMotion.set_target
        else:
            for _ in range(cmd[1]):
                axis.step(DT, speed, accel)
                positions.append(axis.pos)
    return positions


def script_for(low, high, commands, start):
    lines = [f'init {low} {high}', f'set {start} 0 0']   # known position, like the model
    for cmd in commands:
        if cmd[0] == 'set':
            lines.append(f'set {cmd[1]} {cmd[2]} {cmd[3]}')
        else:
            lines.append(f'step {DT} {cmd[1]}')
    return '\n'.join(lines) + '\n'


def compare(driver, commands, low=-90.0, high=90.0):
    c = run_c(driver, script_for(low, high, commands, 0.0))
    py = run_py(low, high, commands)
    assert len(c) == len(py)
    worst = max(abs(a - b) for a, b in zip(c, py))
    assert worst < TOLERANCE_DEG, f'off by {worst:.4f} deg'
    return c


@pytest.mark.parametrize('target,speed,accel', [
    (30.0, 400.0, 4000.0),    # the default quick move
    (30.0, 20.0, 200.0),      # a slow glide
    (-55.0, 60.0, 300.0),
    (0.4, 400.0, 4000.0),     # under a degree
    (88.0, 400.0, 4000.0),
])
def test_single_moves(driver, target, speed, accel):
    c = compare(driver, [('set', target, speed, accel), ('step', 200)])
    assert c[-1] == pytest.approx(target, abs=1e-4)


def test_retarget_mid_move_blends(driver):
    # Reverse halfway (a face moving the other way): keeps its velocity and turns
    compare(driver, [('set', 40.0, 60.0, 300.0), ('step', 25),
                     ('set', -10.0, 60.0, 300.0), ('step', 150)])


def test_stream_of_targets(driver):
    # Tracking: a new target every 0.1 s
    commands = []
    for i in range(30):
        commands += [('set', 20.0 * ((i % 7) / 6.0) - 5.0, 80.0, 500.0), ('step', 5)]
    compare(driver, commands + [('step', 100)])


def test_limits_clamp(driver):
    c = compare(driver, [('set', 50.0, 100.0, 1000.0), ('step', 100)], low=-90.0, high=20.0)
    assert max(c) <= 20.0


def test_head_motion_limits_drive_both_axes(driver):
    # What the head node sends: HeadMotion.limits() per axis (arrive together, stop)
    motion = HeadMotion((-40.0, 38.0), (-36.0, 19.0), speed=60.0, accel=300.0)
    motion.set_target(30.0, -12.0)
    (ps, pa), (ts, ta) = motion.limits()
    pan = [('set', 30.0, ps, pa), ('step', 20)]
    tilt = [('set', -12.0, ts, ta), ('step', 20)]
    for _ in range(20):
        motion.step(0.0, DT)
    target = motion.stop()
    (ps, pa), (ts, ta) = motion.limits()
    pan += [('set', target[0], ps, pa), ('step', 100)]
    tilt += [('set', target[1], ts, ta), ('step', 100)]
    p = compare(driver, pan, -40.0, 38.0)
    t = compare(driver, tilt, -36.0, 19.0)
    for _ in range(100):
        motion.step(0.0, DT)
    assert p[-1] == pytest.approx(motion.pan.pos, abs=TOLERANCE_DEG)
    assert t[-1] == pytest.approx(motion.tilt.pos, abs=TOLERANCE_DEG)


def test_jump_without_speed(driver):
    c = run_c(driver, 'init -90 90\nset 10 0 0\nstep 0.02 1\nset -20 0 0\nstep 0.02 1\n')
    assert c == [10.0, -20.0]


def test_first_goal_jumps(driver):
    # Unknown position at boot: the first goal jumps, later ones move
    c = run_c(driver, 'init -90 90\nset 30 60 300\nstep 0.02 1\nset 0 60 300\nstep 0.02 1\n')
    assert c[0] == 30.0 and 29.0 < c[1] < 30.0
