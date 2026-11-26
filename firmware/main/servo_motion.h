// One servo's motion towards a target with limited speed and acceleration: a
// trapezoid that brakes to stop on the target, stepped at a fixed period. A new
// target keeps the current velocity, so a stream of targets is one continuous
// movement.
//
// The same algorithm as rainbow_drivers/head_motion.py (Axis.step): the Pi runs
// that model to publish /joint_states, so the two must stay in step;
// firmware/test/test_servo_motion.py compares them. Pure C, no ESP-IDF.
#pragma once

#include <stdbool.h>

typedef struct {
    float low, high;      // limits, degrees
    float pos, vel;       // degrees, deg/s
    float target;
    float speed, accel;   // deg/s, deg/s^2; <= 0: jump
    bool known;           // false until the first target: the servo's position is unknown
} servo_motion_t;

void servo_motion_init(servo_motion_t *m, float low, float high);
// Aim at target (clamped). Without a known position, or with speed or accel <= 0,
// the servo jumps there (pos = target, vel = 0).
void servo_motion_set(servo_motion_t *m, float target, float speed, float accel);
// Advance by dt seconds; returns true if pos changed.
bool servo_motion_step(servo_motion_t *m, float dt);
