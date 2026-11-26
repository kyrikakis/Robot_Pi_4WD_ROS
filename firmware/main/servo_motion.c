#include "servo_motion.h"

#include <math.h>

static float clampf(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }

void servo_motion_init(servo_motion_t *m, float low, float high)
{
    *m = (servo_motion_t){.low = low, .high = high};
}

void servo_motion_set(servo_motion_t *m, float target, float speed, float accel)
{
    m->target = clampf(target, m->low, m->high);
    m->speed = speed;
    m->accel = accel;
    if (!m->known || speed <= 0.0f || accel <= 0.0f) {
        m->pos = m->target;
        m->vel = 0.0f;
        m->known = true;
    }
}

bool servo_motion_step(servo_motion_t *m, float dt)
{
    float err = m->target - m->pos;
    if (err == 0.0f && m->vel == 0.0f) return false;
    float a = m->accel;
    if (a <= 0.0f || m->speed <= 0.0f) {  // set() jumps for these; never divide by 0
        m->pos = m->target;
        m->vel = 0.0f;
        return true;
    }
    // The fastest speed from which it can still brake to a stop on the target,
    // braking a * dt per step: v^2 / (2 a) + v dt / 2 = distance left
    float brake = a * (sqrtf(0.25f * dt * dt + 2.0f * fabsf(err) / a) - 0.5f * dt);
    float wanted = copysignf(fminf(m->speed, brake), err);
    float dv = clampf(wanted - m->vel, -a * dt, a * dt);
    m->vel += dv;
    float next = m->pos + m->vel * dt;
    float before = m->pos;
    // Arrived (or would pass it) at a speed it can stop from in one step: stop there
    bool passed = (m->target - next) * err <= 0.0f;
    if (passed && fabsf(m->vel) <= a * dt * (1.0f + 1e-6f)) {
        m->pos = m->target;
        m->vel = 0.0f;
    } else {
        m->pos = clampf(next, m->low, m->high);
    }
    return m->pos != before;
}
