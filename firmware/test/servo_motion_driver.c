// Drives firmware/main/servo_motion.c from stdin for test_servo_motion.py:
//   init LOW HIGH | set TARGET SPEED ACCEL | step DT N (prints pos after each step)
#include <stdio.h>
#include <string.h>

#include "servo_motion.h"

int main(void)
{
    servo_motion_t m;
    char cmd[16];
    while (scanf("%15s", cmd) == 1) {
        if (strcmp(cmd, "init") == 0) {
            float lo, hi;
            if (scanf("%f %f", &lo, &hi) != 2) return 1;
            servo_motion_init(&m, lo, hi);
        } else if (strcmp(cmd, "set") == 0) {
            float t, s, a;
            if (scanf("%f %f %f", &t, &s, &a) != 3) return 1;
            servo_motion_set(&m, t, s, a);
        } else if (strcmp(cmd, "step") == 0) {
            float dt;
            int n;
            if (scanf("%f %d", &dt, &n) != 2) return 1;
            for (int i = 0; i < n; i++) {
                servo_motion_step(&m, dt);
                printf("%.6f\n", m.pos);
            }
        } else {
            return 1;
        }
    }
    return 0;
}
