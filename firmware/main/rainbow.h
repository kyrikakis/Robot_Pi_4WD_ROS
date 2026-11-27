// Rainbow ESP32-S3 firmware: the modules and their interfaces.
//
// Pin map of the Yahboom MicroROS board (ESP32-S3, 4 MB flash):
//   motors M1..M4 PWM A/B   4/5, 15/16, 9/10, 13/14   (M1, M2 left; M3, M4 right)
//   encoders H1..H4 A/B     6/7, 47/48, 11/12, 1/2
//   servos S1 (pan), S2     8, 21
//   buzzer                  46 (active high)
//   battery                 GPIO3 (ADC1 ch2), divider x4.03
//   IMU ICM-42670-P (I2C)   SCL 39, SDA 40
//   lidar MS200 (UART1)     TX 17, RX 18, 230400 baud
//   micro-ROS (UART0)       TX 43, RX 44 -> CP2102 -> Pi
#pragma once

#include <stdbool.h>
#include <stdint.h>

#define RAINBOW_NUM_WHEELS 4

// Motors: wheel speed PID at 100 Hz on core 1.
void motors_init(void);
// Body command (skid-steer): v forward in m/s, w turn rate in rad/s.
void motors_command(float v, float w);
// Zero all wheel targets (agent lost, command timeout).
void motors_stop(void);
// Encoder counts since boot, per wheel (M1..M4), forward positive.
void motors_get_counts(int64_t counts[RAINBOW_NUM_WHEELS]);
// Metres of travel per encoder count.
float motors_metres_per_count(void);
// Half track (m) of the kinematics, shared with the odometry.
float motors_half_track(void);
// PID gains (on encoder counts per 10 ms -> PWM ticks) and feedforward (ticks
// per count/10 ms of target), live-tunable through parameters; boot values are
// the build options.
typedef enum { MOTOR_GAIN_KP, MOTOR_GAIN_KI, MOTOR_GAIN_KD, MOTOR_GAIN_KF, MOTOR_NUM_GAINS } motor_gain_t;
float motors_get_gain(motor_gain_t g);
void motors_set_gain(motor_gain_t g, float value);

// Lidar (MS200): 360 one-degree bins, refreshed once per revolution.
#define LIDAR_POINTS 360
#define LIDAR_MIN_HZ 5
#define LIDAR_MAX_HZ 15
typedef enum { LIDAR_BUSY = 0, LIDAR_OK, LIDAR_FAILED } lidar_result_t;
void lidar_init(void);
// Copies the last full revolution if a new one is ready since the last call;
// period_s is that revolution's duration (0 if unknown).
bool lidar_take_revolution(uint16_t distance_mm[LIDAR_POINTS], uint8_t intensity[LIDAR_POINTS],
                           float *period_s);
// Commands run in the background; each returns a ticket (0: another command is
// still running, or out of range) to poll with lidar_command_result().
// Enable: run / standby (no ranging, no data). Speed: 5..15 Hz.
uint32_t lidar_command_enable(bool on);
uint32_t lidar_command_speed(int hz);
lidar_result_t lidar_command_result(uint32_t ticket);
// Rotation reported by the sensor, Hz (0 while it sends nothing).
float lidar_rotation_hz(void);

// IMU (ICM-42670-P): raw axes of the chip, SI units.
bool imu_init(void);
bool imu_read(float accel_ms2[3], float gyro_rads[3]);

// Servos: angles in degrees, S1 [-90, 90], S2 [-90, 20]. No pulse until the first command.
typedef enum { SERVO_S1 = 0, SERVO_S2 = 1 } servo_id_t;
void servos_init(void);
// Jump to the angle (servo_s1 / servo_s2), cancelling any move.
void servos_set(servo_id_t servo, int angle_deg);
// Move both servos (index SERVO_S1, SERVO_S2) with limited speed (deg/s) and
// acceleration (deg/s^2), blending from the current motion; <= 0 jumps.
void servos_goal(const float angle_deg[2], const float speed[2], const float accel[2]);

// Buzzer: 0 off, 1 on until told otherwise, >= 10 on for that many ms.
void beep_init(void);
void beep_command(uint16_t value);

// Battery voltage (V), averaged.
void battery_init(void);
float battery_volts(void);

// micro-ROS over UART0.
void transport_init(void);
void uros_task(void *arg);
