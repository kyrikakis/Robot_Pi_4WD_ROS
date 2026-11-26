// micro-ROS node YB_Car_Node: the ROS contract of the stock Yahboom firmware,
// plus lidar/enable and esp_firmware. See firmware/README.md and firmware/DELTAS.md.
//
// Connection state machine: wait for the agent (ping), create the entities, spin
// and ping once a second; three missed pings in a row (or an agent restart) tear
// everything down, stop the motors and go back to waiting. So the board needs no
// reset when the agent restarts (the stock firmware connected once only).

#include <math.h>
#include <string.h>

#include "esp_app_desc.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdkconfig.h"

#include <geometry_msgs/msg/twist.h>
#include <nav_msgs/msg/odometry.h>
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <rmw_microros/rmw_microros.h>
#include <sensor_msgs/msg/imu.h>
#include <sensor_msgs/msg/laser_scan.h>
#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/string.h>
#include <std_msgs/msg/u_int16.h>
#include <std_srvs/srv/set_bool.h>

#include "rainbow.h"

#define ODOM_PERIOD_MS 50
#define IMU_PERIOD_MS 40
#define SCAN_POLL_MS 20
#define BATTERY_PERIOD_MS 1000
#define PING_PERIOD_MS 1000
#define PING_MISSES 3
#define TIME_SYNC_PERIOD_MS 60000

#define OK(fn) \
    if ((fn) != RCL_RET_OK) return false;
// Best effort: a failed publish or teardown is not fatal (the ping notices a lost agent).
#define SOFT(fn)                 \
    do {                         \
        if ((fn) != RCL_RET_OK) { \
        }                        \
    } while (0)

static rcl_allocator_t s_alloc;
static rclc_support_t s_support;
static rcl_node_t s_node;
static rclc_executor_t s_exec;
static rcl_publisher_t s_pub_odom, s_pub_imu, s_pub_scan, s_pub_battery, s_pub_version;
static rcl_subscription_t s_sub_cmd, s_sub_s1, s_sub_s2, s_sub_beep;
static rcl_service_t s_srv_lidar;
static rcl_timer_t s_tm_odom, s_tm_imu, s_tm_scan, s_tm_battery;

static nav_msgs__msg__Odometry s_odom;
static sensor_msgs__msg__Imu s_imu;
static sensor_msgs__msg__LaserScan s_scan;
static std_msgs__msg__UInt16 s_battery;
static std_msgs__msg__String s_version;
static geometry_msgs__msg__Twist s_cmd;
static std_msgs__msg__Int32 s_s1, s_s2;
static std_msgs__msg__UInt16 s_beep;
static std_srvs__srv__SetBool_Request s_lidar_req;
static std_srvs__srv__SetBool_Response s_lidar_res;

static float s_ranges[LIDAR_POINTS], s_intensities[LIDAR_POINTS];
static char s_version_text[160];

// Odometry state.
static int64_t s_prev_counts[RAINBOW_NUM_WHEELS];
static int64_t s_prev_odom_us;
static double s_x, s_y, s_heading;

static void set_string(rosidl_runtime_c__String *s, const char *text)
{
    s->data = (char *)text;
    s->size = strlen(text);
    s->capacity = s->size + 1;
}

static void stamp(builtin_interfaces__msg__Time *t)
{
    int64_t ns = rmw_uros_epoch_nanos();
    t->sec = (int32_t)(ns / 1000000000LL);
    t->nanosec = (uint32_t)(ns % 1000000000LL);
}

static void init_messages(void)
{
    set_string(&s_odom.header.frame_id, "odom_frame");
    set_string(&s_odom.child_frame_id, "base_footprint");
    s_odom.pose.pose.orientation.w = 1.0;
    s_odom.pose.covariance[0] = s_odom.pose.covariance[7] = s_odom.pose.covariance[35] = 0.001;
    s_odom.twist.covariance[0] = s_odom.twist.covariance[7] = s_odom.twist.covariance[35] = 0.0001;

    set_string(&s_imu.header.frame_id, "imu_frame");
    s_imu.orientation.w = 1.0;
    s_imu.orientation_covariance[0] = -1.0;  // no orientation estimate (REP 145)

    set_string(&s_scan.header.frame_id, "laser_frame");
    s_scan.angle_min = -M_PI;
    s_scan.angle_max = M_PI;
    s_scan.angle_increment = M_PI / 180.0;
    s_scan.range_min = 0.12f;
    s_scan.range_max = 8.0f;
    s_scan.ranges.data = s_ranges;
    s_scan.ranges.size = s_scan.ranges.capacity = LIDAR_POINTS;
    s_scan.intensities.data = s_intensities;
    s_scan.intensities.size = s_scan.intensities.capacity = LIDAR_POINTS;

    const esp_app_desc_t *app = esp_app_get_description();
    snprintf(s_version_text, sizeof(s_version_text), "rainbow_esp %s (built %s %s, IDF %s)",
             app->version, app->date, app->time, app->idf_ver);
    set_string(&s_version.data, s_version_text);
    set_string(&s_lidar_res.message, "");
}

// --- callbacks ---------------------------------------------------------------

static void on_cmd_vel(const void *msg)
{
    const geometry_msgs__msg__Twist *t = msg;
    motors_command((float)t->linear.x, (float)t->angular.z);  // skid-steer: linear.y ignored
}

static void on_servo_s1(const void *msg) { servos_set(SERVO_S1, ((const std_msgs__msg__Int32 *)msg)->data); }
static void on_servo_s2(const void *msg) { servos_set(SERVO_S2, ((const std_msgs__msg__Int32 *)msg)->data); }
static void on_beep(const void *msg) { beep_command(((const std_msgs__msg__UInt16 *)msg)->data); }

static void on_lidar_enable(const void *req, void *res)
{
    const std_srvs__srv__SetBool_Request *rq = req;
    std_srvs__srv__SetBool_Response *rs = res;
    lidar_set_enabled(rq->data);
    rs->success = true;
    set_string(&rs->message, rq->data ? "lidar on" : "lidar off");
}

static void on_odom_timer(rcl_timer_t *timer, int64_t last_call)
{
    int64_t counts[RAINBOW_NUM_WHEELS];
    motors_get_counts(counts);
    int64_t now = esp_timer_get_time();
    double dt = (now - s_prev_odom_us) / 1e6;
    s_prev_odom_us = now;
    if (dt <= 0) return;

    double m = motors_metres_per_count();
    double d[RAINBOW_NUM_WHEELS];
    for (int i = 0; i < RAINBOW_NUM_WHEELS; i++) {
        d[i] = (counts[i] - s_prev_counts[i]) * m;
        s_prev_counts[i] = counts[i];
    }
    double left = (d[0] + d[1]) / 2, right = (d[2] + d[3]) / 2;
    double ds = (left + right) / 2;
    double dth = (right - left) / 2 / motors_half_track();
    double mid = s_heading + dth / 2;
    s_x += ds * cos(mid);
    s_y += ds * sin(mid);
    s_heading += dth;

    s_odom.pose.pose.position.x = s_x;
    s_odom.pose.pose.position.y = s_y;
    s_odom.pose.pose.orientation.z = sin(s_heading / 2);
    s_odom.pose.pose.orientation.w = cos(s_heading / 2);
    s_odom.twist.twist.linear.x = ds / dt;
    s_odom.twist.twist.angular.z = dth / dt;
    stamp(&s_odom.header.stamp);
    SOFT(rcl_publish(&s_pub_odom, &s_odom, NULL));
}

static void on_imu_timer(rcl_timer_t *timer, int64_t last_call)
{
    float a[3], g[3];
    if (!imu_read(a, g)) return;
    s_imu.linear_acceleration.x = a[0];
    s_imu.linear_acceleration.y = a[1];
    s_imu.linear_acceleration.z = a[2];
    s_imu.angular_velocity.x = g[0];
    s_imu.angular_velocity.y = g[1];
    s_imu.angular_velocity.z = g[2];
    stamp(&s_imu.header.stamp);
    SOFT(rcl_publish(&s_pub_imu, &s_imu, NULL));
}

static void on_scan_timer(rcl_timer_t *timer, int64_t last_call)
{
    static uint16_t dist[LIDAR_POINTS];
    static uint8_t inten[LIDAR_POINTS];
    if (!lidar_take_revolution(dist, inten)) return;
    // ROS index i (angle -180 + i deg, counter-clockwise) <- sensor degree (180 - i), clockwise.
    for (int i = 0; i < LIDAR_POINTS; i++) {
        int k = ((180 - i) % LIDAR_POINTS + LIDAR_POINTS) % LIDAR_POINTS;
        s_ranges[i] = dist[k] / 1000.0f;
        s_intensities[i] = inten[k];
    }
    stamp(&s_scan.header.stamp);
    SOFT(rcl_publish(&s_pub_scan, &s_scan, NULL));
}

static void on_battery_timer(rcl_timer_t *timer, int64_t last_call)
{
    s_battery.data = (uint16_t)lroundf(battery_volts() * 10.0f);
    SOFT(rcl_publish(&s_pub_battery, &s_battery, NULL));
}

// --- entities ------------------------------------------------------------------

static bool create_entities(void)
{
    s_alloc = rcl_get_default_allocator();
    rcl_init_options_t opts = rcl_get_zero_initialized_init_options();
    OK(rcl_init_options_init(&opts, s_alloc));
    OK(rcl_init_options_set_domain_id(&opts, CONFIG_RAINBOW_ROS_DOMAIN_ID));
    rcl_ret_t rc = rclc_support_init_with_options(&s_support, 0, NULL, &opts, &s_alloc);
    SOFT(rcl_init_options_fini(&opts));
    OK(rc);
    OK(rclc_node_init_default(&s_node, "YB_Car_Node", "", &s_support));

    OK(rclc_publisher_init_default(&s_pub_odom, &s_node, ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry),
                                   "odom_raw"));
    OK(rclc_publisher_init_default(&s_pub_imu, &s_node, ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu), "imu"));
    OK(rclc_publisher_init_default(&s_pub_scan, &s_node, ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, LaserScan),
                                   "scan"));
    OK(rclc_publisher_init_default(&s_pub_battery, &s_node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt16),
                                   "battery"));
    rmw_qos_profile_t latched = rmw_qos_profile_default;
    latched.durability = RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL;
    latched.depth = 1;
    OK(rclc_publisher_init(&s_pub_version, &s_node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
                           "esp_firmware", &latched));

    OK(rclc_subscription_init_default(&s_sub_cmd, &s_node, ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
                                      "cmd_vel"));
    OK(rclc_subscription_init_default(&s_sub_s1, &s_node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
                                      "servo_s1"));
    OK(rclc_subscription_init_default(&s_sub_s2, &s_node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
                                      "servo_s2"));
    OK(rclc_subscription_init_default(&s_sub_beep, &s_node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt16),
                                      "beep"));
    OK(rclc_service_init_default(&s_srv_lidar, &s_node, ROSIDL_GET_SRV_TYPE_SUPPORT(std_srvs, srv, SetBool),
                                 "lidar/enable"));

    OK(rclc_timer_init_default2(&s_tm_odom, &s_support, RCL_MS_TO_NS(ODOM_PERIOD_MS), on_odom_timer, true));
    OK(rclc_timer_init_default2(&s_tm_imu, &s_support, RCL_MS_TO_NS(IMU_PERIOD_MS), on_imu_timer, true));
    OK(rclc_timer_init_default2(&s_tm_scan, &s_support, RCL_MS_TO_NS(SCAN_POLL_MS), on_scan_timer, true));
    OK(rclc_timer_init_default2(&s_tm_battery, &s_support, RCL_MS_TO_NS(BATTERY_PERIOD_MS), on_battery_timer,
                                true));

    s_exec = rclc_executor_get_zero_initialized_executor();
    OK(rclc_executor_init(&s_exec, &s_support.context, 9, &s_alloc));
    OK(rclc_executor_add_subscription(&s_exec, &s_sub_cmd, &s_cmd, on_cmd_vel, ON_NEW_DATA));
    OK(rclc_executor_add_subscription(&s_exec, &s_sub_s1, &s_s1, on_servo_s1, ON_NEW_DATA));
    OK(rclc_executor_add_subscription(&s_exec, &s_sub_s2, &s_s2, on_servo_s2, ON_NEW_DATA));
    OK(rclc_executor_add_subscription(&s_exec, &s_sub_beep, &s_beep, on_beep, ON_NEW_DATA));
    OK(rclc_executor_add_service(&s_exec, &s_srv_lidar, &s_lidar_req, &s_lidar_res, on_lidar_enable));
    OK(rclc_executor_add_timer(&s_exec, &s_tm_odom));
    OK(rclc_executor_add_timer(&s_exec, &s_tm_imu));
    OK(rclc_executor_add_timer(&s_exec, &s_tm_scan));
    OK(rclc_executor_add_timer(&s_exec, &s_tm_battery));

    rmw_uros_sync_session(1000);
    motors_get_counts(s_prev_counts);
    s_prev_odom_us = esp_timer_get_time();
    SOFT(rcl_publish(&s_pub_version, &s_version, NULL));
    return true;
}

static void destroy_entities(void)
{
    rmw_context_t *ctx = rcl_context_get_rmw_context(&s_support.context);
    if (ctx) rmw_uros_set_context_entity_destroy_session_timeout(ctx, 0);

    SOFT(rclc_executor_fini(&s_exec));
    SOFT(rcl_timer_fini(&s_tm_odom));
    SOFT(rcl_timer_fini(&s_tm_imu));
    SOFT(rcl_timer_fini(&s_tm_scan));
    SOFT(rcl_timer_fini(&s_tm_battery));
    SOFT(rcl_service_fini(&s_srv_lidar, &s_node));
    SOFT(rcl_subscription_fini(&s_sub_cmd, &s_node));
    SOFT(rcl_subscription_fini(&s_sub_s1, &s_node));
    SOFT(rcl_subscription_fini(&s_sub_s2, &s_node));
    SOFT(rcl_subscription_fini(&s_sub_beep, &s_node));
    SOFT(rcl_publisher_fini(&s_pub_odom, &s_node));
    SOFT(rcl_publisher_fini(&s_pub_imu, &s_node));
    SOFT(rcl_publisher_fini(&s_pub_scan, &s_node));
    SOFT(rcl_publisher_fini(&s_pub_battery, &s_node));
    SOFT(rcl_publisher_fini(&s_pub_version, &s_node));
    SOFT(rcl_node_fini(&s_node));
    SOFT(rclc_support_fini(&s_support));
}

void uros_task(void *arg)
{
    enum { WAITING, CONNECTED } state = WAITING;
    init_messages();
    int64_t last_ping = 0, last_sync = 0;
    int misses = 0;

    while (1) {
        int64_t now_ms = esp_timer_get_time() / 1000;
        if (state == WAITING) {
            if (rmw_uros_ping_agent(100, 1) == RMW_RET_OK) {
                if (create_entities()) {
                    state = CONNECTED;
                    misses = 0;
                    last_ping = last_sync = now_ms;
                } else {
                    destroy_entities();
                    vTaskDelay(pdMS_TO_TICKS(500));
                }
            } else {
                vTaskDelay(pdMS_TO_TICKS(400));
            }
            continue;
        }

        rclc_executor_spin_some(&s_exec, RCL_MS_TO_NS(5));
        if (now_ms - last_ping >= PING_PERIOD_MS) {
            last_ping = now_ms;
            misses = (rmw_uros_ping_agent(100, 1) == RMW_RET_OK) ? 0 : misses + 1;
            if (misses >= PING_MISSES) {
                motors_stop();
                destroy_entities();
                state = WAITING;
                continue;
            }
        }
        if (now_ms - last_sync >= TIME_SYNC_PERIOD_MS) {
            last_sync = now_ms;
            rmw_uros_sync_session(100);
        }
        vTaskDelay(1);
    }
}
