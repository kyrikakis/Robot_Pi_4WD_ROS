// Rainbow ESP32-S3 firmware. See firmware/README.md for the ROS contract.

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdkconfig.h"

#include "rainbow.h"

void app_main(void)
{
    // Outputs first, so nothing moves or beeps while the rest starts.
    motors_init();
    servos_init();
    beep_init();
    battery_init();
#if CONFIG_RAINBOW_PUBLISH_IMU
    imu_init();  // optional: /imu is simply not published without it
#endif
    lidar_init();

    transport_init();
    xTaskCreatePinnedToCore(uros_task, "uros", 16384, NULL, 5, NULL, 0);
}
