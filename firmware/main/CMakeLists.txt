idf_component_register(
    SRCS main.c uros.c transport.c motors.c lidar.c imu.c servos.c servo_motion.c beep.c battery.c
    INCLUDE_DIRS "."
    REQUIRES driver esp_adc esp_timer esp_app_format bdc_motor micro_ros_espidf_component)
