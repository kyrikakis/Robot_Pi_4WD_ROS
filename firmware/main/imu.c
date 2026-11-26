// IMU: TDK ICM-42670-P on I2C (SCL 39, SDA 40), address 0x68 or 0x69.
//
// The board's IMU is mounted wrongly and the EKF does not use it (it fuses the
// BNO055 on the Pi); /imu is published for completeness, in the chip's own axes.
// Ranges: +-4 g, +-500 deg/s, 100 Hz output data rate.

#include <math.h>

#include "driver/i2c_master.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "rainbow.h"

#define IMU_SCL 39
#define IMU_SDA 40
#define REG_ACCEL_DATA_X1 0x0B  // accel X,Y,Z then gyro X,Y,Z, big endian
#define REG_PWR_MGMT0 0x1F
#define REG_GYRO_CONFIG0 0x20
#define REG_ACCEL_CONFIG0 0x21
#define REG_WHO_AM_I 0x75
#define WHO_AM_I_42670P 0x67
#define GYRO_500DPS_100HZ ((2 << 5) | 0x09)
#define ACCEL_4G_100HZ ((2 << 5) | 0x09)
#define PWR_GYRO_ACCEL_LN 0x0F

#define ACCEL_SCALE (4.0f * 9.80665f / 32768.0f)
#define GYRO_SCALE (500.0f * (float)M_PI / 180.0f / 32768.0f)

static i2c_master_dev_handle_t s_dev;
static bool s_ok;

static esp_err_t write_reg(uint8_t reg, uint8_t value)
{
    uint8_t buf[2] = {reg, value};
    return i2c_master_transmit(s_dev, buf, 2, 50);
}

bool imu_init(void)
{
    i2c_master_bus_config_t bus_cfg = {
        .i2c_port = I2C_NUM_0,
        .sda_io_num = IMU_SDA,
        .scl_io_num = IMU_SCL,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    i2c_master_bus_handle_t bus;
    if (i2c_new_master_bus(&bus_cfg, &bus) != ESP_OK) return false;

    uint16_t addr = 0;
    if (i2c_master_probe(bus, 0x68, 50) == ESP_OK) {
        addr = 0x68;
    } else if (i2c_master_probe(bus, 0x69, 50) == ESP_OK) {
        addr = 0x69;
    } else {
        return false;
    }
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = addr,
        .scl_speed_hz = 400000,
    };
    if (i2c_master_bus_add_device(bus, &dev_cfg, &s_dev) != ESP_OK) return false;

    uint8_t reg = REG_WHO_AM_I, who = 0;
    if (i2c_master_transmit_receive(s_dev, &reg, 1, &who, 1, 50) != ESP_OK || who != WHO_AM_I_42670P) {
        return false;
    }
    if (write_reg(REG_GYRO_CONFIG0, GYRO_500DPS_100HZ) != ESP_OK) return false;
    if (write_reg(REG_ACCEL_CONFIG0, ACCEL_4G_100HZ) != ESP_OK) return false;
    if (write_reg(REG_PWR_MGMT0, PWR_GYRO_ACCEL_LN) != ESP_OK) return false;
    vTaskDelay(pdMS_TO_TICKS(50));  // gyro start-up time is 45 ms
    s_ok = true;
    return true;
}

bool imu_read(float accel_ms2[3], float gyro_rads[3])
{
    if (!s_ok) return false;
    uint8_t reg = REG_ACCEL_DATA_X1, d[12];
    if (i2c_master_transmit_receive(s_dev, &reg, 1, d, sizeof(d), 20) != ESP_OK) return false;
    for (int i = 0; i < 3; i++) {
        accel_ms2[i] = (int16_t)((d[2 * i] << 8) | d[2 * i + 1]) * ACCEL_SCALE;
        gyro_rads[i] = (int16_t)((d[6 + 2 * i] << 8) | d[7 + 2 * i]) * GYRO_SCALE;
    }
    return true;
}
