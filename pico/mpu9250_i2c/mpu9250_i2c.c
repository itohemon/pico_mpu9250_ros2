#include <string.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"

#include "mpu9250_i2c.h"

static float gyro_fs_sel[4]  = {131.0, 65.5, 32.8, 16.4};
static float accel_fs_sel[4] = {16384.0, 8192.0, 4096.0, 2048.0};

float ak_asa[3];

static int addr = 0x68;
static int mag_addr = 0x0C;

void mpu9250_reset() {
  uint8_t buf[] = {PWR_MGMT_1, 0x00};
  i2c_write_blocking(i2c_default, addr, buf, 2, false);
}

void mpu9250_init() {
  uint8_t buf[2];
  buf[0] = CONFIG;
  buf[1] = 0b00000110;
  i2c_write_blocking(i2c_default, addr, buf, 2, false);

  buf[0] = GYRO_CONFIG;
  buf[1] = (uint8_t)GYRO_FS_SEL << 3;
  i2c_write_blocking(i2c_default, addr, buf, 2, false);

  buf[0] = ACCEL_CONFIG;
  buf[1] = (uint8_t)ACCEL_FS_SEL << 3;
  i2c_write_blocking(i2c_default, addr, buf, 2, false);

  buf[0] = ACCEL_CONFIG2;
  buf[1] = 0b00000110;
  i2c_write_blocking(i2c_default, addr, buf, 2, false);
}

void mpu9250_enable_bypass() {
  uint8_t buf[] = {INT_PIN_CFG, 0x02};
  i2c_write_blocking(i2c_default, addr, buf, 2, false);
}

float mpu9250_gyro_fs_sel() {
  return gyro_fs_sel[GYRO_FS_SEL];
}
  
float mpu9250_accel_fs_sel() {
  return accel_fs_sel[ACCEL_FS_SEL];
}
  
void mpu9250_read_raw(int16_t accel[3], int16_t gyro[3], int16_t *temp) {
  uint8_t buffer[14];

  // Start reading acceleration registers from register 0x3B for 6 bytes
  uint8_t val = ACCEL_XOUT_H;
  i2c_write_blocking(i2c_default, addr, &val, 1, true); // true to keep master control of bus
  i2c_read_blocking(i2c_default, addr, buffer, 14, false);

  accel[0] = (buffer[ACCEL_XOUT_H - val] << 8 | buffer[ACCEL_XOUT_L - val]);
  accel[1] = (buffer[ACCEL_YOUT_H - val] << 8 | buffer[ACCEL_YOUT_L - val]);
  accel[2] = (buffer[ACCEL_ZOUT_H - val] << 8 | buffer[ACCEL_ZOUT_L - val]);

  *temp    = (buffer[TEMP_OUT_H - val]   << 8 | buffer[TEMP_OUT_L - val]);

  gyro[0]  = (buffer[GYRO_XOUT_H - val]  << 8 | buffer[GYRO_XOUT_L - val]);
  gyro[1]  = (buffer[GYRO_YOUT_H - val]  << 8 | buffer[GYRO_YOUT_L - val]);
  gyro[2]  = (buffer[GYRO_ZOUT_H - val]  << 8 | buffer[GYRO_ZOUT_L - val]);
}

void ak8963_init() {
  uint8_t buf[3];
  buf[0] = AK_CNTL1;
  buf[1] = 0x0F;
  i2c_write_blocking(i2c_default, mag_addr, buf, 2, false);

  uint8_t val = AK_ASAX;
  i2c_write_blocking(i2c_default, mag_addr, &val, 1, true);
  i2c_read_blocking(i2c_default, mag_addr, buf, 3, false);
  
  ak_asa[0] = buf[0];
  ak_asa[1] = buf[1];
  ak_asa[2] = buf[2];

  buf[0] = AK_CNTL1;
  buf[1] = 0x00;
  i2c_write_blocking(i2c_default, mag_addr, buf, 2, false);

  buf[0] = AK_CNTL1;
  buf[1] = 0x16;
  i2c_write_blocking(i2c_default, mag_addr, buf, 2, false);
}

void ak8963_read_raw(int16_t mag[3]) {
  uint8_t buffer[7];
  
  uint8_t val = AK_HXL;
  i2c_write_blocking(i2c_default, mag_addr, &val, 1, true); // true to keep master control of bus
  i2c_read_blocking(i2c_default, mag_addr, buffer, 7, false);

  for (int i = 0; i < 3; i++) {
    mag[i] = (buffer[(i * 2) + 1] << 8 | buffer[i * 2]);
    mag[i] = mag[i] * ((((ak_asa[i] - 128) * 0.5) / 128) + 1);
  }
}
