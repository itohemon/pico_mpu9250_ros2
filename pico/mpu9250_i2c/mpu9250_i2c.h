#ifndef MPU9250_H_
#define MPU9250_H_

#define CONFIG       (0x1A)
#define GYRO_CONFIG  (0x1B)
#define ACCEL_CONFIG (0x1C)
#define ACCEL_CONFIG2 (0x1D)
#define INT_PIN_CFG  (0x37)
#define ACCEL_XOUT_H (0x3B)
#define ACCEL_XOUT_L (0x3C)
#define ACCEL_YOUT_H (0x3D)
#define ACCEL_YOUT_L (0x3E)
#define ACCEL_ZOUT_H (0x3F)
#define ACCEL_ZOUT_L (0x40)
#define TEMP_OUT_H   (0x41)
#define TEMP_OUT_L   (0x42)
#define GYRO_XOUT_H  (0x43)
#define GYRO_XOUT_L  (0x44)
#define GYRO_YOUT_H  (0x45)
#define GYRO_YOUT_L  (0x46)
#define GYRO_ZOUT_H  (0x47)
#define GYRO_ZOUT_L  (0x48)
#define PWR_MGMT_1   (0x6B)

#define GYRO_FS_SEL  0
#define ACCEL_FS_SEL 0

#define AK_HXL   (0x03)
#define AK_CNTL1 (0x0A)
#define AK_ASAX  (0x10)

#define BMP_CALIB00   (0x88)
#define BMP_CTRL_MEAS (0xF4)
#define BMP_CONFIG    (0xF5)
#define BMP_PRESS_MSB (0xF7)

extern void mpu9250_reset();
extern void mpu9250_init();
extern void mpu9250_enable_bypass();
extern void mpu9250_read_raw(int16_t accel[3], int16_t gyro[3], int16_t *temp);
extern float mpu9250_gyro_fs_sel();
extern float mpu9250_accel_fs_sel();

extern void ak8963_init();
extern void ak8963_read_raw(int16_t mag[3]);

extern void bmp280_init();
extern void bmp280_read_raw(int32_t *pressure, int32_t *temperature);

#endif
