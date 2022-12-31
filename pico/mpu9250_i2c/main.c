#include <stdio.h>
#include <math.h>

#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "pico/binary_info.h"
#include "pico/multicore.h"

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <sensor_msgs/msg/imu.h>
#include <sensor_msgs/msg/magnetic_field.h>
#include <rmw_microros/rmw_microros.h>

#include "pico_uart_transports.h"

#include "mpu9250_i2c.h"

const uint LED_PIN = 25;        /* PicoオンボードLED */

const int INTR_HZ = 100;         /* タイマー割り込み周期 */
const float STD_G = 9.80665;    /* 重力加速度([m/s^2] == 1G) */

float gyro_fs_sel;
float accel_fs_sel;

bool led;

sensor_msgs__msg__Imu imu_msg;
sensor_msgs__msg__MagneticField mag_msg;
rcl_publisher_t pub_imu;
rcl_publisher_t pub_mag;

void ledPattern()
{
  static int n = 0;

  while (true) {
    if (n < 1) {
      led = true;
    } else {
      led = false;
    }
    gpio_put(LED_PIN, led);
    
    n++;
    if (n >= 100) {
      n = 0;
    }

    sleep_ms(10);
  }
}


void mag_callback(rcl_timer_t *timer, int64_t last_call_time)
{
  int16_t magnet[3];
  float mx, my, mz;             /* [T] */

  ak8963_read_raw(magnet);
  mx = magnet[1] / 0.15 / 1000000.0;      /* AK8963は加速度・角速度と軸が異なるので変換が必要 */
  my = magnet[0] / 0.15 / 1000000.0;
  mz = (-1.0) * magnet[2] / 0.15 / 1000000.0;

  //  printf("Mag. X = %f, Y = %f, Z = %f\n", mx, my, mz);
  
  mag_msg.magnetic_field.x = mx;
  mag_msg.magnetic_field.y = my;
  mag_msg.magnetic_field.z = mz;

  rcl_ret_t ret;
  ret = rcl_publish(&pub_mag, &mag_msg, NULL);
}

void accelgyro_callback(rcl_timer_t *timer, int64_t last_call_time)
{
  int16_t acceleration[3], gyro[3], temp;
  float ax, ay, az;             /* m/s^2 */
  float gx, gy, gz;             /* rad/s */
  int32_t temp2, press;

  mpu9250_read_raw(acceleration, gyro, &temp);
  
  ax = (float)acceleration[0] / accel_fs_sel * STD_G;
  ay = (float)acceleration[1] / accel_fs_sel * STD_G;
  az = (float)acceleration[2] / accel_fs_sel * STD_G;
  
  gx = (float)gyro[0] / gyro_fs_sel * 0.0174; // 0.0174 = M_PI / 180.0;
  gy = (float)gyro[1] / gyro_fs_sel * 0.0174;
  gz = (float)gyro[2] / gyro_fs_sel * 0.0174;
  
  //  printf("Acc. X = %f, Y = %f, Z = %f\n", ax, ay, az);
  //  printf("Gyro. X = %f, Y = %f, Z = %f\n", gx, gy, gz);
  // Temperature is simple so use the datasheet calculation to get deg C.
  // Note this is chip temperature.
  //  printf("Temp. = %f\n", (temp / 340.0) + 36.53);
  
  //  bmp280_read_raw(&press, &temp2);
  //  printf("Pressure = %dPa\n", press);
  //  printf("Temp. = %.2fC\n", temp2 / 100.0);

  imu_msg.linear_acceleration.x = ax;
  imu_msg.linear_acceleration.y = ay;
  imu_msg.linear_acceleration.z = az;

  imu_msg.angular_velocity.x = gx;
  imu_msg.angular_velocity.y = gy;
  imu_msg.angular_velocity.z = gz;

  rcl_ret_t ret;
  ret = rcl_publish(&pub_imu, &imu_msg, NULL);
}

/*
 * Pico搭載LED設定
 */
void led_pin_init()
{
  gpio_init(LED_PIN);
  gpio_set_dir(LED_PIN, GPIO_OUT);
}

int main() {
  rcl_timer_t timer_A_G;        /* Create timer object */
  rcl_timer_t timer_M;          /* Create timer object */
  rcl_node_t node;
  rcl_allocator_t allocator;
  rclc_support_t support;
  rclc_executor_t executor;
  
  led = true;
  gpio_put(LED_PIN, led);

  // stdio_init_all();
  //  printf("Hello, MPU9250! Reading raw data from registers...\n");

  rmw_uros_set_custom_transport(
    true,
    NULL,
    pico_serial_transport_open,
    pico_serial_transport_close,
    pico_serial_transport_write,
    pico_serial_transport_read
    );
  
  // This example will use I2C0 on the default SDA and SCL pins (4, 5 on a Pico)
  i2c_init(i2c_default, 400 * 1000);
  gpio_set_function(PICO_DEFAULT_I2C_SDA_PIN, GPIO_FUNC_I2C);
  gpio_set_function(PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C);
  gpio_pull_up(PICO_DEFAULT_I2C_SDA_PIN);
  gpio_pull_up(PICO_DEFAULT_I2C_SCL_PIN);
  // Make the I2C pins available to picotool
  bi_decl(bi_2pins_with_func(PICO_DEFAULT_I2C_SDA_PIN, PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C));

  mpu9250_reset();
  mpu9250_init();

  mpu9250_enable_bypass();
  ak8963_init();

  led_pin_init();

  gyro_fs_sel = mpu9250_gyro_fs_sel();
  accel_fs_sel = mpu9250_accel_fs_sel();

  // Initialize micro-ROS allocator
  allocator = rcl_get_default_allocator();

  // Initialize support object
  rclc_support_init(&support, 0, NULL, &allocator);
  // Create node object
  rclc_node_init_default(&node, "mpu9250", "", &support);

  // Create a reliable rcl publisher
  rclc_publisher_init_default(&pub_imu, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
    "data_raw");
  rclc_publisher_init_default(&pub_mag, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, MagneticField),
    "mag");

  // Initialize timer object
  rclc_timer_init_default(&timer_A_G, &support,
    RCL_MS_TO_NS(1000 / INTR_HZ), /* Timer period on nanoseconds */
    accelgyro_callback);
  rclc_timer_init_default(&timer_M, &support,
    RCL_MS_TO_NS(1000 / INTR_HZ), /* Timer period on nanoseconds */
    mag_callback);
  
  // 3つめの引数は通信オブジェクトの数(Timerとsubscriptionの総数)
  // このプログラムはTimer2つで通信オブジェクトは2
  executor = rclc_executor_get_zero_initialized_executor();
  rclc_executor_init(&executor, &support.context, 2, &allocator);
  
  // Add to the executor
  rclc_executor_add_timer(&executor, &timer_A_G);
  rclc_executor_add_timer(&executor, &timer_M);
  
  multicore_launch_core1(ledPattern);

  while (true)
  {
    rclc_executor_spin_some(&executor, RCL_MS_TO_NS(1));
  }

  rcl_timer_fini(&timer_A_G);
  rcl_timer_fini(&timer_M);
  rclc_executor_fini(&executor);
  rcl_publisher_fini(&pub_imu, &node);
  rcl_publisher_fini(&pub_mag, &node);
  rcl_node_fini(&node);
  rclc_support_fini(&support);
  
  return 0;
}
