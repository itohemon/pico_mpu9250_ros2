# pico_mpu9250_ros2
Raspberry Pi PicoとMPU9250をつなぎ、Micro-ROSとROS2でIMUを出力する

## Picoファームウェア開発環境(事前にセットアップされていること)
- Raspberry Pi Pico C/C++ SDK
- [Micro-ROS](https://github.com/micro-ROS/micro_ros_raspberrypi_pico_sdk/tree/galactic)

## MPU9250
- [Waveshare 10 DOF IMU Sensor, Low Power](https://www.waveshare.com/product/10-DOF-IMU-Sensor-C.htm)
- 接続プロトコル：I2C
- SDA ... PicoのGP4と接続
- SCL ... PicoのGP5と接続
- VCC ... PicoのVBUSと接続
- GND ... PicoのGNDと接続

## PCのROS2環境
- galactic
- PicoとはUSBで接続

## 実行方法
1. Picoのファームウェアをコンパイル
```
cd pico/mpu9250_i2c/
mkdir build
cd build
cmake ..
make 
```
2. Picoに書き込む

3. ROS2ノード実行
```
ros2 launch pico_mpu9250_ros2 mpu9250_i2c.launch.py 
```

# LICENSE
This repository is open-sourced under the Apache-2.0 license. See the LICENSE file for details.

