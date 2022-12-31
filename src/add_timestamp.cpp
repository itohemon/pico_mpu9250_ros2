#include <chrono>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>

rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub;
rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub;

rclcpp::Subscription<sensor_msgs::msg::MagneticField>::SharedPtr mag_sub;
rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr mag_pub;

rclcpp::Node::SharedPtr node = nullptr;

void mag_sub_cb(const sensor_msgs::msg::MagneticField::SharedPtr msg_)
{
  auto msg = sensor_msgs::msg::MagneticField();
  
  msg.header.stamp = node->get_clock()->now();
  msg.header.frame_id = "imu_link";
  msg.magnetic_field.x = msg_->magnetic_field.x;
  msg.magnetic_field.y = msg_->magnetic_field.y;
  msg.magnetic_field.z = msg_->magnetic_field.z;
  
  mag_pub->publish(msg);
}
  

void imu_sub_cb(const sensor_msgs::msg::Imu::SharedPtr msg_)
{
  auto msg = sensor_msgs::msg::Imu();

  msg.header.stamp = node->get_clock()->now();
  msg.header.frame_id = "imu_link";
  msg.angular_velocity.x = msg_->angular_velocity.x;
  msg.angular_velocity.y = msg_->angular_velocity.y;
  msg.angular_velocity.z = msg_->angular_velocity.z;
  msg.linear_acceleration.x = msg_->linear_acceleration.x;
  msg.linear_acceleration.y = msg_->linear_acceleration.y;
  msg.linear_acceleration.z = msg_->linear_acceleration.z;

  imu_pub->publish(msg);
}

int main(int argc,  char *argv[])
{
  rclcpp::init(argc, argv);
  node = rclcpp::Node::make_shared("mpu9250_pub");

  imu_sub = node->create_subscription<sensor_msgs::msg::Imu>(
    "data_raw", 10, imu_sub_cb);
  imu_pub = node->create_publisher<sensor_msgs::msg::Imu>(
    "imu/data_raw", 10);
  
  mag_sub = node->create_subscription<sensor_msgs::msg::MagneticField>(
    "mag", 10, mag_sub_cb);
  mag_pub = node->create_publisher<sensor_msgs::msg::MagneticField>(
    "imu/mag", 10);
  
  /*** コールバックの発生を待機 ***/
  rclcpp::spin(node);
  /*** プロセスの終了 ***/
  node = nullptr;
  rclcpp::shutdown();

  return 0;
}
