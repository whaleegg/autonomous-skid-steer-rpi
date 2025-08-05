#include "mpu9250driver/mpu9250driver.h"

#include <chrono>
#include <memory>

#include "LinuxI2cCommunicator.h"

using namespace std::chrono_literals;

MPU9250Driver::MPU9250Driver() : Node("mpu9250publisher")
{
  // Create concrete I2C communicator and pass to sensor
  std::unique_ptr<I2cCommunicator> i2cBus = std::make_unique<LinuxI2cCommunicator>();
  mpu9250_ = std::make_unique<MPU9250Sensor>(std::move(i2cBus));
  // Declare parameters
  declareParameters();
  
  // Set parameters
  mpu9250_->setGyroscopeRange(
      static_cast<MPU9250Sensor::GyroRange>(this->get_parameter("gyro_range").as_int()));
  mpu9250_->setAccelerometerRange(
      static_cast<MPU9250Sensor::AccelRange>(this->get_parameter("accel_range").as_int()));
  mpu9250_->setDlpfBandwidth(
      static_cast<MPU9250Sensor::DlpfBandwidth>(this->get_parameter("dlpf_bandwidth").as_int()));
  mpu9250_->setGyroscopeOffset(this->get_parameter("gyro_x_offset").as_double(),
                               this->get_parameter("gyro_y_offset").as_double(),
                               this->get_parameter("gyro_z_offset").as_double());
  mpu9250_->setAccelerometerOffset(this->get_parameter("accel_x_offset").as_double(),
                                   this->get_parameter("accel_y_offset").as_double(),
                                   this->get_parameter("accel_z_offset").as_double());
  // Check if we want to calibrate the sensor
  if (this->get_parameter("calibrate").as_bool()) {
    RCLCPP_INFO(this->get_logger(), "Calibrating...");
    mpu9250_->calibrate();
  }

  // 2) 값 가져와서 bool 변수에 저장
  //if (this->get_parameter("covariance")).as_bool(){ //why isn't work?
  if (true){
    RCLCPP_INFO(this->get_logger(), "Calc Covariance...");
    mpu9250_->covariance();
  } //add calc covariance;



  mpu9250_->printConfig();
  mpu9250_->printOffsets();
  // Create publisher
  publisher_ = this->create_publisher<sensor_msgs::msg::Imu>("imu/data_raw", 10);
  // 자기장 퍼블리셔 추가
  mag_publisher_ = this->create_publisher<sensor_msgs::msg::MagneticField>(
      "imu/mag", rclcpp::QoS(10));

  std::chrono::duration<int64_t, std::milli> frequency =
      1000ms / this->get_parameter("gyro_range").as_int();
  timer_ = this->create_wall_timer(frequency, std::bind(&MPU9250Driver::handleInput, this));
}

void MPU9250Driver::handleInput()
{
  auto message = sensor_msgs::msg::Imu();
  message.header.stamp = this->get_clock()->now();
  message.header.frame_id = "base_link";
  // Direct measurements
  //message.linear_acceleration_covariance = {0};
  message.linear_acceleration_covariance  = mpu9250_->get_accel_covariance_();
  message.linear_acceleration.x = mpu9250_->getAccelerationX();
  message.linear_acceleration.y = mpu9250_->getAccelerationY();
  message.linear_acceleration.z = mpu9250_->getAccelerationZ();
  //message.angular_velocity_covariance[0] = {0};
  message.angular_velocity_covariance = mpu9250_->get_gyro_covariance_();
  message.angular_velocity.x = mpu9250_->getAngularVelocityX();
  message.angular_velocity.y = mpu9250_->getAngularVelocityY();
  message.angular_velocity.z = mpu9250_->getAngularVelocityZ();
  // Calculate euler angles, convert to quaternion and store in message
  message.orientation_covariance[0] = -1.0;
  //message.orientation_covariance = mpu9250_->get_orientation_covariance_();
  //calculateOrientation(message);
  publisher_->publish(message);


  // 자기장 데이터 읽어서 메시지 생성
  auto mag_msg = sensor_msgs::msg::MagneticField();
  mag_msg.header.stamp = this->get_clock()->now();
  mag_msg.header.frame_id = "imu_link";
  mag_msg.magnetic_field.x = mpu9250_->getMagneticFluxDensityX();
  mag_msg.magnetic_field.y = mpu9250_->getMagneticFluxDensityY();
  mag_msg.magnetic_field.z = mpu9250_->getMagneticFluxDensityZ();
  // 필요시 공분산 설정
  mag_msg.magnetic_field_covariance = mpu9250_->get_mag_covariance_();
  mag_publisher_->publish(mag_msg);
}

void MPU9250Driver::declareParameters()
{
  this->declare_parameter<bool>("calibrate", true);
  this->declare_parameter<bool>("covariance", true);

  this->declare_parameter<int>("gyro_range", MPU9250Sensor::GyroRange::GYR_250_DEG_S);
  this->declare_parameter<int>("accel_range", MPU9250Sensor::AccelRange::ACC_2_G);
  this->declare_parameter<int>("dlpf_bandwidth", MPU9250Sensor::DlpfBandwidth::DLPF_260_HZ);
  this->declare_parameter<double>("gyro_x_offset", 0.0);
  this->declare_parameter<double>("gyro_y_offset", 0.0);
  this->declare_parameter<double>("gyro_z_offset", 0.0);
  this->declare_parameter<double>("accel_x_offset", 0.0);
  this->declare_parameter<double>("accel_y_offset", 0.0);
  this->declare_parameter<double>("accel_z_offset", 0.0);
  this->declare_parameter<int>("frequency", 0.0);
}

void MPU9250Driver::calculateOrientation(sensor_msgs::msg::Imu& imu_message)
{
  // Calculate Euler angles
  double roll, pitch, yaw;
  roll = atan2(imu_message.linear_acceleration.y, imu_message.linear_acceleration.z);
  pitch = atan2(-imu_message.linear_acceleration.y,
                (sqrt(imu_message.linear_acceleration.y * imu_message.linear_acceleration.y +
                      imu_message.linear_acceleration.z * imu_message.linear_acceleration.z)));
  yaw = atan2(mpu9250_->getMagneticFluxDensityY(), mpu9250_->getMagneticFluxDensityX());

  // Convert to quaternion
  double cy = cos(yaw * 0.5);
  double sy = sin(yaw * 0.5);
  double cp = cos(pitch * 0.5);
  double sp = sin(pitch * 0.5);
  double cr = cos(roll * 0.5);
  double sr = sin(roll * 0.5);

  imu_message.orientation.x = cy * cp * sr - sy * sp * cr;
  imu_message.orientation.y = sy * cp * sr + cy * sp * cr;
  imu_message.orientation.z = sy * cp * cr - cy * sp * sr;
  imu_message.orientation.w = cy * cp * cr + sy * sp * sr;
}

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MPU9250Driver>());
  rclcpp::shutdown();
  return 0;
}
