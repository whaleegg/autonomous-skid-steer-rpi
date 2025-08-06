#include "mpu9250sensor.h"

extern "C" {
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
}

#include <iostream>
#include <thread>

MPU9250Sensor::MPU9250Sensor(std::unique_ptr<I2cCommunicator> i2cBus) : i2cBus_(std::move(i2cBus))
{
  initImuI2c();
  // Wake up sensor
  int result = i2cBus_->write(PWR_MGMT_1, 0);
  if (result < 0)
  {
    std::cerr << "Error waking sensor" << std::endl;
  }
  // Enable bypass mode for magnetometer
  enableBypassMode();
  // Set magnetometer to 100 Hz continuous measurement mode
  setContinuousMeasurementMode100Hz();
  // Read current ranges from sensor
  readGyroscopeRange();
  readAccelerometerRange();
  readDlpfConfig();
}

void MPU9250Sensor::initImuI2c() const
{
  if (ioctl(i2cBus_->getFile(), I2C_SLAVE, MPU9250_ADDRESS_DEFAULT) < 0) {
    std::cerr << "Failed to find device address! Check device address!";
    exit(1);
  }
}

void MPU9250Sensor::initMagnI2c() const
{
  if (ioctl(i2cBus_->getFile(), I2C_SLAVE, AK8963_ADDRESS_DEFAULT) < 0) {
    std::cerr << "Failed to find device address! Check device address!";
    exit(1);
  }
}

void MPU9250Sensor::printConfig() const
{
  std::cout << "Accelerometer Range: +-" << accel_range_ << "g\n";
  std::cout << "Gyroscope Range: +-" << gyro_range_ << " degree per sec\n";
  std::cout << "DLPF Range: " << dlpf_range_ << " Hz\n";
}

void MPU9250Sensor::printOffsets() const
{
  std::cout << "Accelerometer Offsets: x: " << accel_x_offset_ << ", y: " << accel_y_offset_
            << ", z: " << accel_z_offset_ << "\n";
  std::cout << "Gyroscope Offsets: x: " << gyro_x_offset_ << ", y: " << gyro_y_offset_
            << ", z: " << gyro_z_offset_ << "\n";
}

void MPU9250Sensor::setContinuousMeasurementMode100Hz()
{
  initMagnI2c();
  // Set to power-down mode first before switching to another mode
  int result = i2cBus_->write(MAGN_MEAS_MODE, 0x00);
  if (result < 0)
  {
    std::cerr << "Error powering down magnometer" << std::endl;
  }
  // Wait until mode changes
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  // Switch to 100 Hz mode
  result = i2cBus_->write(MAGN_MEAS_MODE, 0x16);
  if (result < 0)
  {
    std::cerr << "Error reactivating magnometer" << std::endl;
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  initImuI2c();
}

void MPU9250Sensor::enableBypassMode()
{
  // Disable I2C master interface
  int result = i2cBus_->write(MPU9250_USER_CTRL, 0x00);
  if (result < 0)
  {
    std::cerr << "Error disabling i2c master interface" << std::endl;
  }
  
  // Enable bypass mode
  result = i2cBus_->write(MPU9250_BYPASS_ADDR, 0x02);
  if (result < 0)
  {
    std::cerr << "Error enabling bypass" << std::endl;
  }
}

int MPU9250Sensor::readGyroscopeRange()
{
  int range = i2cBus_->read(GYRO_CONFIG);
  range = range >> GYRO_CONFIG_SHIFT;
  gyro_range_ = GYRO_RANGES[range];
  return gyro_range_;
}

int MPU9250Sensor::readAccelerometerRange()
{
  int range = i2cBus_->read(ACCEL_CONFIG);
  range = range >> ACCEL_CONFIG_SHIFT;
  accel_range_ = ACCEL_RANGES[range];
  return accel_range_;
}

int MPU9250Sensor::readDlpfConfig()
{
  int range = i2cBus_->read(DLPF_CONFIG);
  range = range & 7;  // Read only first 3 bits
  dlpf_range_ = DLPF_RANGES[range];
  return dlpf_range_;
}

void MPU9250Sensor::setGyroscopeRange(MPU9250Sensor::GyroRange range)
{
  int result = i2cBus_->write(GYRO_CONFIG, range << GYRO_CONFIG_SHIFT);
  if (result < 0)
  {
    std::cerr << "Error setting gyroscope range" << std::endl;
  }
  gyro_range_ = GYRO_RANGES[static_cast<size_t>(range)];
}

void MPU9250Sensor::setAccelerometerRange(MPU9250Sensor::AccelRange range)
{
  int result = i2cBus_->write(ACCEL_CONFIG, range << ACCEL_CONFIG_SHIFT);
  if (result < 0)
  {
    std::cerr << "Error setting acc. range" << std::endl;
  }
  accel_range_ = ACCEL_RANGES[static_cast<size_t>(range)];
}

void MPU9250Sensor::setDlpfBandwidth(DlpfBandwidth bandwidth)
{
  int result = i2cBus_->write(DLPF_CONFIG, bandwidth);
  if (result < 0)
  {
    std::cerr << "Error setting bandwidth" << std::endl;
  }
  dlpf_range_ = DLPF_RANGES[static_cast<size_t>(bandwidth)];
}

double MPU9250Sensor::getAccelerationX() const
{
  int16_t accel_x_msb = i2cBus_->read(ACCEL_XOUT_H);
  int16_t accel_x_lsb = i2cBus_->read(ACCEL_XOUT_H + 1);
  int16_t accel_x = accel_x_lsb | accel_x_msb << 8;
  double accel_x_converted = convertRawAccelerometerData(accel_x);
  if (calibrated_) {
    return accel_x_converted - accel_x_offset_;
  }
  return accel_x_converted;
}

double MPU9250Sensor::getAccelerationY() const
{
  int16_t accel_y_msb = i2cBus_->read(ACCEL_YOUT_H);
  int16_t accel_y_lsb = i2cBus_->read(ACCEL_YOUT_H + 1);
  int16_t accel_y = accel_y_lsb | accel_y_msb << 8;
  double accel_y_converted = convertRawAccelerometerData(accel_y);
  if (calibrated_) {
    return accel_y_converted - accel_y_offset_;
  }
  return accel_y_converted;
}

double MPU9250Sensor::getAccelerationZ() const
{
  int16_t accel_z_msb = i2cBus_->read(ACCEL_ZOUT_H);
  int16_t accel_z_lsb = i2cBus_->read(ACCEL_ZOUT_H + 1);
  int16_t accel_z = accel_z_lsb | accel_z_msb << 8;
  double accel_z_converted = convertRawAccelerometerData(accel_z);
  if (calibrated_) {
    return accel_z_converted - accel_z_offset_;
  }
  return accel_z_converted;
}

double MPU9250Sensor::getAngularVelocityX() const
{
  int16_t gyro_x_msb = i2cBus_->read(GYRO_XOUT_H);
  int16_t gyro_x_lsb = i2cBus_->read(GYRO_XOUT_H + 1);
  int16_t gyro_x = gyro_x_lsb | gyro_x_msb << 8;
  double gyro_x_converted = convertRawGyroscopeData(gyro_x);
  if (calibrated_) {
    return gyro_x_converted - gyro_x_offset_;
  }
  return gyro_x_converted;
}

double MPU9250Sensor::getAngularVelocityY() const
{
  int16_t gyro_y_msb = i2cBus_->read(GYRO_YOUT_H);
  int16_t gyro_y_lsb = i2cBus_->read(GYRO_YOUT_H + 1);
  int16_t gyro_y = gyro_y_lsb | gyro_y_msb << 8;
  double gyro_y_converted = convertRawGyroscopeData(gyro_y);
  if (calibrated_) {
    return gyro_y_converted - gyro_y_offset_;
  }
  return gyro_y_converted;
}

double MPU9250Sensor::getAngularVelocityZ() const
{
  int16_t gyro_z_msb = i2cBus_->read(GYRO_ZOUT_H);
  int16_t gyro_z_lsb = i2cBus_->read(GYRO_ZOUT_H + 1);
  int16_t gyro_z = gyro_z_lsb | gyro_z_msb << 8;
  double gyro_z_converted = convertRawGyroscopeData(gyro_z);
  if (calibrated_) {
    return gyro_z_converted - gyro_z_offset_;
  }
  return gyro_z_converted;
}

double MPU9250Sensor::getMagneticFluxDensityX() const
{
  // TODO: check for overflow of magnetic sensor
  initMagnI2c();
  int16_t magn_flux_x_msb = i2cBus_->read(MAGN_XOUT_L + 1);
  int16_t magn_flux_x_lsb = i2cBus_->read(MAGN_XOUT_L);
  int16_t magn_flux_x = magn_flux_x_lsb | magn_flux_x_msb << 8;
  double magn_flux_x_converted = convertRawMagnetometerData(magn_flux_x);
  initImuI2c();
  return magn_flux_x_converted;
}

double MPU9250Sensor::getMagneticFluxDensityY() const
{
  initMagnI2c();
  int16_t magn_flux_y_msb = i2cBus_->read(MAGN_YOUT_L + 1);
  int16_t magn_flux_y_lsb = i2cBus_->read(MAGN_YOUT_L);
  int16_t magn_flux_y = magn_flux_y_lsb | magn_flux_y_msb << 8;
  double magn_flux_y_converted = convertRawMagnetometerData(magn_flux_y);
  initImuI2c();
  return magn_flux_y_converted;
}

double MPU9250Sensor::getMagneticFluxDensityZ() const
{
  initMagnI2c();
  int16_t magn_flux_z_msb = i2cBus_->read(MAGN_ZOUT_L + 1);
  int16_t magn_flux_z_lsb = i2cBus_->read(MAGN_ZOUT_L);
  int16_t magn_flux_z = magn_flux_z_lsb | magn_flux_z_msb << 8;
  double magn_flux_z_converted = convertRawMagnetometerData(magn_flux_z);
  initImuI2c();
  return magn_flux_z_converted;
}

double MPU9250Sensor::convertRawGyroscopeData(int16_t gyro_raw) const
{
  const double ang_vel_in_deg_per_s = static_cast<double>(gyro_raw) / GYRO_SENS_MAP.at(gyro_range_);
  return ang_vel_in_deg_per_s;
}

double MPU9250Sensor::convertRawMagnetometerData(int16_t flux_raw) const
{
  const double magn_flux_in_mu_tesla =
      static_cast<double>(flux_raw) * MAX_CONV_MAGN_FLUX / MAX_RAW_MAGN_FLUX;
  return magn_flux_in_mu_tesla;
}

double MPU9250Sensor::convertRawAccelerometerData(int16_t accel_raw) const
{
  const double accel_in_m_per_s =
      static_cast<double>(accel_raw) / ACCEL_SENS_MAP.at(accel_range_) * GRAVITY;
  return accel_in_m_per_s;
}

void MPU9250Sensor::setGyroscopeOffset(double gyro_x_offset, double gyro_y_offset,
                                       double gyro_z_offset)
{
  gyro_x_offset_ = gyro_x_offset;
  gyro_y_offset_ = gyro_y_offset;
  gyro_z_offset_ = gyro_z_offset;
}

void MPU9250Sensor::setAccelerometerOffset(double accel_x_offset, double accel_y_offset,
                                           double accel_z_offset)
{
  accel_x_offset_ = accel_x_offset;
  accel_y_offset_ = accel_y_offset;
  accel_z_offset_ = accel_z_offset;
}

void MPU9250Sensor::calibrate()
{
  int count = 0;
  while (count < CALIBRATION_COUNT) {
    gyro_x_offset_ += getAngularVelocityX();
    gyro_y_offset_ += getAngularVelocityY();
    gyro_z_offset_ += getAngularVelocityZ();
    accel_x_offset_ += getAccelerationX();
    accel_y_offset_ += getAccelerationY();
    accel_z_offset_ += getAccelerationZ();
    ++count;
  }
  gyro_x_offset_ /= CALIBRATION_COUNT;
  gyro_y_offset_ /= CALIBRATION_COUNT;
  gyro_z_offset_ /= CALIBRATION_COUNT;
  accel_x_offset_ /= CALIBRATION_COUNT;
  accel_y_offset_ /= CALIBRATION_COUNT;
  accel_z_offset_ /= CALIBRATION_COUNT;
  accel_z_offset_ -= GRAVITY;
  calibrated_ = true;
}

void MPU9250Sensor::covariance()
{
    // 데이터 수집용 벡터
    std::vector<std::array<double, 3>>accel_samples;
    std::vector<std::array<double, 3>>gyro_samples;

    std::vector<std::array<double, 3>>mag_samples;
    std::vector<std::array<double, 4>>quaternion_samples;

    
    // 메모리 예약 (성능 최적화)
    accel_samples.reserve(COVARIANCE_SAMPLE_COUNT);
    gyro_samples.reserve(COVARIANCE_SAMPLE_COUNT);
    mag_samples.reserve(COVARIANCE_SAMPLE_COUNT);
    quaternion_samples.reserve(COVARIANCE_SAMPLE_COUNT);

    int count = 0;
    while (count < COVARIANCE_SAMPLE_COUNT) {
        // 센서 데이터 읽기
        double corrected_accel_x = getAccelerationX();
        double corrected_accel_y = getAccelerationY();
        double corrected_accel_z =  getAccelerationZ();
        double corrected_gyro_x = getAngularVelocityX();
        double corrected_gyro_y = getAngularVelocityY();
        double corrected_gyro_z = getAngularVelocityZ();
        
        double corrected_mag_x = getMagneticFluxDensityX();
        double corrected_mag_y = getMagneticFluxDensityY();
        double corrected_mag_z = getMagneticFluxDensityZ();

        // 보정된 데이터를 샘플에 추가
        accel_samples.push_back({corrected_accel_x, corrected_accel_y, corrected_accel_z});
        gyro_samples.push_back({corrected_gyro_x, corrected_gyro_y, corrected_gyro_z});
        quaternion_samples.push_back(calcQuart(corrected_accel_y, corrected_accel_z));
        mag_samples.push_back({corrected_mag_x, corrected_mag_y, corrected_mag_z});
        ++count;

        // 진행 상황 표시 (10%마다)
        if (count % (COVARIANCE_SAMPLE_COUNT / 10) == 0) {
            std::cout << "Covariance calculation progress: " 
                      << (count * 100 / COVARIANCE_SAMPLE_COUNT) << "%" << std::endl;
        }

        // 10ms 대기 (100Hz 샘플링)
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    // 공분산 행렬 계산
    computeCovarianceMatrix(accel_samples, accel_covariance_);
    computeCovarianceMatrix(gyro_samples, gyro_covariance_);
    computeCovarianceMatrix(mag_samples, mag_covariance_);
    computeQuaternionCovariance(quaternion_samples, orientation_covariance_);

    // 방향 공분산은 자이로스코프 기반으로 추정 (간단한 방법)
    // 실제로는 더 복잡한 센서 융합 기반 계산이 필요
    //std::copy(gyro_covariance_.begin(), gyro_covariance_.end(), orientation_covariance_.begin());


    //covariance_calculated_ = true;

    std::cout << "Covariance calculation completed!" << std::endl;
    
    // 결과 출력
    printCovarianceMatrix(accel_covariance_, "Accelerometer");
    printCovarianceMatrix(gyro_covariance_, "Gyroscope");
    printCovarianceMatrix(mag_covariance_, "magnetometer");
    printCovarianceMatrix(orientation_covariance_, "Orientation");
}

void MPU9250Sensor::computeCovarianceMatrix(const std::vector<std::array<double, 3>>& samples,
                                           std::array<double, 9>& covariance)
{
    const int n = samples.size();
    if (n < 2) {
        std::cerr << "Error: Not enough samples for covariance calculation" << std::endl;
        return;
    }

    // 평균 계산
    std::array<double, 3> mean = {0.0, 0.0, 0.0};
    for (const auto& sample : samples) {
        for (int i = 0; i < 3; i++) {
            mean[i] += sample[i];
        }
    }
    for (int i = 0; i < 3; i++) {
        mean[i] /= n;
    }

    // 공분산 행렬 계산 (3x3 행렬을 1차원 배열로 저장)
    std::fill(covariance.begin(), covariance.end(), 0.0);
    
    for (const auto& sample : samples) {
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                covariance[i * 3 + j] += (sample[i] - mean[i]) * (sample[j] - mean[j]);
            }
        }
    }

    // 표본 공분산으로 정규화 (n-1로 나누기)
    for (auto& cov : covariance) {
        cov /= (n - 1);
    }
}
void MPU9250Sensor::printCovarianceMatrix(const std::array<double, 9>& cov, 
                                         const std::string& name)
{
    std::cout << std::endl << name << " Covariance Matrix:" << std::endl;
    //std::cout << std::fixed << std::setprecision(8);
    
    for (int i = 0; i < 3; i++) {
        std::cout << "[";
        for (int j = 0; j < 3; j++) {
            std::cout << std::setw(12) << cov[i * 3 + j];
            if (j < 2) std::cout << " ";
        }
        std::cout << "]" << std::endl;
    }
    
    // 대각선 원소 (분산) 출력
    std::cout << "Variances (diagonal): ["
              << cov[0] << ", " << cov[4] << ", " << cov[8] << "]" << std::endl;
    
    // 표준편차 출력
    std::cout << "Standard Deviations: ["
              << std::sqrt(cov[0]) << ", " << std::sqrt(cov[4]) << ", " << std::sqrt(cov[8]) << "]" << std::endl;
}

std::array<double,4> MPU9250Sensor::calcQuart(double linear_acceleration_y, double linear_acceleration_z){
  double roll = atan2(linear_acceleration_y, linear_acceleration_z);
  double pitch = atan2(-linear_acceleration_y,
                (sqrt(linear_acceleration_y * linear_acceleration_y +
                      linear_acceleration_z * linear_acceleration_z)));
  double yaw = atan2(getMagneticFluxDensityY(), getMagneticFluxDensityX());

  // Convert to quaternion
  double cy = cos(yaw * 0.5);
  double sy = sin(yaw * 0.5);
  double cp = cos(pitch * 0.5);
  double sp = sin(pitch * 0.5);
  double cr = cos(roll * 0.5);
  double sr = sin(roll * 0.5);

  // double형 데이터를 담을 vector 선언
  std::array<double, 4> myDoubleVector;

  // 각 데이터를 vector에 하나씩 추가 (push_back 사용)
  myDoubleVector[0]=(cy * cp * sr - sy * sp * cr);
  myDoubleVector[1]=(sy * cp * sr + cy * sp * cr);
  myDoubleVector[2]=(sy * cp * cr - cy * sp * sr);
  myDoubleVector[3]=(cy * cp * cr + sy * sp * sr);
  return myDoubleVector;
}
void MPU9250Sensor::computeQuaternionCovariance(
    const std::vector<std::array<double, 4>>& quaternion_samples,
    std::array<double, 9>& covariance)
{
    const int n = quaternion_samples.size();
    if (n < 2) return;

    // Quaternion은 단위 제약이 있으므로 특별한 처리 필요
    // Method 1: Quaternion error vector 방법 (3차원)
    std::vector<std::array<double, 3>> error_samples;
    error_samples.reserve(n-1);

    // 기준 quaternion (첫 번째 샘플)
    auto q_ref = quaternion_samples[0];
    
    for (int i = 1; i < n; i++) {
        auto q_current = quaternion_samples[i];
        
        // Quaternion difference -> 3D error vector
        auto q_error = quaternionError(q_ref, q_current);
        error_samples.push_back(q_error);
    }
    
    // 3x3 공분산 계산
    computeCovarianceMatrix(error_samples, covariance);
}

std::array<double, 3> MPU9250Sensor::quaternionError(
    const std::array<double, 4>& q_ref,
    const std::array<double, 4>& q_current)
{
    // q_error = q_current * q_ref^(-1)을 3D 벡터로 변환
    // 작은 각도 근사: q_error ? [1, δθ/2]
    
    // Quaternion conjugate (inverse for unit quaternion)
    std::array<double, 4> q_ref_inv = {q_ref[0], -q_ref[1], -q_ref[2], -q_ref[3]};
    
    // Quaternion multiplication: q_error = q_current * q_ref_inv
    std::array<double, 4> q_error = quaternionMultiply(q_current, q_ref_inv);
    
    // Convert to 3D error vector (2 * vector part for small angles)
    return {2.0 * q_error[1], 2.0 * q_error[2], 2.0 * q_error[3]};
}
std::array<double, 4> MPU9250Sensor::quaternionMultiply(
    const std::array<double, 4>& q1,
    const std::array<double, 4>& q2)
{
    double w1 = q1[0], x1 = q1[1], y1 = q1[2], z1 = q1[3];
    double w2 = q2[0], x2 = q2[1], y2 = q2[2], z2 = q2[3];

    // quaternion 곱: q = q1 * q2
    double w = w1*w2 - x1*x2 - y1*y2 - z1*z2;
    double x = w1*x2 + x1*w2 + y1*z2 - z1*y2;
    double y = w1*y2 - x1*z2 + y1*w2 + z1*x2;
    double z = w1*z2 + x1*y2 - y1*x2 + z1*w2;

    return {w, x, y, z};
}

std::array<double, 9> MPU9250Sensor::get_accel_covariance_(){
  return accel_covariance_;
};
std::array<double, 9> MPU9250Sensor::get_gyro_covariance_(){
  return gyro_covariance_;
};
std::array<double, 9> MPU9250Sensor::get_mag_covariance_(){
  return mag_covariance_;
};
std::array<double, 9> MPU9250Sensor::get_orientation_covariance_(){
  return orientation_covariance_;
}

