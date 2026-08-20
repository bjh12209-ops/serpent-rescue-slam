#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>

#include <algorithm>
#include <array>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <fcntl.h>
#include <memory>
#include <poll.h>
#include <stdexcept>
#include <string>
#include <termios.h>
#include <thread>
#include <unistd.h>
#include <vector>

namespace
{
constexpr double kGravity = 9.80665;
constexpr double kDegToRad = 3.14159265358979323846 / 180.0;
constexpr uint16_t kFirstDataRegister = 0x0034;
constexpr uint16_t kRegisterCount = 12;  // accel, gyro, mag, roll/pitch/yaw
constexpr std::size_t kResponseSize = 5 + 2 * kRegisterCount;

uint16_t modbus_crc(const uint8_t * data, std::size_t length)
{
  uint16_t crc = 0xFFFF;
  for (std::size_t i = 0; i < length; ++i) {
    crc ^= data[i];
    for (int bit = 0; bit < 8; ++bit) {
      crc = (crc & 1U) ? static_cast<uint16_t>((crc >> 1U) ^ 0xA001U) :
        static_cast<uint16_t>(crc >> 1U);
    }
  }
  return crc;
}

int16_t signed_register(const uint8_t * bytes)
{
  return static_cast<int16_t>(
    (static_cast<uint16_t>(bytes[0]) << 8U) | static_cast<uint16_t>(bytes[1]));
}

speed_t baud_constant(int baud)
{
  switch (baud) {
    case 9600: return B9600;
    case 19200: return B19200;
    case 38400: return B38400;
    case 57600: return B57600;
    case 115200: return B115200;
    case 230400: return B230400;
    default: throw std::invalid_argument("unsupported baud rate: " + std::to_string(baud));
  }
}

struct EulerQuaternion
{
  double x;
  double y;
  double z;
  double w;
};

EulerQuaternion quaternion_from_rpy(double roll, double pitch, double yaw)
{
  const double cr = std::cos(roll * 0.5);
  const double sr = std::sin(roll * 0.5);
  const double cp = std::cos(pitch * 0.5);
  const double sp = std::sin(pitch * 0.5);
  const double cy = std::cos(yaw * 0.5);
  const double sy = std::sin(yaw * 0.5);
  return {
    sr * cp * cy - cr * sp * sy,
    cr * sp * cy + sr * cp * sy,
    cr * cp * sy - sr * sp * cy,
    cr * cp * cy + sr * sp * sy};
}
}  // namespace

class SerialPort
{
public:
  SerialPort(const std::string & path, int baud)
  {
    fd_ = ::open(path.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK | O_CLOEXEC);
    if (fd_ < 0) {
      throw std::runtime_error("cannot open " + path + ": " + std::strerror(errno));
    }
    if (::tcgetattr(fd_, &saved_) != 0) {
      const std::string error = std::strerror(errno);
      ::close(fd_);
      fd_ = -1;
      throw std::runtime_error("tcgetattr failed: " + error);
    }
    termios config = saved_;
    ::cfmakeraw(&config);
    config.c_cflag &= static_cast<tcflag_t>(~(PARENB | CSTOPB | CSIZE | CRTSCTS));
    config.c_cflag |= static_cast<tcflag_t>(CS8 | CREAD | CLOCAL);
    const speed_t speed = baud_constant(baud);
    ::cfsetispeed(&config, speed);
    ::cfsetospeed(&config, speed);
    config.c_cc[VMIN] = 0;
    config.c_cc[VTIME] = 0;
    if (::tcsetattr(fd_, TCSANOW, &config) != 0) {
      const std::string error = std::strerror(errno);
      ::close(fd_);
      fd_ = -1;
      throw std::runtime_error("tcsetattr failed: " + error);
    }
    ::tcflush(fd_, TCIOFLUSH);
  }

  ~SerialPort()
  {
    if (fd_ >= 0) {
      ::tcsetattr(fd_, TCSANOW, &saved_);
      ::close(fd_);
    }
  }

  SerialPort(const SerialPort &) = delete;
  SerialPort & operator=(const SerialPort &) = delete;

  bool transact(
    uint8_t sensor_id, std::array<uint8_t, kResponseSize> & response,
    std::chrono::microseconds timeout)
  {
    std::array<uint8_t, 8> request{
      sensor_id, 0x03,
      static_cast<uint8_t>(kFirstDataRegister >> 8U),
      static_cast<uint8_t>(kFirstDataRegister & 0xFFU),
      static_cast<uint8_t>(kRegisterCount >> 8U),
      static_cast<uint8_t>(kRegisterCount & 0xFFU), 0, 0};
    const uint16_t crc = modbus_crc(request.data(), 6);
    request[6] = static_cast<uint8_t>(crc & 0xFFU);
    request[7] = static_cast<uint8_t>(crc >> 8U);

    ::tcflush(fd_, TCIFLUSH);
    std::size_t sent = 0;
    while (sent < request.size()) {
      const ssize_t count = ::write(fd_, request.data() + sent, request.size() - sent);
      if (count > 0) {
        sent += static_cast<std::size_t>(count);
      } else if (count < 0 && errno != EINTR && errno != EAGAIN) {
        return false;
      }
    }
    if (::tcdrain(fd_) != 0) {
      return false;
    }

    const auto deadline = std::chrono::steady_clock::now() + timeout;
    std::size_t received = 0;
    while (received < response.size()) {
      const auto now = std::chrono::steady_clock::now();
      if (now >= deadline) {
        return false;
      }
      const auto remaining = std::chrono::duration_cast<std::chrono::microseconds>(deadline - now);
      pollfd descriptor{fd_, POLLIN, 0};
      const int timeout_ms = std::max(1, static_cast<int>((remaining.count() + 999) / 1000));
      const int ready = ::poll(&descriptor, 1, timeout_ms);
      if (ready == 0) {
        return false;
      }
      if (ready < 0) {
        if (errno == EINTR) {
          continue;
        }
        return false;
      }
      const ssize_t count = ::read(fd_, response.data() + received, response.size() - received);
      if (count > 0) {
        received += static_cast<std::size_t>(count);
      } else if (count < 0 && errno != EINTR && errno != EAGAIN) {
        return false;
      }
    }

    const uint16_t received_crc = static_cast<uint16_t>(response[response.size() - 2]) |
      (static_cast<uint16_t>(response[response.size() - 1]) << 8U);
    return response[0] == sensor_id && response[1] == 0x03 &&
           response[2] == 2 * kRegisterCount &&
           modbus_crc(response.data(), response.size() - 2) == received_crc;
  }

private:
  int fd_{-1};
  termios saved_{};
};

class Wt901c485Node : public rclcpp::Node
{
public:
  Wt901c485Node()
  : Node("wt901c485_node")
  {
    const std::string port = declare_parameter<std::string>(
      "port", "/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0");
    const int baud = declare_parameter<int>("baud_rate", 115200);
    const auto ids = declare_parameter<std::vector<int64_t>>("sensor_ids", {81});
    rate_hz_ = declare_parameter<double>("rate_hz", 60.0);
    timeout_ = std::chrono::microseconds(
      declare_parameter<int>("response_timeout_us", 12000));
    offline_retry_ = std::chrono::milliseconds(
      declare_parameter<int>("offline_retry_ms", 1000));
    const double yaw_zero_duration = declare_parameter<double>("yaw_zero_duration_sec", 2.0);
    frame_prefix_ = declare_parameter<std::string>("frame_prefix", "imu");
    const auto orientation_covariance = declare_parameter<std::vector<double>>(
      "orientation_covariance",
      {0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.25});
    const auto angular_velocity_covariance = declare_parameter<std::vector<double>>(
      "angular_velocity_covariance",
      {0.0025, 0.0, 0.0, 0.0, 0.0025, 0.0, 0.0, 0.0, 0.0025});
    const auto linear_acceleration_covariance = declare_parameter<std::vector<double>>(
      "linear_acceleration_covariance",
      {0.04, 0.0, 0.0, 0.0, 0.04, 0.0, 0.0, 0.0, 0.04});

    if (rate_hz_ <= 0.0 || yaw_zero_duration < 0.0 || ids.empty()) {
      throw std::invalid_argument("rate_hz must be positive and sensor_ids must not be empty");
    }
    if (
      orientation_covariance.size() != 9 ||
      angular_velocity_covariance.size() != 9 ||
      linear_acceleration_covariance.size() != 9)
    {
      throw std::invalid_argument("all IMU covariance parameters must contain 9 values");
    }
    std::copy(
      orientation_covariance.begin(), orientation_covariance.end(),
      orientation_covariance_.begin());
    std::copy(
      angular_velocity_covariance.begin(), angular_velocity_covariance.end(),
      angular_velocity_covariance_.begin());
    std::copy(
      linear_acceleration_covariance.begin(), linear_acceleration_covariance.end(),
      linear_acceleration_covariance_.begin());
    yaw_zero_samples_ = std::max<std::size_t>(
      1, static_cast<std::size_t>(std::llround(rate_hz_ * yaw_zero_duration)));
    for (const int64_t id : ids) {
      if (id < 1 || id > 247) {
        throw std::invalid_argument("Modbus sensor ID must be in [1, 247]");
      }
      Sensor sensor;
      sensor.id = static_cast<uint8_t>(id);
      const std::string suffix = hex_id(sensor.id);
      sensor.imu_pub = create_publisher<sensor_msgs::msg::Imu>("imu_" + suffix + "/data", 10);
      sensor.mag_pub = create_publisher<sensor_msgs::msg::MagneticField>("imu_" + suffix + "/mag", 10);
      sensor.euler_pub = create_publisher<geometry_msgs::msg::Vector3Stamped>(
        "imu_" + suffix + "/euler_deg", 10);
      sensors_.push_back(std::move(sensor));
    }

    serial_ = std::make_unique<SerialPort>(port, baud);
    RCLCPP_INFO(
      get_logger(), "Opened %s at %d baud; polling %zu sensor(s) at %.1f Hz",
      port.c_str(), baud, sensors_.size(), rate_hz_);
    RCLCPP_INFO(
      get_logger(), "Keep the robot stationary: collecting %zu samples for yaw zeroing",
      yaw_zero_samples_);
    worker_ = std::thread(&Wt901c485Node::io_loop, this);
  }

  ~Wt901c485Node() override
  {
    stop_.store(true, std::memory_order_release);
    if (worker_.joinable()) {
      worker_.join();
    }
  }

private:
  struct Sensor
  {
    uint8_t id{0};
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub;
    rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr mag_pub;
    rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr euler_pub;
    uint64_t success{0};
    uint64_t failure{0};
    unsigned consecutive_failures{0};
    bool online{true};
    std::chrono::steady_clock::time_point retry_at{};
    double yaw_sin_sum{0.0};
    double yaw_cos_sum{0.0};
    double yaw_offset_rad{0.0};
    std::size_t yaw_sample_count{0};
    bool yaw_zeroed{false};
  };

  static std::string hex_id(uint8_t id)
  {
    constexpr char digits[] = "0123456789abcdef";
    std::string result(2, '0');
    result[0] = digits[(id >> 4U) & 0x0FU];
    result[1] = digits[id & 0x0FU];
    return result;
  }

  void io_loop()
  {
    using Clock = std::chrono::steady_clock;
    const auto period = std::chrono::duration_cast<Clock::duration>(
      std::chrono::duration<double>(1.0 / rate_hz_));
    auto next_cycle = Clock::now();
    auto next_report = next_cycle + std::chrono::seconds(5);
    std::vector<uint64_t> previous_success(sensors_.size(), 0);

    while (!stop_.load(std::memory_order_acquire) && rclcpp::ok()) {
      next_cycle += period;
      for (auto & sensor : sensors_) {
        const auto now = Clock::now();
        if (!sensor.online && now < sensor.retry_at) {
          continue;
        }
        std::array<uint8_t, kResponseSize> response{};
        if (serial_->transact(sensor.id, response, timeout_)) {
          if (!sensor.online) {
            RCLCPP_INFO(get_logger(), "IMU 0x%02X is online", sensor.id);
          }
          sensor.online = true;
          sensor.consecutive_failures = 0;
          ++sensor.success;
          publish(sensor, response);
        } else {
          ++sensor.failure;
          ++sensor.consecutive_failures;
          if (sensor.consecutive_failures >= 3) {
            if (sensor.online) {
              RCLCPP_WARN(get_logger(), "IMU 0x%02X is not responding", sensor.id);
            }
            sensor.online = false;
            sensor.retry_at = Clock::now() + offline_retry_;
          }
        }
      }

      const auto now = Clock::now();
      if (now >= next_report) {
        for (std::size_t i = 0; i < sensors_.size(); ++i) {
          const auto samples = sensors_[i].success - previous_success[i];
          previous_success[i] = sensors_[i].success;
          RCLCPP_INFO(
            get_logger(), "IMU 0x%02X: %.1f Hz, ok=%lu, failed=%lu, %s",
            sensors_[i].id, samples / 5.0, sensors_[i].success, sensors_[i].failure,
            sensors_[i].online ? "online" : "offline");
        }
        next_report = now + std::chrono::seconds(5);
      }

      if (next_cycle > Clock::now()) {
        std::this_thread::sleep_until(next_cycle);
      } else {
        next_cycle = Clock::now();
      }
    }
  }

  void publish(Sensor & sensor, const std::array<uint8_t, kResponseSize> & response)
  {
    std::array<int16_t, kRegisterCount> value{};
    for (std::size_t i = 0; i < value.size(); ++i) {
      value[i] = signed_register(&response[3 + 2 * i]);
    }

    const double accel_scale = 16.0 * kGravity / 32768.0;
    const double gyro_scale = 2000.0 * kDegToRad / 32768.0;
    const double angle_scale = 180.0 * kDegToRad / 32768.0;
    const double roll = value[9] * angle_scale;
    const double pitch = value[10] * angle_scale;
    const double raw_yaw = value[11] * angle_scale;

    if (!sensor.yaw_zeroed) {
      sensor.yaw_sin_sum += std::sin(raw_yaw);
      sensor.yaw_cos_sum += std::cos(raw_yaw);
      ++sensor.yaw_sample_count;
      if (sensor.yaw_sample_count < yaw_zero_samples_) {
        return;
      }
      sensor.yaw_offset_rad = std::atan2(sensor.yaw_sin_sum, sensor.yaw_cos_sum);
      sensor.yaw_zeroed = true;
      RCLCPP_INFO(
        get_logger(), "IMU 0x%02X yaw zeroed: raw offset %.3f deg",
        sensor.id, sensor.yaw_offset_rad / kDegToRad);
    }

    const double relative_yaw = std::remainder(
      raw_yaw - sensor.yaw_offset_rad, 2.0 * 3.14159265358979323846);
    const auto quaternion = quaternion_from_rpy(
      roll, pitch, relative_yaw);
    const auto stamp = now();
    const std::string frame = frame_prefix_ + "_" + hex_id(sensor.id) + "_link";

    sensor_msgs::msg::Imu imu;
    imu.header.stamp = stamp;
    imu.header.frame_id = frame;
    imu.orientation.x = quaternion.x;
    imu.orientation.y = quaternion.y;
    imu.orientation.z = quaternion.z;
    imu.orientation.w = quaternion.w;
    imu.orientation_covariance = orientation_covariance_;
    imu.angular_velocity.x = value[3] * gyro_scale;
    imu.angular_velocity.y = value[4] * gyro_scale;
    imu.angular_velocity.z = value[5] * gyro_scale;
    imu.angular_velocity_covariance = angular_velocity_covariance_;
    imu.linear_acceleration.x = value[0] * accel_scale;
    imu.linear_acceleration.y = value[1] * accel_scale;
    imu.linear_acceleration.z = value[2] * accel_scale;
    imu.linear_acceleration_covariance = linear_acceleration_covariance_;
    sensor.imu_pub->publish(imu);

    // WT901C485 magnetometer resolution: 0.0667 mGauss/LSB = 6.67e-9 tesla/LSB.
    constexpr double magnetic_scale = 6.67e-9;
    sensor_msgs::msg::MagneticField magnetic;
    magnetic.header = imu.header;
    magnetic.magnetic_field.x = value[6] * magnetic_scale;
    magnetic.magnetic_field.y = value[7] * magnetic_scale;
    magnetic.magnetic_field.z = value[8] * magnetic_scale;
    sensor.mag_pub->publish(magnetic);

    geometry_msgs::msg::Vector3Stamped euler;
    euler.header = imu.header;
    euler.vector.x = roll / kDegToRad;
    euler.vector.y = pitch / kDegToRad;
    euler.vector.z = relative_yaw / kDegToRad;
    sensor.euler_pub->publish(euler);
  }

  std::unique_ptr<SerialPort> serial_;
  std::vector<Sensor> sensors_;
  std::thread worker_;
  std::atomic<bool> stop_{false};
  double rate_hz_{60.0};
  std::chrono::microseconds timeout_{12000};
  std::chrono::milliseconds offline_retry_{1000};
  std::size_t yaw_zero_samples_{120};
  std::string frame_prefix_{"imu"};
  std::array<double, 9> orientation_covariance_{};
  std::array<double, 9> angular_velocity_covariance_{};
  std::array<double, 9> linear_acceleration_covariance_{};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<Wt901c485Node>());
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("wt901c485_node"), "%s", error.what());
  }
  rclcpp::shutdown();
  return 0;
}
