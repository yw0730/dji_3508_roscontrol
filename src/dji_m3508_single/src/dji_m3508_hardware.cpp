#include "dji_m3508_single/dji_m3508_hardware.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <iterator>
#include <limits>
#include <string>
#include <unistd.h>

#include "dji_m3508_single/can_utils.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"

namespace
{
constexpr double kTwoPi = 2.0 * M_PI;
constexpr uint32_t kControlFrameId = 0x200;
constexpr int kCanBitrate = 1000000;
}  // namespace

namespace dji_m3508_single
{

hardware_interface::CallbackReturn DjiM3508Hardware::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) !=
      hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (info_.joints.size() != 1) {
    RCLCPP_FATAL(rclcpp::get_logger("DjiM3508Hardware"), "Only one joint is supported.");
    return hardware_interface::CallbackReturn::ERROR;
  }

  // 初始化 command/state 缓冲区。
  hw_positions_.assign(1, 0.0);
  hw_velocities_.assign(1, 0.0);
  hw_commands_.assign(1, 0.0);

  if (info_.hardware_parameters.count("can_interface") > 0) {
    can_interface_ = info_.hardware_parameters.at("can_interface");
  }
  if (info_.hardware_parameters.count("motor_id") > 0) {
    motor_id_ = std::stoi(info_.hardware_parameters.at("motor_id"));
  }
  if (info_.hardware_parameters.count("kp") > 0) {
    kp_ = std::stod(info_.hardware_parameters.at("kp"));
  } else {
    kp_ = 80.0;
  }
  if (info_.hardware_parameters.count("ki") > 0) {
    ki_ = std::stod(info_.hardware_parameters.at("ki"));
  } else {
    ki_ = 0.0;
  }
  if (info_.hardware_parameters.count("kd") > 0) {
    kd_ = std::stod(info_.hardware_parameters.at("kd"));
  } else {
    kd_ = 0.0;
  }
  if (info_.hardware_parameters.count("min_effective_current") > 0) {
    min_effective_current_ = static_cast<int16_t>(
      std::stoi(info_.hardware_parameters.at("min_effective_current")));
  }
  if (info_.hardware_parameters.count("max_current") > 0) {
    max_current_ = static_cast<int16_t>(std::stoi(info_.hardware_parameters.at("max_current")));
  }
  if (info_.hardware_parameters.count("error_deadband") > 0) {
    error_deadband_ = std::stod(info_.hardware_parameters.at("error_deadband"));
  }
  if (info_.hardware_parameters.count("startup_error_threshold") > 0) {
    startup_error_threshold_ = std::stod(info_.hardware_parameters.at("startup_error_threshold"));
  }
  if (info_.hardware_parameters.count("current_slew_rate_per_sec") > 0) {
    current_slew_rate_per_sec_ = std::stod(info_.hardware_parameters.at("current_slew_rate_per_sec"));
  }
  if (info_.hardware_parameters.count("velocity_lpf_alpha") > 0) {
    velocity_lpf_alpha_ = std::stod(info_.hardware_parameters.at("velocity_lpf_alpha"));
  }

  if (motor_id_ < 1 || motor_id_ > 4) {
    RCLCPP_FATAL(rclcpp::get_logger("DjiM3508Hardware"), "motor_id must be in [1,4].");
    return hardware_interface::CallbackReturn::ERROR;
  }
  if (max_current_ <= 0 || max_current_ > 16384) {
    RCLCPP_FATAL(rclcpp::get_logger("DjiM3508Hardware"), "max_current must be in [1,16384].");
    return hardware_interface::CallbackReturn::ERROR;
  }
  if (min_effective_current_ < 0 || min_effective_current_ > max_current_) {
    RCLCPP_FATAL(
      rclcpp::get_logger("DjiM3508Hardware"),
      "min_effective_current must be in [0,max_current].");
    return hardware_interface::CallbackReturn::ERROR;
  }
  if (error_deadband_ < 0.0 || startup_error_threshold_ < 0.0 || current_slew_rate_per_sec_ <= 0.0) {
    RCLCPP_FATAL(
      rclcpp::get_logger("DjiM3508Hardware"),
      "error_deadband/startup_error_threshold must be >=0 and current_slew_rate_per_sec must be >0.");
    return hardware_interface::CallbackReturn::ERROR;
  }
  if (velocity_lpf_alpha_ < 0.0 || velocity_lpf_alpha_ > 1.0) {
    RCLCPP_FATAL(
      rclcpp::get_logger("DjiM3508Hardware"),
      "velocity_lpf_alpha must be in [0,1].");
    return hardware_interface::CallbackReturn::ERROR;
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> DjiM3508Hardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  state_interfaces.emplace_back(info_.joints[0].name, hardware_interface::HW_IF_POSITION, &hw_positions_[0]);
  state_interfaces.emplace_back(info_.joints[0].name, hardware_interface::HW_IF_VELOCITY, &hw_velocities_[0]);
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> DjiM3508Hardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  command_interfaces.emplace_back(
    info_.joints[0].name, hardware_interface::HW_IF_VELOCITY, &hw_commands_[0]);
  return command_interfaces;
}

hardware_interface::CallbackReturn DjiM3508Hardware::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // 尝试在启动时把 CAN 口设置为 1Mbps。
  if (!configure_can_interface(can_interface_, kCanBitrate)) {
    RCLCPP_WARN(
      rclcpp::get_logger("DjiM3508Hardware"),
      "Failed to configure %s bitrate to 1Mbps. Continue opening socket.",
      can_interface_.c_str());
  }

  can_fd_ = open_can_socket(can_interface_);
  if (can_fd_ < 0) {
    RCLCPP_ERROR(
      rclcpp::get_logger("DjiM3508Hardware"),
      "Failed to open CAN socket on %s",
      can_interface_.c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (!set_socket_nonblocking(can_fd_)) {
    RCLCPP_ERROR(rclcpp::get_logger("DjiM3508Hardware"), "Failed to set CAN socket nonblocking.");
    return hardware_interface::CallbackReturn::ERROR;
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DjiM3508Hardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // 激活时清空控制器内部状态，避免历史误差造成冲击。
  integral_error_ = 0.0;
  prev_error_ = 0.0;
  first_feedback_ = true;
  turn_count_ = 0;
  last_raw_angle_ = 0;
  hw_commands_[0] = 0.0;
  last_current_cmd_ = 0.0;
  filtered_velocity_ = 0.0;
  send_zero_current();

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DjiM3508Hardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // 停止时先下发零电流，再关闭 CAN fd。
  send_zero_current();

  if (can_fd_ >= 0) {
    close(can_fd_);
    can_fd_ = -1;
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type DjiM3508Hardware::read(
  const rclcpp::Time & /*time*/,
  const rclcpp::Duration & /*period*/)
{
  if (can_fd_ < 0) {
    return hardware_interface::return_type::ERROR;
  }

  can_frame frame {};
  // 非阻塞读取，尽量消费本周期内所有缓存帧。
  while (read_can_frame(can_fd_, frame)) {
    parse_feedback_frame(frame);
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type DjiM3508Hardware::write(
  const rclcpp::Time & /*time*/,
  const rclcpp::Duration & period)
{
  if (can_fd_ < 0) {
    return hardware_interface::return_type::ERROR;
  }

  const double dt_sec = std::max(period.seconds(), 1e-6);
  const int16_t current_cmd = compute_current_command(dt_sec);

  can_frame tx {};
  tx.can_id = kControlFrameId;
  tx.can_dlc = 8;
  std::fill(std::begin(tx.data), std::end(tx.data), 0);

  // 0x200 帧同时控制 4 个电机，按 motor_id 写入对应两个字节。
  const int index = (motor_id_ - 1) * 2;
  tx.data[index] = static_cast<uint8_t>((current_cmd >> 8) & 0xFF);
  tx.data[index + 1] = static_cast<uint8_t>(current_cmd & 0xFF);

  if (!write_can_frame(can_fd_, tx)) {
    RCLCPP_WARN(rclcpp::get_logger("DjiM3508Hardware"), "Failed to write CAN frame 0x200");
    return hardware_interface::return_type::ERROR;
  }

  return hardware_interface::return_type::OK;
}

bool DjiM3508Hardware::parse_feedback_frame(const can_frame & frame)
{
  const uint32_t expected_id = static_cast<uint32_t>(0x200 + motor_id_);
  if ((frame.can_id & CAN_SFF_MASK) != expected_id || frame.can_dlc < 6) {
    return false;
  }

  const uint16_t raw_angle = static_cast<uint16_t>((frame.data[0] << 8) | frame.data[1]);
  const int16_t raw_rpm = static_cast<int16_t>((frame.data[2] << 8) | frame.data[3]);

  if (first_feedback_) {
    last_raw_angle_ = raw_angle;
    first_feedback_ = false;
  }

  // 基于编码器角度跨零点跳变判断圈数，实现多圈位置累计。
  const int delta = static_cast<int>(raw_angle) - static_cast<int>(last_raw_angle_);
  if (delta > 4096) {
    turn_count_--;
  } else if (delta < -4096) {
    turn_count_++;
  }
  last_raw_angle_ = raw_angle;

  const int32_t multi_turn_ticks = turn_count_ * 8192 + static_cast<int32_t>(raw_angle);

  // 按需求使用 8191 归一化，将编码器值映射到弧度。
  hw_positions_[0] = static_cast<double>(multi_turn_ticks) / 8191.0 * kTwoPi;
  const double raw_velocity = static_cast<double>(raw_rpm) * kTwoPi / 60.0;
  if (first_feedback_) {
    filtered_velocity_ = raw_velocity;
  } else {
    filtered_velocity_ = velocity_lpf_alpha_ * raw_velocity + (1.0 - velocity_lpf_alpha_) * filtered_velocity_;
  }
  hw_velocities_[0] = raw_velocity;

  return true;
}

void DjiM3508Hardware::send_zero_current()
{
  if (can_fd_ < 0) {
    return;
  }

  can_frame tx {};
  tx.can_id = kControlFrameId;
  tx.can_dlc = 8;
  std::fill(std::begin(tx.data), std::end(tx.data), 0);
  write_can_frame(can_fd_, tx);
}

int16_t DjiM3508Hardware::compute_current_command(double dt_sec)
{
  // velocity(rad/s) -> current 的内部 PID。
  const double raw_error = hw_commands_[0] - filtered_velocity_;
  const double error = (std::abs(raw_error) < error_deadband_) ? 0.0 : raw_error;
  integral_error_ += error * dt_sec;

  // Anti-windup: cap integral term to keep i-output within current limit.
  if (ki_ > 1e-9 || ki_ < -1e-9) {
    const double i_cap = static_cast<double>(max_current_) / std::abs(ki_);
    integral_error_ = std::clamp(integral_error_, -i_cap, i_cap);
  }

  const double derivative = (error - prev_error_) / dt_sec;
  prev_error_ = error;

  const double current = kp_ * error + ki_ * integral_error_ + kd_ * derivative;
  double clamped = std::clamp(
    current,
    -static_cast<double>(max_current_),
    static_cast<double>(max_current_));

  // Only apply startup boost when far from target, to avoid steady-state chatter.
  if (
    std::abs(error) > startup_error_threshold_ &&
    std::abs(filtered_velocity_) < 5.0 &&
    std::abs(clamped) < min_effective_current_)
  {
    clamped = std::copysign(static_cast<double>(min_effective_current_), error);
  }

  // Slew-limit current command to reduce abrupt torque reversals and vibration.
  const double max_step = current_slew_rate_per_sec_ * dt_sec;
  const double delta = std::clamp(clamped - last_current_cmd_, -max_step, max_step);
  const double slewed = last_current_cmd_ + delta;
  last_current_cmd_ = slewed;

  return static_cast<int16_t>(std::lround(slewed));
}

}  // namespace dji_m3508_single

PLUGINLIB_EXPORT_CLASS(dji_m3508_single::DjiM3508Hardware, hardware_interface::SystemInterface)
