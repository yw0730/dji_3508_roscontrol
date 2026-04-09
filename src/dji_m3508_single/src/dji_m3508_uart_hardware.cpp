#include "dji_m3508_single/dji_m3508_uart_hardware.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <thread>
#include <vector>

#include "dji_m3508_single/serial_utils.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"

namespace dji_m3508_single
{

namespace
{
constexpr std::size_t kPayloadSetVelocityLen = 7;
constexpr std::size_t kPayloadSetModeLen = 1;
constexpr std::size_t kPayloadMotorStateLen = 14;
constexpr std::size_t kPayloadDiagStateLen = 8;
constexpr std::size_t kPayloadAckLen = 5;

constexpr auto kAckWaitTimeout = std::chrono::milliseconds(100);
constexpr auto kMotorStateWarningTimeout = std::chrono::milliseconds(100);
constexpr auto kMotorStateErrorTimeout = std::chrono::milliseconds(500);
constexpr auto kWarningLogInterval = std::chrono::milliseconds(1000);

uint16_t read_le16(const std::vector<uint8_t> & payload, std::size_t offset)
{
  return static_cast<uint16_t>(
    static_cast<uint16_t>(payload[offset]) |
    (static_cast<uint16_t>(payload[offset + 1]) << 8));
}

int16_t read_le16_signed(const std::vector<uint8_t> & payload, std::size_t offset)
{
  return static_cast<int16_t>(read_le16(payload, offset));
}

float read_le_float32(const std::vector<uint8_t> & payload, std::size_t offset)
{
  uint32_t raw =
    static_cast<uint32_t>(payload[offset]) |
    (static_cast<uint32_t>(payload[offset + 1]) << 8) |
    (static_cast<uint32_t>(payload[offset + 2]) << 16) |
    (static_cast<uint32_t>(payload[offset + 3]) << 24);

  float out = 0.0F;
  std::memcpy(&out, &raw, sizeof(float));
  return out;
}

void append_le16(std::vector<uint8_t> & out, uint16_t value)
{
  out.push_back(static_cast<uint8_t>(value & 0xFF));
  out.push_back(static_cast<uint8_t>((value >> 8) & 0xFF));
}

void append_le_float32(std::vector<uint8_t> & out, float value)
{
  uint32_t raw = 0;
  std::memcpy(&raw, &value, sizeof(float));
  out.push_back(static_cast<uint8_t>(raw & 0xFF));
  out.push_back(static_cast<uint8_t>((raw >> 8) & 0xFF));
  out.push_back(static_cast<uint8_t>((raw >> 16) & 0xFF));
  out.push_back(static_cast<uint8_t>((raw >> 24) & 0xFF));
}

}  // namespace

hardware_interface::CallbackReturn DjiM3508UartHardware::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (info_.joints.size() != 1) {
    RCLCPP_FATAL(rclcpp::get_logger("DjiM3508UartHardware"), "Only one joint is supported.");
    return hardware_interface::CallbackReturn::ERROR;
  }

  hw_positions_.assign(1, 0.0);
  hw_velocities_.assign(1, 0.0);
  hw_commands_.assign(1, 0.0);

  if (info_.hardware_parameters.count("serial_port") > 0) {
    serial_port_ = info_.hardware_parameters.at("serial_port");
  }
  if (info_.hardware_parameters.count("baudrate") > 0) {
    baudrate_ = std::stoi(info_.hardware_parameters.at("baudrate"));
  }
  if (info_.hardware_parameters.count("motor_id") > 0) {
    motor_id_ = std::stoi(info_.hardware_parameters.at("motor_id"));
  }
  if (info_.hardware_parameters.count("cmd_timeout_ms") > 0) {
    cmd_timeout_ms_ = static_cast<uint16_t>(std::stoul(info_.hardware_parameters.at("cmd_timeout_ms")));
  }
  if (info_.hardware_parameters.count("heartbeat_period_ms") > 0) {
    heartbeat_period_ms_ = static_cast<uint16_t>(
      std::stoul(info_.hardware_parameters.at("heartbeat_period_ms")));
  }

  if (motor_id_ < 1 || motor_id_ > 4) {
    RCLCPP_FATAL(rclcpp::get_logger("DjiM3508UartHardware"), "motor_id must be in [1,4].");
    return hardware_interface::CallbackReturn::ERROR;
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> DjiM3508UartHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  state_interfaces.emplace_back(info_.joints[0].name, hardware_interface::HW_IF_POSITION, &hw_positions_[0]);
  state_interfaces.emplace_back(info_.joints[0].name, hardware_interface::HW_IF_VELOCITY, &hw_velocities_[0]);
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> DjiM3508UartHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  command_interfaces.emplace_back(
    info_.joints[0].name, hardware_interface::HW_IF_VELOCITY, &hw_commands_[0]);
  return command_interfaces;
}

hardware_interface::CallbackReturn DjiM3508UartHardware::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  serial_fd_ = open_serial_port(serial_port_, baudrate_);
  if (serial_fd_ < 0) {
    RCLCPP_ERROR(
      rclcpp::get_logger("DjiM3508UartHardware"),
      "Failed to open serial device %s at %d bps",
      serial_port_.c_str(),
      baudrate_);
    return hardware_interface::CallbackReturn::ERROR;
  }

  decoder_.reset();
  tx_seq_ = 0;
  reset_runtime_state();

  RCLCPP_INFO(
    rclcpp::get_logger("DjiM3508UartHardware"),
    "UART configured on %s (%d 8N1 raw, non-blocking)",
    serial_port_.c_str(),
    baudrate_);

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DjiM3508UartHardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  uint16_t mode_seq = 0;
  if (!send_set_mode(kModeVelocity, &mode_seq)) {
    RCLCPP_ERROR(rclcpp::get_logger("DjiM3508UartHardware"), "Failed to send SET_MODE=VELOCITY");
    return hardware_interface::CallbackReturn::ERROR;
  }

  AckState ack;
  if (!wait_for_ack(mode_seq, kAckWaitTimeout, &ack)) {
    RCLCPP_ERROR(
      rclcpp::get_logger("DjiM3508UartHardware"),
      "No ACK for SET_MODE seq=%u",
      static_cast<unsigned int>(mode_seq));
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (ack.result != kAckOk) {
    if (ack.result == kAckWarn) {
      RCLCPP_WARN(
        rclcpp::get_logger("DjiM3508UartHardware"),
        "SET_MODE ACK=WARN seq=%u err_code=%u, continue activation",
        static_cast<unsigned int>(ack.ack_seq),
        static_cast<unsigned int>(ack.err_code));
    } else {
      RCLCPP_ERROR(
        rclcpp::get_logger("DjiM3508UartHardware"),
        "SET_MODE ACK=ERROR seq=%u err_code=%u",
        static_cast<unsigned int>(ack.ack_seq),
        static_cast<unsigned int>(ack.err_code));
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  hw_commands_[0] = 0.0;
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DjiM3508UartHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  send_set_mode(kModeDisable, nullptr);

  if (serial_fd_ >= 0) {
    close_serial_port(serial_fd_);
    serial_fd_ = -1;
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type DjiM3508UartHardware::read(
  const rclcpp::Time & /*time*/,
  const rclcpp::Duration & /*period*/)
{
  if (serial_fd_ < 0) {
    return hardware_interface::return_type::ERROR;
  }

  std::array<uint8_t, 512> rx_buf {};
  while (true) {
    const ssize_t n = read_serial_bytes(serial_fd_, rx_buf.data(), rx_buf.size());
    if (n < 0) {
      RCLCPP_ERROR(rclcpp::get_logger("DjiM3508UartHardware"), "Serial read failed");
      return hardware_interface::return_type::ERROR;
    }
    if (n == 0) {
      break;
    }

    const auto frames = decoder_.push_bytes(rx_buf.data(), static_cast<std::size_t>(n));
    for (const auto & frame : frames) {
      process_frame(frame);
    }
  }

  const auto now = std::chrono::steady_clock::now();
  const auto silence = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_motor_state_time_);

  if (silence > kMotorStateErrorTimeout) {
    RCLCPP_ERROR(
      rclcpp::get_logger("DjiM3508UartHardware"),
      "No MOTOR_STATE for %lld ms, entering ERROR",
      static_cast<long long>(silence.count()));
    return hardware_interface::return_type::ERROR;
  }

  if (silence > kMotorStateWarningTimeout && (now - last_warning_log_time_) > kWarningLogInterval) {
    RCLCPP_WARN(
      rclcpp::get_logger("DjiM3508UartHardware"),
      "No MOTOR_STATE for %lld ms",
      static_cast<long long>(silence.count()));
    last_warning_log_time_ = now;
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type DjiM3508UartHardware::write(
  const rclcpp::Time & /*time*/,
  const rclcpp::Duration & /*period*/)
{
  if (serial_fd_ < 0) {
    return hardware_interface::return_type::ERROR;
  }

  if (!send_set_velocity(hw_commands_[0], cmd_timeout_ms_, nullptr)) {
    RCLCPP_ERROR(rclcpp::get_logger("DjiM3508UartHardware"), "Failed to send SET_VELOCITY");
    return hardware_interface::return_type::ERROR;
  }

  const auto now = std::chrono::steady_clock::now();
  const auto elapsed_heartbeat = std::chrono::duration_cast<std::chrono::milliseconds>(
    now - last_heartbeat_time_);
  if (elapsed_heartbeat.count() >= heartbeat_period_ms_) {
    if (!send_heartbeat()) {
      RCLCPP_WARN(rclcpp::get_logger("DjiM3508UartHardware"), "Failed to send HEARTBEAT");
    }
    last_heartbeat_time_ = now;
  }

  return hardware_interface::return_type::OK;
}

bool DjiM3508UartHardware::send_frame(
  uint8_t type,
  const std::vector<uint8_t> & payload,
  uint16_t * seq_out)
{
  const uint16_t seq = tx_seq_++;
  const std::vector<uint8_t> frame = encode_frame(type, seq, now_ms(), payload);
  if (!write_serial_bytes(serial_fd_, frame.data(), frame.size())) {
    return false;
  }

  if (seq_out != nullptr) {
    *seq_out = seq;
  }
  return true;
}

bool DjiM3508UartHardware::send_set_mode(uint8_t mode, uint16_t * seq_out)
{
  std::vector<uint8_t> payload;
  payload.reserve(kPayloadSetModeLen);
  payload.push_back(mode);
  return send_frame(kMsgSetMode, payload, seq_out);
}

bool DjiM3508UartHardware::send_set_velocity(double target_vel, uint16_t timeout_ms, uint16_t * seq_out)
{
  std::vector<uint8_t> payload;
  payload.reserve(kPayloadSetVelocityLen);
  payload.push_back(static_cast<uint8_t>(motor_id_));
  append_le_float32(payload, static_cast<float>(target_vel));
  append_le16(payload, timeout_ms);
  return send_frame(kMsgSetVelocity, payload, seq_out);
}

bool DjiM3508UartHardware::send_heartbeat()
{
  std::vector<uint8_t> payload;
  payload.reserve(1);
  payload.push_back(static_cast<uint8_t>(motor_id_));
  return send_frame(kMsgHeartbeat, payload, nullptr);
}

bool DjiM3508UartHardware::wait_for_ack(
  uint16_t target_seq,
  std::chrono::milliseconds timeout,
  AckState * out_ack)
{
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  std::array<uint8_t, 256> rx_buf {};

  while (std::chrono::steady_clock::now() < deadline) {
    const ssize_t n = read_serial_bytes(serial_fd_, rx_buf.data(), rx_buf.size());
    if (n < 0) {
      return false;
    }

    if (n > 0) {
      const auto frames = decoder_.push_bytes(rx_buf.data(), static_cast<std::size_t>(n));
      for (const auto & frame : frames) {
        process_frame(frame);
        if (
          last_ack_.valid &&
          last_ack_.ack_seq == target_seq)
        {
          if (out_ack != nullptr) {
            *out_ack = last_ack_;
          }
          return true;
        }
      }
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }

  return false;
}

void DjiM3508UartHardware::process_frame(const DecodedFrame & frame)
{
  switch (frame.type) {
    case kMsgMotorState:
      parse_motor_state(frame.payload);
      break;
    case kMsgDiagState:
      parse_diag_state(frame.payload);
      break;
    case kMsgAck:
      parse_ack(frame.payload);
      break;
    default:
      break;
  }
}

void DjiM3508UartHardware::parse_motor_state(const std::vector<uint8_t> & payload)
{
  if (payload.size() != kPayloadMotorStateLen) {
    RCLCPP_WARN(rclcpp::get_logger("DjiM3508UartHardware"), "Invalid MOTOR_STATE len=%zu", payload.size());
    return;
  }

  const uint8_t motor_id = payload[0];
  if (motor_id != static_cast<uint8_t>(motor_id_)) {
    return;
  }

  const float position = read_le_float32(payload, 1);
  const float velocity = read_le_float32(payload, 5);
  const int16_t current = read_le16_signed(payload, 9);
  const uint8_t temperature = payload[11];
  const uint16_t status_bits = read_le16(payload, 12);

  hw_positions_[0] = static_cast<double>(position);
  hw_velocities_[0] = static_cast<double>(velocity);
  last_motor_state_time_ = std::chrono::steady_clock::now();

  if (status_bits != 0) {
    RCLCPP_WARN(
      rclcpp::get_logger("DjiM3508UartHardware"),
      "MOTOR_STATE status_bits=0x%04x current=%d temp=%u",
      static_cast<unsigned int>(status_bits),
      static_cast<int>(current),
      static_cast<unsigned int>(temperature));
  }
}

void DjiM3508UartHardware::parse_diag_state(const std::vector<uint8_t> & payload)
{
  if (payload.size() != kPayloadDiagStateLen) {
    RCLCPP_WARN(rclcpp::get_logger("DjiM3508UartHardware"), "Invalid DIAG_STATE len=%zu", payload.size());
    return;
  }

  diag_.can_rx_hz = read_le16(payload, 0);
  diag_.can_tx_hz = read_le16(payload, 2);
  diag_.last_err = read_le16(payload, 4);
  diag_.bus_off_cnt = read_le16(payload, 6);
}

void DjiM3508UartHardware::parse_ack(const std::vector<uint8_t> & payload)
{
  if (payload.size() != kPayloadAckLen) {
    RCLCPP_WARN(rclcpp::get_logger("DjiM3508UartHardware"), "Invalid ACK len=%zu", payload.size());
    return;
  }

  last_ack_.valid = true;
  last_ack_.ack_seq = read_le16(payload, 0);
  last_ack_.result = payload[2];
  last_ack_.err_code = read_le16(payload, 3);
  last_ack_.recv_time = std::chrono::steady_clock::now();

  if (last_ack_.result == kAckWarn) {
    RCLCPP_WARN(
      rclcpp::get_logger("DjiM3508UartHardware"),
      "ACK WARN seq=%u err_code=%u",
      static_cast<unsigned int>(last_ack_.ack_seq),
      static_cast<unsigned int>(last_ack_.err_code));
  } else if (last_ack_.result == kAckError) {
    RCLCPP_WARN(
      rclcpp::get_logger("DjiM3508UartHardware"),
      "ACK ERROR seq=%u err_code=%u",
      static_cast<unsigned int>(last_ack_.ack_seq),
      static_cast<unsigned int>(last_ack_.err_code));
  } else {
    RCLCPP_DEBUG(
      rclcpp::get_logger("DjiM3508UartHardware"),
      "ACK OK seq=%u",
      static_cast<unsigned int>(last_ack_.ack_seq));
  }
}

void DjiM3508UartHardware::reset_runtime_state()
{
  start_time_ = std::chrono::steady_clock::now();
  last_motor_state_time_ = start_time_;
  last_warning_log_time_ = start_time_;
  last_heartbeat_time_ = start_time_;
  last_ack_ = AckState{};
  diag_ = DiagState{};
}

uint32_t DjiM3508UartHardware::now_ms() const
{
  const auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
    std::chrono::steady_clock::now() - start_time_);
  return static_cast<uint32_t>(elapsed.count());
}

}  // namespace dji_m3508_single

PLUGINLIB_EXPORT_CLASS(dji_m3508_single::DjiM3508UartHardware, hardware_interface::SystemInterface)
