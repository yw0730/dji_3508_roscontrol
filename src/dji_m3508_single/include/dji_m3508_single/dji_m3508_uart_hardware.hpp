#ifndef DJI_M3508_SINGLE__DJI_M3508_UART_HARDWARE_HPP_
#define DJI_M3508_SINGLE__DJI_M3508_UART_HARDWARE_HPP_

#include <chrono>
#include <cstdint>
#include <string>
#include <vector>

#include "dji_m3508_single/uart_protocol.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace dji_m3508_single
{

class DjiM3508UartHardware : public hardware_interface::SystemInterface
{
public:
  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type read(
    const rclcpp::Time & time,
    const rclcpp::Duration & period) override;
  hardware_interface::return_type write(
    const rclcpp::Time & time,
    const rclcpp::Duration & period) override;

private:
  struct DiagState
  {
    uint16_t can_rx_hz {0};
    uint16_t can_tx_hz {0};
    uint16_t last_err {0};
    uint16_t bus_off_cnt {0};
  };

  struct AckState
  {
    bool valid {false};
    uint16_t ack_seq {0};
    uint8_t result {kAckError};
    uint16_t err_code {0};
    std::chrono::steady_clock::time_point recv_time {};
  };

  bool send_frame(uint8_t type, const std::vector<uint8_t> & payload, uint16_t * seq_out = nullptr);
  bool send_set_mode(uint8_t mode, uint16_t * seq_out = nullptr);
  bool send_set_velocity(double target_vel, uint16_t timeout_ms, uint16_t * seq_out = nullptr);
  bool send_heartbeat();

  bool wait_for_ack(uint16_t target_seq, std::chrono::milliseconds timeout, AckState * out_ack);
  void process_frame(const DecodedFrame & frame);

  void parse_motor_state(const std::vector<uint8_t> & payload);
  void parse_diag_state(const std::vector<uint8_t> & payload);
  void parse_ack(const std::vector<uint8_t> & payload);

  void reset_runtime_state();
  uint32_t now_ms() const;

  std::vector<double> hw_positions_;
  std::vector<double> hw_velocities_;
  std::vector<double> hw_commands_;

  std::string serial_port_ {"/dev/ttyUSB0"};
  int baudrate_ {921600};
  int motor_id_ {1};
  uint16_t cmd_timeout_ms_ {100};
  uint16_t heartbeat_period_ms_ {20};
  bool strict_activate_ack_ {false};
  bool strict_motor_state_timeout_ {false};

  int serial_fd_ {-1};
  FrameDecoder decoder_;
  uint16_t tx_seq_ {0};

  DiagState diag_;
  AckState last_ack_;

  std::chrono::steady_clock::time_point start_time_;
  std::chrono::steady_clock::time_point last_motor_state_time_;
  std::chrono::steady_clock::time_point last_warning_log_time_;
  std::chrono::steady_clock::time_point last_heartbeat_time_;
};

}  // namespace dji_m3508_single

#endif  // DJI_M3508_SINGLE__DJI_M3508_UART_HARDWARE_HPP_
