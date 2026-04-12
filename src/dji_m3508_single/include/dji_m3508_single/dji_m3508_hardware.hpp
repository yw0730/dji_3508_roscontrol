#ifndef DJI_M3508_SINGLE__DJI_M3508_HARDWARE_HPP_
#define DJI_M3508_SINGLE__DJI_M3508_HARDWARE_HPP_

#include <cstdint>
#include <linux/can.h>
#include <string>
#include <vector>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace dji_m3508_single
{

class DjiM3508Hardware : public hardware_interface::SystemInterface
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
  bool parse_feedback_frame(const can_frame & frame);
  void send_zero_current();
  int16_t compute_current_command(double dt_sec);

  std::vector<double> hw_positions_;
  std::vector<double> hw_velocities_;
  std::vector<double> hw_commands_;

  std::string can_interface_ {"can0"};
  int motor_id_ {1};
  int can_fd_ {-1};

  double kp_ {10.0};
  double ki_ {0.1};
  double kd_ {0.01};
  int16_t min_effective_current_ {800};
  int16_t max_current_ {16384};
  double error_deadband_ {0.6};
  double startup_error_threshold_ {2.0};
  double current_slew_rate_per_sec_ {8000.0};
  double velocity_lpf_alpha_ {0.2};
  double last_current_cmd_ {0.0};
  double filtered_velocity_ {0.0};

  double integral_error_ {0.0};
  double prev_error_ {0.0};

  bool first_feedback_ {true};
  int32_t turn_count_ {0};
  uint16_t last_raw_angle_ {0};
};

}  // namespace dji_m3508_single

#endif  // DJI_M3508_SINGLE__DJI_M3508_HARDWARE_HPP_
