#pragma once

#include "cubemars_ak_hardware/socketcan.hpp"

#include <hardware_interface/system_interface.hpp>
#include <rclcpp/clock.hpp>
#include <rclcpp/logger.hpp>

#include <cstdint>
#include <string>
#include <unordered_map>
#include <vector>

namespace cubemars_ak_hardware
{

class CubeMarsAKSystemHardware : public hardware_interface::SystemInterface
{
public:
  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
  hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  struct Joint
  {
    std::string name;

    // Configuration (from URDF ros2_control params)
    uint8_t can_id{0};          // motor node ID (0..255) (your current case: 111)
    int pole_pairs{14};         // AK45-36: 14 pole pairs
    double gear_ratio{36.0};    // AK45-36: 36:1 reduction
    double kt{0.11};            // Nm/A at motor (approx)
    int direction{+1};          // +1 or -1 (flip if motor is mounted/wired reversed)
    double vel_limit_rad_s{0};  // 0 = no limit
    bool read_only{false};      // if true, never send commands

    // Command (what controllers write)
    double cmd_vel_rad_s{0.0};

    // State (what controllers read)
    double pos_rad{0.0};
    double vel_rad_s{0.0};
    double effort_nm{0.0};
    double temperature_c{0.0};

    // Diagnostics
    uint8_t error_code{0};
    bool got_status_this_cycle{false};
  };

  // Identity for logs (set from URDF ros2_control "name=")
  std::string component_name_;

  // Logger and clock for throttle macros (Humble-safe; lvalues, not temporaries)
  rclcpp::Logger logger_{rclcpp::get_logger("cubemars_ak_system")};
  rclcpp::Clock throttle_clock_{RCL_STEADY_TIME};

  // From <ros2_control><hardware><param name="can_interface">
  std::string can_interface_{"can0"};

  // If true, we do not rely on motor status frames for feedback. We synthesize state
  // from the commanded velocity (useful when you have duplicate node IDs and/or disabled status).
  bool open_loop_feedback_{false};

  SocketCAN can_;

  std::vector<Joint> joints_;
  std::unordered_map<uint8_t, std::size_t> canid_to_index_;

  // Helpers
  static double clamp(double v, double lo, double hi);

  // Apply vel_limit clamp in joint-space without applying direction.
  // This is used for both write() and open_loop read() so state matches what we actually command.
  static double apply_vel_limit(double cmd_rad_s, double vel_limit_rad_s);

  bool send_speed_command_erpm(uint8_t can_id, int32_t erpm);
  void handle_status_frame(const struct can_frame & frame, const rclcpp::Duration & period);
};

}  // namespace cubemars_ak_hardware
