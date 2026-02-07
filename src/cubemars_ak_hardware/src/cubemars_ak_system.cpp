#include "cubemars_ak_hardware/cubemars_ak_system.hpp"

#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/logging.hpp>

#include <linux/can.h>

#include <cmath>
#include <cstring>
#include <string>

namespace cubemars_ak_hardware
{

double CubeMarsAKSystemHardware::clamp(double v, double lo, double hi)
{
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

double CubeMarsAKSystemHardware::apply_vel_limit(double cmd_rad_s, double vel_limit_rad_s)
{
  if (vel_limit_rad_s > 0.0) {
    return clamp(cmd_rad_s, -vel_limit_rad_s, +vel_limit_rad_s);
  }
  return cmd_rad_s;
}

hardware_interface::CallbackReturn CubeMarsAKSystemHardware::on_init(const hardware_interface::HardwareInfo & info)
{
  // Let the base class store `info_` and do basic validation.
  if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  component_name_ = info_.name;

  // Read required CAN interface name from <hardware> parameters.
  auto it = info_.hardware_parameters.find("can_interface");
  if (it == info_.hardware_parameters.end()) {
    RCLCPP_ERROR(logger_, "[%s] Missing required hardware param 'can_interface' (e.g. can0/can1)",
                component_name_.c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }
  can_interface_ = it->second;

  // Optional: open-loop feedback mode (recommended while IDs are duplicated / feedback is unreliable).
  auto it_fb = info_.hardware_parameters.find("open_loop_feedback");
  if (it_fb != info_.hardware_parameters.end()) {
    open_loop_feedback_ = (std::stoi(it_fb->second) != 0);
  }

  joints_.clear();
  canid_to_index_.clear();

  // Build Joint configs from URDF joint entries.
  for (std::size_t i = 0; i < info_.joints.size(); ++i) {
    const auto & ji = info_.joints[i];

    // We expect to command velocity for wheels.
    bool has_velocity_cmd = false;
    for (const auto & ci : ji.command_interfaces) {
      if (ci.name == hardware_interface::HW_IF_VELOCITY) {
        has_velocity_cmd = true;
      }
    }
    if (!has_velocity_cmd) {
      RCLCPP_ERROR(logger_, "[%s] Joint '%s' must expose a 'velocity' command_interface",
                  component_name_.c_str(), ji.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }

    Joint j;
    j.name = ji.name;

    // Helper lambda to fetch joint params with friendly errors.
    auto require_param = [&](const std::string & key) -> std::string {
      auto pit = ji.parameters.find(key);
      if (pit == ji.parameters.end()) {
        RCLCPP_ERROR(logger_, "[%s] Joint '%s' missing required param '%s'",
                    component_name_.c_str(), ji.name.c_str(), key.c_str());
        throw std::runtime_error("missing_param");
      }
      return pit->second;
    };

    try {
      // stoi(..., base=0) accepts "10" or "0x0A"
      j.can_id = static_cast<uint8_t>(std::stoi(require_param("can_id"), nullptr, 0));
      j.pole_pairs = std::stoi(require_param("pole_pairs"));
      j.gear_ratio = std::stod(require_param("gear_ratio"));
      j.kt = std::stod(require_param("kt"));

      // Optional params
      auto opt = ji.parameters.find("direction");
      if (opt != ji.parameters.end()) {
        j.direction = std::stoi(opt->second);
        if (j.direction != 1 && j.direction != -1) {
          RCLCPP_ERROR(logger_, "[%s] Joint '%s' param 'direction' must be +1 or -1",
                      component_name_.c_str(), ji.name.c_str());
          return hardware_interface::CallbackReturn::ERROR;
        }
      }

      opt = ji.parameters.find("vel_limit");
      if (opt != ji.parameters.end()) {
        j.vel_limit_rad_s = std::stod(opt->second);
        if (j.vel_limit_rad_s < 0.0) {
          RCLCPP_ERROR(logger_, "[%s] Joint '%s' param 'vel_limit' must be >= 0",
                      component_name_.c_str(), ji.name.c_str());
          return hardware_interface::CallbackReturn::ERROR;
        }
      }

      opt = ji.parameters.find("read_only");
      if (opt != ji.parameters.end()) {
        j.read_only = (std::stoi(opt->second) != 0);
      }
    } catch (...) {
      return hardware_interface::CallbackReturn::ERROR;
    }

    // Within a SINGLE hardware instance (one CAN interface), we still require unique can_id.
    // If your bus truly has multiple devices with same ID, you should model ONE actuated joint and
    // let the extra wheels be mimic joints in URDF (robot_state_publisher), or wait until IDs are unique.
    if (canid_to_index_.count(j.can_id)) {
      RCLCPP_ERROR(logger_, "[%s] Duplicate can_id %u within one hardware instance. "
                           "Model only one actuated joint per duplicated-ID bus.",
                  component_name_.c_str(), j.can_id);
      return hardware_interface::CallbackReturn::ERROR;
    }

    canid_to_index_[j.can_id] = joints_.size();
    joints_.push_back(j);
  }

  RCLCPP_INFO(logger_, "[%s] Initialized CubeMarsAKSystemHardware with %zu joint(s) on %s (open_loop_feedback=%d)",
              component_name_.c_str(), joints_.size(), can_interface_.c_str(), open_loop_feedback_ ? 1 : 0);

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> CubeMarsAKSystemHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> out;
  out.reserve(joints_.size() * 4);

  for (auto & j : joints_) {
    out.emplace_back(j.name, hardware_interface::HW_IF_POSITION, &j.pos_rad);
    out.emplace_back(j.name, hardware_interface::HW_IF_VELOCITY, &j.vel_rad_s);
    out.emplace_back(j.name, hardware_interface::HW_IF_EFFORT, &j.effort_nm);

    // Non-standard but useful for debugging. Controllers may ignore it.
    out.emplace_back(j.name, "temperature", &j.temperature_c);
  }
  return out;
}

std::vector<hardware_interface::CommandInterface> CubeMarsAKSystemHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> out;
  out.reserve(joints_.size());

  for (auto & j : joints_) {
    out.emplace_back(j.name, hardware_interface::HW_IF_VELOCITY, &j.cmd_vel_rad_s);
  }
  return out;
}

hardware_interface::CallbackReturn CubeMarsAKSystemHardware::on_activate(const rclcpp_lifecycle::State &)
{
  std::string err;
  if (!can_.open(can_interface_, &err)) {
    RCLCPP_ERROR(logger_, "[%s] Failed to open SocketCAN interface '%s': %s",
                component_name_.c_str(), can_interface_.c_str(), err.c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Zero commands/states on activation.
  for (auto & j : joints_) {
    j.cmd_vel_rad_s = 0.0;
    j.vel_rad_s = 0.0;
    j.effort_nm = 0.0;
    j.temperature_c = 0.0;
    j.error_code = 0;
    j.got_status_this_cycle = false;
  }

  // Send a zero-speed command to each motor (unless read_only).
  for (const auto & j : joints_) {
    if (!j.read_only) {
      (void)send_speed_command_erpm(j.can_id, 0);
    }
  }

  RCLCPP_INFO(logger_, "[%s] Activated on %s", component_name_.c_str(), can_interface_.c_str());
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn CubeMarsAKSystemHardware::on_deactivate(const rclcpp_lifecycle::State &)
{
  // Best effort: stop motors.
  for (const auto & j : joints_) {
    if (!j.read_only) {
      (void)send_speed_command_erpm(j.can_id, 0);
    }
  }

  can_.close();
  RCLCPP_INFO(logger_, "[%s] Deactivated", component_name_.c_str());
  return hardware_interface::CallbackReturn::SUCCESS;
}

bool CubeMarsAKSystemHardware::send_speed_command_erpm(uint8_t can_id, int32_t erpm)
{
  // CubeMars servo mode uses extended CAN frames. Speed mode is 3.
  // Extended ID: (mode << 8) | can_id
  const uint32_t CONTROL_MODE_SPEED = 3u;
  const uint32_t ext_id = (CONTROL_MODE_SPEED << 8) | static_cast<uint32_t>(can_id);

  can_frame frame;
  std::memset(&frame, 0, sizeof(frame));

  frame.can_id = CAN_EFF_FLAG | (ext_id & CAN_EFF_MASK);
  frame.can_dlc = 4;

  // Big-endian int32 payload
  frame.data[0] = static_cast<uint8_t>((erpm >> 24) & 0xFF);
  frame.data[1] = static_cast<uint8_t>((erpm >> 16) & 0xFF);
  frame.data[2] = static_cast<uint8_t>((erpm >>  8) & 0xFF);
  frame.data[3] = static_cast<uint8_t>((erpm >>  0) & 0xFF);

  std::string err;
  if (!can_.send(frame, &err)) {
    RCLCPP_WARN_THROTTLE(logger_, throttle_clock_, 2000,
                         "[%s] CAN send failed (if=%s id=%u): %s",
                         component_name_.c_str(), can_interface_.c_str(), can_id, err.c_str());
    return false;
  }
  return true;
}

void CubeMarsAKSystemHardware::handle_status_frame(const can_frame & frame, const rclcpp::Duration & period)
{
  // Only parse extended frames with 8-byte payloads (per CubeMars status upload format).
  if ((frame.can_id & CAN_EFF_FLAG) == 0) {
    return;
  }
  if (frame.can_dlc != 8) {
    return;
  }

  const uint32_t raw_id = frame.can_id & CAN_EFF_MASK;
  const uint8_t can_id = static_cast<uint8_t>(raw_id & 0xFF);

  auto it = canid_to_index_.find(can_id);
  if (it == canid_to_index_.end()) {
    return;  // not one of our motors
  }

  Joint & j = joints_[it->second];

  // Decode (big-endian):
  // position int16: Data[0..1] -> pos_deg = pos_int * 0.1
  // speed int16:    Data[2..3] -> erpm    = spd_int * 10.0
  // current int16:  Data[4..5] -> amps    = cur_int * 0.01
  // temp int8:      Data[6]
  // error uint8:    Data[7]
  const int16_t pos_int = static_cast<int16_t>((static_cast<uint16_t>(frame.data[0]) << 8) | frame.data[1]);
  const int16_t spd_int = static_cast<int16_t>((static_cast<uint16_t>(frame.data[2]) << 8) | frame.data[3]);
  const int16_t cur_int = static_cast<int16_t>((static_cast<uint16_t>(frame.data[4]) << 8) | frame.data[5]);
  const int8_t temp_i8 = static_cast<int8_t>(frame.data[6]);
  const uint8_t err_u8 = frame.data[7];

  (void)pos_int;  // We integrate position from velocity for continuity.

  const double erpm = static_cast<double>(spd_int) * 10.0;
  const double current_a = static_cast<double>(cur_int) * 0.01;

  // Convert ERPM -> wheel rad/s:
  // rotor_rpm = erpm / pole_pairs
  // output_rpm = rotor_rpm / gear_ratio
  // rad/s = rpm * 2*pi/60
  const double rotor_rpm = erpm / static_cast<double>(j.pole_pairs);
  const double output_rpm = rotor_rpm / j.gear_ratio;
  const double wheel_rad_s = output_rpm * (2.0 * M_PI / 60.0);

  // Estimate output torque
  const double torque_out = current_a * j.kt * j.gear_ratio;

  // Apply direction so joint sign matches URDF/controller convention.
  j.vel_rad_s = wheel_rad_s * static_cast<double>(j.direction);
  j.effort_nm = torque_out * static_cast<double>(j.direction);
  j.temperature_c = static_cast<double>(temp_i8);
  j.error_code = err_u8;
  j.got_status_this_cycle = true;

  // Integrate wheel position using velocity and control period.
  const double dt = period.seconds();
  j.pos_rad += j.vel_rad_s * dt;
}

hardware_interface::return_type CubeMarsAKSystemHardware::read(
  const rclcpp::Time &, const rclcpp::Duration & period)
{
  if (open_loop_feedback_) {
    // Synthesize state from command.
    // We apply the same vel_limit clamp used in write(), so joint_states reflect
    // what we actually command.
    const double dt = period.seconds();

    for (auto & j : joints_) {
      const double effective_cmd = apply_vel_limit(j.cmd_vel_rad_s, j.vel_limit_rad_s);

      j.vel_rad_s = effective_cmd;
      j.pos_rad += j.vel_rad_s * dt;

      // Unknown in open-loop (leave at 0). You could also set NaN if you prefer.
      j.effort_nm = 0.0;
      j.temperature_c = 0.0;
      j.error_code = 0;
      j.got_status_this_cycle = true;
    }

    return hardware_interface::return_type::OK;
  }

  // Normal feedback mode: reset status flags each cycle.
  for (auto & j : joints_) {
    j.got_status_this_cycle = false;
  }

  // Drain all available CAN frames this cycle.
  for (;;) {
    std::string err;
    auto opt = can_.recv(&err);
    if (!opt.has_value()) {
      break;
    }
    handle_status_frame(opt.value(), period);
  }

  // Warn if feedback is missing. (Skip in open_loop_feedback_ mode.)
  for (const auto & j : joints_) {
    if (!j.got_status_this_cycle) {
      RCLCPP_WARN_THROTTLE(logger_, throttle_clock_, 2000,
                           "[%s] No status frame received this cycle for joint '%s' (if=%s can_id=%u)",
                           component_name_.c_str(), j.name.c_str(), can_interface_.c_str(), j.can_id);
    }
    if (j.error_code != 0) {
      RCLCPP_WARN_THROTTLE(logger_, throttle_clock_, 2000,
                           "[%s] Motor reported error_code=%u for joint '%s' (if=%s can_id=%u)",
                           component_name_.c_str(), j.error_code, j.name.c_str(), can_interface_.c_str(), j.can_id);
    }
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type CubeMarsAKSystemHardware::write(
  const rclcpp::Time &, const rclcpp::Duration &)
{
  // Convert each joint's cmd_vel (rad/s) to ERPM and send.
  // CubeMars speed mode command is int32 ERPM; we clamp to a conservative range.
  for (const auto & j : joints_) {
    if (j.read_only) {
      continue;
    }

    // Clamp in joint-space first.
    double cmd_joint = apply_vel_limit(j.cmd_vel_rad_s, j.vel_limit_rad_s);

    // Apply direction for the motor command.
    const double cmd_motor_joint = cmd_joint * static_cast<double>(j.direction);

    // wheel rad/s -> output RPM
    const double output_rpm = cmd_motor_joint * (60.0 / (2.0 * M_PI));

    // output RPM -> rotor RPM -> ERPM
    const double rotor_rpm = output_rpm * j.gear_ratio;
    const double erpm_f = rotor_rpm * static_cast<double>(j.pole_pairs);

    // Clamp to a conservative ERPM range
    const double erpm_clamped = clamp(erpm_f, -100000.0, +100000.0);
    const int32_t erpm_i = static_cast<int32_t>(std::llround(erpm_clamped));

    (void)send_speed_command_erpm(j.can_id, erpm_i);
  }

  return hardware_interface::return_type::OK;
}

}  // namespace cubemars_ak_hardware

PLUGINLIB_EXPORT_CLASS(
  cubemars_ak_hardware::CubeMarsAKSystemHardware,
  hardware_interface::SystemInterface)
