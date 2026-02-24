#include <algorithm>
#include <cmath>
#include <limits>
#include <string>
#include <vector>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"

namespace slow_fake_gripper_hardware
{

class SlowFakeSystem : public hardware_interface::SystemInterface
{
public:
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override
  {
    if (hardware_interface::SystemInterface::on_init(info) !=
        hardware_interface::CallbackReturn::SUCCESS)
    {
      return hardware_interface::CallbackReturn::ERROR;
    }

    // Param: max_velocity (units match your joint position units per second)
    // For your prismatic finger joint [0..0.025], something like 0.02 m/s feels reasonable.
    max_velocity_ = 0.02;  // default
    if (info_.hardware_parameters.count("max_velocity"))
    {
      max_velocity_ = std::stod(info_.hardware_parameters.at("max_velocity"));
    }

    // Allocate for all joints defined in <ros2_control> block
    const size_t n = info_.joints.size();
    cmd_pos_.assign(n, 0.0);
    state_pos_.assign(n, 0.0);
    state_vel_.assign(n, 0.0);

    // Initialize from URDF <state_interface initial_value> if provided
    for (size_t i = 0; i < n; ++i)
    {
      state_pos_[i] = 0.0;
      for (const auto & si : info_.joints[i].state_interfaces)
      {
        if (si.name == hardware_interface::HW_IF_POSITION)
        {
          auto it = si.parameters.find("initial_value");
          if (it != si.parameters.end())
          {
            state_pos_[i] = std::stod(it->second);
          }
        }
      }
      cmd_pos_[i] = state_pos_[i];
    }

    RCLCPP_INFO(
      rclcpp::get_logger("SlowFakeSystem"),
      "Initialized slow fake system with %zu joint(s), max_velocity=%.6f units/s",
      n, max_velocity_);

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override
  {
    std::vector<hardware_interface::StateInterface> state_interfaces;
    state_interfaces.reserve(info_.joints.size() * 2);

    for (size_t i = 0; i < info_.joints.size(); ++i)
    {
      state_interfaces.emplace_back(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION, &state_pos_[i]);
      state_interfaces.emplace_back(
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &state_vel_[i]);
    }
    return state_interfaces;
  }

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override
  {
    std::vector<hardware_interface::CommandInterface> command_interfaces;
    command_interfaces.reserve(info_.joints.size());

    for (size_t i = 0; i < info_.joints.size(); ++i)
    {
      command_interfaces.emplace_back(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION, &cmd_pos_[i]);
    }
    return command_interfaces;
  }

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State &) override
  {
    // keep current state as command to avoid jump
    cmd_pos_ = state_pos_;
    std::fill(state_vel_.begin(), state_vel_.end(), 0.0);
    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State &) override
  {
    std::fill(state_vel_.begin(), state_vel_.end(), 0.0);
    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::return_type read(
    const rclcpp::Time &, const rclcpp::Duration & period) override
  {
    const double dt = std::max(0.0, period.seconds());
    if (dt <= 0.0)
    {
      std::fill(state_vel_.begin(), state_vel_.end(), 0.0);
      return hardware_interface::return_type::OK;
    }

    // Rate-limit position: state moves toward command at max_velocity_
    for (size_t i = 0; i < state_pos_.size(); ++i)
    {
      const double prev = state_pos_[i];
      const double err = cmd_pos_[i] - state_pos_[i];

      const double max_step = max_velocity_ * dt;
      const double step = std::clamp(err, -max_step, +max_step);

      state_pos_[i] += step;
      state_vel_[i] = (state_pos_[i] - prev) / dt;
    }

    return hardware_interface::return_type::OK;
  }

  hardware_interface::return_type write(
    const rclcpp::Time &, const rclcpp::Duration &) override
  {
    // No-op: commands are already stored in cmd_pos_ via command interfaces.
    // Real hardware would send cmd_pos_ to actuators here.
    return hardware_interface::return_type::OK;
  }

private:
  double max_velocity_{0.02};
  std::vector<double> cmd_pos_;
  std::vector<double> state_pos_;
  std::vector<double> state_vel_;
};

}  // namespace slow_fake_gripper_hardware

PLUGINLIB_EXPORT_CLASS(
  slow_fake_gripper_hardware::SlowFakeSystem,
  hardware_interface::SystemInterface)