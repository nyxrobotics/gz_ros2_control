#pragma once

#include <map>
#include <memory>
#include <string>
#include <vector>

#include <hardware_interface/system_interface.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp>
#include <rclcpp_lifecycle/state.hpp>

#include <gz_ros2_control/gz_system_interface.hpp>

namespace gz_ros2_pid_control {

using CallbackReturn =
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class GazeboSimPIDSystemPrivate;

// Hardware plugin implementing gz_ros2_control::GazeboSimSystemInterface.
// This is a fork of gz_ros2_control::GazeboSimSystem with PID position->effort
// control.
class GazeboSimPIDSystem : public gz_ros2_control::GazeboSimSystemInterface {
public:
  CallbackReturn
  on_init(const hardware_interface::HardwareInfo &system_info) override;
  CallbackReturn
  on_configure(const rclcpp_lifecycle::State &previous_state) override;

  std::vector<hardware_interface::StateInterface>
  export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface>
  export_command_interfaces() override;

  CallbackReturn
  on_activate(const rclcpp_lifecycle::State &previous_state) override;
  CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State &previous_state) override;

  hardware_interface::return_type perform_command_mode_switch(
      const std::vector<std::string> &start_interfaces,
      const std::vector<std::string> &stop_interfaces) override;

  hardware_interface::return_type read(const rclcpp::Time &time,
                                       const rclcpp::Duration &period) override;

  hardware_interface::return_type
  write(const rclcpp::Time &time, const rclcpp::Duration &period) override;

  bool initSim(rclcpp::Node::SharedPtr &model_nh,
               std::map<std::string, sim::Entity> &joints,
               const hardware_interface::HardwareInfo &hardware_info,
               sim::EntityComponentManager &_ecm, int &update_rate) override;

private:
  void registerSensors(const hardware_interface::HardwareInfo &hardware_info);

  std::unique_ptr<GazeboSimPIDSystemPrivate> dataPtr;
};

} // namespace gz_ros2_pid_control
