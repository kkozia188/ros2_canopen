// Copyright (c) 2022, StoglRobotics
// Copyright (c) 2022, Stogl Robotics Consulting UG (haftungsbeschränkt) (template)
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

//----------------------------------------------------------------------
/*!\file
 *
 * \author  Lovro Ivanov lovro.ivanov@gmail.com
 * \date    2022-08-01
 *
 */
//----------------------------------------------------------------------

#ifndef CANOPEN_ROS2_CONTROL__CIA402_SYSTEM_HPP_
#define CANOPEN_ROS2_CONTROL__CIA402_SYSTEM_HPP_

#include <atomic>

#include "canopen_402_driver/cia402_driver.hpp"
#include "canopen_ros2_control/canopen_system.hpp"

constexpr double kResponseOk = 1.0;
constexpr double kResponseFail = 0.0;

namespace canopen_ros2_control
{

struct MotorTriggerCommand
{
  double ons_cmd{std::numeric_limits<double>::quiet_NaN()};
  double resp{std::numeric_limits<double>::quiet_NaN()};

  bool is_commanded()
  {
    bool tmp = !std::isnan(ons_cmd);
    ons_cmd = std::numeric_limits<double>::quiet_NaN();
    return tmp;
  }

  void set_response(bool response) { resp = response ? kResponseOk : kResponseFail; }
};

struct MotorTarget : public MotorTriggerCommand
{
  double position_value;
  double velocity_value;
  double torque_value;
};

struct MotorNodeData
{
  // feedback
  double actual_position;
  double actual_speed;

  // basic control
  MotorTriggerCommand init;
  MotorTriggerCommand halt;
  MotorTriggerCommand recover;

  // mode control
  MotorTriggerCommand position_mode;
  MotorTriggerCommand velocity_mode;
  MotorTriggerCommand cyclic_velocity_mode;
  MotorTriggerCommand cyclic_position_mode;
  MotorTriggerCommand torque_mode;
  MotorTriggerCommand interpolated_position_mode;

  // setpoint
  MotorTarget target;
};

using namespace ros2_canopen;
class Cia402System : public CanopenSystem
{
public:
  CANOPEN_ROS2_CONTROL__VISIBILITY_PUBLIC
  Cia402System();
  CANOPEN_ROS2_CONTROL__VISIBILITY_PUBLIC
  ~Cia402System() noexcept override;
  CANOPEN_ROS2_CONTROL__VISIBILITY_PUBLIC
  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info);

  CANOPEN_ROS2_CONTROL__VISIBILITY_PUBLIC
  hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state);

  CANOPEN_ROS2_CONTROL__VISIBILITY_PUBLIC
  hardware_interface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State & previous_state);

  CANOPEN_ROS2_CONTROL__VISIBILITY_PUBLIC
  hardware_interface::CallbackReturn on_shutdown(const rclcpp_lifecycle::State & previous_state);

  CANOPEN_ROS2_CONTROL__VISIBILITY_PUBLIC
  hardware_interface::CallbackReturn on_error(
    const rclcpp_lifecycle::State & previous_state) override;

  CANOPEN_ROS2_CONTROL__VISIBILITY_PUBLIC
  std::vector<hardware_interface::StateInterface> export_state_interfaces();

  CANOPEN_ROS2_CONTROL__VISIBILITY_PUBLIC
  std::vector<hardware_interface::CommandInterface> export_command_interfaces();

  CANOPEN_ROS2_CONTROL__VISIBILITY_PUBLIC
  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state);

  CANOPEN_ROS2_CONTROL__VISIBILITY_PUBLIC
  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state);

  CANOPEN_ROS2_CONTROL__VISIBILITY_PUBLIC
  hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period);

  CANOPEN_ROS2_CONTROL__VISIBILITY_PUBLIC
  hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period);

protected:
  // can stuff
  std::map<uint, MotorNodeData> motor_data_;

  bool isConfiguredMotorMode(uint8_t node_id, uint16_t operation_mode) const;

  virtual hardware_interface::CallbackReturn configureCommunication();

  virtual hardware_interface::CallbackReturn activateConfiguredMotors();

  virtual hardware_interface::CallbackReturn deactivateConfiguredMotors();

  virtual hardware_interface::return_type readConfiguredMotors();

  bool isMotorSessionActive() const
  {
    return motor_session_active_.load(std::memory_order_acquire);
  }

private:
  struct MotorConfiguration
  {
    std::string joint_name;
    uint8_t node_id;
    uint16_t operation_mode;
  };

  bool configureMotorTopology(const hardware_interface::HardwareInfo & info);

  bool isConfiguredMotorNode(uint8_t node_id) const;

  void switchModes(uint id, const std::shared_ptr<ros2_canopen::Cia402Driver> & driver);

  void handleInit(uint id, const std::shared_ptr<ros2_canopen::Cia402Driver> & driver);

  void handleRecover(uint id, const std::shared_ptr<ros2_canopen::Cia402Driver> & driver);

  void handleHalt(uint id, const std::shared_ptr<ros2_canopen::Cia402Driver> & driver);

  void initDeviceContainer();

  std::vector<MotorConfiguration> motor_configurations_;
  bool configured_mode_locks_valid_{false};
  std::atomic<bool> mode_drift_latched_{false};
  std::atomic<bool> motor_session_active_{false};
};

}  // namespace canopen_ros2_control

#endif  // CANOPEN_ROS2_CONTROL__CIA402_SYSTEM_HPP_
