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

#include "canopen_ros2_control/cia402_system.hpp"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>
#include <hardware_interface/types/hardware_interface_type_values.hpp>

namespace
{
auto const kLogger = rclcpp::get_logger("Cia402System");
}

namespace canopen_ros2_control
{

Cia402System::Cia402System() : CanopenSystem() {}

hardware_interface::CallbackReturn Cia402System::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (CanopenSystem::on_init(info) != CallbackReturn::SUCCESS)
  {
    return CallbackReturn::ERROR;
  }

  return CallbackReturn::SUCCESS;
}

void Cia402System::initDeviceContainer()
{
  std::string tmp_master_bin = (info_.hardware_parameters["master_bin"] == "\"\"")
                                 ? ""
                                 : info_.hardware_parameters["master_bin"];

  device_container_->init(
    info_.hardware_parameters["can_interface_name"], info_.hardware_parameters["master_config"],
    info_.hardware_parameters["bus_config"], tmp_master_bin);
  auto drivers = device_container_->get_registered_drivers();
  RCLCPP_INFO(kLogger, "Number of registered drivers: '%lu'", device_container_->count_drivers());
  for (auto it = drivers.begin(); it != drivers.end(); it++)
  {
    auto driver = std::static_pointer_cast<ros2_canopen::Cia402Driver>(it->second);

    auto nmt_state_cb = [this](canopen::NmtState nmt_state, uint8_t id)
    { canopen_data_[id].nmt_state.set_state(nmt_state); };
    // register callback
    driver->register_nmt_state_cb(nmt_state_cb);

    auto rpdo_cb = [this](ros2_canopen::COData data, uint8_t id)
    { canopen_data_[id].rpdo_data.set_data(data); };
    // register callback
    driver->register_rpdo_cb(rpdo_cb);

    auto emcy_cb = [this](ros2_canopen::COEmcy emcy, uint8_t id)
    {
      if (emcy.eec == 0U || (id != 2U && id != 3U))
      {
        return;
      }
      RCLCPP_ERROR(
        kLogger, "Track node %u reported EMCY 0x%04X; requesting all-node NMT Stop", id,
        emcy.eec);
      if (!device_container_->request_nmt_stop_all_nodes())
      {
        RCLCPP_ERROR(kLogger, "All-node NMT Stop request failed after track EMCY");
      }
    };
    driver->register_emcy_cb(emcy_cb);

    RCLCPP_INFO(
      kLogger, "\nRegistered driver:\n    name: '%s'\n    node_id: '0x%X'",
      it->second->get_node_base_interface()->get_name(), it->first);
  }

  RCLCPP_INFO(device_container_->get_logger(), "Initialisation successful.");
}

hardware_interface::CallbackReturn Cia402System::on_configure(
  const rclcpp_lifecycle::State & previous_state)
{
  executor_ = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
  device_container_ = std::make_shared<ros2_canopen::DeviceContainer>(executor_);
  executor_->add_node(device_container_);

  // threads
  spin_thread_ = std::make_unique<std::thread>(&Cia402System::spin, this);
  init_thread_ = std::make_unique<std::thread>(&Cia402System::initDeviceContainer, this);

  // actually wait for init phase to end
  if (init_thread_->joinable())
  {
    init_thread_->join();

    // TODO(livanov93): see how to handle configure once LifecycleCia402Driver is introduced
    /*
    auto drivers = device_container_->get_registered_drivers();
    for (auto it = drivers.begin(); it != drivers.end(); it++) {
        auto d = std::static_pointer_cast<ros2_canopen::LifecycleCia402Driver>(it->second);
        d->configure();
    }
    */
  }
  else
  {
    RCLCPP_ERROR(kLogger, "Could not join init thread!");
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> Cia402System::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  // underlying base class export first
  state_interfaces = CanopenSystem::export_state_interfaces();

  for (uint i = 0; i < info_.joints.size(); i++)
  {
    if (info_.joints[i].parameters.find("node_id") == info_.joints[i].parameters.end())
    {
      // skip adding motor canopen interfaces
      continue;
    }
    const uint8_t node_id = static_cast<uint8_t>(std::stoi(info_.joints[i].parameters["node_id"]));

    // actual position
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION,
      &motor_data_[node_id].actual_position));
    // actual speed
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY,
      &motor_data_[node_id].actual_speed));
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> Cia402System::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  // underlying base class export first
  command_interfaces = CanopenSystem::export_command_interfaces();

  for (uint i = 0; i < info_.joints.size(); i++)
  {
    if (info_.joints[i].parameters.find("node_id") == info_.joints[i].parameters.end())
    {
      // skip adding canopen interfaces
      continue;
    }

    const uint8_t node_id = static_cast<uint8_t>(std::stoi(info_.joints[i].parameters["node_id"]));

    // target
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION,
      &motor_data_[node_id].target.position_value));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY,
      &motor_data_[node_id].target.velocity_value));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_EFFORT,
      &motor_data_[node_id].target.torque_value));
    // init
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, "init_cmd", &motor_data_[node_id].init.ons_cmd));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, "init_fbk", &motor_data_[node_id].init.resp));

    // halt
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, "halt_cmd", &motor_data_[node_id].halt.ons_cmd));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, "halt_fbk", &motor_data_[node_id].halt.resp));

    // recover
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, "recover_cmd", &motor_data_[node_id].recover.ons_cmd));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, "recover_fbk", &motor_data_[node_id].recover.resp));

    // set position mode
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, "position_mode_cmd", &motor_data_[node_id].position_mode.ons_cmd));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, "position_mode_fbk", &motor_data_[node_id].position_mode.resp));

    // set velocity mode
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, "velocity_mode_cmd", &motor_data_[node_id].velocity_mode.ons_cmd));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, "velocity_mode_fbk", &motor_data_[node_id].velocity_mode.resp));

    // set cyclic velocity mode
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, "cyclic_velocity_mode_cmd",
      &motor_data_[node_id].cyclic_velocity_mode.ons_cmd));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, "cyclic_velocity_mode_fbk",
      &motor_data_[node_id].cyclic_velocity_mode.resp));
    // set cyclic position mode
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, "cyclic_position_mode_cmd",
      &motor_data_[node_id].cyclic_position_mode.ons_cmd));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, "cyclic_position_mode_fbk",
      &motor_data_[node_id].cyclic_position_mode.resp));
    // set interpolated position mode
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, "interpolated_position_mode_cmd",
      &motor_data_[node_id].interpolated_position_mode.ons_cmd));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, "interpolated_position_mode_fbk",
      &motor_data_[node_id].interpolated_position_mode.resp));
  }

  return command_interfaces;
}

hardware_interface::CallbackReturn Cia402System::on_activate(
  const rclcpp_lifecycle::State & previous_state)
{
  if (CanopenSystem::on_activate(previous_state) != CallbackReturn::SUCCESS)
  {
    return CallbackReturn::ERROR;
  }

  struct MotorConfiguration
  {
    uint8_t node_id;
    uint16_t operation_mode;
  };

  std::vector<MotorConfiguration> configurations;
  for (const auto & joint : info_.joints)
  {
    const auto node_parameter = joint.parameters.find("node_id");
    if (node_parameter == joint.parameters.end())
    {
      continue;
    }
    const auto mode_parameter = joint.parameters.find("operation_mode");
    if (mode_parameter == joint.parameters.end())
    {
      RCLCPP_ERROR(kLogger, "Joint %s has no operation_mode parameter", joint.name.c_str());
      return CallbackReturn::ERROR;
    }
    try
    {
      const auto parsed_node_id = std::stoul(node_parameter->second);
      const auto parsed_mode = std::stoul(mode_parameter->second);
      if (
        parsed_node_id > std::numeric_limits<uint8_t>::max() ||
        parsed_mode > std::numeric_limits<uint16_t>::max())
      {
        throw std::out_of_range("node_id or operation_mode is outside its storage range");
      }
      const auto node_id = static_cast<uint8_t>(parsed_node_id);
      const auto mode = static_cast<uint16_t>(parsed_mode);
      if (
        (node_id != 2U && node_id != 3U) || mode != MotorBase::Profiled_Velocity)
      {
        RCLCPP_ERROR(
          kLogger, "Unsupported CANopen activation mapping for joint %s: node=%u mode=%u",
          joint.name.c_str(), node_id, mode);
        return CallbackReturn::ERROR;
      }
      configurations.push_back({node_id, mode});
    }
    catch (const std::exception & exception)
    {
      RCLCPP_ERROR(
        kLogger, "Invalid CANopen parameters for joint %s: %s", joint.name.c_str(),
        exception.what());
      return CallbackReturn::ERROR;
    }
  }
  std::sort(
    configurations.begin(), configurations.end(),
    [](const MotorConfiguration & lhs, const MotorConfiguration & rhs)
    { return lhs.node_id < rhs.node_id; });

  if (
    configurations.size() != 2U || configurations[0].node_id != 2U ||
    configurations[1].node_id != 3U)
  {
    RCLCPP_ERROR(kLogger, "Expected exactly CANopen track motor nodes 2 and 3");
    return CallbackReturn::ERROR;
  }

  auto drivers = device_container_->get_registered_drivers();
  std::vector<uint8_t> completed_nodes;
  std::string primary_failure;
  uint8_t primary_node = 0U;
  for (const auto & configuration : configurations)
  {
    const auto driver_iterator = drivers.find(configuration.node_id);
    if (driver_iterator == drivers.end())
    {
      primary_node = configuration.node_id;
      primary_failure = "driver_lookup";
      break;
    }
    auto driver = std::static_pointer_cast<ros2_canopen::Cia402Driver>(driver_iterator->second);
    ros2_canopen::COData mode_command = {
      0x6060U, 0x00U, static_cast<uint32_t>(configuration.operation_mode)};
    if (!driver->tpdo_transmit(mode_command))
    {
      primary_node = configuration.node_id;
      primary_failure = "prime_operation_mode";
      break;
    }
    RCLCPP_INFO(
      kLogger, "Primed node %u RPDO mode cache with fixed mode %u", configuration.node_id,
      configuration.operation_mode);
    if (!driver->init_motor())
    {
      primary_node = configuration.node_id;
      primary_failure = "init_motor";
      break;
    }
    if (
      driver->get_mode() != configuration.operation_mode &&
      !driver->set_operation_mode(configuration.operation_mode))
    {
      primary_node = configuration.node_id;
      primary_failure = "set_operation_mode";
      break;
    }

    motor_data_[configuration.node_id].target.velocity_value = 0.0;
    const bool target_seeded = driver->set_target(0.0);
    if (!target_seeded)
    {
      primary_node = configuration.node_id;
      primary_failure = "seed_safe_target";
      break;
    }
    completed_nodes.push_back(configuration.node_id);
  }

  if (primary_failure.empty())
  {
    return CallbackReturn::SUCCESS;
  }

  std::string rollback_failure;
  uint8_t rollback_node = 0U;
  for (const auto & configuration : configurations)
  {
    const auto driver_iterator = drivers.find(configuration.node_id);
    if (driver_iterator == drivers.end())
    {
      continue;
    }
    auto driver = std::static_pointer_cast<ros2_canopen::Cia402Driver>(driver_iterator->second);
    bool target_valid = driver->get_mode() == configuration.operation_mode;
    bool target_result = true;
    if (target_valid)
    {
      motor_data_[configuration.node_id].target.velocity_value = 0.0;
      target_result = driver->set_target(0.0);
    }
    if (target_valid && !target_result && rollback_failure.empty())
    {
      rollback_node = configuration.node_id;
      rollback_failure = "seed_safe_target";
    }
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(20));

  std::vector<uint8_t> shutdown_order(completed_nodes.rbegin(), completed_nodes.rend());
  for (const auto & configuration : configurations)
  {
    if (
      std::find(completed_nodes.begin(), completed_nodes.end(), configuration.node_id) ==
      completed_nodes.end())
    {
      shutdown_order.push_back(configuration.node_id);
    }
  }
  for (const auto node_id : shutdown_order)
  {
    const auto driver_iterator = drivers.find(node_id);
    if (driver_iterator == drivers.end())
    {
      if (rollback_failure.empty())
      {
        rollback_node = node_id;
        rollback_failure = "driver_lookup";
      }
      continue;
    }
    auto driver = std::static_pointer_cast<ros2_canopen::Cia402Driver>(driver_iterator->second);
    if (!driver->shutdown_motor() && rollback_failure.empty())
    {
      rollback_node = node_id;
      rollback_failure = "shutdown_motor";
    }
  }
  if (!rollback_failure.empty())
  {
    if (!device_container_->request_nmt_stop_all_nodes())
    {
      RCLCPP_ERROR(kLogger, "Activation rollback all-node NMT Stop request failed");
    }
    RCLCPP_ERROR(
      kLogger, "CANopen activation failed at node %u/%s; first rollback failure node %u/%s",
      primary_node, primary_failure.c_str(), rollback_node, rollback_failure.c_str());
  }
  else
  {
    RCLCPP_ERROR(
      kLogger, "CANopen activation failed at node %u/%s; rollback completed", primary_node,
      primary_failure.c_str());
  }
  return CallbackReturn::ERROR;
}

hardware_interface::CallbackReturn Cia402System::on_deactivate(
  const rclcpp_lifecycle::State & previous_state)
{
  auto drivers = device_container_->get_registered_drivers();
  bool failed = false;
  uint8_t first_failure_node = 0U;
  std::string first_failure_stage;

  for (const uint8_t node_id : {2U, 3U})
  {
    const auto driver_iterator = drivers.find(node_id);
    if (driver_iterator == drivers.end())
    {
      if (!failed)
      {
        failed = true;
        first_failure_node = node_id;
        first_failure_stage = "driver_lookup";
      }
      continue;
    }
    auto driver = std::static_pointer_cast<ros2_canopen::Cia402Driver>(driver_iterator->second);
    if (driver->get_state() == State402::Switch_On_Disabled)
    {
      continue;
    }
    motor_data_[node_id].target.velocity_value = 0.0;
    const bool target_result = driver->set_target(0.0);
    if (!target_result && !failed)
    {
      failed = true;
      first_failure_node = node_id;
      first_failure_stage = "seed_safe_target";
    }
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(20));

  for (const uint8_t node_id : {2U, 3U})
  {
    const auto driver_iterator = drivers.find(node_id);
    if (driver_iterator == drivers.end())
    {
      continue;
    }
    auto driver = std::static_pointer_cast<ros2_canopen::Cia402Driver>(driver_iterator->second);
    if (!driver->shutdown_motor() && !failed)
    {
      failed = true;
      first_failure_node = node_id;
      first_failure_stage = "shutdown_motor";
    }
  }

  if (!device_container_->request_nmt_stop_all_nodes() && !failed)
  {
    failed = true;
    first_failure_stage = "nmt_stop";
  }

  stop_callback_executor();

  if (!device_container_->shutdown_drivers() && !failed)
  {
    failed = true;
    first_failure_stage = "driver_shutdown";
  }

  const auto base_result = CanopenSystem::on_deactivate(previous_state);
  if (failed || base_result != CallbackReturn::SUCCESS)
  {
    RCLCPP_ERROR(
      kLogger, "CANopen deactivation failed at node %u/%s", first_failure_node,
      first_failure_stage.c_str());
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

hardware_interface::return_type Cia402System::read(
  const rclcpp::Time & time, const rclcpp::Duration & period)
{
  // TODO(anyone): read robot states

  auto ret_val = CanopenSystem::read(time, period);

  auto drivers = device_container_->get_registered_drivers();

  for (auto it = canopen_data_.begin(); it != canopen_data_.end(); ++it)
  {
    auto motion_controller_driver =
      std::static_pointer_cast<ros2_canopen::Cia402Driver>(drivers[it->first]);
    // get position
    motor_data_[it->first].actual_position = motion_controller_driver->get_position();
    // get speed
    motor_data_[it->first].actual_speed = motion_controller_driver->get_speed();
  }

  return ret_val;
}

hardware_interface::return_type Cia402System::write(
  const rclcpp::Time & time, const rclcpp::Duration & period)
{
  auto drivers = device_container_->get_registered_drivers();

  for (auto it = canopen_data_.begin(); it != canopen_data_.end(); ++it)
  {
    // TODO(livanov93): check casting
    auto motion_controller_driver =
      std::static_pointer_cast<ros2_canopen::Cia402Driver>(drivers[it->first]);
    // do same as in proxy system first - handle nmt, tpdo, rpdo
    // reset node nmt
    if (it->second.nmt_state.reset_command())
    {
      motion_controller_driver->reset_node_nmt_command();
    }

    // start nmt
    if (it->second.nmt_state.start_command())
    {
      motion_controller_driver->start_node_nmt_command();
    }

    // tpdo data one shot mechanism
    if (it->second.tpdo_data.write_command())
    {
      it->second.tpdo_data.prepare_data();
      motion_controller_driver->tpdo_transmit(it->second.tpdo_data.original_data);
    }

    // initialisation
    handleInit(it->first, motion_controller_driver);

    // halt
    handleHalt(it->first, motion_controller_driver);

    // recover
    handleRecover(it->first, motion_controller_driver);

    // mode switching
    switchModes(it->first, motion_controller_driver);

    const uint16_t & mode = motion_controller_driver->get_mode();

    switch (mode)
    {
      case MotorBase::No_Mode:
        break;
      case MotorBase::Profiled_Position:
      case MotorBase::Cyclic_Synchronous_Position:
      case MotorBase::Interpolated_Position:
        motion_controller_driver->set_target(motor_data_[it->first].target.position_value);
        break;
      case MotorBase::Profiled_Velocity:
      case MotorBase::Cyclic_Synchronous_Velocity:
        motion_controller_driver->set_target(motor_data_[it->first].target.velocity_value);
        break;
      case MotorBase::Profiled_Torque:
        motion_controller_driver->set_target(motor_data_[it->first].target.torque_value);
        break;
      default:
        RCLCPP_INFO(kLogger, "Mode %u not supported", mode);
    }
  }

  return hardware_interface::return_type::OK;
}

void Cia402System::switchModes(uint id, const std::shared_ptr<ros2_canopen::Cia402Driver> & driver)
{
  if (motor_data_[id].position_mode.is_commanded())
  {
    motor_data_[id].position_mode.set_response(
      driver->set_operation_mode(MotorBase::Profiled_Position));
  }

  if (motor_data_[id].cyclic_position_mode.is_commanded())
  {
    motor_data_[id].cyclic_position_mode.set_response(
      driver->set_operation_mode(MotorBase::Cyclic_Synchronous_Position));
  }

  if (motor_data_[id].velocity_mode.is_commanded())
  {
    motor_data_[id].velocity_mode.set_response(
      driver->set_operation_mode(MotorBase::Profiled_Velocity));
  }

  if (motor_data_[id].cyclic_velocity_mode.is_commanded())
  {
    motor_data_[id].cyclic_velocity_mode.set_response(
      driver->set_operation_mode(MotorBase::Cyclic_Synchronous_Velocity));
  }

  if (motor_data_[id].torque_mode.is_commanded())
  {
    motor_data_[id].torque_mode.set_response(
      driver->set_operation_mode(MotorBase::Profiled_Torque));
  }

  if (motor_data_[id].interpolated_position_mode.is_commanded())
  {
    motor_data_[id].interpolated_position_mode.set_response(
      driver->set_operation_mode(MotorBase::Interpolated_Position));
  }
}

void Cia402System::handleInit(uint id, const std::shared_ptr<ros2_canopen::Cia402Driver> & driver)
{
  if (motor_data_[id].init.is_commanded())
  {
    motor_data_[id].init.set_response(driver->init_motor());
  }
}

void Cia402System::handleRecover(
  uint id, const std::shared_ptr<ros2_canopen::Cia402Driver> & driver)
{
  if (motor_data_[id].recover.is_commanded())
  {
    motor_data_[id].recover.set_response(driver->recover_motor());
  }
}

void Cia402System::handleHalt(uint id, const std::shared_ptr<ros2_canopen::Cia402Driver> & driver)
{
  if (motor_data_[id].halt.is_commanded())
  {
    motor_data_[id].halt.set_response(driver->halt_motor());
  }
}

}  // namespace canopen_ros2_control

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(canopen_ros2_control::Cia402System, hardware_interface::SystemInterface)
