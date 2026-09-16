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
#include <exception>
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
constexpr uint16_t kTargetVelocityIndex = 0x60FFU;

template <typename Unsigned>
Unsigned parseUnsignedParameter(const std::string & value, const char * parameter_name)
{
  if (value.empty() || value.find_first_not_of("0123456789") != std::string::npos)
  {
    throw std::invalid_argument(std::string(parameter_name) + " must be an unsigned integer");
  }
  std::size_t parsed_characters = 0U;
  const auto parsed = std::stoul(value, &parsed_characters, 10);
  if (parsed_characters != value.size() || parsed > std::numeric_limits<Unsigned>::max())
  {
    throw std::out_of_range(std::string(parameter_name) + " is outside its valid range");
  }
  return static_cast<Unsigned>(parsed);
}
}

namespace canopen_ros2_control
{

Cia402System::Cia402System() : CanopenSystem() {}

Cia402System::~Cia402System() noexcept
{
  if (!clean())
  {
    RCLCPP_FATAL(
      kLogger,
      "Cia402System destruction could not quiesce CANopen callbacks; terminating before motor "
      "state is destroyed");
    std::terminate();
  }
}

hardware_interface::CallbackReturn Cia402System::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (CanopenSystem::on_init(info) != CallbackReturn::SUCCESS)
  {
    return CallbackReturn::ERROR;
  }

  return configureMotorTopology(info) ? CallbackReturn::SUCCESS : CallbackReturn::ERROR;
}

bool Cia402System::configureMotorTopology(const hardware_interface::HardwareInfo & info)
{
  std::vector<MotorConfiguration> configurations;
  configurations.reserve(info.joints.size());
  for (const auto & joint : info.joints)
  {
    const auto node_parameter = joint.parameters.find("node_id");
    const auto mode_parameter = joint.parameters.find("operation_mode");
    if (
      node_parameter == joint.parameters.end() ||
      mode_parameter == joint.parameters.end())
    {
      RCLCPP_ERROR(
        kLogger, "Joint %s must define node_id and operation_mode together", joint.name.c_str());
      return false;
    }
    try
    {
      const auto node_id = parseUnsignedParameter<uint8_t>(node_parameter->second, "node_id");
      const auto mode =
        parseUnsignedParameter<uint16_t>(mode_parameter->second, "operation_mode");
      if (node_id == 0U || node_id > 127U)
      {
        RCLCPP_ERROR(
          kLogger, "Invalid CANopen node for joint %s: node=%u", joint.name.c_str(), node_id);
        return false;
      }
      if (mode != MotorBase::Profiled_Velocity)
      {
        RCLCPP_ERROR(
          kLogger,
          "Joint %s uses mode %u, but this system can only seed a safe Profiled Velocity target",
          joint.name.c_str(), mode);
        return false;
      }
      configurations.push_back({joint.name, node_id, mode});
    }
    catch (const std::exception & exception)
    {
      RCLCPP_ERROR(
        kLogger, "Invalid CANopen parameters for joint %s: %s", joint.name.c_str(),
        exception.what());
      return false;
    }
  }
  std::sort(
    configurations.begin(), configurations.end(),
    [](const MotorConfiguration & lhs, const MotorConfiguration & rhs)
    { return lhs.node_id < rhs.node_id; });
  if (configurations.empty())
  {
    RCLCPP_ERROR(kLogger, "No configured CiA402 motor joints");
    return false;
  }
  const auto duplicate = std::adjacent_find(
    configurations.begin(), configurations.end(),
    [](const MotorConfiguration & lhs, const MotorConfiguration & rhs)
    { return lhs.node_id == rhs.node_id; });
  if (duplicate != configurations.end())
  {
    RCLCPP_ERROR(kLogger, "CANopen node %u is assigned to more than one joint", duplicate->node_id);
    return false;
  }

  motor_configurations_ = std::move(configurations);
  return true;
}

bool Cia402System::isConfiguredMotorNode(uint8_t node_id) const
{
  return std::any_of(
    motor_configurations_.begin(), motor_configurations_.end(),
    [node_id](const MotorConfiguration & configuration)
    { return configuration.node_id == node_id; });
}

bool Cia402System::isConfiguredMotorMode(uint8_t node_id, uint16_t operation_mode) const
{
  const auto configuration = std::find_if(
    motor_configurations_.begin(), motor_configurations_.end(),
    [node_id](const MotorConfiguration & candidate) { return candidate.node_id == node_id; });
  return configuration != motor_configurations_.end() &&
         configuration->operation_mode == operation_mode;
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

  bool mode_locks_valid = true;
  for (const auto & configuration : motor_configurations_)
  {
    const auto driver_iterator = drivers.find(configuration.node_id);
    if (driver_iterator == drivers.end())
    {
      RCLCPP_ERROR(
        kLogger, "Cannot lock configured mode for missing CANopen node %u",
        configuration.node_id);
      mode_locks_valid = false;
      continue;
    }
    auto driver = std::static_pointer_cast<ros2_canopen::Cia402Driver>(driver_iterator->second);
    if (!driver->lock_operation_mode(configuration.operation_mode))
    {
      RCLCPP_ERROR(
        kLogger, "Cannot lock CANopen node %u to configured mode %u", configuration.node_id,
        configuration.operation_mode);
      mode_locks_valid = false;
    }
  }
  configured_mode_locks_valid_ = mode_locks_valid;
  if (!configured_mode_locks_valid_)
  {
    RCLCPP_ERROR(kLogger, "CANopen configured operation-mode locks are incomplete");
    return;
  }

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
      if (emcy.eec == 0U || !isConfiguredMotorNode(id))
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
  const rclcpp_lifecycle::State &)
{
  motor_session_active_.store(false, std::memory_order_release);
  return configureCommunication();
}

hardware_interface::CallbackReturn Cia402System::configureCommunication()
{
  configured_mode_locks_valid_ = false;
  executor_ = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
  auto container_options = rclcpp::NodeOptions().use_global_arguments(false);
  container_options.parameter_overrides(
    {rclcpp::Parameter("expose_mutating_ros_api", false)});
  device_container_ = std::make_shared<ros2_canopen::DeviceContainer>(
    executor_, "device_container", container_options);
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
  if (!configured_mode_locks_valid_)
  {
    RCLCPP_ERROR(kLogger, "CANopen configuration rejected: operation-mode lock failed");
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn Cia402System::on_cleanup(
  const rclcpp_lifecycle::State & previous_state)
{
  motor_session_active_.store(false, std::memory_order_release);
  return CanopenSystem::on_cleanup(previous_state);
}

hardware_interface::CallbackReturn Cia402System::on_shutdown(
  const rclcpp_lifecycle::State & previous_state)
{
  motor_session_active_.store(false, std::memory_order_release);
  return CanopenSystem::on_shutdown(previous_state);
}

hardware_interface::CallbackReturn Cia402System::on_error(
  const rclcpp_lifecycle::State & previous_state)
{
  motor_session_active_.store(false, std::memory_order_release);
  return CanopenSystem::on_error(previous_state);
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

  command_interfaces.reserve(motor_configurations_.size());
  for (const auto & configuration : motor_configurations_)
  {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      configuration.joint_name, hardware_interface::HW_IF_VELOCITY,
      &motor_data_[configuration.node_id].target.velocity_value));
  }

  return command_interfaces;
}

hardware_interface::CallbackReturn Cia402System::on_activate(
  const rclcpp_lifecycle::State & previous_state)
{
  motor_session_active_.store(false, std::memory_order_release);
  if (mode_drift_latched_.load(std::memory_order_acquire))
  {
    RCLCPP_ERROR(kLogger, "CANopen activation rejected after a latched operation-mode drift");
    return CallbackReturn::ERROR;
  }
  if (CanopenSystem::on_activate(previous_state) != CallbackReturn::SUCCESS)
  {
    return CallbackReturn::ERROR;
  }

  const auto activation_result = activateConfiguredMotors();
  if (activation_result == CallbackReturn::SUCCESS)
  {
    motor_session_active_.store(true, std::memory_order_release);
  }
  return activation_result;
}

hardware_interface::CallbackReturn Cia402System::activateConfiguredMotors()
{
  auto drivers = device_container_->get_registered_drivers();
  std::vector<uint8_t> completed_nodes;
  std::string primary_failure;
  uint8_t primary_node = 0U;
  for (const auto & configuration : motor_configurations_)
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
    if (driver->get_actual_mode() != configuration.operation_mode)
    {
      primary_node = configuration.node_id;
      primary_failure = "verify_operation_mode";
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
  for (const auto & configuration : motor_configurations_)
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
  for (const auto & configuration : motor_configurations_)
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
  const bool was_active = motor_session_active_.exchange(false, std::memory_order_acq_rel);
  const auto motor_result =
    was_active ? deactivateConfiguredMotors() : CallbackReturn::SUCCESS;
  const auto base_result = CanopenSystem::on_deactivate(previous_state);
  if (motor_result != CallbackReturn::SUCCESS || base_result != CallbackReturn::SUCCESS)
  {
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn Cia402System::deactivateConfiguredMotors()
{
  auto drivers = device_container_->get_registered_drivers();
  bool failed = false;
  uint8_t first_failure_node = 0U;
  std::string first_failure_stage;

  for (const auto & configuration : motor_configurations_)
  {
    const uint8_t node_id = configuration.node_id;
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
    bool target_result = false;
    if (isConfiguredMotorMode(node_id, driver->get_actual_mode()))
    {
      target_result = driver->set_target(0.0);
    }
    else
    {
      ros2_canopen::COData safe_velocity_command = {kTargetVelocityIndex, 0x00U, 0U};
      target_result = driver->tpdo_transmit(safe_velocity_command);
    }
    if (!target_result && !failed)
    {
      failed = true;
      first_failure_node = node_id;
      first_failure_stage = "seed_safe_target";
    }
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(20));

  for (const auto & configuration : motor_configurations_)
  {
    const uint8_t node_id = configuration.node_id;
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

  if (failed && !device_container_->request_nmt_stop_all_nodes())
  {
    RCLCPP_ERROR(kLogger, "CANopen deactivation fallback all-node NMT Stop request failed");
  }

  if (failed)
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
  if (ret_val != hardware_interface::return_type::OK)
  {
    return ret_val;
  }
  return readConfiguredMotors();
}

hardware_interface::return_type Cia402System::readConfiguredMotors()
{
  if (!device_container_)
  {
    return hardware_interface::return_type::ERROR;
  }
  auto drivers = device_container_->get_registered_drivers();

  for (const auto & configuration : motor_configurations_)
  {
    const auto driver_iterator = drivers.find(configuration.node_id);
    if (driver_iterator == drivers.end())
    {
      return hardware_interface::return_type::ERROR;
    }
    auto motion_controller_driver =
      std::static_pointer_cast<ros2_canopen::Cia402Driver>(driver_iterator->second);
    // get position
    motor_data_[configuration.node_id].actual_position = motion_controller_driver->get_position();
    // get speed
    motor_data_[configuration.node_id].actual_speed = motion_controller_driver->get_speed();
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type Cia402System::write(
  const rclcpp::Time &, const rclcpp::Duration &)
{
  if (mode_drift_latched_.load(std::memory_order_acquire))
  {
    for (const auto & configuration : motor_configurations_)
    {
      motor_data_[configuration.node_id].target.velocity_value = 0.0;
    }
    return hardware_interface::return_type::ERROR;
  }

  if (!motor_session_active_.load(std::memory_order_acquire))
  {
    for (const auto & configuration : motor_configurations_)
    {
      motor_data_[configuration.node_id].target.velocity_value = 0.0;
    }
    return hardware_interface::return_type::OK;
  }

  auto drivers = device_container_->get_registered_drivers();

  uint8_t drift_node = 0U;
  uint16_t expected_mode = MotorBase::No_Mode;
  uint16_t observed_mode = MotorBase::No_Mode;
  bool driver_missing = false;
  for (const auto & configuration : motor_configurations_)
  {
    const auto driver_iterator = drivers.find(configuration.node_id);
    if (driver_iterator == drivers.end())
    {
      drift_node = configuration.node_id;
      expected_mode = configuration.operation_mode;
      driver_missing = true;
      break;
    }
    auto driver = std::static_pointer_cast<ros2_canopen::Cia402Driver>(driver_iterator->second);
    observed_mode = driver->get_actual_mode();
    if (!isConfiguredMotorMode(configuration.node_id, observed_mode))
    {
      drift_node = configuration.node_id;
      expected_mode = configuration.operation_mode;
      break;
    }
  }

  if (drift_node != 0U)
  {
    bool safe_velocity_failed = false;
    for (const auto & configuration : motor_configurations_)
    {
      motor_data_[configuration.node_id].target.velocity_value = 0.0;
      const auto driver_iterator = drivers.find(configuration.node_id);
      if (driver_iterator == drivers.end())
      {
        safe_velocity_failed = true;
        continue;
      }
      auto driver = std::static_pointer_cast<ros2_canopen::Cia402Driver>(driver_iterator->second);
      ros2_canopen::COData safe_velocity_command = {kTargetVelocityIndex, 0x00U, 0U};
      if (!driver->tpdo_transmit(safe_velocity_command))
      {
        safe_velocity_failed = true;
      }
    }
    const bool nmt_stop_requested = device_container_->request_nmt_stop_all_nodes();
    mode_drift_latched_.store(true, std::memory_order_release);
    RCLCPP_ERROR(
      kLogger,
      "CANopen operation-mode contract failed at node %u: expected=%u observed=%u%s; "
      "safe_velocity=%s nmt_stop=%s; restart required",
      drift_node, expected_mode, observed_mode, driver_missing ? " driver_missing" : "",
      safe_velocity_failed ? "failed" : "requested", nmt_stop_requested ? "requested" : "failed");
    return hardware_interface::return_type::ERROR;
  }

  for (const auto & configuration : motor_configurations_)
  {
    const auto driver =
      std::static_pointer_cast<ros2_canopen::Cia402Driver>(drivers.at(configuration.node_id));
    driver->set_target(motor_data_[configuration.node_id].target.velocity_value);
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
