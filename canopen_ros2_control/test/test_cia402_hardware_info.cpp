#include <gtest/gtest.h>

#include <chrono>
#include <future>
#include <initializer_list>
#include <memory>
#include <set>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "canopen_402_driver/base.hpp"
#include "canopen_402_driver/cia402_driver.hpp"
#include "canopen_402_driver/motor.hpp"
#include "canopen_ros2_control/cia402_system.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "rclcpp/executors/multi_threaded_executor.hpp"
#include "rclcpp/rclcpp.hpp"

namespace canopen_ros2_control
{
namespace
{

using JointParameters = std::pair<std::string, std::string>;

class InspectableCia402System : public Cia402System
{
public:
  using Cia402System::isConfiguredMotorMode;
};

class LifecycleInspectableCia402System : public Cia402System
{
public:
  bool motorSessionActive() const { return isMotorSessionActive(); }

  unsigned int activationCount() const { return activation_count_; }

  unsigned int deactivationCount() const { return deactivation_count_; }

  unsigned int configurationCount() const { return configuration_count_; }

  unsigned int readCount() const { return read_count_; }

  unsigned int terminalBarrierCount() const { return terminal_barrier_count_; }

protected:
  hardware_interface::CallbackReturn configureCommunication() override
  {
    ++configuration_count_;
    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::CallbackReturn activateConfiguredMotors() override
  {
    ++activation_count_;
    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::CallbackReturn deactivateConfiguredMotors() override
  {
    ++deactivation_count_;
    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::return_type readConfiguredMotors() override
  {
    ++read_count_;
    return hardware_interface::return_type::OK;
  }

  bool stop_callback_executor() override
  {
    ++terminal_barrier_count_;
    return true;
  }

private:
  unsigned int configuration_count_{0U};
  unsigned int activation_count_{0U};
  unsigned int deactivation_count_{0U};
  unsigned int read_count_{0U};
  unsigned int terminal_barrier_count_{0U};
};

class InspectableDeviceContainer : public ros2_canopen::DeviceContainer
{
public:
  using ros2_canopen::DeviceContainer::DeviceContainer;

  void simulateLifecycleOperation() { lifecycle_operation_ = true; }

  bool hasLifecycleManager() const { return lifecycle_manager_ != nullptr; }

protected:
  void add_node_to_executor(
    rclcpp::node_interfaces::NodeBaseInterface::SharedPtr) override
  {
  }
};

struct TerminalCleanupResults
{
  bool nmt_stop{true};
  bool callback_barrier{true};
  bool drivers{true};
  bool master{true};
};

class TerminalDeviceContainer : public ros2_canopen::DeviceContainer
{
public:
  TerminalDeviceContainer(
    std::vector<std::string> & events, const TerminalCleanupResults & results,
    const std::string & node_name)
  : DeviceContainer(
      std::weak_ptr<rclcpp::Executor>(), node_name,
      rclcpp::NodeOptions().use_global_arguments(false)),
    events_(events),
    results_(results)
  {
  }

  bool request_nmt_stop_all_nodes() override
  {
    events_.push_back("nmt_stop");
    return results_.nmt_stop;
  }

  bool shutdown_drivers() override
  {
    events_.push_back("drivers");
    return results_.drivers;
  }

  bool shutdown_master() override
  {
    events_.push_back("master");
    return results_.master;
  }

private:
  std::vector<std::string> & events_;
  const TerminalCleanupResults & results_;
};

class TerminalInspectableCia402System : public Cia402System
{
public:
  TerminalInspectableCia402System(
    std::vector<std::string> & events, const TerminalCleanupResults & results)
  : events_(events), results_(results)
  {
  }

  void installContainer(const std::shared_ptr<ros2_canopen::DeviceContainer> & container)
  {
    device_container_ = container;
  }

  bool hasInstalledContainer() const { return device_container_ != nullptr; }

protected:
  bool stop_callback_executor() override
  {
    events_.push_back("callback_barrier");
    return results_.callback_barrier;
  }

private:
  std::vector<std::string> & events_;
  const TerminalCleanupResults & results_;
};

class TerminalInspectableCanopenSystem : public CanopenSystem
{
public:
  TerminalInspectableCanopenSystem(
    std::vector<std::string> & events, const TerminalCleanupResults & results)
  : events_(events), results_(results)
  {
  }

  void installContainer(const std::shared_ptr<ros2_canopen::DeviceContainer> & container)
  {
    device_container_ = container;
  }

  bool hasInstalledContainer() const { return device_container_ != nullptr; }

protected:
  bool stop_callback_executor() override
  {
    events_.push_back("callback_barrier");
    return results_.callback_barrier;
  }

private:
  std::vector<std::string> & events_;
  const TerminalCleanupResults & results_;
};

template <typename SystemT>
class SelfDeletingSystem : public SystemT
{
public:
  void deleteOnOwnedThread(
    std::shared_future<void> release, std::promise<void> & deletion_started)
  {
    this->spin_thread_ = std::make_unique<std::thread>(
      [this, release = std::move(release), &deletion_started]() {
        release.wait();
        deletion_started.set_value();
        delete this;
      });
  }
};

hardware_interface::HardwareInfo makeHardwareInfo(
  std::initializer_list<std::pair<std::string, JointParameters>> joints)
{
  hardware_interface::HardwareInfo info;
  info.name = "test_canopen_system";
  info.type = "system";
  info.hardware_class_type = "canopen_ros2_control/Cia402System";
  info.hardware_parameters = {
    {"bus_config", "/tmp/bus.yml"},
    {"master_config", "/tmp/master.dcf"},
    {"master_bin", "/tmp/master.bin"},
    {"can_interface_name", "can0"},
  };
  for (const auto & [joint_name, parameters] : joints)
  {
    hardware_interface::ComponentInfo joint;
    joint.name = joint_name;
    joint.type = "joint";
    if (!parameters.first.empty())
    {
      joint.parameters.emplace("node_id", parameters.first);
    }
    if (!parameters.second.empty())
    {
      joint.parameters.emplace("operation_mode", parameters.second);
    }
    info.joints.push_back(std::move(joint));
  }
  return info;
}

TEST(Cia402HardwareInfoTest, AcceptsArbitraryUniqueCanopenNodeIds)
{
  Cia402System system;
  const auto info = makeHardwareInfo({
    {"front_track", {"7", "3"}},
    {"middle_track", {"11", "3"}},
    {"rear_track", {"42", "3"}},
  });

  EXPECT_EQ(system.on_init(info), hardware_interface::CallbackReturn::SUCCESS);
}

TEST(Cia402HardwareInfoTest, ExposesOnlyConfiguredProfiledVelocityCommandSurface)
{
  Cia402System system;
  const auto info = makeHardwareInfo({
    {"left_track", {"2", "3"}},
    {"right_track", {"3", "3"}},
  });
  ASSERT_EQ(system.on_init(info), hardware_interface::CallbackReturn::SUCCESS);

  const auto interfaces = system.export_command_interfaces();
  std::set<std::string> interface_names;
  for (const auto & interface : interfaces)
  {
    interface_names.insert(interface.get_name());
  }

  EXPECT_EQ(
    interface_names,
    (std::set<std::string>{"left_track/velocity", "right_track/velocity"}));
}

TEST(Cia402HardwareInfoTest, DetectsConfiguredModeDriftAndUnknownNodes)
{
  InspectableCia402System system;
  const auto info = makeHardwareInfo({
    {"left_track", {"2", "3"}},
    {"right_track", {"3", "3"}},
  });
  ASSERT_EQ(system.on_init(info), hardware_interface::CallbackReturn::SUCCESS);

  EXPECT_TRUE(system.isConfiguredMotorMode(2U, MotorBase::Profiled_Velocity));
  EXPECT_FALSE(system.isConfiguredMotorMode(2U, MotorBase::Profiled_Position));
  EXPECT_FALSE(system.isConfiguredMotorMode(2U, MotorBase::Cyclic_Synchronous_Velocity));
  EXPECT_FALSE(system.isConfiguredMotorMode(2U, MotorBase::Profiled_Torque));
  EXPECT_FALSE(system.isConfiguredMotorMode(42U, MotorBase::Profiled_Velocity));
}

TEST(Cia402HardwareInfoTest, InactiveReadWriteIsSafeAndTheMotorSessionCanReactivate)
{
  LifecycleInspectableCia402System system;
  ASSERT_EQ(
    system.on_init(makeHardwareInfo({{"left_track", {"2", "3"}}})),
    hardware_interface::CallbackReturn::SUCCESS);
  auto command_interfaces = system.export_command_interfaces();
  ASSERT_EQ(command_interfaces.size(), 1U);

  const rclcpp_lifecycle::State state;
  const rclcpp::Time now(0, 0, RCL_ROS_TIME);
  const rclcpp::Duration period(0, 4000000);
  ASSERT_EQ(system.on_configure(state), hardware_interface::CallbackReturn::SUCCESS);
  EXPECT_EQ(system.configurationCount(), 1U);
  EXPECT_FALSE(system.motorSessionActive());

  command_interfaces.front().set_value(1.0);
  EXPECT_EQ(system.write(now, period), hardware_interface::return_type::OK);
  EXPECT_DOUBLE_EQ(command_interfaces.front().get_value(), 0.0);

  ASSERT_EQ(system.on_activate(state), hardware_interface::CallbackReturn::SUCCESS);
  EXPECT_TRUE(system.motorSessionActive());
  EXPECT_EQ(system.activationCount(), 1U);

  ASSERT_EQ(system.on_deactivate(state), hardware_interface::CallbackReturn::SUCCESS);
  EXPECT_FALSE(system.motorSessionActive());
  EXPECT_EQ(system.deactivationCount(), 1U);
  EXPECT_EQ(system.terminalBarrierCount(), 0U);

  EXPECT_EQ(system.read(now, period), hardware_interface::return_type::OK);
  EXPECT_EQ(system.readCount(), 1U);
  command_interfaces.front().set_value(1.0);
  EXPECT_EQ(system.write(now, period), hardware_interface::return_type::OK);
  EXPECT_DOUBLE_EQ(command_interfaces.front().get_value(), 0.0);

  EXPECT_EQ(system.on_activate(state), hardware_interface::CallbackReturn::SUCCESS);
  EXPECT_TRUE(system.motorSessionActive());
  EXPECT_EQ(system.activationCount(), 2U);

  ASSERT_EQ(system.on_deactivate(state), hardware_interface::CallbackReturn::SUCCESS);
  ASSERT_EQ(system.on_cleanup(state), hardware_interface::CallbackReturn::SUCCESS);
  ASSERT_EQ(system.on_configure(state), hardware_interface::CallbackReturn::SUCCESS);
  EXPECT_EQ(system.configurationCount(), 2U);
  EXPECT_EQ(system.on_activate(state), hardware_interface::CallbackReturn::SUCCESS);
  EXPECT_TRUE(system.motorSessionActive());
  EXPECT_EQ(system.activationCount(), 3U);
}

TEST(Cia402HardwareInfoTest, TerminalCleanupPreservesOrderAndPropagatesEveryFailure)
{
  rclcpp::init(0, nullptr);
  {
    const std::vector<TerminalCleanupResults> cases = {
      {},
      {false, true, true, true},
      {true, true, false, true},
      {true, true, true, false},
    };
    for (std::size_t index = 0U; index < cases.size(); ++index)
    {
      std::vector<std::string> events;
      TerminalInspectableCia402System system(events, cases[index]);
      auto container = std::make_shared<TerminalDeviceContainer>(
        events, cases[index], "terminal_container_" + std::to_string(index));
      system.installContainer(container);

      const auto result = system.on_cleanup(rclcpp_lifecycle::State());
      EXPECT_EQ(
        result, index == 0U ? hardware_interface::CallbackReturn::SUCCESS
                            : hardware_interface::CallbackReturn::ERROR);
      EXPECT_EQ(
        events,
        (std::vector<std::string>{"nmt_stop", "callback_barrier", "drivers", "master"}));

      const auto completed_event_count = events.size();
      EXPECT_EQ(
        system.on_cleanup(rclcpp_lifecycle::State()),
        hardware_interface::CallbackReturn::SUCCESS);
      EXPECT_EQ(events.size(), completed_event_count);
    }
  }
  rclcpp::shutdown();
}

TEST(Cia402HardwareInfoTest, TerminalCallbackBarrierFailureBlocksTeardownAndCanRetry)
{
  rclcpp::init(0, nullptr);
  {
    TerminalCleanupResults results{false, false, false, false};
    std::vector<std::string> events;
    TerminalInspectableCia402System system(events, results);
    auto container = std::make_shared<TerminalDeviceContainer>(
      events, results, "terminal_barrier_retry_container");
    system.installContainer(container);

    EXPECT_EQ(
      system.on_cleanup(rclcpp_lifecycle::State()),
      hardware_interface::CallbackReturn::ERROR);
    EXPECT_EQ(events, (std::vector<std::string>{"nmt_stop", "callback_barrier"}));
    EXPECT_TRUE(system.hasInstalledContainer());

    results = TerminalCleanupResults{};
    EXPECT_EQ(
      system.on_cleanup(rclcpp_lifecycle::State()),
      hardware_interface::CallbackReturn::SUCCESS);
    EXPECT_EQ(
      events,
      (std::vector<std::string>{
        "nmt_stop", "callback_barrier", "nmt_stop", "callback_barrier", "drivers", "master"}));
    EXPECT_FALSE(system.hasInstalledContainer());

    const auto completed_event_count = events.size();
    EXPECT_EQ(
      system.on_cleanup(rclcpp_lifecycle::State()),
      hardware_interface::CallbackReturn::SUCCESS);
    EXPECT_EQ(events.size(), completed_event_count);
  }
  rclcpp::shutdown();
}

TEST(Cia402HardwareInfoTest, TerminalShutdownPropagatesCleanupFailure)
{
  rclcpp::init(0, nullptr);
  {
    const TerminalCleanupResults results{true, true, true, false};
    std::vector<std::string> events;
    TerminalInspectableCia402System system(events, results);
    auto container = std::make_shared<TerminalDeviceContainer>(
      events, results, "terminal_shutdown_container");
    system.installContainer(container);

    EXPECT_EQ(
      system.on_shutdown(rclcpp_lifecycle::State()),
      hardware_interface::CallbackReturn::ERROR);
    EXPECT_EQ(
      events,
      (std::vector<std::string>{"nmt_stop", "callback_barrier", "drivers", "master"}));
  }
  rclcpp::shutdown();
}

TEST(Cia402HardwareInfoTest, ErrorTransitionRunsTerminalCleanupBeforeFinalization)
{
  rclcpp::init(0, nullptr);
  {
    const TerminalCleanupResults results;
    std::vector<std::string> events;
    TerminalInspectableCia402System system(events, results);
    auto container = std::make_shared<TerminalDeviceContainer>(
      events, results, "terminal_error_container");
    system.installContainer(container);

    EXPECT_EQ(
      system.on_error(rclcpp_lifecycle::State()),
      hardware_interface::CallbackReturn::SUCCESS);
    EXPECT_EQ(
      events,
      (std::vector<std::string>{"nmt_stop", "callback_barrier", "drivers", "master"}));
    EXPECT_FALSE(system.hasInstalledContainer());
  }
  {
    TerminalCleanupResults results{true, false, true, true};
    std::vector<std::string> events;
    TerminalInspectableCia402System system(events, results);
    auto container = std::make_shared<TerminalDeviceContainer>(
      events, results, "terminal_error_barrier_container");
    system.installContainer(container);

    EXPECT_EQ(
      system.on_error(rclcpp_lifecycle::State()),
      hardware_interface::CallbackReturn::ERROR);
    EXPECT_EQ(events, (std::vector<std::string>{"nmt_stop", "callback_barrier"}));
    EXPECT_TRUE(system.hasInstalledContainer());

    results = TerminalCleanupResults{};
    EXPECT_EQ(
      system.on_cleanup(rclcpp_lifecycle::State()),
      hardware_interface::CallbackReturn::SUCCESS);
  }
  rclcpp::shutdown();
}

TEST(Cia402HardwareInfoTest, BaseErrorTransitionRunsTerminalCleanupBeforeFinalization)
{
  rclcpp::init(0, nullptr);
  {
    const TerminalCleanupResults results;
    std::vector<std::string> events;
    TerminalInspectableCanopenSystem system(events, results);
    auto container = std::make_shared<TerminalDeviceContainer>(
      events, results, "base_terminal_error_container");
    system.installContainer(container);

    EXPECT_EQ(
      system.on_error(rclcpp_lifecycle::State()),
      hardware_interface::CallbackReturn::SUCCESS);
    EXPECT_EQ(
      events,
      (std::vector<std::string>{"nmt_stop", "callback_barrier", "drivers", "master"}));
    EXPECT_FALSE(system.hasInstalledContainer());
  }
  rclcpp::shutdown();
}

TEST(Cia402HardwareInfoTest, DerivedDestructorCleanupIsIdempotentWithBaseDestructor)
{
  rclcpp::init(0, nullptr);
  {
    const TerminalCleanupResults results;
    std::vector<std::string> events;
    {
      TerminalInspectableCia402System system(events, results);
      auto container = std::make_shared<TerminalDeviceContainer>(
        events, results, "terminal_destructor_container");
      system.installContainer(container);
    }
    EXPECT_EQ(events, (std::vector<std::string>{"nmt_stop", "drivers", "master"}));
  }
  rclcpp::shutdown();
}

TEST(Cia402HardwareInfoTest, DerivedDestructorFailStopsIfCallbackBarrierCannotJoin)
{
  ::testing::FLAGS_gtest_death_test_style = "threadsafe";
  EXPECT_DEATH(
    {
      std::promise<void> release;
      std::promise<void> deletion_started;
      auto deletion_started_future = deletion_started.get_future();
      auto * system = new SelfDeletingSystem<Cia402System>();
      system->deleteOnOwnedThread(release.get_future().share(), deletion_started);
      release.set_value();
      deletion_started_future.wait();
      std::this_thread::sleep_for(std::chrono::seconds(5));
    },
    "Cia402System destruction could not quiesce CANopen callbacks");
}

TEST(Cia402HardwareInfoTest, BaseDestructorFailStopsIfCallbackBarrierCannotJoin)
{
  ::testing::FLAGS_gtest_death_test_style = "threadsafe";
  EXPECT_DEATH(
    {
      std::promise<void> release;
      std::promise<void> deletion_started;
      auto deletion_started_future = deletion_started.get_future();
      auto * system = new SelfDeletingSystem<CanopenSystem>();
      system->deleteOnOwnedThread(release.get_future().share(), deletion_started);
      release.set_value();
      deletion_started_future.wait();
      std::this_thread::sleep_for(std::chrono::seconds(5));
    },
    "CanopenSystem destruction could not quiesce CANopen callbacks");
}

TEST(Cia402HardwareInfoTest, ActualOperationModeCacheStartsFailClosed)
{
  ros2_canopen::Motor402 motor(
    nullptr, ros2_canopen::State402::Switch_On_Disabled, 0);

  EXPECT_EQ(motor.getActualMode(), MotorBase::No_Mode);
}

TEST(Cia402HardwareInfoTest, DriverModeLockIsCompatibleUntilExplicitlyFrozen)
{
  rclcpp::init(0, nullptr);
  {
    ros2_canopen::Cia402Driver driver(
      rclcpp::NodeOptions().use_global_arguments(false));

    EXPECT_TRUE(driver.is_operation_mode_allowed(MotorBase::Profiled_Position));
    EXPECT_TRUE(driver.is_operation_mode_allowed(MotorBase::Cyclic_Synchronous_Velocity));
    EXPECT_FALSE(driver.lock_operation_mode(MotorBase::No_Mode));
    EXPECT_TRUE(driver.is_operation_mode_allowed(MotorBase::Profiled_Torque));
    EXPECT_TRUE(driver.lock_operation_mode(MotorBase::Profiled_Velocity));
    EXPECT_TRUE(driver.lock_operation_mode(MotorBase::Profiled_Velocity));
    EXPECT_TRUE(driver.is_operation_mode_allowed(MotorBase::Profiled_Velocity));
    EXPECT_FALSE(driver.is_operation_mode_allowed(MotorBase::Profiled_Position));
    EXPECT_FALSE(driver.is_operation_mode_allowed(MotorBase::Cyclic_Synchronous_Velocity));
    EXPECT_FALSE(driver.is_operation_mode_allowed(MotorBase::Profiled_Torque));
    EXPECT_FALSE(driver.lock_operation_mode(MotorBase::Profiled_Position));
  }
  {
    ros2_canopen::Cia402Driver concurrent_driver(
      rclcpp::NodeOptions().use_global_arguments(false));
    bool velocity_lock = false;
    bool position_lock = false;
    std::thread velocity_thread(
      [&concurrent_driver, &velocity_lock]()
      { velocity_lock = concurrent_driver.lock_operation_mode(MotorBase::Profiled_Velocity); });
    std::thread position_thread(
      [&concurrent_driver, &position_lock]()
      { position_lock = concurrent_driver.lock_operation_mode(MotorBase::Profiled_Position); });
    velocity_thread.join();
    position_thread.join();

    EXPECT_NE(velocity_lock, position_lock);
    EXPECT_NE(
      concurrent_driver.is_operation_mode_allowed(MotorBase::Profiled_Velocity),
      concurrent_driver.is_operation_mode_allowed(MotorBase::Profiled_Position));
  }
  rclcpp::shutdown();
}

TEST(Cia402HardwareInfoTest, EmbeddedContainerOmitsInitDriverService)
{
  rclcpp::init(0, nullptr);
  {
    auto executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
    auto options = rclcpp::NodeOptions().use_global_arguments(false);
    options.parameter_overrides(
      {rclcpp::Parameter("expose_mutating_ros_api", false)});
    auto container = std::make_shared<ros2_canopen::DeviceContainer>(
      executor, "restricted_device_container", options);

    const auto services = container->get_service_names_and_types_by_node(
      container->get_name(), container->get_namespace());
    EXPECT_EQ(services.count("/restricted_device_container/init_driver"), 0U);
  }
  rclcpp::shutdown();
}

TEST(Cia402HardwareInfoTest, EmbeddedContainerRejectsLifecycleManagerMutationSurface)
{
  rclcpp::init(0, nullptr);
  {
    auto executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
    auto options = rclcpp::NodeOptions().use_global_arguments(false);
    options.parameter_overrides(
      {rclcpp::Parameter("expose_mutating_ros_api", false)});
    auto container = std::make_shared<InspectableDeviceContainer>(
      executor, "restricted_lifecycle_device_container", options);
    container->simulateLifecycleOperation();

    EXPECT_FALSE(container->load_manager());
    EXPECT_FALSE(container->hasLifecycleManager());
  }
  rclcpp::shutdown();
}

TEST(Cia402HardwareInfoTest, EmbeddedDriverOmitsEveryMutatingRosEntryPoint)
{
  rclcpp::init(0, nullptr);
  {
    auto options = rclcpp::NodeOptions().use_global_arguments(false);
    options.arguments(
      {"--ros-args", "-r", "__node:=restricted_cia402_driver"});
    options.parameter_overrides(
      {rclcpp::Parameter("expose_mutating_ros_api", false)});
    ros2_canopen::Cia402Driver driver(options);
    driver.get_node_canopen_driver_interface()->init();

    const auto services =
      driver.get_service_names_and_types_by_node(driver.get_name(), driver.get_namespace());
    EXPECT_EQ(services.count("/restricted_cia402_driver/sdo_read"), 1U);
    const std::vector<std::string> forbidden_services = {
      "/restricted_cia402_driver/sdo_write",
      "/restricted_cia402_driver/nmt_reset_node",
      "/restricted_cia402_driver/nmt_start_node",
      "/restricted_cia402_driver/init",
      "/restricted_cia402_driver/enable",
      "/restricted_cia402_driver/disable",
      "/restricted_cia402_driver/halt",
      "/restricted_cia402_driver/recover",
      "/restricted_cia402_driver/position_mode",
      "/restricted_cia402_driver/velocity_mode",
      "/restricted_cia402_driver/cyclic_velocity_mode",
      "/restricted_cia402_driver/cyclic_position_mode",
      "/restricted_cia402_driver/interpolated_position_mode",
      "/restricted_cia402_driver/torque_mode",
      "/restricted_cia402_driver/cyclic_torque_mode",
      "/restricted_cia402_driver/target",
    };
    for (const auto & service_name : forbidden_services)
    {
      EXPECT_EQ(services.count(service_name), 0U) << service_name;
    }

    const auto topics = driver.get_topic_names_and_types();
    EXPECT_EQ(topics.count("/restricted_cia402_driver/tpdo"), 0U);
  }
  rclcpp::shutdown();
}

TEST(Cia402HardwareInfoTest, StandaloneDriverKeepsUpstreamMutatingRosApiByDefault)
{
  rclcpp::init(0, nullptr);
  {
    auto options = rclcpp::NodeOptions().use_global_arguments(false);
    options.arguments(
      {"--ros-args", "-r", "__node:=standalone_cia402_driver"});
    ros2_canopen::Cia402Driver driver(options);
    driver.get_node_canopen_driver_interface()->init();

    const auto services =
      driver.get_service_names_and_types_by_node(driver.get_name(), driver.get_namespace());
    EXPECT_EQ(services.count("/standalone_cia402_driver/sdo_read"), 1U);
    EXPECT_EQ(services.count("/standalone_cia402_driver/sdo_write"), 1U);
    EXPECT_EQ(services.count("/standalone_cia402_driver/target"), 1U);
    EXPECT_EQ(services.count("/standalone_cia402_driver/position_mode"), 1U);

    const auto topics = driver.get_topic_names_and_types();
    EXPECT_EQ(topics.count("/standalone_cia402_driver/tpdo"), 1U);
  }
  rclcpp::shutdown();
}

TEST(Cia402HardwareInfoTest, RejectsEmptyOrDuplicateMotorTopology)
{
  Cia402System empty_system;
  EXPECT_EQ(
    empty_system.on_init(makeHardwareInfo({})), hardware_interface::CallbackReturn::ERROR);

  Cia402System duplicate_system;
  EXPECT_EQ(
    duplicate_system.on_init(makeHardwareInfo({
      {"left_track", {"2", "3"}},
      {"right_track", {"2", "3"}},
    })),
    hardware_interface::CallbackReturn::ERROR);
}

class InvalidCia402ParameterTest :
  public ::testing::TestWithParam<std::pair<std::string, JointParameters>>
{};

TEST_P(InvalidCia402ParameterTest, RejectsMalformedOrUnsafeParameters)
{
  Cia402System system;
  const auto & [label, parameters] = GetParam();
  EXPECT_EQ(
    system.on_init(makeHardwareInfo({{label, parameters}})),
    hardware_interface::CallbackReturn::ERROR)
    << label;
}

INSTANTIATE_TEST_SUITE_P(
  InvalidParameters, InvalidCia402ParameterTest,
  ::testing::Values(
    std::make_pair("both_missing", JointParameters{"", ""}),
    std::make_pair("missing_node", JointParameters{"", "3"}),
    std::make_pair("missing_mode", JointParameters{"2", ""}),
    std::make_pair("non_numeric_node", JointParameters{"two", "3"}),
    std::make_pair("trailing_node", JointParameters{"2x", "3"}),
    std::make_pair("leading_space_node", JointParameters{" 2", "3"}),
    std::make_pair("negative_node", JointParameters{"-2", "3"}),
    std::make_pair("fractional_node", JointParameters{"2.0", "3"}),
    std::make_pair("zero_node", JointParameters{"0", "3"}),
    std::make_pair("large_node", JointParameters{"128", "3"}),
    std::make_pair("unsafe_mode", JointParameters{"2", "9"})));

}  // namespace
}  // namespace canopen_ros2_control
