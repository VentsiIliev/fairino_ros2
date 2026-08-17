// Copyright 2022 ICUBE Laboratory, University of Strasbourg
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

#include "ethercat_driver/ethercat_driver.hpp"

#include <tinyxml2.h>
#include <string>
#include <regex>
#include <thread>
#include <atomic>
#include <pthread.h>
#include <sched.h>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace ethercat_driver
{
namespace
{
std::string trim_xml_text(const char * text)
{
  if (text == nullptr) {
    return {};
  }

  const std::string value(text);
  const auto first = value.find_first_not_of(" \t\r\n");
  if (first == std::string::npos) {
    return {};
  }
  const auto last = value.find_last_not_of(" \t\r\n");
  return value.substr(first, last - first + 1);
}
}  // namespace

CallbackReturn EthercatDriver::on_init(
  const hardware_interface::HardwareComponentInterfaceParams & params)
{
  if (hardware_interface::SystemInterface::on_init(params) != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }

  const std::lock_guard<std::mutex> lock(ec_mutex_);
  activated_ = false;
  freeze_on_slave_fault_ = false;

  hw_joint_states_.resize(info_.joints.size());
  for (uint j = 0; j < info_.joints.size(); j++) {
    hw_joint_states_[j].resize(
      info_.joints[j].state_interfaces.size(),
      0.0);
  }
  hw_sensor_states_.resize(info_.sensors.size());
  for (uint s = 0; s < info_.sensors.size(); s++) {
    hw_sensor_states_[s].resize(
      info_.sensors[s].state_interfaces.size(),
      0.0);
  }
  hw_gpio_states_.resize(info_.gpios.size());
  for (uint g = 0; g < info_.gpios.size(); g++) {
    hw_gpio_states_[g].resize(
      info_.gpios[g].state_interfaces.size(),
      0.0);
  }
  hw_joint_commands_.resize(info_.joints.size());
  for (uint j = 0; j < info_.joints.size(); j++) {
    hw_joint_commands_[j].resize(
      info_.joints[j].command_interfaces.size(),
      std::numeric_limits<double>::quiet_NaN());
  }
  hw_sensor_commands_.resize(info_.sensors.size());
  for (uint s = 0; s < info_.sensors.size(); s++) {
    hw_sensor_commands_[s].resize(
      info_.sensors[s].command_interfaces.size(),
      std::numeric_limits<double>::quiet_NaN());
  }
  hw_gpio_commands_.resize(info_.gpios.size());
  for (uint g = 0; g < info_.gpios.size(); g++) {
    hw_gpio_commands_[g].resize(
      info_.gpios[g].command_interfaces.size(),
      std::numeric_limits<double>::quiet_NaN());
  }

  for (uint j = 0; j < info_.joints.size(); j++) {
    RCLCPP_INFO(rclcpp::get_logger("EthercatDriver"), "joints");
    // check all joints for EC modules and load into ec_modules_
    auto module_params = getEcModuleParam(info_.original_xml, info_.joints[j].name, "joint");
    ec_module_parameters_.insert(
      ec_module_parameters_.end(), module_params.begin(), module_params.end());
    for (auto i = 0ul; i < module_params.size(); i++) {
      for (auto k = 0ul; k < info_.joints[j].state_interfaces.size(); k++) {
        module_params[i]["state_interface/" +
          info_.joints[j].state_interfaces[k].name] = std::to_string(k);
      }
      for (auto k = 0ul; k < info_.joints[j].command_interfaces.size(); k++) {
        module_params[i]["command_interface/" +
          info_.joints[j].command_interfaces[k].name] = std::to_string(k);
      }
      try {
        auto module = ec_loader_.createSharedInstance(module_params[i].at("plugin"));
        if (!module->setupSlave(
            module_params[i], &hw_joint_states_[j], &hw_joint_commands_[j]))
        {
          RCLCPP_FATAL(
            rclcpp::get_logger("EthercatDriver"),
            "Setup of Joint module %li FAILED.", i + 1);
          return CallbackReturn::ERROR;
        }
        ec_modules_.push_back(module);
      } catch (pluginlib::PluginlibException & ex) {
        RCLCPP_FATAL(
          rclcpp::get_logger("EthercatDriver"),
          "The plugin of %s failed to load for some reason. Error: %s\n",
          info_.joints[j].name.c_str(), ex.what());
      }
    }
  }
  for (uint g = 0; g < info_.gpios.size(); g++) {
    RCLCPP_INFO(rclcpp::get_logger("EthercatDriver"), "gpios");
    // check all gpios for EC modules and load into ec_modules_
    auto module_params = getEcModuleParam(info_.original_xml, info_.gpios[g].name, "gpio");
    ec_module_parameters_.insert(
      ec_module_parameters_.end(), module_params.begin(), module_params.end());
    for (auto i = 0ul; i < module_params.size(); i++) {
      for (auto k = 0ul; k < info_.gpios[g].state_interfaces.size(); k++) {
        module_params[i]["state_interface/" +
          info_.gpios[g].state_interfaces[k].name] = std::to_string(k);
      }
      for (auto k = 0ul; k < info_.gpios[g].command_interfaces.size(); k++) {
        module_params[i]["command_interface/" +
          info_.gpios[g].command_interfaces[k].name] = std::to_string(k);
      }
      try {
        auto module = ec_loader_.createSharedInstance(module_params[i].at("plugin"));
        if (!module->setupSlave(
            module_params[i], &hw_gpio_states_[g], &hw_gpio_commands_[g]))
        {
          RCLCPP_FATAL(
            rclcpp::get_logger("EthercatDriver"),
            "Setup of GPIO module %li FAILED.", i + 1);
          return CallbackReturn::ERROR;
        }
        ec_modules_.push_back(module);
      } catch (pluginlib::PluginlibException & ex) {
        RCLCPP_FATAL(
          rclcpp::get_logger("EthercatDriver"),
          "The plugin of %s failed to load for some reason. Error: %s\n",
          info_.gpios[g].name.c_str(), ex.what());
      }
    }
  }
  for (uint s = 0; s < info_.sensors.size(); s++) {
    RCLCPP_INFO(rclcpp::get_logger("EthercatDriver"), "sensors");
    // check all sensors for EC modules and load into ec_modules_
    auto module_params = getEcModuleParam(info_.original_xml, info_.sensors[s].name, "sensor");
    ec_module_parameters_.insert(
      ec_module_parameters_.end(), module_params.begin(), module_params.end());
    for (auto i = 0ul; i < module_params.size(); i++) {
      for (auto k = 0ul; k < info_.sensors[s].state_interfaces.size(); k++) {
        module_params[i]["state_interface/" +
          info_.sensors[s].state_interfaces[k].name] = std::to_string(k);
      }
      for (auto k = 0ul; k < info_.sensors[s].command_interfaces.size(); k++) {
        module_params[i]["command_interface/" +
          info_.sensors[s].command_interfaces[k].name] = std::to_string(k);
      }
      try {
        auto module = ec_loader_.createSharedInstance(module_params[i].at("plugin"));
        if (!module->setupSlave(
            module_params[i], &hw_sensor_states_[s], &hw_sensor_commands_[s]))
        {
          RCLCPP_FATAL(
            rclcpp::get_logger("EthercatDriver"),
            "Setup of Sensor module %li FAILED.", i + 1);
          return CallbackReturn::ERROR;
        }
        ec_modules_.push_back(module);
      } catch (pluginlib::PluginlibException & ex) {
        RCLCPP_FATAL(
          rclcpp::get_logger("EthercatDriver"),
          "The plugin of %s failed to load for some reason. Error: %s\n",
          info_.sensors[s].name.c_str(), ex.what());
      }
    }
  }

  RCLCPP_INFO(rclcpp::get_logger("EthercatDriver"), "Got %li modules", ec_modules_.size());

  return CallbackReturn::SUCCESS;
}

CallbackReturn EthercatDriver::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
EthercatDriver::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  // export joint state interface
  for (uint j = 0; j < info_.joints.size(); j++) {
    for (uint i = 0; i < info_.joints[j].state_interfaces.size(); i++) {
      state_interfaces.emplace_back(
        hardware_interface::StateInterface(
          info_.joints[j].name,
          info_.joints[j].state_interfaces[i].name,
          &hw_joint_states_[j][i]));
    }
  }
  // export sensor state interface
  for (uint s = 0; s < info_.sensors.size(); s++) {
    for (uint i = 0; i < info_.sensors[s].state_interfaces.size(); i++) {
      state_interfaces.emplace_back(
        hardware_interface::StateInterface(
          info_.sensors[s].name,
          info_.sensors[s].state_interfaces[i].name,
          &hw_sensor_states_[s][i]));
    }
  }
  // export gpio state interface
  for (uint g = 0; g < info_.gpios.size(); g++) {
    for (uint i = 0; i < info_.gpios[g].state_interfaces.size(); i++) {
      state_interfaces.emplace_back(
        hardware_interface::StateInterface(
          info_.gpios[g].name,
          info_.gpios[g].state_interfaces[i].name,
          &hw_gpio_states_[g][i]));
    }
  }
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
EthercatDriver::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  // export joint command interface
  std::vector<double> test;
  for (uint j = 0; j < info_.joints.size(); j++) {
    for (uint i = 0; i < info_.joints[j].command_interfaces.size(); i++) {
      command_interfaces.emplace_back(
        hardware_interface::CommandInterface(
          info_.joints[j].name,
          info_.joints[j].command_interfaces[i].name,
          &hw_joint_commands_[j][i]));
    }
  }
  // export sensor command interface
  for (uint s = 0; s < info_.sensors.size(); s++) {
    for (uint i = 0; i < info_.sensors[s].command_interfaces.size(); i++) {
      command_interfaces.emplace_back(
        hardware_interface::CommandInterface(
          info_.sensors[s].name,
          info_.sensors[s].command_interfaces[i].name,
          &hw_sensor_commands_[s][i]));
    }
  }
  // export gpio command interface
  for (uint g = 0; g < info_.gpios.size(); g++) {
    for (uint i = 0; i < info_.gpios[g].command_interfaces.size(); i++) {
      command_interfaces.emplace_back(
        hardware_interface::CommandInterface(
          info_.gpios[g].name,
          info_.gpios[g].command_interfaces[i].name,
          &hw_gpio_commands_[g][i]));
    }
  }
  return command_interfaces;
}

CallbackReturn EthercatDriver::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  const std::lock_guard<std::mutex> lock(ec_mutex_);
  if (activated_) {
    RCLCPP_FATAL(rclcpp::get_logger("EthercatDriver"), "Double on_activate()");
    return CallbackReturn::ERROR;
  }
  RCLCPP_INFO(rclcpp::get_logger("EthercatDriver"), "Starting EtherCAT...");
  if (info_.hardware_parameters.find("control_frequency") == info_.hardware_parameters.end()) {
    control_frequency_ = 100;
  } else {
    control_frequency_ = std::stod(info_.hardware_parameters["control_frequency"]);
  }
  if (control_frequency_ <= 0) {
    RCLCPP_FATAL(rclcpp::get_logger("EthercatDriver"), "Invalid control frequency!");
    return CallbackReturn::ERROR;
  }

  if (info_.hardware_parameters.find("freeze_on_slave_fault") != info_.hardware_parameters.end()) {
    const auto & value = info_.hardware_parameters["freeze_on_slave_fault"];
    freeze_on_slave_fault_ = (value == "true" || value == "1" || value == "yes" || value == "on");
  }
  if (info_.hardware_parameters.find("control_thread_cpu") != info_.hardware_parameters.end()) {
    control_thread_cpu_ = std::stoi(info_.hardware_parameters["control_thread_cpu"]);
  } else {
    control_thread_cpu_ = -1;
  }

  master_.setCtrlFrequency(control_frequency_);

  for (auto i = 0ul; i < ec_modules_.size(); i++) {
    master_.addSlave(
      std::stod(ec_module_parameters_[i]["alias"]),
      std::stod(ec_module_parameters_[i]["position"]),
      ec_modules_[i].get());
  }

  for (auto i = 0ul; i < ec_modules_.size(); i++) {
    for (auto & sdo : ec_modules_[i]->sdo_config) {
      uint32_t abort_code;
      int ret = master_.configSlaveSdo(
        std::stod(ec_module_parameters_[i]["position"]),
        sdo,
        &abort_code
      );
      if (ret) {
        RCLCPP_INFO(
          rclcpp::get_logger("EthercatDriver"),
          "Failed SDO config module pos %s err %d",
          ec_module_parameters_[i]["position"].c_str(), abort_code);
      }
    }
  }

  if (!master_.activate()) {
    RCLCPP_ERROR(rclcpp::get_logger("EthercatDriver"), "Activate EcMaster failed");
    return CallbackReturn::ERROR;
  }
  RCLCPP_INFO(rclcpp::get_logger("EthercatDriver"), "EcMaster activated, launching cycle thread");

  all_modules_operational_.store(false);
  run_ctrl_.store(true);
  control_thread_ = std::thread([this]() {
    if (control_thread_cpu_ >= 0) {
      cpu_set_t cpuset;
      CPU_ZERO(&cpuset);
      CPU_SET(control_thread_cpu_, &cpuset);
      if (pthread_setaffinity_np(pthread_self(), sizeof(cpu_set_t), &cpuset) != 0) {
        RCLCPP_WARN(
          rclcpp::get_logger("EthercatDriver"),
          "Failed to set EtherCAT control thread affinity to CPU %d.",
          control_thread_cpu_);
      }
    }

    // Set thread priority
    struct sched_param param;
    param.sched_priority = 80;  // High priority for EtherCAT
    if (pthread_setschedparam(pthread_self(), SCHED_FIFO, &param) != 0) {
      RCLCPP_WARN(rclcpp::get_logger("EthercatDriver"),
                  "Failed to set real-time priority. Run with sudo or configure permissions.");
    }

    struct timespec t;
    clock_gettime(CLOCK_MONOTONIC, &t);
    t.tv_sec += 1;
    uint32_t interval = master_.getInterval();
    uint64_t cycle_count = 0;
    uint64_t missed_cycles = 0;
    uint64_t consecutive_misses = 0;
    struct timespec last_t = t;

    while (run_ctrl_.load()) {
      struct timespec now;
      clock_gettime(CLOCK_MONOTONIC, &now);

      int64_t time_diff_ns = (now.tv_sec - last_t.tv_sec) * 1000000000LL +
                             (now.tv_nsec - last_t.tv_nsec);

      // Detect missed cycles (tolerance: 150% over target to handle transient CPU spikes)
      if (cycle_count > 0 && time_diff_ns > (interval * 2.5)) {
        missed_cycles++;
        consecutive_misses++;

        if (consecutive_misses >= 5) {
          RCLCPP_WARN(
            rclcpp::get_logger("EthercatDriver"),
            "⚠️  %lu consecutive missed cycles! Reduce CPU load to prevent slave state drops.",
            consecutive_misses);
        }

        if (missed_cycles % 200 == 1) {
          RCLCPP_WARN(
            rclcpp::get_logger("EthercatDriver"),
            "Missed cycle! Expected: %u ns, Actual: %ld ns (%.1f%% overrun). "
            "Total missed: %lu/%lu (%.2f%%)",
            interval, time_diff_ns,
            ((time_diff_ns - interval) * 100.0 / interval),
            missed_cycles, cycle_count,
            (missed_cycles * 100.0 / cycle_count));
        }
      } else {
        consecutive_misses = 0;
      }

      clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &t, NULL);
      {
        std::lock_guard<std::mutex> lk(ec_mutex_);
        master_.update();

        bool all_operational = true;
        for (auto & m : ec_modules_) {
          all_operational &= m->initialized();
        }

        const bool was_operational = all_modules_operational_.exchange(all_operational);
        if (freeze_on_slave_fault_ && !all_operational) {
          freeze_command_interfaces();
          if (was_operational) {
            RCLCPP_ERROR(
              rclcpp::get_logger("EthercatDriver"),
              "One or more EtherCAT modules lost operational state; freezing all motion commands.");
          }
        } else if (freeze_on_slave_fault_ && !was_operational) {
          RCLCPP_INFO(
            rclcpp::get_logger("EthercatDriver"),
            "All EtherCAT modules operational again; motion commands re-enabled.");
        }
      }

      last_t = t;
      t.tv_nsec += interval;
      while (t.tv_nsec >= 1000000000) { t.tv_nsec -= 1000000000; t.tv_sec++; }
      cycle_count++;
    }

    if (missed_cycles > 0) {
      RCLCPP_INFO(
        rclcpp::get_logger("EthercatDriver"),
        "EtherCAT cycle statistics: %lu missed cycles out of %lu total (%.2f%%)",
        missed_cycles, cycle_count, (missed_cycles * 100.0 / cycle_count));
    }
  });

  // wait until all modules initialized or timeout
  auto start = std::chrono::steady_clock::now();
  const auto timeout = std::chrono::seconds(30);
  int last_init_count = 0;
  while (true) {
    bool all_init = true;
    int init_count = 0;
    for (auto & m : ec_modules_) {
      if (m->initialized()) init_count++;
      all_init &= m->initialized();
    }

    if (init_count != last_init_count) {
      RCLCPP_INFO(
        rclcpp::get_logger("EthercatDriver"),
        "Initialization progress: %d/%lu modules operational",
        init_count, ec_modules_.size());
      last_init_count = init_count;
    }

    if (all_init) break;
    if (std::chrono::steady_clock::now() - start > timeout) {
      RCLCPP_ERROR(
        rclcpp::get_logger("EthercatDriver"),
        "Initialization timeout - only %d/%lu modules operational",
        init_count, ec_modules_.size());
      run_ctrl_.store(false);
      if (control_thread_.joinable()) control_thread_.join();
      return CallbackReturn::ERROR;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }

  activated_ = true;
  all_modules_operational_.store(true);
  RCLCPP_INFO(rclcpp::get_logger("EthercatDriver"), "System started (modules operational)");
  return CallbackReturn::SUCCESS;
}

CallbackReturn EthercatDriver::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  const std::lock_guard<std::mutex> lock(ec_mutex_);
  activated_ = false;
  RCLCPP_INFO(rclcpp::get_logger("EthercatDriver"), "Stopping EtherCAT...");
  run_ctrl_.store(false);
  if (control_thread_.joinable()) { control_thread_.join(); }
  master_.stop();
  RCLCPP_INFO(rclcpp::get_logger("EthercatDriver"), "System stopped");
  return CallbackReturn::SUCCESS;
}

hardware_interface::return_type EthercatDriver::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // state already updated in background thread, nothing extra
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type EthercatDriver::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  if (freeze_on_slave_fault_ && !all_modules_operational_.load()) {
    freeze_command_interfaces();
    return hardware_interface::return_type::ERROR;
  }

  // commands picked up directly by background thread during master_.update()
  return hardware_interface::return_type::OK;
}

void EthercatDriver::freeze_command_interfaces()
{
  const double nan = std::numeric_limits<double>::quiet_NaN();

  for (auto & joint_cmds : hw_joint_commands_) {
    for (auto & value : joint_cmds) {
      value = nan;
    }
  }

  for (auto & sensor_cmds : hw_sensor_commands_) {
    for (auto & value : sensor_cmds) {
      value = nan;
    }
  }

  for (auto & gpio_cmds : hw_gpio_commands_) {
    for (auto & value : gpio_cmds) {
      value = nan;
    }
  }
}

std::vector<std::unordered_map<std::string, std::string>> EthercatDriver::getEcModuleParam(
  std::string & urdf,
  std::string component_name,
  std::string component_type)
{
  // Check if everything OK with URDF string
  if (urdf.empty()) {
    throw std::runtime_error("empty URDF passed to robot");
  }
  tinyxml2::XMLDocument doc;
  if (!doc.Parse(urdf.c_str()) && doc.Error()) {
    throw std::runtime_error("invalid URDF passed in to robot parser");
  }
  if (doc.Error()) {
    throw std::runtime_error("invalid URDF passed in to robot parser");
  }

  tinyxml2::XMLElement * robot_it = doc.RootElement();
  if (std::string("robot").compare(robot_it->Name())) {
    throw std::runtime_error("the robot tag is not root element in URDF");
  }

  const tinyxml2::XMLElement * ros2_control_it = robot_it->FirstChildElement("ros2_control");
  if (!ros2_control_it) {
    throw std::runtime_error("no ros2_control tag");
  }

  std::vector<std::unordered_map<std::string, std::string>> module_params;
  std::unordered_map<std::string, std::string> module_param;

  while (ros2_control_it) {
    const auto * ros2_control_child_it = ros2_control_it->FirstChildElement(component_type.c_str());
    while (ros2_control_child_it) {
      if (!component_name.compare(ros2_control_child_it->Attribute("name"))) {
        const auto * ec_module_it = ros2_control_child_it->FirstChildElement("ec_module");
        while (ec_module_it) {
          module_param.clear();
          module_param["name"] = ec_module_it->Attribute("name");
          const auto * plugin_it = ec_module_it->FirstChildElement("plugin");
          if (NULL != plugin_it) {
            module_param["plugin"] = trim_xml_text(plugin_it->GetText());
          }
          const auto * param_it = ec_module_it->FirstChildElement("param");
          while (param_it) {
            module_param[param_it->Attribute("name")] = trim_xml_text(param_it->GetText());
            param_it = param_it->NextSiblingElement("param");
          }
          module_params.push_back(module_param);
          ec_module_it = ec_module_it->NextSiblingElement("ec_module");
        }
      }
      ros2_control_child_it = ros2_control_child_it->NextSiblingElement(component_type.c_str());
    }
    ros2_control_it = ros2_control_it->NextSiblingElement("ros2_control");
  }

  return module_params;
}

}  // namespace ethercat_driver

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  ethercat_driver::EthercatDriver, hardware_interface::SystemInterface)
