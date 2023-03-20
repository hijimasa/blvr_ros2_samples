// Copyright 2021 ros2_control Development Team
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

#ifndef BLVR_DIFFBOT_HARDWARE__BLVR_DIFFBOT_SYSTEM_HPP_
#define BLVR_DIFFBOT_HARDWARE__BLVR_DIFFBOT_SYSTEM_HPP_

#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "blvr_diffbot_hardware/visibility_control.h"
#include <blvr_diffbot_hardware/blvr_comunicator.h>

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace blvr_diffbot_hardware
{
class BlvrDiffbotSystemHardware : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(BlvrDiffbotSystemHardware);

  BLVR_DIFFBOT_HARDWARE_PUBLIC
  CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;

  BLVR_DIFFBOT_HARDWARE_PUBLIC
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  BLVR_DIFFBOT_HARDWARE_PUBLIC
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  BLVR_DIFFBOT_HARDWARE_PUBLIC
  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;

  BLVR_DIFFBOT_HARDWARE_PUBLIC
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  BLVR_DIFFBOT_HARDWARE_PUBLIC
  hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;

  BLVR_DIFFBOT_HARDWARE_PUBLIC
  hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;
  
  rclcpp::Duration getPeriod() const { return rclcpp::Duration(0, 100000000); } // loop period is 0.1s


private:
  // Parameters for the BlvrDiffbot simulation
  double hw_start_sec_;
  double hw_stop_sec_;

  // Store the command for the simulated robot
  std::vector<double> hw_commands_;
  std::vector<double> hw_positions_;
  std::vector<double> hw_velocities_;
  std::vector<double> hw_efforts_;
  
  std::vector<int> prev_step_;
  std::string device_name_;
  double gear_ratio_;
  double encoder_resolution_;
  std::shared_ptr<BlvrComunicator> serial_port_;

};

}  // namespace blvr_diffbot_hardware

#endif  // BLVR_DIFFBOT_HARDWARE__BLVR_DIFFBOT_SYSTEM_HPP_
