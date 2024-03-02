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

#include "diff_control/diffbot_system.hpp"

#include <chrono>
#include <cmath>
#include <cstddef>
#include <limits>
#include <memory>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace diff_control
{
    hardware_interface::CallbackReturn DiffBotSystemHardware::on_init(
        const hardware_interface::HardwareInfo &info)
    {
        if (
            hardware_interface::SystemInterface::on_init(info) !=
            hardware_interface::CallbackReturn::SUCCESS)
        {
            return hardware_interface::CallbackReturn::ERROR;
        }

        // rclcpp::init(0, nullptr);
        // rclcpp::spin(comms_);

        cfg_.left_wheel_name = info_.hardware_parameters["left_wheel_name"]; 
        cfg_.right_wheel_name = info_.hardware_parameters["right_wheel_name"];  
        cfg_.loop_rate = std::stof(info_.hardware_parameters["loop_rate"]); 
        cfg_.device = info_.hardware_parameters["device"];  
        cfg_.baud_rate = std::stoi(info_.hardware_parameters["baud_rate"]);  
        cfg_.timeout_ms = std::stoi(info_.hardware_parameters["timeout_ms"]);  
        cfg_.enc_counts_per_rev = std::stoi(info_.hardware_parameters["enc_counts_per_rev"]);  
        wheel_l_.setup(cfg_.left_wheel_name, cfg_.enc_counts_per_rev);
        wheel_r_.setup(cfg_.right_wheel_name, cfg_.enc_counts_per_rev);

        for (const hardware_interface::ComponentInfo &joint : info_.joints)
        {
            // DiffBotSystem has exactly two states and one command interface on each joint
            if (joint.command_interfaces.size() != 1)
            {
                RCLCPP_FATAL(
                    rclcpp::get_logger("DiffBotSystemHardware"),
                    "Joint '%s' has %zu command interfaces found. 1 expected.", joint.name.c_str(),
                    joint.command_interfaces.size());
                return hardware_interface::CallbackReturn::ERROR;
            }

            if (joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
            {
                RCLCPP_FATAL(
                    rclcpp::get_logger("DiffBotSystemHardware"),
                    "Joint '%s' have %s command interfaces found. '%s' expected.", joint.name.c_str(),
                    joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_VELOCITY);
                return hardware_interface::CallbackReturn::ERROR;
            }

            if (joint.state_interfaces.size() != 2)
            {
                RCLCPP_FATAL(
                    rclcpp::get_logger("DiffBotSystemHardware"),
                    "Joint '%s' has %zu state interface. 2 expected.", joint.name.c_str(),
                    joint.state_interfaces.size());
                return hardware_interface::CallbackReturn::ERROR;
            }

            if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
            {
                RCLCPP_FATAL(
                    rclcpp::get_logger("DiffBotSystemHardware"),
                    "Joint '%s' have '%s' as first state interface. '%s' expected.", joint.name.c_str(),
                    joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
                return hardware_interface::CallbackReturn::ERROR;
            }

            if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY)
            {
                RCLCPP_FATAL(
                    rclcpp::get_logger("DiffBotSystemHardware"),
                    "Joint '%s' have '%s' as second state interface. '%s' expected.", joint.name.c_str(),
                    joint.state_interfaces[1].name.c_str(), hardware_interface::HW_IF_VELOCITY);
                return hardware_interface::CallbackReturn::ERROR;
            }
        }

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    std::vector<hardware_interface::StateInterface> DiffBotSystemHardware::export_state_interfaces()
    {
        std::vector<hardware_interface::StateInterface> state_interfaces;

        state_interfaces.emplace_back(hardware_interface::StateInterface(
            wheel_l_.name, hardware_interface::HW_IF_POSITION, &wheel_l_.pos));

        state_interfaces.emplace_back(hardware_interface::StateInterface(
            wheel_l_.name, hardware_interface::HW_IF_VELOCITY, &wheel_l_.vel));

        state_interfaces.emplace_back(hardware_interface::StateInterface(
            wheel_r_.name, hardware_interface::HW_IF_POSITION, &wheel_r_.pos));
        
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            wheel_r_.name, hardware_interface::HW_IF_VELOCITY, &wheel_r_.vel));

        return state_interfaces;
    }

    std::vector<hardware_interface::CommandInterface> DiffBotSystemHardware::export_command_interfaces()
    {
        std::vector<hardware_interface::CommandInterface> command_interfaces;

        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            wheel_l_.name, hardware_interface::HW_IF_VELOCITY, &wheel_l_.cmd));

        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            wheel_r_.name, hardware_interface::HW_IF_VELOCITY, &wheel_r_.cmd));

        return command_interfaces;
    }

    hardware_interface::CallbackReturn DiffBotSystemHardware::on_configure(
        const rclcpp_lifecycle::State & /*previous_state*/)
    {
        // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
        RCLCPP_INFO(rclcpp::get_logger("DiffBotSystemHardware"), "Confiugring ...please wait...");

        // comms_.connect(cfg_.device, cfg_.baud_rate, cfg_.timeout_ms);

        RCLCPP_INFO(rclcpp::get_logger("DiffBotSystemHardware"), "Successfully configured!");

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn DiffBotSystemHardware::on_cleanup(
        const rclcpp_lifecycle::State & /*previous_state*/)
    {
        // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
        RCLCPP_INFO(rclcpp::get_logger("DiffBotSystemHardware"), "Cleaning up ...please wait...");

        // comms_.disconnect();

        RCLCPP_INFO(rclcpp::get_logger("DiffBotSystemHardware"), "Successfully cleaned up!");

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn DiffBotSystemHardware::on_activate(
        const rclcpp_lifecycle::State & /*previous_state*/)
    {
        // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
        RCLCPP_INFO(rclcpp::get_logger("DiffBotSystemHardware"), "Activating ...please wait...");

        // comms_->set_pid_values(30, 20, 0, 100);

        RCLCPP_INFO(rclcpp::get_logger("DiffBotSystemHardware"), "Successfully activated!");

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn DiffBotSystemHardware::on_deactivate(
        const rclcpp_lifecycle::State & /*previous_state*/)
    {
        // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
        RCLCPP_INFO(rclcpp::get_logger("DiffBotSystemHardware"), "Deactivating ...please wait...");
        RCLCPP_INFO(rclcpp::get_logger("DiffBotSystemHardware"), "Successfully deactivated!");

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::return_type DiffBotSystemHardware::read(
        const rclcpp::Time & /*time*/, const rclcpp::Duration &period)
    {
        comms_->read_encoder_values(wheel_l_.enc, wheel_r_.enc);

        //  wheel velocities are in rings per minute, need to convert to radians per second
        wheel_l_.vel = (double)wheel_l_.enc * 2 * M_PI / 60 / 19;
        wheel_r_.vel = (double)wheel_r_.enc * 2 * M_PI / 60 / 19;

        wheel_l_.pos = wheel_l_.pos + wheel_l_.vel * period.seconds();
        wheel_r_.pos = wheel_r_.pos + wheel_r_.vel * period.seconds();

        // RCLCPP_INFO(
        //     rclcpp::get_logger("DiffBotSystemHardware"),
        //     "Got position state %.5f and velocity state %.5f (rad/s) for '%s'!", wheel_l_.pos,
        //     wheel_l_.vel, wheel_l_.name.c_str());

        // double pos_prev_l = wheel_l_.pos;  
        // double pos_prev_r = wheel_l_.pos;  

        // wheel_l_.pos = wheel_l_.calc_enc_angle();
        // wheel_r_.pos = wheel_r_.calc_enc_angle();

        // wheel_l_.vel = (wheel_l_.pos - pos_prev_l) / period.seconds();
        // wheel_r_.vel = (wheel_r_.pos - pos_prev_r) / period.seconds();

        // // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
        // for (std::size_t i = 0; i < hw_velocities_.size(); i++)
        // {
        //   // Simulate DiffBot wheels's movement as a first-order system
        //   // Update the joint status: this is a revolute joint without any limit.
        //   // Simply integrates
        //   hw_positions_[i] = hw_positions_[i] + period.seconds() * hw_velocities_[i];

        //   RCLCPP_INFO(
        //     rclcpp::get_logger("DiffBotSystemHardware"),
        //     "Got position state %.5f and velocity state %.5f for '%s'!", hw_positions_[i],
        //     hw_velocities_[i], info_.joints[i].name.c_str());
        // }
        // // END: This part here is for exemplary purposes - Please do not copy to your production code

        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type diff_control ::DiffBotSystemHardware::write(
        const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
    {
        // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
        // RCLCPP_INFO(rclcpp::get_logger("DiffBotSystemHardware"), "Writing...");
        
        double motor_l_counts_per_loop = wheel_l_.cmd * 60 / (2 * M_PI) * 19;
        double motor_r_counts_per_loop = wheel_r_.cmd * 60 / (2 * M_PI) * 19;

        comms_->set_motor_values((int)motor_l_counts_per_loop, (int)motor_r_counts_per_loop);
        // for (auto i = 0u; i < hw_commands_.size(); i++)
        // {
        //     // Simulate sending commands to the hardware
        //     RCLCPP_INFO(
        //         rclcpp::get_logger("DiffBotSystemHardware"), "Got command %.5f for '%s'!", hw_commands_[i],
        //         info_.joints[i].name.c_str());

        //     hw_velocities_[i] = hw_commands_[i];
        // }
        // RCLCPP_INFO(rclcpp::get_logger("DiffBotSystemHardware"), "Joints successfully written!");
        // END: This part here is for exemplary purposes - Please do not copy to your production code

        return hardware_interface::return_type::OK;
    }

} // namespace diff_control

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
    diff_control::DiffBotSystemHardware, hardware_interface::SystemInterface)
