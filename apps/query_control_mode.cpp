// Copyright 2025 Enactic, Inc.
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

#include <atomic>
#include <chrono>
#include <csignal>
#include <cstddef>
#include <iostream>
#include <openarm/can/socket/openarm.hpp>
#include <openarm/damiao_motor/dm_motor_constants.hpp>
#include <thread>
#include <algorithm>

int main(int argc, char *argv[]) {

    if(argc != 2 && argc != 3) {
        std::cerr << "Usage: " << argv[0] << " " << " <can interface> <optional fd mode: true/false. default: false>" << std::endl;
        return 1;
    }

    std::string can_interface { argv[1] };

    bool can_fd { false };
    if(argc == 3) {
        std::string can_fd_string { argv[2] };

        std::transform(can_fd_string.begin(), can_fd_string.end(), can_fd_string.begin(),
        [](unsigned char c){ return std::tolower(c); });

        can_fd = (can_fd_string == "true");
    }

    try {

        std::cout << "=== Query Control Mode ===" << std::endl;
        std::cout << "Query robot motor control mode" << std::endl;

        using openarm::damiao_motor::MotorControlMode;
        using openarm::damiao_motor::GetMotorControlModeString;

        // Print avaiable modes
        std::cout << "Available modes:\n";
        for(size_t i {static_cast<size_t>(MotorControlMode::minMode) + 1} ; i < static_cast<size_t>(MotorControlMode::maxMode); ++i) 
        {
            std::cout << std::to_string(i) << ":" << GetMotorControlModeString(static_cast<MotorControlMode>(i)) << '\n';
        }

        // Initialize OpenArm with CAN interface and enable CAN-FD
        std::cout << "Initializing OpenArm CAN..." << std::endl;
        std::cout << "Interface: " << can_interface << std::endl;
        std::cout << "Flexible Data Mode: " << ((can_fd) ? "true" : "false") << std::endl;
        openarm::can::socket::OpenArm openarm(can_interface, can_fd);  // Use CAN-FD on can0 interface

        // Initialize arm motors
        std::vector<openarm::damiao_motor::MotorType> motor_types = {
            openarm::damiao_motor::MotorType::DM8009,
            openarm::damiao_motor::MotorType::DM8009,
            openarm::damiao_motor::MotorType::DM4340,
            openarm::damiao_motor::MotorType::DM4340,
            openarm::damiao_motor::MotorType::DM4310,
            openarm::damiao_motor::MotorType::DM4310,
            openarm::damiao_motor::MotorType::DM4310
        };
        std::vector<uint32_t> send_can_ids = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07};
        std::vector<uint32_t> recv_can_ids = {0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17};

        openarm.init_arm_motors(motor_types, send_can_ids, recv_can_ids);

        // Initialize gripper
        std::cout << "Initializing gripper..." << std::endl;
        openarm.init_gripper_motor(openarm::damiao_motor::MotorType::DM4310, 0x08, 0x18);

        // Query motor param
        using openarm::damiao_motor::RID;

        RID param_to_query { static_cast<int>(RID::CTRL_MODE) };

        openarm.set_callback_mode_all(openarm::damiao_motor::CallbackMode::PARAM);

        openarm.query_param_all(static_cast<int>(param_to_query));
        openarm.recv_all(2000);

        // Arm
        for (const auto& motor : openarm.get_arm().get_motors()) {
            uint8_t control_mode_code { static_cast<uint8_t>(std::round(motor.get_param(static_cast<int>(param_to_query)))) };
            MotorControlMode control_mode { static_cast<MotorControlMode>(control_mode_code) };
            std::cout << "Arm Motor: " << motor.get_send_can_id() << " Motor Control Mode: "
                      << std::to_string(control_mode_code) << " - " << GetMotorControlModeString(control_mode)
                      << std::endl;
        }

        // Gripper
        for (const auto& motor : openarm.get_gripper().get_motors()) {
            uint8_t control_mode_code { static_cast<uint8_t>(std::round(motor.get_param(static_cast<int>(param_to_query)))) };
            MotorControlMode control_mode { static_cast<MotorControlMode>(control_mode_code) };
            std::cout << "Gripper Motor: " << motor.get_send_can_id() << " Motor Control Mode: "
                      << std::to_string(control_mode_code) << " - " << GetMotorControlModeString(control_mode)
                      << std::endl;
        }

        // Turn off motor
        openarm.set_callback_mode_all(openarm::damiao_motor::CallbackMode::STATE);

        openarm.disable_all();
        openarm.recv_all(1000);

    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return -1;
    }

    return 0;
}
