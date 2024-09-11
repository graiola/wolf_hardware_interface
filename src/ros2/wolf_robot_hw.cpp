/*
 * Copyright (C) 2022 Gennaro Raiola
 * Author: Gennaro Raiola
 * email:  gennaro.raiola@gmail.com
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>
*/

#include <wolf_hardware_interface/wolf_robot_hw.h>
#include <urdf_parser/urdf_parser.h>
#include <srdfdom/srdf_writer.h>
#include <rclcpp/rclcpp.hpp>

using namespace hardware_interface;
using namespace wolf_controller_utils;

WolfRobotHwInterface::WolfRobotHwInterface()
{
}

WolfRobotHwInterface::~WolfRobotHwInterface()
{
}

hardware_interface::return_type WolfRobotHwInterface::configure(const hardware_interface::HardwareInfo & info)
{
    // Perform necessary initializations here
    return hardware_interface::return_type::OK;
}

std::vector<hardware_interface::StateInterface> WolfRobotHwInterface::export_state_interfaces()
{
    std::vector<hardware_interface::StateInterface> state_interfaces;

    // Joint state interfaces
    for (unsigned int j = 0; j < n_dof_; j++) {
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            joint_names_[j], hardware_interface::HW_IF_POSITION, &joint_position_[j]));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            joint_names_[j], hardware_interface::HW_IF_VELOCITY, &joint_velocity_[j]));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            joint_names_[j], hardware_interface::HW_IF_EFFORT, &joint_effort_[j]));
    }

    // IMU state interfaces
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        imu_data_.header.frame_id, "orientation.x", &imu_orientation_[0]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        imu_data_.header.frame_id, "orientation.y", &imu_orientation_[1]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        imu_data_.header.frame_id, "orientation.z", &imu_orientation_[2]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        imu_data_.header.frame_id, "orientation.w", &imu_orientation_[3]));

    state_interfaces.emplace_back(hardware_interface::StateInterface(
        imu_data_.header.frame_id, "angular_velocity.x", &imu_ang_vel_[0]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        imu_data_.header.frame_id, "angular_velocity.y", &imu_ang_vel_[1]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        imu_data_.header.frame_id, "angular_velocity.z", &imu_ang_vel_[2]));

    state_interfaces.emplace_back(hardware_interface::StateInterface(
        imu_data_.header.frame_id, "linear_acceleration.x", &imu_lin_acc_[0]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        imu_data_.header.frame_id, "linear_acceleration.y", &imu_lin_acc_[1]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        imu_data_.header.frame_id, "linear_acceleration.z", &imu_lin_acc_[2]));

    // Add other sensor state interfaces as needed

    return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> WolfRobotHwInterface::export_command_interfaces()
{
    std::vector<hardware_interface::CommandInterface> command_interfaces;

    for (unsigned int j = 0; j < n_dof_; j++) {
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            joint_names_[j], hardware_interface::HW_IF_EFFORT, &joint_effort_command_[j]));
    }

    return command_interfaces;
}

hardware_interface::return_type WolfRobotHwInterface::start()
{
    // Hardware startup logic if necessary
    return hardware_interface::return_type::OK;
}

hardware_interface::return_type WolfRobotHwInterface::stop()
{
    // Hardware shutdown logic if necessary
    return hardware_interface::return_type::OK;
}

hardware_interface::return_type WolfRobotHwInterface::read()
{
    // Update joint states (position, velocity, effort) from hardware or simulation
    // Update IMU states
    return hardware_interface::return_type::OK;
}

hardware_interface::return_type WolfRobotHwInterface::write()
{
    // Send commands (effort) to the actuators
    return hardware_interface::return_type::OK;
}

void WolfRobotHwInterface::parseSRDF(const std::string& robot_namespace)
{
    srdf_parser_.parseSRDF(robot_namespace);
}

void WolfRobotHwInterface::initializeJointsInterface(const std::vector<std::string>& joint_names)
{
    n_dof_ = static_cast<unsigned int>(joint_names.size());
    joint_names_ = joint_names;
    joint_position_.resize(n_dof_);
    joint_velocity_.resize(n_dof_);
    joint_effort_.resize(n_dof_);
    joint_effort_command_.resize(n_dof_);

    // Initialize joint states to default values
    for (unsigned int j = 0; j < n_dof_; j++) {
        joint_position_[j] = 0.0;
        joint_velocity_[j] = 0.0;
        joint_effort_[j] = 0.0;
        joint_effort_command_[j] = 0.0;
    }
}

void WolfRobotHwInterface::initializeImuInterface(const std::string& imu_link_name)
{
    imu_orientation_.resize(4, 0.0);
    imu_ang_vel_.resize(3, 0.0);
    imu_lin_acc_.resize(3, 0.0);

    imu_data_.header.frame_id = imu_link_name;
}

void WolfRobotHwInterface::initializeGroundTruthInterface(const std::string& base_link_name)
{
    base_orientation_.resize(4, 0.0);
    base_ang_vel_.resize(3, 0.0);
    base_ang_acc_.resize(3, 0.0);
    base_lin_acc_.resize(3, 0.0);
    base_lin_pos_.resize(3, 0.0);
    base_lin_vel_.resize(3, 0.0);
}

void WolfRobotHwInterface::initializeContactSensorsInterface(const std::vector<std::string>& contact_names)
{
    // Create handle for each contact sensor
    contact_.resize(contact_names.size(), false);
    force_.resize(contact_names.size(), std::vector<double>(3, 0.0));
    torque_.resize(contact_names.size(), std::vector<double>(3, 0.0));
    normal_.resize(contact_names.size(), std::vector<double>(3, 0.0));
}

std::vector<std::string> WolfRobotHwInterface::loadJointNamesFromSRDF()
{
    return srdf_parser_.getJointNames();
}

std::string WolfRobotHwInterface::loadImuLinkNameFromSRDF()
{
    return srdf_parser_.getImuLinkName();
}

std::string WolfRobotHwInterface::loadBaseLinkNameFromSRDF()
{
    return srdf_parser_.getBaseLinkName();
}

std::vector<std::string> WolfRobotHwInterface::loadContactNamesFromSRDF()
{
    return srdf_parser_.getContactNames();
}
