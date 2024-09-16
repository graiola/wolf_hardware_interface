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

#ifndef WOLF_ROBOT_HW_INTERFACE_H
#define WOLF_ROBOT_HW_INTERFACE_H

#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/handle.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp> // For IMU message type
#include <urdf/model.h>
#include <srdfdom/model.h>
#include <wolf_controller_utils/srdf_parser.h>
#include <deque>
#include <vector>
#include <string>

namespace hardware_interface
{

class WolfRobotHwInterface
{
public:

    const std::string CLASS_NAME = "WolfRobotHwInterface";

    WolfRobotHwInterface();
    virtual ~WolfRobotHwInterface();

    std::vector<hardware_interface::StateInterface> exportStateInterfaces();
    std::vector<hardware_interface::CommandInterface> exportCommandInterfaces();

    void parseSRDF(const std::string& robot_namespace);
    void initializeJointsInterface(const std::vector<std::string>& joint_names);
    void initializeImuInterface(const std::string& imu_link_name);
    void initializeGroundTruthInterface(const std::string& base_link_name);
    void initializeContactSensorsInterface(const std::vector<std::string>& contact_names);

    std::string getRobotName() {return robot_name_;}
    unsigned int getNdof() {return n_dof_;}

    std::vector<std::string> loadJointNamesFromSRDF();
    std::string loadImuLinkNameFromSRDF();
    std::string loadBaseLinkNameFromSRDF();
    std::vector<std::string> loadContactNamesFromSRDF();

protected:

    std::string robot_name_;

    // Joint and sensor data
    std::vector<double> joint_position_;
    std::vector<double> joint_velocity_;
    std::vector<double> joint_effort_;
    std::vector<double> joint_effort_command_;

    unsigned int n_dof_;
    sensor_msgs::msg::Imu imu_data_;
    std::vector<std::string> joint_names_;
    std::vector<std::string> contact_sensor_names_;
    // IMU Data
    std::vector<double> imu_orientation_;
    std::vector<double> imu_ang_vel_;
    std::vector<double> imu_lin_acc_;

    // Force-Torque Sensor Data
    std::vector<std::vector<double>> force_;
    std::vector<std::vector<double>> torque_;

    std::vector<double> base_orientation_;
    std::vector<double> base_ang_vel_;
    std::vector<double> base_ang_vel_prev_;
    std::vector<double> base_ang_acc_;
    std::vector<double> base_lin_acc_;
    std::vector<double> base_lin_pos_;
    std::vector<double> base_lin_vel_;
    std::vector<double> base_lin_vel_prev_;

    std::vector<std::string> leg_name_;
    std::vector<std::vector<double>> normal_;
    std::deque<bool> contact_;

private:

    wolf_controller_utils::SRDFParser srdf_parser_;
};

}  // namespace hardware_interface

#endif  // WOLF_ROBOT_HW_INTERFACE_H
