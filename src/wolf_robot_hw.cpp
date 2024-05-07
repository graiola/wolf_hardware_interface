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

using namespace hardware_interface;
using namespace wolf_controller_utils;

WolfRobotHwInterface::WolfRobotHwInterface()
{
}

WolfRobotHwInterface::~WolfRobotHwInterface()
{
}

void WolfRobotHwInterface::parseSRDF(const std::string& robot_namespace)
{
  srdf_parser_.parseSRDF(robot_namespace);
}

void WolfRobotHwInterface::initializeJointsInterface(const std::vector<std::string>& joint_names)
{
  // Resize vectors to our DOF
  n_dof_ = static_cast<unsigned int>(joint_names.size());
  joint_names_.resize(n_dof_);
  joint_types_.resize(n_dof_);
  joint_effort_limits_.resize(n_dof_);
  joint_position_.resize(n_dof_);
  joint_velocity_.resize(n_dof_);
  joint_effort_.resize(n_dof_);
  joint_effort_command_.resize(n_dof_);

  for (unsigned int j=0; j < n_dof_; j++)
  {

    ROS_DEBUG_STREAM_NAMED(CLASS_NAME,"Loading joint: "<< joint_names[j]);

    joint_names_[j]          = joint_names[j];
    joint_position_[j]       = 1.0;
    joint_velocity_[j]       = 0.0;
    joint_effort_[j]         = 0.0;  // N/m for continuous joints
    joint_effort_command_[j] = 0.0;

    // Create joint state interface for all joints
    joint_state_interface_.registerHandle(hardware_interface::JointStateHandle(
                                            joint_names_[j], &joint_position_[j], &joint_velocity_[j], &joint_effort_[j]));

    joint_effort_interface_.registerHandle(JointHandle(joint_state_interface_.getHandle(joint_names_[j]), &joint_effort_command_[j]));
  }
}

void WolfRobotHwInterface::initializeImuInterface(const std::string& imu_link_name)
{
  imu_orientation_.resize(4);
  imu_ang_vel_.resize(3);
  imu_lin_acc_.resize(3);

  imu_data_.name = "imu";
  imu_data_.frame_id = imu_link_name;
  imu_data_.orientation = &imu_orientation_[0];
  imu_data_.angular_velocity = &imu_ang_vel_[0];
  imu_data_.linear_acceleration = &imu_lin_acc_[0];
  imu_sensor_interface_.registerHandle(hardware_interface::ImuSensorHandle(imu_data_));
}

void WolfRobotHwInterface::initializeGroundTruthInterface(const std::string& base_link_name)
{
  base_orientation_.resize(4);
  base_ang_vel_.resize(3);
  base_ang_vel_prev_.resize(3);
  base_ang_acc_.resize(3);
  base_lin_acc_.resize(3);
  base_lin_pos_.resize(3);
  base_lin_vel_.resize(3);
  base_lin_vel_prev_.resize(3);

  gt_data_.name = "ground_truth";
  gt_data_.frame_id = base_link_name;
  gt_data_.orientation = &base_orientation_[0];
  gt_data_.angular_velocity = &base_ang_vel_[0];
  gt_data_.angular_acceleration = &base_ang_acc_[0];
  gt_data_.linear_acceleration = &base_lin_acc_[0];
  gt_data_.linear_position = &base_lin_pos_[0];
  gt_data_.linear_velocity = &base_lin_vel_[0];
  ground_truth_interface_.registerHandle(hardware_interface::GroundTruthHandle(gt_data_));
}

void WolfRobotHwInterface::initializeContactSensorsInterface(const std::vector<std::string>& contact_names)
{
  // Note the _contact_sensor here, this is appended because
  // of the fact that we are using the SRDF file to get on which link the contact sensors
  // are mounted
  for(unsigned int i=0;i<contact_names.size();i++)
    contact_sensor_names_.push_back(contact_names[i]+"_contact_sensor");

  // Create the handle for each contact sensor,
  contact_.resize(contact_sensor_names_.size());
  force_.resize(contact_sensor_names_.size());
  torque_.resize(contact_sensor_names_.size());
  normal_.resize(contact_sensor_names_.size());
  for (unsigned int i=0;i<contact_sensor_names_.size();i++)
  {
    contact_[i] = false;
    force_[i].resize(3,0);
    torque_[i].resize(3,0);
    normal_[i].resize(3,0);
    contact_sensor_interface_.registerHandle(hardware_interface::ContactSwitchSensorHandle(contact_names[i], &contact_[i], &force_[i][0], &torque_[i][0], &normal_[i][0]));
  }
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
