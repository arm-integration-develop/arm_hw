/*******************************************************************************
 * BSD 3-Clause License
 *
 * Copyright (c) 2021, Qiayuan Liao
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * * Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *******************************************************************************/

//
// Created by qiayuan on 12/21/20.
//

#pragma once

#include <vector>
#include <string>
#include <memory>
#include <unordered_map>

// ROS
#include <ros/ros.h>
#include <urdf/model.h>
#include <XmlRpcValue.h>

// ROS control
#include <hardware_interface/robot_hw.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/actuator_state_interface.h>
#include <hardware_interface/actuator_command_interface.h>
#include <transmission_interface/transmission_interface_loader.h>
#include <joint_limits_interface/joint_limits_interface.h>
#include <arm_common/interface/hardware_interface/robot_state_interface.h>
#include <arm_common/interface/hardware_interface/actuator_extra_interface.h>
#include <arm_common/tools/can_motor.h>
#include <arm_msgs/ActuatorState.h>
#include <joint_limits_interface/joint_limits_urdf.h>
#include <joint_limits_interface/joint_limits_rosparam.h>
#include "realtime_tools/realtime_publisher.h"

// #include

namespace arm_hw
{
class ArmRobotHW : public hardware_interface::RobotHW
{
public:
  ArmRobotHW() = default;
  /** \brief Get necessary params from param server. Init hardware_interface.
   *
   * Get params from param server and check whether these params are set. Load urdf of robot. Set up transmission and
   * joint limit. Get configuration of can bus and create data pointer which point to data received from Can bus.
   *
   * @param root_nh Root node-handle of a ROS node.
   * @param robot_hw_nh Node-handle for robot hardware.
   * @return True when init successful, False when failed.
   */
  bool init(ros::NodeHandle& root_nh, ros::NodeHandle& robot_hw_nh) override;
  /** \brief Comunicate with hardware. Get datas, status of robot.
   *
   * Call @ref rm_hw::CanBus::read(). Check whether temperature of actuator is too high and whether actuator is offline.
   * Propagate actuator state to joint state for the stored transmission. Set all cmd to zero to avoid crazy soft limit
   * oscillation when not controller loaded(all controllers update after read()).
   *
   * @param time Current time
   * @param period Current time - last time
   */
  void read(const ros::Time& time, const ros::Duration& period) override;

  /** \brief Comunicate with hardware. Publish command to robot.
   *
   * Propagate joint state to actuator state for the stored
   * transmission. Limit cmd_effort into suitable value. Call @ref rm_hw::CanBus::write(). Publish actuator current state.
   *
   * @param time Current time
   * @param period Current time - last time
   */
  void write(const ros::Time& time, const ros::Duration& period) override;

  void setCanBusThreadPriority(int thread_priority);

  void registerROSInterface(XmlRpc::XmlRpcValue& act_datas);

  void testMotor();

  void closeMotor();

private:
  /** \brief Load urdf of robot from param server.
   *
   * Load urdf of robot from param server.
   *
   * @param root_nh Root node-handle of a ROS node
   * @return True if successful.
   */
  bool loadUrdf(ros::NodeHandle& root_nh);
  bool setupTransmission(ros::NodeHandle& root_nh);
  bool setupJointLimit(ros::NodeHandle& root_nh);
  void publishActuatorState(const ros::Time& time);

  ros::ServiceServer service_server_;

  bool is_actuator_specified_ = false;
  int thread_priority_;
  // ROS Interface
  hardware_interface::RobotStateInterface robot_state_interface_;
  hardware_interface::ActuatorStateInterface act_state_interface_;
  hardware_interface::EffortActuatorInterface effort_act_interface_;
  hardware_interface::JointStateInterface jnt_state_interface_;
  hardware_interface::ActuatorExtraInterface act_extra_interface_;
  std::vector<hardware_interface::JointHandle> effort_joint_handles_{};
  std::unique_ptr<transmission_interface::TransmissionInterfaceLoader> transmission_loader_{};
  transmission_interface::RobotTransmissions robot_transmissions_;
  transmission_interface::ActuatorToJointStateInterface* act_to_jnt_state_{};
  transmission_interface::JointToActuatorEffortInterface* jnt_to_act_effort_{};
  joint_limits_interface::EffortJointSaturationInterface effort_jnt_saturation_interface_;
  joint_limits_interface::EffortJointSoftLimitsInterface effort_jnt_soft_limits_interface_;

  // URDF model of the robot
  std::string urdf_string_;                  // for transmission
  std::shared_ptr<urdf::Model> urdf_model_;  // for limit

  // Actuator
  can_interface::CanMotor can_motor_{};

  ros::Time last_publish_time_;
  std::shared_ptr<realtime_tools::RealtimePublisher<arm_msgs::ActuatorState>> actuator_state_pub_;
};

}  // namespace arm_hw
