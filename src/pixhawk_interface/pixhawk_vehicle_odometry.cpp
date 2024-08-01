/***********************************************************************************************************************
 * Copyright (c) 2024 Mattia Gramuglia, Giri M. Kumar, Andrea L'Afflitto. All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without modification, are permitted provided that the
 * following conditions are met:
 * 
 * 1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following
 *    disclaimer.
 * 
 * 2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the
 *    following disclaimer in the documentation and/or other materials provided with the distribution.
 * 
 * 3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote
 *    products derived from this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS “AS IS” AND ANY EXPRESS OR IMPLIED WARRANTIES,
 * INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 **********************************************************************************************************************/

/***********************************************************************************************************************
 * File:        pixhawk_vehicle_odometry.cpp
 * Author:      Mattia Gramuglia
 * Date:        April 9, 2024
 * For info:    Andrea L'Afflitto 
 *              a.lafflitto@vt.edu
 * 
 * Description: Callback that is executed everytime a VehicleOdometry message is made available by Pixhawk.
 * 
 * GitHub:    https://github.com/andrealaffly/ACSL_flightstack_X8.git
 **********************************************************************************************************************/

#include "multi_threaded_node.hpp"
#include "vehicle_state.hpp"

using namespace std::chrono_literals;

/*
  Callback function that is executed everytime a VehicleOdometry message is made available by Pixhawk
*/
void MultiThreadedNode::subscriber_pixhawk_odometry_callback(const px4_msgs::msg::VehicleOdometry::UniquePtr msg)
{
  /* // Extract current thread
  auto thread_string = "THREAD " + string_thread_id();
  std::cout << "\n\n";
  std::cout << "PIXHAWK ODOMETRY " << thread_string << std::endl;
  */

  // If it's the "offset" message, store the offset state
  if (time_current_ > global_params_.OFFSET_ODOMETRY_TIME_SECONDS && 
        time_current_ < global_params_.TAKEOFF_START_TIME_SECONDS && 
        vehicle_state_->getIsOffsetOdometryMsg())
  {
    // std::cout << "OFFSET ODOMETRY MESSAGE" << std::endl;
    vehicle_state_->setPositionOffset(Eigen::Vector3d(msg->position[0],
                                                      msg->position[1], 
                                                      msg->position[2]));
    vehicle_state_->setQuaternionOffset(Eigen::Quaterniond(msg->q[0], 
                                                           msg->q[1], 
                                                           msg->q[2], 
                                                           msg->q[3]));
    vehicle_state_->setEulerAnglesRPYOffset(Eigen::Quaterniond(msg->q[0], 
                                                               msg->q[1], 
                                                               msg->q[2], 
                                                               msg->q[3]));
    vehicle_state_->setYawQuaternionInverseOffset();

    // Set is_offset_odometry_msg_ to false
    vehicle_state_->setIsOffsetOdometryMsg(false);
  }


  // Assign data from msg to vehicle_state_ using setter methods
  vehicle_state_->setTimestamp(msg->timestamp);
  vehicle_state_->setTimeOdometryInSeconds();
  vehicle_state_->setPosition(Eigen::Vector3d(msg->position[0] - vehicle_state_->getPositionOffset()(0),
                                              msg->position[1] - vehicle_state_->getPositionOffset()(1), 
                                              msg->position[2] - vehicle_state_->getPositionOffset()(2)));

  Eigen::Quaterniond raw_quaternion = Eigen::Quaterniond(msg->q[0], 
                                                         msg->q[1], 
                                                         msg->q[2], 
                                                         msg->q[3]);

  Eigen::Quaterniond yaw_unbiased_quaternion = vehicle_state_->getYawQuaternionInverseOffset() * raw_quaternion;

  // vehicle_state_->setQuaternion(Eigen::Quaterniond(msg->q[0], 
  //                                                  msg->q[1], 
  //                                                  msg->q[2], 
  //                                                  msg->q[3]));
  vehicle_state_->setQuaternion(yaw_unbiased_quaternion);

  vehicle_state_->setVelocity(Eigen::Vector3d(msg->velocity[0], 
                                              msg->velocity[1], 
                                              msg->velocity[2]));
                                              
  vehicle_state_->setAngularVelocity(Eigen::Vector3d(msg->angular_velocity[0], 
                                                     msg->angular_velocity[1], 
                                                     msg->angular_velocity[2]));
  // vehicle_state_->setEulerAnglesRPY(Eigen::Quaterniond(msg->q[0], 
  //                                                      msg->q[1], 
  //                                                      msg->q[2], 
  //                                                      msg->q[3]));
  vehicle_state_->setEulerAnglesRPY(yaw_unbiased_quaternion);


  /* 
  // Print to console for debugging 
  std::cout << "pose_frame: ";
  switch (msg->pose_frame) {
    case px4_msgs::msg::VehicleOdometry::POSE_FRAME_UNKNOWN:
      std::cout << "POSE_FRAME_UNKNOWN" << std::endl;
      break;
    case px4_msgs::msg::VehicleOdometry::POSE_FRAME_NED:
      std::cout << "POSE_FRAME_NED" << std::endl;
      break;
    case px4_msgs::msg::VehicleOdometry::POSE_FRAME_FRD:
      std::cout << "POSE_FRAME_FRD" << std::endl;
      break;
    default:
      std::cout << "Invalid value" << std::endl;
  }

  std::cout << "velocity_frame: ";
  switch (msg->velocity_frame) {
    case px4_msgs::msg::VehicleOdometry::VELOCITY_FRAME_UNKNOWN:
      std::cout << "VELOCITY_FRAME_UNKNOWN" << std::endl;
      break;
    case px4_msgs::msg::VehicleOdometry::VELOCITY_FRAME_NED:
      std::cout << "VELOCITY_FRAME_NED" << std::endl;
      break;
    case px4_msgs::msg::VehicleOdometry::VELOCITY_FRAME_FRD:
      std::cout << "VELOCITY_FRAME_FRD" << std::endl;
      break;
    case px4_msgs::msg::VehicleOdometry::VELOCITY_FRAME_BODY_FRD:
      std::cout << "VELOCITY_FRAME_BODY_FRD" << std::endl;
      break;
    default:
      std::cout << "Invalid value" << std::endl;
  }
  */

  // Print to console for debugging the members assigned to vehicle_state_
  /*
  std::cout << "\n\n";
  std::cout << thread_string << std::endl;
  std::cout << "VehicleState CLASS"   << std::endl;
  std::cout << "=================="   << std::endl;
  std::cout << "timestamp: " << vehicle_state_->getTimestamp() << std::endl;
  std::cout << "initial timestamp: " << vehicle_state_->getInitialTimestamp() << std::endl;
  std::cout << "time odometry: " << vehicle_state_->getTimeOdometryInSeconds() << std::endl;
  std::cout << "x: " << vehicle_state_->getPosition()(0)  << std::endl;
  std::cout << "y: " << vehicle_state_->getPosition()(1)  << std::endl;
  std::cout << "z: " << vehicle_state_->getPosition()(2)  << std::endl;
  std::cout << "q0: " << vehicle_state_->getQuaternion().w() << std::endl;
  std::cout << "q1: " << vehicle_state_->getQuaternion().x() << std::endl;
  std::cout << "q2: " << vehicle_state_->getQuaternion().y() << std::endl;
  std::cout << "q3: " << vehicle_state_->getQuaternion().z() << std::endl;
  std::cout << "vx: " << vehicle_state_->getVelocity()(0) << std::endl;
  std::cout << "vy: " << vehicle_state_->getVelocity()(1) << std::endl;
  std::cout << "vz: " << vehicle_state_->getVelocity()(2) << std::endl;						
  std::cout << "rollspeed: " << vehicle_state_->getAngularVelocity()(0) << std::endl;
  std::cout << "pitchspeed: " << vehicle_state_->getAngularVelocity()(1) << std::endl;
  std::cout << "yawspeed: " << vehicle_state_->getAngularVelocity()(2) << std::endl;

  std::cout << "DEBUUUUUUUUUUUUUG"   << std::endl;
  std::cout << "roll: " << vehicle_state_->getEulerAnglesRPY()(0) << std::endl;
  std::cout << "pitch: " << vehicle_state_->getEulerAnglesRPY()(1) << std::endl;
  std::cout << "yaw: " << vehicle_state_->getEulerAnglesRPY()(2) << std::endl;
  std::cout << "euler_angles_rpy_offset: " << vehicle_state_->getEulerAnglesRPYOffset() << std::endl;
  */



}

