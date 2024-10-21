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
 * File:        user_defined_trajectory.cpp
 * Author:      Mattia Gramuglia
 * Date:        April 12, 2024
 * For info:    Andrea L'Afflitto 
 *              a.lafflitto@vt.edu
 * 
 * Description: Class that contains the common members that each type of user-defined trajectory class will inherit
 * 
 * GitHub:    https://github.com/andrealaffly/ACSL-flightstack.git
 **********************************************************************************************************************/

/**
 * @file user_defined_trajectory.cpp
 * @brief Class that contains the common members that each type of user-defined trajectory class will inherit
 */

#include "user_defined_trajectory.hpp"
#include "multi_threaded_node.hpp"

// Constructor
/**
 * @see UserDefinedTrajectory
 * 
 * @param node 
 */
UserDefinedTrajectory::UserDefinedTrajectory(MultiThreadedNode& node) :
	node_(node),
	timestamp_initial_(node.getInitialTimestamp()), // Initialize timestamp_initial_ with the value from MultiThreadedNode
	user_defined_position_(Eigen::Vector3d::Zero()),
	user_defined_velocity_(Eigen::Vector3d::Zero()),
	user_defined_acceleration_(Eigen::Vector3d::Zero()),
	user_defined_yaw_(0),
	user_defined_yaw_dot_(0),
	user_defined_yaw_dot_dot_(0)
{}

// Getter function that returns a reference to the current MultiThreadedNode object
MultiThreadedNode& UserDefinedTrajectory::getNode() const {
  return node_;
}

// Getter function for the initial timestamp generated when the MultiThreadedNode is started
const std::atomic<uint64_t>& UserDefinedTrajectory::getInitialTimestamp() const {
  return timestamp_initial_;
}

// Getter function for the user_defined_position_
Eigen::Vector3d& UserDefinedTrajectory::getUserDefinedPosition() {
  return user_defined_position_;
}

// Getter function for the user_defined_velocity_
Eigen::Vector3d& UserDefinedTrajectory::getUserDefinedVelocity() {
  return user_defined_velocity_;
}

// Getter function for the user_defined_acceleration_
Eigen::Vector3d& UserDefinedTrajectory::getUserDefinedAcceleration() {
  return user_defined_acceleration_;
}

// Getter function for the user_defined_yaw_
double& UserDefinedTrajectory::getUserDefinedYaw() {
  return user_defined_yaw_;
}

// Getter function for the user_defined_yaw_dot_
double& UserDefinedTrajectory::getUserDefinedYawDot() {
  return user_defined_yaw_dot_;
}

// Getter function for the user_defined_yaw_dot_dot_
double& UserDefinedTrajectory::getUserDefinedYawDotDot() {
  return user_defined_yaw_dot_dot_;
}

// Setter function for the user_defined_position_
void UserDefinedTrajectory::setUserDefinedPosition(const Eigen::Vector3d& user_defined_position) {
  user_defined_position_ = user_defined_position;
}

// Setter function for the user_defined_velocity_
void UserDefinedTrajectory::setUserDefinedVelocity(const Eigen::Vector3d& user_defined_velocity) {
  user_defined_velocity_ = user_defined_velocity;
}

// Setter function for the user_defined_acceleration_
void UserDefinedTrajectory::setUserDefinedAcceleration(const Eigen::Vector3d& user_defined_acceleration) {
  user_defined_acceleration_ = user_defined_acceleration;
}

// Setter function for the user_defined_yaw_
void UserDefinedTrajectory::setUserDefinedYaw(const double& user_defined_yaw) {
  user_defined_yaw_ = user_defined_yaw;
}

// Setter function for the user_defined_yaw_dot_
void UserDefinedTrajectory::setUserDefinedYawDot(const double& user_defined_yaw_dot) {
  user_defined_yaw_dot_ = user_defined_yaw_dot;
}

// Setter function for the user_defined_yaw_dot_dot_
void UserDefinedTrajectory::setUserDefinedYawDotDot(const double& user_defined_yaw_dot_dot) {
  user_defined_yaw_dot_dot_ = user_defined_yaw_dot_dot;
}