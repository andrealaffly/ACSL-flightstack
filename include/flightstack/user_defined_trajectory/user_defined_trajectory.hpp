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
 * File:        user_defined_trajectory.hpp
 * Author:      Mattia Gramuglia
 * Date:        April 12, 2024
 * For info:    Andrea L'Afflitto 
 *              a.lafflitto@vt.edu
 * 
 * Description: Class that contains the common members that each type of user-defined trajectory class will inherit
 * 
 * GitHub:    https://github.com/andrealaffly/ACSL-flightstack.git
 **********************************************************************************************************************/

#ifndef USER_DEFINED_TRAJECTORY_HPP
#define USER_DEFINED_TRAJECTORY_HPP

#include <cmath>

#include <Eigen/Dense>

// Forward declaration of MultiThreadedNode class
class MultiThreadedNode;

class UserDefinedTrajectory
{
public:

  // Constructor
  UserDefinedTrajectory(MultiThreadedNode& node);

  // Getter functions
  MultiThreadedNode& getNode() const;
  const std::atomic<uint64_t>& getInitialTimestamp() const;
  Eigen::Vector3d& getUserDefinedPosition();
  Eigen::Vector3d& getUserDefinedVelocity();
  Eigen::Vector3d& getUserDefinedAcceleration();
  double& getUserDefinedYaw();
  double& getUserDefinedYawDot();
  double& getUserDefinedYawDotDot();

  // Setter functions
  void setUserDefinedPosition(const Eigen::Vector3d& position);
  void setUserDefinedVelocity(const Eigen::Vector3d& user_defined_velocity);
  void setUserDefinedAcceleration(const Eigen::Vector3d& user_defined_acceleration);
  void setUserDefinedYaw(const double& user_defined_yaw);
  void setUserDefinedYawDot(const double& user_defined_yaw_dot);
  void setUserDefinedYawDotDot(const double& user_defined_yaw_dot_dot);

private:

  MultiThreadedNode& node_;

  const std::atomic<uint64_t>& timestamp_initial_;  // Reference to the initial timestamp
  // const std::atomic<double>& time_current_;  // Reference to the current time

  Eigen::Vector3d user_defined_position_;
  Eigen::Vector3d user_defined_velocity_;
  Eigen::Vector3d user_defined_acceleration_;
  double user_defined_yaw_;
  double user_defined_yaw_dot_;
  double user_defined_yaw_dot_dot_;

};


#endif // USER_DEFINED_TRAJECTORY_HPP