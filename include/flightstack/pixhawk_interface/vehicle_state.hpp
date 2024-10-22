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
 * File:        vehicle_state.hpp
 * Author:      Mattia Gramuglia
 * Date:        April 9, 2024
 * For info:    Andrea L'Afflitto 
 *              a.lafflitto@vt.edu
 * 
 * Description: Class storing the state of the vehicle coming from the flight controller (Pixhawk)
 * 
 * GitHub:    https://github.com/andrealaffly/ACSL-flightstack.git
 **********************************************************************************************************************/

/**
 * @file vehicle_state.hpp
 * @brief Class storing the state of the vehicle coming from the flight controller (Pixhawk)
 */

#ifndef VEHICLE_STATE_HPP
#define VEHICLE_STATE_HPP

#include <string>
#include <cmath>
#include <atomic>

#include <Eigen/Dense>

// Forward declaration of MultiThreadedNode class
/**
 * @class MultiThreadedNode
 */
class MultiThreadedNode;

/**
 * @class VehicleState
 * @brief Orientation of the vehicle at a given instant of time
 */
class VehicleState 
{
public:

  // Constructor
  /**
   * @brief Construct a new Vehicle State object
   * 
   * @param node 
   */
  VehicleState(MultiThreadedNode& node);

  // Getter functions
  uint64_t getTimestamp() const;
  Eigen::Vector3d& getPosition();
  Eigen::Quaterniond& getQuaternion();
  Eigen::Vector3d& getVelocity();
  Eigen::Vector3d& getAngularVelocity();
  Eigen::Vector3d& getEulerAnglesRPY();
  const std::atomic<uint64_t>& getInitialTimestamp() const;
  double& getTimeOdometryInSeconds();
  bool getIsOffsetOdometryMsg() const;
  uint64_t getTimestampOffset() const;
  Eigen::Vector3d& getPositionOffset();
  Eigen::Quaterniond& getQuaternionOffset();
  Eigen::Vector3d& getEulerAnglesRPYOffset();
  Eigen::Quaterniond& getYawQuaternionInverseOffset();

  // Setter functions
  void setTimestamp(uint64_t timestamp);
  void setPosition(const Eigen::Vector3d& position);
  void setQuaternion(const Eigen::Quaterniond& q);
  void setVelocity(const Eigen::Vector3d& velocity);
  void setAngularVelocity(const Eigen::Vector3d& angular_velocity);
  void setEulerAnglesRPY(const Eigen::Quaterniond& q);
  void setTimeOdometryInSeconds();
  void setIsOffsetOdometryMsg(bool value);
  void setTimestampOffset(uint64_t timestamp_offset);
  void setPositionOffset(const Eigen::Vector3d& position_offset);
  void setQuaternionOffset(const Eigen::Quaterniond& q_offset);
  void setEulerAnglesRPYOffset(const Eigen::Quaterniond& q_offset);
  void setYawQuaternionInverseOffset();

  // Transform quaternion to euler angles
  Eigen::Vector3d quaternionToEulerAnglesRPY(const Eigen::Quaterniond& q);

private:

  MultiThreadedNode& node_;
  const std::atomic<uint64_t>& timestamp_initial_;  // Reference to the initial timestamp

  uint64_t timestamp_;                 // [us]
  Eigen::Vector3d position_;           // [m] By default expressed in NED earth-fixed frame
  Eigen::Quaterniond q_;               // [-] By default expressed in NED earth-fixed frame
  Eigen::Vector3d velocity_;           // [m/s] By default expressed in NED earth-fixed frame 
  Eigen::Vector3d angular_velocity_;   // [rad/s] By default expressed in FRD body-fixed frame

  Eigen::Vector3d euler_angles_rpy_;   // [rad]

  double time_odometry_;  // [s]

  // Offset variables used to position Pixhawk in (0, 0, 0) with 0 yaw angle 
  // after the "offset" odometry message is received
  bool is_offset_odometry_msg_;
  uint64_t timestamp_offset_;
  Eigen::Vector3d position_offset_;
  Eigen::Quaterniond q_offset_;
  Eigen::Vector3d euler_angles_rpy_offset_;
  Eigen::Quaterniond yaw_quaternion_inverse_offset_;

};


#endif // VEHICLE_STATE_HPP
