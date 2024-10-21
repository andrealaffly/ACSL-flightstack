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
 * File:        vehicle_state.cpp
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
 * @file vehicle_state.cpp
 * @brief Class storing the state of the vehicle coming from the flight controller (Pixhawk)
 */
#include "vehicle_state.hpp"
#include "multi_threaded_node.hpp"

// Constructor
/**
 * @brief Construct a new Vehicle State:: Vehicle State object
 * 
 * @param node 
 */
VehicleState::VehicleState(MultiThreadedNode& node) :
  node_(node),
  timestamp_initial_(node.getInitialTimestamp()), // Initialize timestamp_initial_ with the value from MultiThreadedNode
  timestamp_(node.getInitialTimestamp()),
  position_(Eigen::Vector3d::Zero()),
  q_(Eigen::Quaterniond::Identity()),
  velocity_(Eigen::Vector3d::Zero()),
  angular_velocity_(Eigen::Vector3d::Zero()),
  euler_angles_rpy_(Eigen::Vector3d::Zero()),
  time_odometry_(0.0),
  is_offset_odometry_msg_(true),
  timestamp_offset_(0),
  position_offset_(Eigen::Vector3d::Zero()),
  q_offset_(Eigen::Quaterniond::Identity()),
  euler_angles_rpy_offset_(Eigen::Vector3d::Zero()),
  yaw_quaternion_inverse_offset_(Eigen::Quaterniond::Identity())
{}

// Getter function for the timestamp
uint64_t VehicleState::getTimestamp() const {
  return timestamp_;
}

// Getter function for the position
Eigen::Vector3d& VehicleState::getPosition() {
  return position_;
}

// Getter function for the quaternion
Eigen::Quaterniond& VehicleState::getQuaternion() {
  return q_;
}

// Getter function for the velocity
Eigen::Vector3d& VehicleState::getVelocity() {
  return velocity_;
}

// Getter function for the angular velocity
Eigen::Vector3d& VehicleState::getAngularVelocity() {
  return angular_velocity_;
}

// Getter function for the euler angles
Eigen::Vector3d& VehicleState::getEulerAnglesRPY() {
  return euler_angles_rpy_;
}

// Getter function for the initial timestamp generated when the MultiThreadedNode is started
const std::atomic<uint64_t>& VehicleState::getInitialTimestamp() const {
  return timestamp_initial_;
}

// Getter function for the time in seconds that has passed from the initial timestamp to the latest received message
double& VehicleState::getTimeOdometryInSeconds()
{
  return time_odometry_;
}

// Getter function for the is_offset_odometry_msg_
bool VehicleState::getIsOffsetOdometryMsg() const {
  return is_offset_odometry_msg_;
}

// Getter function for the timestamp_offset_
uint64_t VehicleState::getTimestampOffset() const {
  return timestamp_offset_;
}

// Getter function for the position_offset_
Eigen::Vector3d& VehicleState::getPositionOffset() {
  return position_offset_;
}

// Getter function for the q_offset_
Eigen::Quaterniond& VehicleState::getQuaternionOffset() {
  return q_offset_;
}

// Getter function for the euler_angles_rpy_offset_
Eigen::Vector3d& VehicleState::getEulerAnglesRPYOffset() {
  return euler_angles_rpy_offset_;
}

// Getter function for the yaw_quaternion_inverse_offset_
Eigen::Quaterniond& VehicleState::getYawQuaternionInverseOffset() { 
  return yaw_quaternion_inverse_offset_;
}


// Setter function for the timestamp
void VehicleState::setTimestamp(uint64_t timestamp) {
  timestamp_ = timestamp;
}

// Setter function for the position
void VehicleState::setPosition(const Eigen::Vector3d& position) {
  position_ = position;
}

// Setter function for the quaternion
void VehicleState::setQuaternion(const Eigen::Quaterniond& q) {
  q_ = q;
}

// Setter function for the velocity
void VehicleState::setVelocity(const Eigen::Vector3d& velocity) {
  velocity_ = velocity;
}

// Setter function for the angular velocity
void VehicleState::setAngularVelocity(const Eigen::Vector3d& angular_velocity) {
  angular_velocity_ = angular_velocity;
}

// Setter function for the euler angles
void VehicleState::setEulerAnglesRPY(const Eigen::Quaterniond& q) {
  Eigen::Vector3d euler_angles_rpy = quaternionToEulerAnglesRPY(q);
  euler_angles_rpy_ = euler_angles_rpy;
}

// Setter function for the time in seconds that has passed from the initial timestamp to the latest received message
void VehicleState::setTimeOdometryInSeconds()
{
  time_odometry_ = static_cast<double>(timestamp_ - timestamp_initial_) / 1e6;
}

// Setter function for the is_offset_odometry_msg_
void VehicleState::setIsOffsetOdometryMsg(bool value) {
  is_offset_odometry_msg_ = value;
}

// Setter function for the timestamp_offset_
void VehicleState::setTimestampOffset(uint64_t timestamp_offset) {
  timestamp_offset_ = timestamp_offset;
}

// Setter function for the position_offset_
void VehicleState::setPositionOffset(const Eigen::Vector3d& position_offset) {
  position_offset_ = position_offset;
}

// Setter function for the quaternion
void VehicleState::setQuaternionOffset(const Eigen::Quaterniond& q_offset) {
  q_offset_ = q_offset;
}

// Setter function for the euler_angles_rpy_offset_
void VehicleState::setEulerAnglesRPYOffset(const Eigen::Quaterniond& q_offset) {
  Eigen::Vector3d euler_angles_rpy_offset = quaternionToEulerAnglesRPY(q_offset);
  euler_angles_rpy_offset_ = euler_angles_rpy_offset;
}

/*
  Setter function for the yaw_quaternion_inverse_offset_
*/
void VehicleState::setYawQuaternionInverseOffset()
{
  Eigen::Quaterniond yaw_quaternion = Eigen::Quaterniond(cos(euler_angles_rpy_offset_(2)/2),
                                                         0,
                                                         0,
                                                         sin(euler_angles_rpy_offset_(2)/2));

  yaw_quaternion_inverse_offset_ = yaw_quaternion.inverse();

}

/*
  Computing Euler Angles from the unit quaternion following a 3-2-1 rotation sequence
  The Euler Angles are ordered as: roll, pitch, yaw
  For reference:
    - https://github.com/PX4/PX4-Matrix/blob/master/matrix/Euler.hpp
    - https://github.com/PX4/PX4-Matrix/blob/master/matrix/Dcm.hpp
*/
Eigen::Vector3d VehicleState::quaternionToEulerAnglesRPY(const Eigen::Quaterniond& q) {
  Eigen::Vector3d euler_angles_rpy;

  // Compute elements of Direction Cosine Matrix (DCM) from quaternion
  double a = q.w();
  double b = q.x();
  double c = q.y();
  double d = q.z();
  double aa = a * a;
  double ab = a * b;
  double ac = a * c;
  double ad = a * d;
  double bb = b * b;
  double bc = b * c;
  double bd = b * d;
  double cc = c * c;
  double cd = c * d;
  double dd = d * d;

  // Compute DCM elements
  double dcm00 = aa + bb - cc - dd;
  // double dcm01 = 2 * (bc - ad); // not needed by the algorithm
  double dcm02 = 2 * (ac + bd);
  double dcm10 = 2 * (bc + ad);
  // double dcm11 = aa - bb + cc - dd; // not needed by the algorithm
  double dcm12 = 2 * (cd - ab);
  double dcm20 = 2 * (bd - ac);
  double dcm21 = 2 * (ab + cd);
  double dcm22 = aa - bb - cc + dd;

  // Compute Euler angles from DCM
  euler_angles_rpy.y() = asin(-dcm20);

  if (fabs(euler_angles_rpy.y() - M_PI / 2) < 1.0e-3) {
    euler_angles_rpy.x() = 0;
    euler_angles_rpy.z() = atan2(dcm12, dcm02);
  } else if (fabs(euler_angles_rpy.y() + M_PI / 2) < 1.0e-3) {
    euler_angles_rpy.x() = 0;
    euler_angles_rpy.z() = atan2(-dcm12, -dcm02);
  } else {
    euler_angles_rpy.x() = atan2(dcm21, dcm22);
    euler_angles_rpy.z() = atan2(dcm10, dcm00);
  }

  return euler_angles_rpy;
}
