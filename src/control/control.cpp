///@cond
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
///@endcond
/***********************************************************************************************************************
 * File:        control.cpp
 * Author:      Mattia Gramuglia
 * Date:        April 19, 2024
 * For info:    Andrea L'Afflitto 
 *              a.lafflitto@vt.edu
 * 
 * Description: General class that contains common members used by most of the control algorithms.
 *              Control algorithm classes inherit from this class.
 * 
 * GitHub:    https://github.com/andrealaffly/ACSL-flightstack.git
 **********************************************************************************************************************/

/**
 * @file control.cpp
 * @brief This file contains the general class containing the common memvers used by most of the control algorithms.
 * Control algorithm classes inherit from this class.
 */

#include "multi_threaded_node.hpp"
#include "control.hpp"


// Constructor
/**
 * @class Control
 */
Control::ControlInternalMembers::ControlInternalMembers() : 
  mu_translational_raw(Eigen::Vector3d::Zero()),
  mu_translational(Eigen::Vector3d::Zero()),
  U_control_inputs(Eigen::Vector4d::Zero()),
  U2_U3_U4(U_control_inputs.segment<3>(1)),
  angular_position_desired(Eigen::Vector3d::Zero()),
  angular_position_desired_dot(Eigen::Vector3d::Zero()),
  angular_position_desired_dot_dot(Eigen::Vector3d::Zero()),
  roll_desired(angular_position_desired[0]),
  pitch_desired(angular_position_desired[1]),
  roll_desired_dot(angular_position_desired_dot[0]),
  pitch_desired_dot(angular_position_desired_dot[1]),
  roll_desired_dot_dot(angular_position_desired_dot_dot[0]),
  pitch_desired_dot_dot(angular_position_desired_dot_dot[1]),
  internal_state_roll_des_filter(Eigen::Vector2d::Zero()),
  internal_state_pitch_des_filter(Eigen::Vector2d::Zero()),
  internal_state_roll_dot_des_filter(Eigen::Vector2d::Zero()),
  internal_state_pitch_dot_des_filter(Eigen::Vector2d::Zero()),
  jacobian_matrix_inverse(Eigen::Matrix3d::Zero()),
  rotation_matrix_321_global_to_local(Eigen::Matrix3d::Zero()),
  euler_angles_rpy_dot(Eigen::Vector3d::Zero()),
  angular_error(Eigen::Vector3d::Zero()),
  translational_position_error(Eigen::Vector3d::Zero()),
  thrust_vector(Eigen::Matrix<double, 8, 1>::Zero()),
  thrust_vector_normalized(Eigen::Matrix<double, 8, 1>::Zero()),
  thrust_vector_quadcopter(Eigen::Vector4d::Zero()),
  thrust_vector_quadcopter_normalized(Eigen::Vector4d::Zero())
{}

//Constructor
Control::ControlReferences::ControlReferences(MultiThreadedNode& node) :
  user_defined_position(node.getUserDefinedTrajectory()->getUserDefinedPosition()),
  user_defined_velocity(node.getUserDefinedTrajectory()->getUserDefinedVelocity()),
  user_defined_acceleration(node.getUserDefinedTrajectory()->getUserDefinedAcceleration()),
  user_defined_yaw(node.getUserDefinedTrajectory()->getUserDefinedYaw()),
  user_defined_yaw_dot(node.getUserDefinedTrajectory()->getUserDefinedYawDot()),
  user_defined_yaw_dot_dot(node.getUserDefinedTrajectory()->getUserDefinedYawDotDot()),
  position(node.getVehicleState()->getPosition()),
  velocity(node.getVehicleState()->getVelocity()),
  euler_angles_rpy(node.getVehicleState()->getEulerAnglesRPY()),
  angular_velocity(node.getVehicleState()->getAngularVelocity())
{}

// Constructor
Control::Control(MultiThreadedNode& node) :
  cr(node),
  node_(node)
{
  this->readJSONdifferentiatorFile();
}

/*
  Function to read the differentiator gains coming from the .json file and assign it to the vehicle info 
  in vehicle_info.hpp
*/


// This function reads the differentiator gains from a specified JSON file and updates the vehicle_info structure accordingly.
void Control::readJSONdifferentiatorFile()
{
  // Define the JSON file where the differentiator gains are located
  const std::string jsonFile = "./src/flightstack/params/control/differentiator_gains.json";

  std::ifstream file(jsonFile);
  if (!file.is_open()) {
    throw std::runtime_error("Could not open file: " + jsonFile);
  }

	nlohmann::json j;
	file >> j;
	
  this->vehicle_info.A_filter_roll_des = extractMatrixFromJSON<double, 2, 2>(j["A_filter_roll_des"]);
  this->vehicle_info.B_filter_roll_des = extractMatrixFromJSON<double, 2, 1>(j["B_filter_roll_des"]);
  this->vehicle_info.C_filter_roll_des = extractMatrixFromJSON<double, 1, 2>(j["C_filter_roll_des"]);
  this->vehicle_info.D_filter_roll_des = j["D_filter_roll_des"];

  this->vehicle_info.A_filter_pitch_des = extractMatrixFromJSON<double, 2, 2>(j["A_filter_pitch_des"]);
  this->vehicle_info.B_filter_pitch_des = extractMatrixFromJSON<double, 2, 1>(j["B_filter_pitch_des"]);
  this->vehicle_info.C_filter_pitch_des = extractMatrixFromJSON<double, 1, 2>(j["C_filter_pitch_des"]);
  this->vehicle_info.D_filter_pitch_des = j["D_filter_pitch_des"];

  this->vehicle_info.A_filter_roll_dot_des = extractMatrixFromJSON<double, 2, 2>(j["A_filter_roll_dot_des"]);
  this->vehicle_info.B_filter_roll_dot_des = extractMatrixFromJSON<double, 2, 1>(j["B_filter_roll_dot_des"]);
  this->vehicle_info.C_filter_roll_dot_des = extractMatrixFromJSON<double, 1, 2>(j["C_filter_roll_dot_des"]);
  this->vehicle_info.D_filter_roll_dot_des = j["D_filter_roll_dot_des"];

  this->vehicle_info.A_filter_pitch_dot_des = extractMatrixFromJSON<double, 2, 2>(j["A_filter_pitch_dot_des"]);
  this->vehicle_info.B_filter_pitch_dot_des = extractMatrixFromJSON<double, 2, 1>(j["B_filter_pitch_dot_des"]);
  this->vehicle_info.C_filter_pitch_dot_des = extractMatrixFromJSON<double, 1, 2>(j["C_filter_pitch_dot_des"]);
  this->vehicle_info.D_filter_pitch_dot_des = j["D_filter_pitch_dot_des"];
}

// Getter function that returns a reference to the current MultiThreadedNode object
MultiThreadedNode& Control::getNode() const {
  return node_;
}

// Getter function that returns a reference to the current VehicleInfo object
const VehicleInfo& Control::getVehicleInfo() const 
{
  return vehicle_info;
}

// Getter function that returns a reference to the current ControlInternalMembers object
const Control::ControlInternalMembers& Control::getControlInternalMembers() const
{
  return cim;
}

// Getter function 
const std::chrono::duration<double, std::micro>& Control::getAlgorithmExecutionTimeMicroseconds() const
{
  return algorithm_execution_time_microseconds_;
}

// Setter function
void Control::setAlgorithmExecutionTimeMicroseconds(std::chrono::duration<double, std::micro> duration)
{
  algorithm_execution_time_microseconds_ = duration;
}

// Function that computes the Jacobian matrix given roll and pitch angles
Eigen::Matrix3d Control::jacobianMatrix(const double& roll, const double& pitch)
{
  Eigen::Matrix3d jacobian_matrix;

  jacobian_matrix(0, 0) = 1;
  jacobian_matrix(0, 1) = 0;
  jacobian_matrix(0, 2) = -std::sin(pitch);

  jacobian_matrix(1, 0) = 0;
  jacobian_matrix(1, 1) = std::cos(roll);
  jacobian_matrix(1, 2) = std::sin(roll) * std::cos(pitch);

  jacobian_matrix(2, 0) = 0;
  jacobian_matrix(2, 1) = -std::sin(roll);
  jacobian_matrix(2, 2) = std::cos(roll) * std::cos(pitch);

  return jacobian_matrix;
}

// Function that computes the time derivative of the Jacobian matrix given
// roll and pitch angles and their time derivatives
Eigen::Matrix3d Control::jacobianMatrixDot(const double& roll,
                                           const double& pitch,
                                           const double& roll_dot, 
                                           const double& pitch_dot)
{
  Eigen::Matrix3d jacobian_matrix_dot;

  jacobian_matrix_dot(0, 0) = 0;
  jacobian_matrix_dot(0, 1) = 0;
  jacobian_matrix_dot(0, 2) = -std::cos(pitch) * pitch_dot;

  jacobian_matrix_dot(1, 0) = 0;
  jacobian_matrix_dot(1, 1) = -std::sin(roll) * roll_dot;
  jacobian_matrix_dot(1, 2) = std::cos(roll) * std::cos(pitch) * roll_dot
                              - std::sin(roll) * std::sin(pitch) * pitch_dot;

  jacobian_matrix_dot(2, 0) = 0;
  jacobian_matrix_dot(2, 1) = -std::cos(roll) * roll_dot;
  jacobian_matrix_dot(2, 2) = -std::sin(roll) * std::cos(pitch) * roll_dot
                              - std::cos(roll) * std::sin(pitch) * pitch_dot;

  return jacobian_matrix_dot;
}

// Function that returns a 3x3 Jacobian matrix inverse given roll and pitch angles.
Eigen::Matrix3d Control::jacobianMatrixInverse(const double& roll, const double& pitch)
{
  // Ensure pitch is not near +/- pi/2 to avoid division by zero
  if (std::abs(std::cos(pitch)) < 1e-6) {
    throw std::invalid_argument("Pitch value results in division by zero.");
  }

  Eigen::Matrix3d jacobian_inverse;

  jacobian_inverse(0, 0) = 1;
  jacobian_inverse(0, 1) = (std::sin(roll) * std::sin(pitch)) / std::cos(pitch);
  jacobian_inverse(0, 2) = (std::cos(roll) * std::sin(pitch)) / std::cos(pitch);

  jacobian_inverse(1, 0) = 0;
  jacobian_inverse(1, 1) = std::cos(roll);
  jacobian_inverse(1, 2) = -std::sin(roll);

  jacobian_inverse(2, 0) = 0;
  jacobian_inverse(2, 1) = std::sin(roll) / std::cos(pitch);
  jacobian_inverse(2, 2) = std::cos(roll) / std::cos(pitch);

  return jacobian_inverse;
}

/*
  Function that computes the 321 rotation matrix that transforms a vector from global to local coordinates.

  rotation_matrix_321_local_to_global = R3 * R2 * R1; // hence
  rotation_matrix_321_global_to_local = R1_transpose * R2_transpose * R3_transpose;
*/
Eigen::Matrix3d Control::rotationMatrix321GlobalToLocal(const double& roll, const double& pitch, const double& yaw)
{
  Eigen::Matrix3d rotation_matrix_321_global_to_local;
  Eigen::Matrix3d R1_transpose;
  Eigen::Matrix3d R2_transpose;
  Eigen::Matrix3d R3_transpose;

  R1_transpose = (Eigen::Matrix3d() << 
    1,  0,               0,
    0,  std::cos(roll),  std::sin(roll),
    0, -std::sin(roll),  std::cos(roll)
  ).finished();

  R2_transpose = (Eigen::Matrix3d() << 
    std::cos(pitch),  0,  -std::sin(pitch),
    0,                1,   0,
    std::sin(pitch),  0,   std::cos(pitch)
  ).finished();

  R3_transpose = (Eigen::Matrix3d() << 
     std::cos(yaw),  std::sin(yaw),  0,
    -std::sin(yaw),  std::cos(yaw),  0,
     0,              0,              1
  ).finished();

  rotation_matrix_321_global_to_local = R1_transpose * R2_transpose * R3_transpose;

  return rotation_matrix_321_global_to_local;
}

/*
  Function that takes as input U1,U2,U3,U4. It computes the thrust that each motor needs to generate in Newton
  and converts it in the normalized value comprised between 0 and 1 according to the experimental 
  thrust/motor curve obtained experimentally using the thrust stand.
  For QUADCOPTER 
*/


void Control::computeNormalizedThrustQuadcopterMode(ControlInternalMembers& cim, VehicleInfo& vehicle_info)
{
  /*
    Compute the thrust in Newton that each motor needs to produce
  */
  cim.thrust_vector_quadcopter = vehicle_info.mixer_matrix_quadcopter * cim.U_control_inputs;

  /* 
  Apply upper saturation at UPPER_MOTOR_THRUST_SATURATION_LIMIT_IN_NEWTON and
  lower saturation at LOWER_MOTOR_THRUST_SATURATION_LIMIT_IN_NEWTON
  */
  Eigen::Vector4d thrust_vector_quadcopter_saturated = (cim.thrust_vector_quadcopter.
    cwiseMin(UPPER_MOTOR_THRUST_SATURATION_LIMIT_IN_NEWTON).
    cwiseMax(LOWER_MOTOR_THRUST_SATURATION_LIMIT_IN_NEWTON));

  cim.thrust_vector_quadcopter_normalized[0] = 
    evaluatePolynomial(vehicle_info.thrust_polynomial_coefficients_quadcopter,
                       thrust_vector_quadcopter_saturated[0]);

  cim.thrust_vector_quadcopter_normalized[1] = 
    evaluatePolynomial(vehicle_info.thrust_polynomial_coefficients_quadcopter,
                       thrust_vector_quadcopter_saturated[1]);

  cim.thrust_vector_quadcopter_normalized[2] = 
    evaluatePolynomial(vehicle_info.thrust_polynomial_coefficients_quadcopter,
                       thrust_vector_quadcopter_saturated[2]);

  cim.thrust_vector_quadcopter_normalized[3] = 
    evaluatePolynomial(vehicle_info.thrust_polynomial_coefficients_quadcopter,
                       thrust_vector_quadcopter_saturated[3]);

}

/*
  Function that takes as input U1,U2,U3,U4. It computes the thrust that each motor needs to generate in Newton
  and converts it in the normalized value comprised between 0 and 1 according to the experimental 
  thrust/motor curve obtained experimentally using the thrust stand.
  For X8-COPTER 
*/
void Control::computeNormalizedThrust(ControlInternalMembers& cim, VehicleInfo& vehicle_info)
{
  /*
    Compute the thrust in Newton that each motor needs to produce
  */
  cim.thrust_vector = vehicle_info.mixer_matrix * cim.U_control_inputs;

  /* 
  Apply upper saturation at UPPER_MOTOR_THRUST_SATURATION_LIMIT_IN_NEWTON and
  lower saturation at LOWER_MOTOR_THRUST_SATURATION_LIMIT_IN_NEWTON
  */
  Eigen::Matrix<double, 8, 1> thrust_vector_saturated = (cim.thrust_vector.
    cwiseMin(UPPER_MOTOR_THRUST_SATURATION_LIMIT_IN_NEWTON).
    cwiseMax(LOWER_MOTOR_THRUST_SATURATION_LIMIT_IN_NEWTON));

  cim.thrust_vector_normalized[0] = 
    evaluatePolynomial(vehicle_info.thrust_polynomial_coefficients_quadcopter,
                       thrust_vector_saturated[0]);

  cim.thrust_vector_normalized[1] = 
    evaluatePolynomial(vehicle_info.thrust_polynomial_coefficients_quadcopter,
                       thrust_vector_saturated[1]);

  cim.thrust_vector_normalized[2] = 
    evaluatePolynomial(vehicle_info.thrust_polynomial_coefficients_quadcopter,
                       thrust_vector_saturated[2]);

  cim.thrust_vector_normalized[3] = 
    evaluatePolynomial(vehicle_info.thrust_polynomial_coefficients_quadcopter,
                       thrust_vector_saturated[3]);

  cim.thrust_vector_normalized[4] = 
    evaluatePolynomial(vehicle_info.thrust_polynomial_coefficients_quadcopter,
                       thrust_vector_saturated[4]);

  cim.thrust_vector_normalized[5] = 
    evaluatePolynomial(vehicle_info.thrust_polynomial_coefficients_quadcopter,
                       thrust_vector_saturated[5]);

  cim.thrust_vector_normalized[6] = 
    evaluatePolynomial(vehicle_info.thrust_polynomial_coefficients_quadcopter,
                       thrust_vector_saturated[6]);

  cim.thrust_vector_normalized[7] = 
    evaluatePolynomial(vehicle_info.thrust_polynomial_coefficients_quadcopter,
                       thrust_vector_saturated[7]);

}

/*
  Function that computes:
    the Total Thrust U1,
    the desired roll angle RollDes,
    the desired pitch angle PitchDes.
  U1 will be fed to mixer matrix, while RollDes and PitchDes will be fed to the inner loop.
*/
void Control::compute_U1_RollDes_PitchDes(ControlInternalMembers& cim)
{
  // Convert the mu_translational from global to local coordinates
  cim.mu_translational = cim.rotation_matrix_321_global_to_local * cim.mu_translational_raw; 

  cim.U_control_inputs[0] = std::sqrt(
      std::pow(cim.mu_translational[0], 2)
    + std::pow(cim.mu_translational[1], 2)
    + std::pow(cim.mu_translational[2], 2)
  );

  double temporaryvar_roll_desired_1 = cim.mu_translational[1] / cim.U_control_inputs[0];
  cim.roll_desired= std::atan2(
    temporaryvar_roll_desired_1,
    std::sqrt(1 - std::pow(temporaryvar_roll_desired_1, 2))
  );

  cim.pitch_desired = std::atan2(-cim.mu_translational[0], -cim.mu_translational[2]);
}

/*
  Function that computes the translational position error
*/
void Control::computeTranslationalPositionError(ControlInternalMembers& cim,
                                                const Eigen::Vector3d& position,
                                                const Eigen::Vector3d& desired_position)
{
  cim.translational_position_error = position - desired_position;
}

/*
  Function to compute the angular error for roll, pitch, and yaw angles.
  This function calculates the difference between the current euler angles
  (roll, pitch, yaw) and the desired angles, taking into account the
  wrapping of the yaw angle to ensure it remains within the range [-pi, pi]
  to handle the discontinuity at +/- pi.
*/
void Control::computeAngularError(ControlInternalMembers& cim,
                                  const Eigen::Vector3d& euler_angles_rpy,
                                  const Eigen::Vector3d& angular_position_desired)
{
  cim.angular_error[0] = euler_angles_rpy[0] - angular_position_desired[0];
  cim.angular_error[1] = euler_angles_rpy[1] - angular_position_desired[1];
  cim.angular_error[2] = makeYawAngularErrorContinuous(euler_angles_rpy[2], angular_position_desired[2]);
}

/*
  Function to compute various rotational parameters.
  This function sets the desired angle, angular rate and angular acceleration for yaw,
  computes the inverse of the Jacobian matrix, the derivative of the Euler angles,
  and the rotation matrix to go from global to local coordinates.
*/
void Control::computeRotationalParameters(ControlInternalMembers& cim, ControlReferences& cr)
{
  // Set angular_position_desired (angular_position_desired[0] and angular_position_desired[1] are
  // automatically updated as they are referenced by roll_desired and pitch_desired)
  cim.angular_position_desired[2] = cr.user_defined_yaw;

  // Set angular_position_desired_dot (angular_position_desired_dot[0] and angular_position_desired_dot[1] are
  // automatically updated as they are referenced by roll_desired_dot and pitch_desired_dot)
  cim.angular_position_desired_dot[2] = cr.user_defined_yaw_dot; 

  // Set angular_position_desired_dot_dot (angular_position_desired_dot_dot[0] and 
  // angular_position_desired_dot_dot[1] are automatically updated as they are referenced by roll_desired_dot_dot
  // and pitch_desired_dot_dot)
  cim.angular_position_desired_dot_dot[2] = cr.user_defined_yaw_dot_dot;

  cim.jacobian_matrix_inverse = this->jacobianMatrixInverse(cr.euler_angles_rpy[0], cr.euler_angles_rpy[1]);

  cim.euler_angles_rpy_dot = cim.jacobian_matrix_inverse * cr.angular_velocity;

  cim.rotation_matrix_321_global_to_local = this->rotationMatrix321GlobalToLocal(cr.euler_angles_rpy[0],
                                                                                 cr.euler_angles_rpy[1],
                                                                                 cr.euler_angles_rpy[2]);
}

/*
  Function that takes as input an angle alpha and wraps it in the interval [-pi, pi]
*/
double Control::wrapAngleToMinusPiAndPi(double alpha)
{
  return alpha - 2 * M_PI * std::floor((alpha + M_PI) / (2 * M_PI));
}

/*
  Function that makes the yaw angular error continuos considering the discontinuity/wrapping that occurs
  at +/- pi for both the yaw and the user-defined yaw
*/
double Control::makeYawAngularErrorContinuous(double yaw, double user_defined_yaw)
{
  double continuos_error;
  
  user_defined_yaw = wrapAngleToMinusPiAndPi(user_defined_yaw);
  
  double raw_error = yaw - user_defined_yaw;  
  
  if ( std::abs(raw_error) >= M_PI && std::abs(raw_error) < 2*M_PI && raw_error < 0 )
  {
    continuos_error = std::fmod(raw_error, M_PI) + M_PI;
  }
  else if ( std::abs(raw_error) >= M_PI && std::abs(raw_error) < 2*M_PI && raw_error > 0 )
  {
    continuos_error = std::fmod(raw_error, M_PI) - M_PI;
  }
  else
  {
    continuos_error = std::fmod(raw_error, M_PI);   
  }
  
  return continuos_error;
}



