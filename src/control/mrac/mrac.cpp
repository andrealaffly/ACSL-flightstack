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
 * File:        mrac.cpp
 * Author:      Mattia Gramuglia
 * Date:        June 13, 2024
 * For info:    Andrea L'Afflitto 
 *              a.lafflitto@vt.edu
 * 
 * Description: Implementation of the MRAC controller.
 * 
 * GitHub:    https://github.com/andrealaffly/ACSL-flightstack.git
 **********************************************************************************************************************/
#include "config.hpp"

#if SELECTED_CONTROLLER == __MRAC__

#include "multi_threaded_node.hpp"
#include "mrac.hpp"
#include "logging_mrac.hpp"
#include "json_parser.hpp"

using namespace nlohmann;

// Constructor
MRAC::MRAC(MultiThreadedNode& node) : 
  Control(node)
{
  // Initialize controller gains
  json j = readJsonFile("./src/flightstack/params/control/mrac/gains_mrac.json");
  this->loadGains(j);
  this->initializeControllerParameters(this->vehicle_info, this->gains_);

  log_data_ = std::make_shared<LogData_MRAC>(node, *this);

  // Initialize logging
	log_data_->logInitializeLogging();
}

// Constructor initializes the full_state and sets the Eigen::Map references
MRAC::StateController::StateController() : 
  full_state(110, 0.0), // full_state vector
  state_roll_des_filter(full_state.data()), // Maps the elements [0:1]
  state_pitch_des_filter(full_state.data() + 2), // Maps the elements [2:3]
  state_roll_dot_des_filter(full_state.data() + 4), // Maps the elements [4:5]
  state_pitch_dot_des_filter(full_state.data() + 6), // Maps the elements [6:7]
  integral_translational_position_error(full_state.data() + 8), // Maps the elements [8:10]
  integral_angular_error(full_state.data() + 11), // Maps the elements [11:13]
  x_ref_translational(full_state.data() + 14), // Maps the elements [14:19]
  integral_translational_position_error_ref(full_state.data() + 20), // Maps the elements [20:22]
  K_hat_x_translational(full_state.data() + 23), // Maps the elements [23:40]
  K_hat_r_translational(full_state.data() + 41), // Maps the elements [41:49]
  Theta_hat_translational(full_state.data() + 50), // Maps the elements [50:67]
  omega_ref_rotational(full_state.data() + 68), // Maps the elements [68:70]
  K_hat_x_rotational(full_state.data() + 71), // Maps the elements [71:79]
  K_hat_r_rotational(full_state.data() + 80), // Maps the elements [80:88]
  Theta_hat_rotational(full_state.data() + 89), // Maps the elements [89:106]
  integral_angular_velocity_error_ref(full_state.data() + 107) // Maps the elements [107:109]
{}

// Constructor
MRAC::ControllerSpecificInternalMembers::ControllerSpecificInternalMembers() : 
  translational_position_error_ref(Eigen::Matrix<double, 3, 1>::Zero()),
  r_cmd_translational(Eigen::Matrix<double, 3, 1>::Zero()),
  x_ref_dot_translational(Eigen::Matrix<double, 6, 1>::Zero()),
  mu_PID_baseline_translational(Eigen::Matrix<double, 3, 1>::Zero()),
  augmented_regressor_vector_translational(Eigen::Matrix<double, 6, 1>::Zero()),
  K_hat_x_dot_translational(Eigen::Matrix<double, 6, 3>::Zero()),
  K_hat_r_dot_translational(Eigen::Matrix<double, 3, 3>::Zero()),
  Theta_hat_dot_translational(Eigen::Matrix<double, 6, 3>::Zero()),
  mu_adaptive_translational(Eigen::Matrix<double, 3, 1>::Zero()),
  dead_zone_value_translational(0.0),
  outer_loop_P(Eigen::Vector3d::Zero()),
  outer_loop_I(Eigen::Vector3d::Zero()),
  outer_loop_D(Eigen::Vector3d::Zero()),
  outer_loop_dynamic_inversion(Eigen::Vector3d::Zero()),
  jacobian_matrix(Eigen::Matrix3d::Zero()),
  jacobian_matrix_dot(Eigen::Matrix3d::Zero()),
  omega_cmd_rotational(Eigen::Vector3d::Zero()),
  omega_cmd_dot_rotational(Eigen::Vector3d::Zero()),
  angular_velocity_error_ref(Eigen::Vector3d::Zero()),
  omega_ref_dot_rotational(Eigen::Vector3d::Zero()),
  r_cmd_rotational(Eigen::Vector3d::Zero()),
  tau_PID_baseline_rotational(Eigen::Vector3d::Zero()),
  augmented_regressor_vector_rotational(Eigen::Matrix<double, 6, 1>::Zero()),
  K_hat_x_dot_rotational(Eigen::Matrix<double, 3, 3>::Zero()),
  K_hat_r_dot_rotational(Eigen::Matrix<double, 3, 3>::Zero()),
  Theta_hat_dot_rotational(Eigen::Matrix<double, 6, 3>::Zero()),
  tau_adaptive_rotational(Eigen::Vector3d::Zero()),
  dead_zone_value_rotational(0.0),
  inner_loop_P(Eigen::Vector3d::Zero()),
  inner_loop_I(Eigen::Vector3d::Zero()),
  inner_loop_D(Eigen::Vector3d::Zero()),
  inner_loop_dynamic_inversion(Eigen::Vector3d::Zero())
{}

MRAC::StateController& MRAC::getStateController()
{
  return state_;
}

const double& MRAC::getTimeStepRK4() const
{
  return time_step_rk4_;
}

const GainsMRAC& MRAC::getGains() const
{
  return gains_;
}

// Getter function that returns a reference to the current getControllerSpecificInternalMembers object
const MRAC::ControllerSpecificInternalMembers& MRAC::getControllerSpecificInternalMembers() const
{
  return csim_;
}

/*
  Getter function that returns a pointer to the log_data_ instance
*/
std::shared_ptr<LogData_MRAC> MRAC::getLogData() const
{
  return log_data_;
}

const std::string& MRAC::getControllerName()
{
  return controller_name_;
}

/*
  Function to read the tuning gains coming from the .json file and assign it to the
  gains_ struct instantiated in the MRAC class
*/

void MRAC::loadGains(json j){
  
  gains_.KP_refmod_translational = extractMatrixFromJSON<double, 3, 3>(j["BASELINE"]["KP_refmod_translational"]);
	gains_.KD_refmod_translational = extractMatrixFromJSON<double, 3, 3>(j["BASELINE"]["KD_refmod_translational"]);
  gains_.KI_refmod_translational = extractMatrixFromJSON<double, 3, 3>(j["BASELINE"]["KI_refmod_translational"]);
  gains_.KP_translational = extractMatrixFromJSON<double, 3, 3>(j["BASELINE"]["KP_translational"]);
	gains_.KD_translational = extractMatrixFromJSON<double, 3, 3>(j["BASELINE"]["KD_translational"]);
  gains_.KI_translational = extractMatrixFromJSON<double, 3, 3>(j["BASELINE"]["KI_translational"]);
  gains_.KP_omega_cmd_rotational = extractMatrixFromJSON<double, 3, 3>(j["BASELINE"]["KP_omega_cmd_rotational"]);
  gains_.KI_omega_cmd_rotational = extractMatrixFromJSON<double, 3, 3>(j["BASELINE"]["KI_omega_cmd_rotational"]);
  gains_.KP_omega_ref_rotational = extractMatrixFromJSON<double, 3, 3>(j["BASELINE"]["KP_omega_ref_rotational"]);
  gains_.KI_omega_ref_rotational = extractMatrixFromJSON<double, 3, 3>(j["BASELINE"]["KI_omega_ref_rotational"]);
	gains_.KP_rotational = extractMatrixFromJSON<double, 3, 3>(j["BASELINE"]["KP_rotational"]);
  gains_.KD_rotational = extractMatrixFromJSON<double, 3, 3>(j["BASELINE"]["KD_rotational"]);
  gains_.KI_rotational = extractMatrixFromJSON<double, 3, 3>(j["BASELINE"]["KI_rotational"]);
  gains_.Gamma_x_translational = extractMatrixFromJSON<double, 6, 6>(j["ADAPTIVE"]["Gamma_x_translational"]);
	gains_.Gamma_r_translational = extractMatrixFromJSON<double, 3, 3>(j["ADAPTIVE"]["Gamma_r_translational"]);
  gains_.Gamma_Theta_translational = extractMatrixFromJSON<double, 6, 6>(j["ADAPTIVE"]["Gamma_Theta_translational"]);
  gains_.Q_translational = extractMatrixFromJSON<double, 6, 6>(j["ADAPTIVE"]["Q_translational"]);
  gains_.sigma_x_translational = j["ADAPTIVE"]["sigma_x_translational"];
  gains_.sigma_r_translational = j["ADAPTIVE"]["sigma_r_translational"];
  gains_.sigma_Theta_translational = j["ADAPTIVE"]["sigma_Theta_translational"];
  gains_.dead_zone_delta_translational = j["ADAPTIVE"]["dead_zone_delta_translational"];
  gains_.dead_zone_e0_translational = j["ADAPTIVE"]["dead_zone_e0_translational"];
  gains_.x_e_x_translational = extractMatrixFromJSON<double, 18, 1>(j["ADAPTIVE"]["x_e_x_translational"]);
  gains_.S_diagonal_x_translational = extractMatrixFromJSON<double, 18, 1>(j["ADAPTIVE"]["S_diagonal_x_translational"]);
  gains_.alpha_x_translational = j["ADAPTIVE"]["alpha_x_translational"];
  gains_.x_e_r_translational = extractMatrixFromJSON<double, 9, 1>(j["ADAPTIVE"]["x_e_r_translational"]);
  gains_.S_diagonal_r_translational = extractMatrixFromJSON<double, 9, 1>(j["ADAPTIVE"]["S_diagonal_r_translational"]);
  gains_.alpha_r_translational = j["ADAPTIVE"]["alpha_r_translational"];
  gains_.x_e_Theta_translational = extractMatrixFromJSON<double, 18, 1>(j["ADAPTIVE"]["x_e_Theta_translational"]);
  gains_.S_diagonal_Theta_translational = extractMatrixFromJSON<double, 18, 1>(j["ADAPTIVE"]["S_diagonal_Theta_translational"]);
  gains_.alpha_Theta_translational = j["ADAPTIVE"]["alpha_Theta_translational"];
  gains_.Gamma_x_rotational = extractMatrixFromJSON<double, 3, 3>(j["ADAPTIVE"]["Gamma_x_rotational"]);
	gains_.Gamma_r_rotational = extractMatrixFromJSON<double, 3, 3>(j["ADAPTIVE"]["Gamma_r_rotational"]);
  gains_.Gamma_Theta_rotational = extractMatrixFromJSON<double, 6, 6>(j["ADAPTIVE"]["Gamma_Theta_rotational"]);
  gains_.Q_rotational = extractMatrixFromJSON<double, 3, 3>(j["ADAPTIVE"]["Q_rotational"]);
  gains_.sigma_x_rotational = j["ADAPTIVE"]["sigma_x_rotational"];
  gains_.sigma_r_rotational = j["ADAPTIVE"]["sigma_r_rotational"];
  gains_.sigma_Theta_rotational = j["ADAPTIVE"]["sigma_Theta_rotational"];
  gains_.dead_zone_delta_rotational = j["ADAPTIVE"]["dead_zone_delta_rotational"];
  gains_.dead_zone_e0_rotational = j["ADAPTIVE"]["dead_zone_e0_rotational"];
  gains_.x_e_x_rotational = extractMatrixFromJSON<double, 9, 1>(j["ADAPTIVE"]["x_e_x_rotational"]);
  gains_.S_diagonal_x_rotational = extractMatrixFromJSON<double, 9, 1>(j["ADAPTIVE"]["S_diagonal_x_rotational"]);
  gains_.alpha_x_rotational = j["ADAPTIVE"]["alpha_x_rotational"];
  gains_.x_e_r_rotational = extractMatrixFromJSON<double, 9, 1>(j["ADAPTIVE"]["x_e_r_rotational"]);
  gains_.S_diagonal_r_rotational = extractMatrixFromJSON<double, 9, 1>(j["ADAPTIVE"]["S_diagonal_r_rotational"]);
  gains_.alpha_r_rotational = j["ADAPTIVE"]["alpha_r_rotational"];
  gains_.x_e_Theta_rotational = extractMatrixFromJSON<double, 18, 1>(j["ADAPTIVE"]["x_e_Theta_rotational"]);
  gains_.S_diagonal_Theta_rotational = extractMatrixFromJSON<double, 18, 1>(j["ADAPTIVE"]["S_diagonal_Theta_rotational"]);
  gains_.alpha_Theta_rotational = j["ADAPTIVE"]["alpha_Theta_rotational"];
}

/*
  Function that given the gains read from the JSON file, initializes the rest of the parameters accordingly
*/
void MRAC::initializeControllerParameters(ParamsVehicle& vehicle_info, GainsMRAC& gains_)
{
  // Initialize to zero the 6x6 matrix and set the top-right 3x3 block as follows
  gains_.A_translational.setZero();
  gains_.A_translational.block<3, 3>(0, 3) = Eigen::Matrix3d::Identity();

  // Initialize to zero the 6x3 matrix and set the bottom 3x3 block as follows
  gains_.B_translational.setZero();
  gains_.B_translational.block<3, 3>(3, 0) = Eigen::Matrix3d::Identity();

  // Initialize to zero the 6x6 matrix 
  gains_.A_ref_translational.setZero();
  // Set the top-right 3x3 block as follows
  gains_.A_ref_translational.block<3, 3>(0, 3) = Eigen::Matrix3d::Identity();
  // Set the bottom-left 3x3 block as follows
  gains_.A_ref_translational.block<3, 3>(3, 0) = -gains_.KP_refmod_translational;
  // Set the bottom-right 3x3 block as follows
  gains_.A_ref_translational.block<3, 3>(3, 3) = -gains_.KD_refmod_translational;

  // Initialize to zero the 6x3 matrix and set the bottom 3x3 block as follows
  gains_.B_ref_translational.setZero();
  gains_.B_ref_translational.block<3, 3>(3, 0) = (1 / vehicle_info.mass) * Eigen::Matrix3d::Identity();

  // Initialize to zero the 3x3 matrix 
  gains_.A_rotational.setZero();

  // Set the 3x3 matrix as follows
  gains_.B_rotational = Eigen::Matrix3d::Identity();

  // Set the 3x3 matrix as follows
  gains_.A_ref_rotational = -gains_.KP_omega_ref_rotational;

  // Set the 3x3 matrix as follows
  gains_.B_ref_rotational = Eigen::Matrix3d::Identity();

  // Solve the continuos Lyapunov equation to compute P_translational
  gains_.P_translational = RealContinuousLyapunovEquation(gains_.A_ref_translational,
                                                          gains_.Q_translational);

  // Solve the continuos Lyapunov equation to compute P_rotational
  gains_.P_rotational = RealContinuousLyapunovEquation(gains_.A_ref_rotational,
                                                       gains_.Q_rotational);

  // Projection operator. Generate the S matrices from their ellipsoid semi-axis terms contained in S_diagonal
  gains_.S_x_translational = projection_operator::generateEllipsoidMatrixFromDiagonal(gains_.S_diagonal_x_translational);
  gains_.S_r_translational = projection_operator::generateEllipsoidMatrixFromDiagonal(gains_.S_diagonal_r_translational);
  gains_.S_Theta_translational = projection_operator::generateEllipsoidMatrixFromDiagonal(gains_.S_diagonal_Theta_translational);
  gains_.S_x_rotational = projection_operator::generateEllipsoidMatrixFromDiagonal(gains_.S_diagonal_x_rotational);
  gains_.S_r_rotational = projection_operator::generateEllipsoidMatrixFromDiagonal(gains_.S_diagonal_r_rotational);
  gains_.S_Theta_rotational = projection_operator::generateEllipsoidMatrixFromDiagonal(gains_.S_diagonal_Theta_rotational);

  // Projection operator. Compute epsilon from alpha
  gains_.epsilon_x_translational = projection_operator::computeEpsilonFromAlpha(gains_.alpha_x_translational);
  gains_.epsilon_r_translational = projection_operator::computeEpsilonFromAlpha(gains_.alpha_r_translational);
  gains_.epsilon_Theta_translational = projection_operator::computeEpsilonFromAlpha(gains_.alpha_Theta_translational);
  gains_.epsilon_x_rotational = projection_operator::computeEpsilonFromAlpha(gains_.alpha_x_rotational);
  gains_.epsilon_r_rotational = projection_operator::computeEpsilonFromAlpha(gains_.alpha_r_rotational);
  gains_.epsilon_Theta_rotational = projection_operator::computeEpsilonFromAlpha(gains_.alpha_Theta_rotational);
}

/*
  Assigning the right-hand side of the differential equations to the first time derivative of the full_state.
*/
void MRAC::assignSystemToDxdt(state_type /* &x */, state_type &dxdt, const double /* t */) 
{
  int current_index = 0; // Starting index for assignments

  // Assign to elements [0:1] of 'dxdt' --> 'internal_state_roll_des_filter'
  this->assignElementsToDxdt(this->cim.internal_state_roll_des_filter, dxdt, current_index);
  // Assign to elements [2:3] of 'dxdt' --> 'internal_state_pitch_des_filter'
  this->assignElementsToDxdt(this->cim.internal_state_pitch_des_filter, dxdt, current_index);
  // Assign to elements [4:5] of 'dxdt' --> 'internal_state_roll_dot_des_filter'
  this->assignElementsToDxdt(this->cim.internal_state_roll_dot_des_filter, dxdt, current_index);
  // Assign to elements [6:7] of 'dxdt' --> 'internal_state_pitch_dot_des_filter'
  this->assignElementsToDxdt(this->cim.internal_state_pitch_dot_des_filter, dxdt, current_index);
  // Assign to elements [8:10] of 'dxdt' --> 'translational_position_error'
  this->assignElementsToDxdt(this->cim.translational_position_error, dxdt, current_index);
  // Assign to elements [11:13] of 'dxdt' --> 'angular_error'
  this->assignElementsToDxdt(this->cim.angular_error, dxdt, current_index);
  // Assign to elements [14:19] of 'dxdt' --> 'x_ref_dot_translational'
  this->assignElementsToDxdt(this->csim_.x_ref_dot_translational, dxdt, current_index);
  // Assign to elements [20:22] of 'dxdt' --> 'translational_position_error_ref'
  this->assignElementsToDxdt(this->csim_.translational_position_error_ref, dxdt, current_index);
  // Assign to elements [23:40] of 'dxdt' --> 'K_hat_x_dot_translational'
  this->assignElementsToDxdt(this->csim_.K_hat_x_dot_translational, dxdt, current_index);
  // Assign to elements [41:49] of 'dxdt' --> 'K_hat_r_dot_translational'
  this->assignElementsToDxdt(this->csim_.K_hat_r_dot_translational, dxdt, current_index);
  // Assign to elements [50:67] of 'dxdt' --> 'Theta_hat_dot_translational'
  this->assignElementsToDxdt(this->csim_.Theta_hat_dot_translational, dxdt, current_index);
  // Assign to elements [68:70] of 'dxdt' --> 'omega_ref_dot_rotational'
  this->assignElementsToDxdt(this->csim_.omega_ref_dot_rotational, dxdt, current_index);
  // Assign to elements [71:79] of 'dxdt' --> 'K_hat_x_dot_rotational'
  this->assignElementsToDxdt(this->csim_.K_hat_x_dot_rotational, dxdt, current_index);
  // Assign to elements [80:88] of 'dxdt' --> 'K_hat_r_dot_rotational'
  this->assignElementsToDxdt(this->csim_.K_hat_r_dot_rotational, dxdt, current_index);
  // Assign to elements [89:106] of 'dxdt' --> 'Theta_hat_dot_rotational'
  this->assignElementsToDxdt(this->csim_.Theta_hat_dot_rotational, dxdt, current_index);
  // Assign to elements [107:109] of 'dxdt' --> 'angular_velocity_error_ref'
  this->assignElementsToDxdt(this->csim_.angular_velocity_error_ref, dxdt, current_index);
}

/*
  Function to compute the variables used for the differentiator of the desired roll and pitch angles.
  This function calculates the internal states and derivatives (first and second) of the roll and pitch
  desired angles using the provided filter coefficients and the current state variables.
*/
void MRAC::computeFilterDifferentiatorVariables(ControlInternalMembers& cim, 
                                                ParamsVehicle& vehicle_info, 
                                                StateController& state_)
{
  cim.internal_state_roll_des_filter = vehicle_info.A_filter_roll_des * state_.state_roll_des_filter 
                                     + vehicle_info.B_filter_roll_des * cim.roll_desired;

  cim.internal_state_pitch_des_filter = vehicle_info.A_filter_pitch_des * state_.state_pitch_des_filter 
                                      + vehicle_info.B_filter_pitch_des * cim.pitch_desired;

  cim.roll_desired_dot = vehicle_info.C_filter_roll_des * state_.state_roll_des_filter;
  cim.pitch_desired_dot = vehicle_info.C_filter_pitch_des * state_.state_pitch_des_filter;


  cim.internal_state_roll_dot_des_filter = vehicle_info.A_filter_roll_dot_des * state_.state_roll_dot_des_filter 
                                         + vehicle_info.B_filter_pitch_des * cim.roll_desired_dot;

  cim.internal_state_pitch_dot_des_filter = vehicle_info.A_filter_pitch_dot_des * state_.state_pitch_dot_des_filter 
                                          + vehicle_info.B_filter_pitch_dot_des * cim.pitch_desired_dot;

  cim.roll_desired_dot_dot = vehicle_info.C_filter_roll_dot_des * state_.state_roll_dot_des_filter;
  cim.pitch_desired_dot_dot = vehicle_info.C_filter_pitch_dot_des * state_.state_pitch_dot_des_filter;
}

/*
  OUTER LOOP CONTROLLER
*/
void MRAC::computeOuterLoop(ControlInternalMembers& cim,
                            ParamsVehicle& vehicle_info,
                            StateController& state_, 
                            ControlReferences& cr,
                            GainsMRAC& gains_,
                            ControllerSpecificInternalMembers& csim_)
{
  // Compute the translational_position_error [actual - reference_model]
  this->computeTranslationalPositionError(cim, cr.position, state_.x_ref_translational.head<3>());
  // Compute the translational_position_error_ref [reference_model - user_defined_trajectory]
  csim_.translational_position_error_ref = state_.x_ref_translational.head<3>() - cr.user_defined_position;

  // Reference command input [reference_model - user_defined_trajectory]
  csim_.r_cmd_translational = vehicle_info.mass * (
    - gains_.KI_refmod_translational * state_.integral_translational_position_error_ref
    + gains_.KP_refmod_translational * cr.user_defined_position
    + gains_.KD_refmod_translational * cr.user_defined_velocity
    + cr.user_defined_acceleration
  );

  // Reference model
  csim_.x_ref_dot_translational = gains_.A_ref_translational * state_.x_ref_translational
                                + gains_.B_ref_translational * csim_.r_cmd_translational;

  // Baseline PID controller terms [actual - reference_model]
  csim_.outer_loop_dynamic_inversion = - vehicle_info.mass * GRAVITATIONAL_ACCELERATION * this->e3_basis;
  csim_.outer_loop_P = - gains_.KP_translational * cim.translational_position_error;
  csim_.outer_loop_D = - gains_.KD_translational * (cr.velocity - state_.x_ref_translational.tail<3>());
  csim_.outer_loop_I = - gains_.KI_translational * state_.integral_translational_position_error;
  // Baseline PID controller [actual - reference_model]
  csim_.mu_PID_baseline_translational = csim_.outer_loop_dynamic_inversion + 
    vehicle_info.mass * (
      csim_.outer_loop_P
    + csim_.outer_loop_D
    + csim_.outer_loop_I
    + csim_.x_ref_dot_translational.tail<3>()
  );

  // Velocity expressed in body/local reference frame
  Eigen::Vector3d velocity_expr_in_local = cim.rotation_matrix_321_global_to_local * cr.velocity;
  // Regressor vector
  Eigen::Vector3d regressor_vector_translational = -0.5 * velocity_expr_in_local.norm() * velocity_expr_in_local;

  // Augmented regressor vector [includes the baseline controller]
  csim_.augmented_regressor_vector_translational.head<3>() = csim_.mu_PID_baseline_translational;
  csim_.augmented_regressor_vector_translational.tail<3>() = regressor_vector_translational;

  // Translational dynamics state
  Eigen::Matrix<double, 6, 1> x_translational;
  x_translational.head<3>() = cr.position;
  x_translational.tail<3>() = cr.velocity;

  // Tracking error [transposed]
  Eigen::Matrix<double, 1, 6> e_translational_transposed = (x_translational - state_.x_ref_translational).transpose();

  // Precomputing (e^T * P * B)
  Eigen::Matrix<double, 1, 3> eTranspose_P_B_translational = e_translational_transposed * 
                                                             gains_.P_translational * 
                                                             gains_.B_translational;

  // Precomputing the norm of (e^T * P * B)
  double eTranspose_P_B_norm_translational = eTranspose_P_B_translational.norm();

  // Computing the scalar value output from the dead-zone modification modulation function
  csim_.dead_zone_value_translational = deadZoneModulationFunction(e_translational_transposed,
                                                                    gains_.dead_zone_delta_translational,
                                                                    gains_.dead_zone_e0_translational);

  // Adaptive law K_hat_x
  csim_.K_hat_x_dot_translational = - gains_.Gamma_x_translational * csim_.dead_zone_value_translational * (
                                        x_translational * 
                                        eTranspose_P_B_translational
                                        + gains_.sigma_x_translational * 
                                          eTranspose_P_B_norm_translational * 
                                          state_.K_hat_x_translational);
  // Projection operator K_hat_x
  csim_.K_hat_x_dot_translational = 
    projection_operator::ellipsoid::projectionMatrix(state_.K_hat_x_translational, 
                                                     csim_.K_hat_x_dot_translational,
                                                     gains_.x_e_x_translational,
                                                     gains_.S_x_translational,
                                                     gains_.epsilon_x_translational);

  // Adaptive law K_hat_r
  csim_.K_hat_r_dot_translational = - gains_.Gamma_r_translational * csim_.dead_zone_value_translational * (
                                        csim_.r_cmd_translational * 
                                        eTranspose_P_B_translational
                                        + gains_.sigma_r_translational * 
                                          eTranspose_P_B_norm_translational * 
                                          state_.K_hat_r_translational);
  // Projection operator K_hat_r
  csim_.K_hat_r_dot_translational = 
    projection_operator::ellipsoid::projectionMatrix(state_.K_hat_r_translational, 
                                                     csim_.K_hat_r_dot_translational,
                                                     gains_.x_e_r_translational,
                                                     gains_.S_r_translational,
                                                     gains_.epsilon_r_translational);

  // Adaptive law Theta_hat
  csim_.Theta_hat_dot_translational = gains_.Gamma_Theta_translational * csim_.dead_zone_value_translational * (
                                        csim_.augmented_regressor_vector_translational * 
                                        eTranspose_P_B_translational
                                        - gains_.sigma_Theta_translational * 
                                          eTranspose_P_B_norm_translational * 
                                          state_.Theta_hat_translational);
  // Projection operator Theta_hat
  csim_.Theta_hat_dot_translational = 
    projection_operator::ellipsoid::projectionMatrix(state_.Theta_hat_translational, 
                                                     csim_.Theta_hat_dot_translational,
                                                     gains_.x_e_Theta_translational,
                                                     gains_.S_Theta_translational,
                                                     gains_.epsilon_Theta_translational);

  // Adaptive control law
  csim_.mu_adaptive_translational = 
      state_.K_hat_x_translational.transpose() * x_translational
    + state_.K_hat_r_translational.transpose() * csim_.r_cmd_translational
    - state_.Theta_hat_translational.transpose() * csim_.augmented_regressor_vector_translational;

  // Total virtual control input
  cim.mu_translational_raw = csim_.mu_PID_baseline_translational + 
                             csim_.mu_adaptive_translational;

}

/*
  OUTER LOOP CONTROLLER USED FOR DEBUGGING, using 'user' instead of 'reference-model' in the baseline
*/
void MRAC::computeOuterLoopDEBUGGING(ControlInternalMembers& cim,
                                      ParamsVehicle& vehicle_info,
                                      StateController& state_, 
                                      ControlReferences& cr,
                                      GainsMRAC& gains_,
                                      ControllerSpecificInternalMembers& csim_)
{
  // Compute the translational_position_error [actual - reference_model]
  this->computeTranslationalPositionError(cim, cr.position, cr.user_defined_position);

  // Compute the translational_position_error_ref [reference_model - user_defined_trajectory]
  csim_.translational_position_error_ref = state_.x_ref_translational.head<3>() - cr.user_defined_position;

  // Reference command input [reference_model - user_defined_trajectory]
  csim_.r_cmd_translational = vehicle_info.mass * (
    - gains_.KI_refmod_translational * state_.integral_translational_position_error_ref
    + gains_.KP_refmod_translational * cr.user_defined_position
    + gains_.KD_refmod_translational * cr.user_defined_velocity
    + cr.user_defined_acceleration
  );

  // Reference model
  csim_.x_ref_dot_translational = gains_.A_ref_translational * state_.x_ref_translational
                                + gains_.B_ref_translational * csim_.r_cmd_translational;

  // Baseline PID controller terms [actual - user]
  csim_.outer_loop_dynamic_inversion = - vehicle_info.mass * GRAVITATIONAL_ACCELERATION * this->e3_basis;
  csim_.outer_loop_P = - gains_.KP_translational * cim.translational_position_error;
  csim_.outer_loop_D = - gains_.KD_translational * (cr.velocity - cr.user_defined_velocity);
  csim_.outer_loop_I = - gains_.KI_translational * state_.integral_translational_position_error;
  // Baseline PID controller [actual - user]
  csim_.mu_PID_baseline_translational = csim_.outer_loop_dynamic_inversion + 
    vehicle_info.mass * (
      csim_.outer_loop_P
    + csim_.outer_loop_D
    + csim_.outer_loop_I
    + cr.user_defined_acceleration
  );

  // Velocity expressed in body/local reference frame
  Eigen::Vector3d velocity_expr_in_local = cim.rotation_matrix_321_global_to_local * cr.velocity;
  // Regressor vector
  Eigen::Vector3d regressor_vector_translational = -0.5 * velocity_expr_in_local.norm() * velocity_expr_in_local;

  // Augmented regressor vector [includes the baseline controller]
  csim_.augmented_regressor_vector_translational.head<3>() = csim_.mu_PID_baseline_translational;
  csim_.augmented_regressor_vector_translational.tail<3>() = regressor_vector_translational;

  // Translational dynamics state
  Eigen::Matrix<double, 6, 1> x_translational;
  x_translational.head<3>() = cr.position;
  x_translational.tail<3>() = cr.velocity;

  // Tracking error [transposed]
  Eigen::Matrix<double, 1, 6> e_translational_transposed = (x_translational - state_.x_ref_translational).transpose();

  // Precomputing (e^T * P * B)
  Eigen::Matrix<double, 1, 3> eTranspose_P_B_translational = e_translational_transposed * 
                                                             gains_.P_translational * 
                                                             gains_.B_translational;

  // Precomputing the norm of (e^T * P * B)
  double eTranspose_P_B_norm_translational = eTranspose_P_B_translational.norm();

  // Computing the scalar value output from the dead-zone modification modulation function
  csim_.dead_zone_value_translational = deadZoneModulationFunction(e_translational_transposed,
                                                                    gains_.dead_zone_delta_translational,
                                                                    gains_.dead_zone_e0_translational);

  // Adaptive law K_hat_x
  csim_.K_hat_x_dot_translational = - gains_.Gamma_x_translational * csim_.dead_zone_value_translational * (
                                        x_translational * 
                                        eTranspose_P_B_translational
                                        + gains_.sigma_x_translational * 
                                          eTranspose_P_B_norm_translational * 
                                          state_.K_hat_x_translational);
  // Projection operator K_hat_x
  csim_.K_hat_x_dot_translational = 
    projection_operator::ellipsoid::projectionMatrix(state_.K_hat_x_translational, 
                                                     csim_.K_hat_x_dot_translational,
                                                     gains_.x_e_x_translational,
                                                     gains_.S_x_translational,
                                                     gains_.epsilon_x_translational);

  // Adaptive law K_hat_r
  csim_.K_hat_r_dot_translational = - gains_.Gamma_r_translational * csim_.dead_zone_value_translational * (
                                        csim_.r_cmd_translational * 
                                        eTranspose_P_B_translational
                                        + gains_.sigma_r_translational * 
                                          eTranspose_P_B_norm_translational * 
                                          state_.K_hat_r_translational);
  // Projection operator K_hat_r
  csim_.K_hat_r_dot_translational = 
    projection_operator::ellipsoid::projectionMatrix(state_.K_hat_r_translational, 
                                                     csim_.K_hat_r_dot_translational,
                                                     gains_.x_e_r_translational,
                                                     gains_.S_r_translational,
                                                     gains_.epsilon_r_translational);

  // Adaptive law Theta_hat
  csim_.Theta_hat_dot_translational = gains_.Gamma_Theta_translational * csim_.dead_zone_value_translational * (
                                        csim_.augmented_regressor_vector_translational * 
                                        eTranspose_P_B_translational
                                        - gains_.sigma_Theta_translational * 
                                          eTranspose_P_B_norm_translational * 
                                          state_.Theta_hat_translational);
  // Projection operator Theta_hat
  csim_.Theta_hat_dot_translational = 
    projection_operator::ellipsoid::projectionMatrix(state_.Theta_hat_translational, 
                                                     csim_.Theta_hat_dot_translational,
                                                     gains_.x_e_Theta_translational,
                                                     gains_.S_Theta_translational,
                                                     gains_.epsilon_Theta_translational);
                                                     
  // Adaptive control law
  csim_.mu_adaptive_translational = 
      state_.K_hat_x_translational.transpose() * x_translational
    + state_.K_hat_r_translational.transpose() * csim_.r_cmd_translational
    - state_.Theta_hat_translational.transpose() * csim_.augmented_regressor_vector_translational;

  // Total virtual control input
  cim.mu_translational_raw = csim_.mu_PID_baseline_translational + 
                             csim_.mu_adaptive_translational;

}

/*
  INNER LOOP CONTROLLER
*/
void MRAC::computeInnerLoop(ControlInternalMembers& cim,
                            ParamsVehicle& vehicle_info,
                            StateController& state_, 
                            ControlReferences& cr,
                            GainsMRAC& gains_,
                            ControllerSpecificInternalMembers& csim_)
{
  // Compute angular_error [actual - desired]
  this->computeAngularError(cim, cr.euler_angles_rpy, cim.angular_position_desired);

  // Compute jacobian matrix
  csim_.jacobian_matrix = this->jacobianMatrix(cr.euler_angles_rpy[0], cr.euler_angles_rpy[1]);

  // Compute time derivative of jacobian matrix
  csim_.jacobian_matrix_dot = this->jacobianMatrixDot(cr.euler_angles_rpy[0],
                                                      cr.euler_angles_rpy[1],
                                                      cim.euler_angles_rpy_dot[0],
                                                      cim.euler_angles_rpy_dot[1]);

  // Compute command angular velocity
  csim_.omega_cmd_rotational = csim_.jacobian_matrix * (
    - gains_.KP_omega_cmd_rotational * cim.angular_error
    - gains_.KI_omega_cmd_rotational * state_.integral_angular_error
    + cim.angular_position_desired_dot
  );

  // Compute time derivative of command angular velocity
  csim_.omega_cmd_dot_rotational =
    csim_.jacobian_matrix_dot * (
      - gains_.KP_omega_cmd_rotational * cim.angular_error
      - gains_.KI_omega_cmd_rotational * state_.integral_angular_error
      + cim.angular_position_desired_dot) 
    + csim_.jacobian_matrix * (
      - gains_.KP_omega_cmd_rotational * (cim.euler_angles_rpy_dot - cim.angular_position_desired_dot)
      - gains_.KI_omega_cmd_rotational * cim.angular_error
      + cim.angular_position_desired_dot_dot
  );

  // Compute the angular_velocity_error_ref [refmod - cmd]
  csim_.angular_velocity_error_ref = state_.omega_ref_rotational - csim_.omega_cmd_rotational;

  // Reference command input [refmod - cmd]
  csim_.r_cmd_rotational = 
    gains_.KP_omega_ref_rotational * csim_.omega_cmd_rotational
    - gains_.KI_omega_ref_rotational * state_.integral_angular_velocity_error_ref
    + csim_.omega_cmd_dot_rotational;

  // Reference model 
  csim_.omega_ref_dot_rotational = 
    - gains_.KP_omega_ref_rotational * state_.omega_ref_rotational
    + csim_.r_cmd_rotational;

  // Baseline PID controller terms [actual - reference_model]
  csim_.inner_loop_dynamic_inversion = cr.angular_velocity.cross(vehicle_info.inertia_matrix * cr.angular_velocity);
  csim_.inner_loop_P = - gains_.KP_rotational * cim.angular_error;
  csim_.inner_loop_D = - gains_.KD_rotational * (cr.angular_velocity - state_.omega_ref_rotational);
  csim_.inner_loop_I = - gains_.KI_rotational * state_.integral_angular_error;
  // Baseline PID controller [actual - reference_model]
  csim_.tau_PID_baseline_rotational = csim_.inner_loop_dynamic_inversion + 
    vehicle_info.inertia_matrix * (
      csim_.inner_loop_P 
    + csim_.inner_loop_D
    + csim_.inner_loop_I
    + csim_.omega_ref_dot_rotational
  );

  // Regressor vector
  Eigen::Vector3d regressor_vector_rotational;
  regressor_vector_rotational << 
    cr.angular_velocity[1] * cr.angular_velocity[2],
    cr.angular_velocity[0] * cr.angular_velocity[2],
    cr.angular_velocity[0] * cr.angular_velocity[1];

  // Augmented regressor vector [includes the baseline controller]
  csim_.augmented_regressor_vector_rotational.head<3>() = csim_.tau_PID_baseline_rotational;
  csim_.augmented_regressor_vector_rotational.tail<3>() = regressor_vector_rotational;

  // Tracking error [transposed]
  Eigen::Matrix<double, 1, 3> e_rotational_transposed = (cr.angular_velocity - state_.omega_ref_rotational).transpose();

  // Precomputing (e^T * P * B)
  Eigen::Matrix<double, 1, 3> eTranspose_P_B_rotational = e_rotational_transposed * 
                                                          gains_.P_rotational * 
                                                          gains_.B_rotational;

  // Precomputing the norm of (e^T * P * B)
  double eTranspose_P_B_norm_rotational = eTranspose_P_B_rotational.norm();

  // Computing the scalar value output from the dead-zone modification modulation function
  csim_.dead_zone_value_rotational = deadZoneModulationFunction(e_rotational_transposed,
                                                                 gains_.dead_zone_delta_rotational,
                                                                 gains_.dead_zone_e0_rotational);

  // Adaptive law K_hat_x
  csim_.K_hat_x_dot_rotational = - gains_.Gamma_x_rotational * csim_.dead_zone_value_rotational * (
                                    cr.angular_velocity * 
                                    eTranspose_P_B_rotational
                                    + gains_.sigma_x_rotational * 
                                      eTranspose_P_B_norm_rotational * 
                                      state_.K_hat_x_rotational);
  // Projection operator K_hat_x
  csim_.K_hat_x_dot_rotational = 
    projection_operator::ellipsoid::projectionMatrix(state_.K_hat_x_rotational, 
                                                     csim_.K_hat_x_dot_rotational,
                                                     gains_.x_e_x_rotational,
                                                     gains_.S_x_rotational,
                                                     gains_.epsilon_x_rotational);
            
  // Adaptive law K_hat_r
  csim_.K_hat_r_dot_rotational = - gains_.Gamma_r_rotational * csim_.dead_zone_value_rotational * (
                                    csim_.r_cmd_rotational * 
                                    eTranspose_P_B_rotational
                                    + gains_.sigma_r_rotational * 
                                      eTranspose_P_B_norm_rotational * 
                                      state_.K_hat_r_rotational);
  // Projection operator K_hat_r
  csim_.K_hat_r_dot_rotational = 
    projection_operator::ellipsoid::projectionMatrix(state_.K_hat_r_rotational, 
                                                     csim_.K_hat_r_dot_rotational,
                                                     gains_.x_e_r_rotational,
                                                     gains_.S_r_rotational,
                                                     gains_.epsilon_r_rotational);

  // Adaptive law Theta_hat
  csim_.Theta_hat_dot_rotational = gains_.Gamma_Theta_rotational * csim_.dead_zone_value_rotational * (
                                    csim_.augmented_regressor_vector_rotational * 
                                    eTranspose_P_B_rotational
                                    - gains_.sigma_Theta_rotational * 
                                      eTranspose_P_B_norm_rotational * 
                                      state_.Theta_hat_rotational);
  // Projection operator Theta_hat
  csim_.Theta_hat_dot_rotational = 
    projection_operator::ellipsoid::projectionMatrix(state_.Theta_hat_rotational, 
                                                     csim_.Theta_hat_dot_rotational,
                                                     gains_.x_e_Theta_rotational,
                                                     gains_.S_Theta_rotational,
                                                     gains_.epsilon_Theta_rotational);

  // Adaptive control law
  csim_.tau_adaptive_rotational = 
      state_.K_hat_x_rotational.transpose() * cr.angular_velocity
    + state_.K_hat_r_rotational.transpose() * csim_.r_cmd_rotational
    - state_.Theta_hat_rotational.transpose() * csim_.augmented_regressor_vector_rotational;

  // Total control input
    cim.U2_U3_U4 = csim_.tau_PID_baseline_rotational + 
                   csim_.tau_adaptive_rotational;

}

/*
  INNER LOOP CONTROLLER USED FOR DEBUGGING, using 'desired' instead of 'reference-model' in the baseline
*/
void MRAC::computeInnerLoopDEBUGGING(ControlInternalMembers& cim,
                                      ParamsVehicle& vehicle_info,
                                      StateController& state_, 
                                      ControlReferences& cr,
                                      GainsMRAC& gains_,
                                      ControllerSpecificInternalMembers& csim_)
{
  // Compute angular_error [actual - desired]
  this->computeAngularError(cim, cr.euler_angles_rpy, cim.angular_position_desired);

  // Compute jacobian matrix
  csim_.jacobian_matrix = this->jacobianMatrix(cr.euler_angles_rpy[0], cr.euler_angles_rpy[1]);

  // Compute time derivative of jacobian matrix
  csim_.jacobian_matrix_dot = this->jacobianMatrixDot(cr.euler_angles_rpy[0],
                                                      cr.euler_angles_rpy[1],
                                                      cim.euler_angles_rpy_dot[0],
                                                      cim.euler_angles_rpy_dot[1]);

  // Compute command angular velocity
  csim_.omega_cmd_rotational = csim_.jacobian_matrix * (
    - gains_.KP_omega_cmd_rotational * cim.angular_error
    - gains_.KI_omega_cmd_rotational * state_.integral_angular_error
    + cim.angular_position_desired_dot
  );

  // Compute time derivative of command angular velocity
  csim_.omega_cmd_dot_rotational =
    csim_.jacobian_matrix_dot * (
      - gains_.KP_omega_cmd_rotational * cim.angular_error
      - gains_.KI_omega_cmd_rotational * state_.integral_angular_error
      + cim.angular_position_desired_dot) 
    + csim_.jacobian_matrix * (
      - gains_.KP_omega_cmd_rotational * (cim.euler_angles_rpy_dot - cim.angular_position_desired_dot)
      - gains_.KI_omega_cmd_rotational * cim.angular_error
      + cim.angular_position_desired_dot_dot
  );

  // Compute the angular_velocity_error_ref [refmod - cmd]
  csim_.angular_velocity_error_ref = state_.omega_ref_rotational - csim_.omega_cmd_rotational;

  // Reference command input [refmod - cmd]
  csim_.r_cmd_rotational = 
    gains_.KP_omega_ref_rotational * csim_.omega_cmd_rotational
    - gains_.KI_omega_ref_rotational * state_.integral_angular_velocity_error_ref
    + csim_.omega_cmd_dot_rotational;

  // Reference model 
  csim_.omega_ref_dot_rotational = 
    - gains_.KP_omega_ref_rotational * state_.omega_ref_rotational
    + csim_.r_cmd_rotational;

  // Baseline PID controller terms [actual - desired]
  csim_.inner_loop_dynamic_inversion = cr.angular_velocity.cross(vehicle_info.inertia_matrix * cr.angular_velocity);
  csim_.inner_loop_P = - gains_.KP_rotational * cim.angular_error;
  csim_.inner_loop_D = - gains_.KD_rotational * (cim.euler_angles_rpy_dot - cim.angular_position_desired_dot);
  csim_.inner_loop_I = - gains_.KI_rotational * state_.integral_angular_error;
  // Baseline PID controller [actual - desired]

  csim_.tau_PID_baseline_rotational = csim_.inner_loop_dynamic_inversion + 
    vehicle_info.inertia_matrix * (
      csim_.inner_loop_P 
    + csim_.inner_loop_D
    + csim_.inner_loop_I
    + cim.angular_position_desired_dot_dot
  );

  // Regressor vector
  Eigen::Vector3d regressor_vector_rotational;
  regressor_vector_rotational << 
    cr.angular_velocity[1] * cr.angular_velocity[2],
    cr.angular_velocity[0] * cr.angular_velocity[2],
    cr.angular_velocity[0] * cr.angular_velocity[1];

  // Augmented regressor vector [includes the baseline controller]
  csim_.augmented_regressor_vector_rotational.head<3>() = csim_.tau_PID_baseline_rotational;
  csim_.augmented_regressor_vector_rotational.tail<3>() = regressor_vector_rotational;

  // Tracking error [transposed]
  Eigen::Matrix<double, 1, 3> e_rotational_transposed = (cr.angular_velocity - state_.omega_ref_rotational).transpose();

  // Precomputing (e^T * P * B)
  Eigen::Matrix<double, 1, 3> eTranspose_P_B_rotational = e_rotational_transposed * 
                                                          gains_.P_rotational * 
                                                          gains_.B_rotational;

  // Precomputing the norm of (e^T * P * B)
  double eTranspose_P_B_norm_rotational = eTranspose_P_B_rotational.norm();

  // Computing the scalar value output from the dead-zone modification modulation function
  csim_.dead_zone_value_rotational = deadZoneModulationFunction(e_rotational_transposed,
                                                                 gains_.dead_zone_delta_rotational,
                                                                 gains_.dead_zone_e0_rotational);

  // Adaptive law K_hat_x
  csim_.K_hat_x_dot_rotational = - gains_.Gamma_x_rotational * csim_.dead_zone_value_rotational * (
                                    cr.angular_velocity * 
                                    eTranspose_P_B_rotational
                                    + gains_.sigma_x_rotational * 
                                      eTranspose_P_B_norm_rotational * 
                                      state_.K_hat_x_rotational);
  // Projection operator K_hat_x
  csim_.K_hat_x_dot_rotational = 
    projection_operator::ellipsoid::projectionMatrix(state_.K_hat_x_rotational, 
                                                     csim_.K_hat_x_dot_rotational,
                                                     gains_.x_e_x_rotational,
                                                     gains_.S_x_rotational,
                                                     gains_.epsilon_x_rotational);

  // Adaptive law K_hat_r
  csim_.K_hat_r_dot_rotational = - gains_.Gamma_r_rotational * csim_.dead_zone_value_rotational * (
                                    csim_.r_cmd_rotational * 
                                    eTranspose_P_B_rotational
                                    + gains_.sigma_r_rotational * 
                                      eTranspose_P_B_norm_rotational * 
                                      state_.K_hat_r_rotational);
  // Projection operator K_hat_r
  csim_.K_hat_r_dot_rotational = 
    projection_operator::ellipsoid::projectionMatrix(state_.K_hat_r_rotational, 
                                                     csim_.K_hat_r_dot_rotational,
                                                     gains_.x_e_r_rotational,
                                                     gains_.S_r_rotational,
                                                     gains_.epsilon_r_rotational);

  // Adaptive law Theta_hat
  csim_.Theta_hat_dot_rotational = gains_.Gamma_Theta_rotational * csim_.dead_zone_value_rotational * (
                                    csim_.augmented_regressor_vector_rotational * 
                                    eTranspose_P_B_rotational
                                    - gains_.sigma_Theta_rotational * 
                                      eTranspose_P_B_norm_rotational * 
                                      state_.Theta_hat_rotational);
  // Projection operator Theta_hat
  csim_.Theta_hat_dot_rotational = 
    projection_operator::ellipsoid::projectionMatrix(state_.Theta_hat_rotational, 
                                                     csim_.Theta_hat_dot_rotational,
                                                     gains_.x_e_Theta_rotational,
                                                     gains_.S_Theta_rotational,
                                                     gains_.epsilon_Theta_rotational);

  // Adaptive control law
  csim_.tau_adaptive_rotational = 
      state_.K_hat_x_rotational.transpose() * cr.angular_velocity
    + state_.K_hat_r_rotational.transpose() * csim_.r_cmd_rotational
    - state_.Theta_hat_rotational.transpose() * csim_.augmented_regressor_vector_rotational;

  // Total control input
    cim.U2_U3_U4 = csim_.tau_PID_baseline_rotational + 
                   csim_.tau_adaptive_rotational;

}


/*
  MRAC Control algorithm
*/
void MRAC::computeControlAlgorithm() 
{

  /*
    Function to compute various rotational parameters.
    This function sets the desired angle, angular rate and angular acceleration for yaw,
    computes the inverse of the Jacobian matrix, the derivative of the Euler angles,
    and the rotation matrix to go from global to local coordinates.
  */
  this->computeRotationalParameters(cim, cr);

  /*
    Compute OUTER LOOP CONTROLLER
  */
  this->computeOuterLoop(cim, vehicle_info, state_, cr, gains_, csim_);
  // this->computeOuterLoopDEBUGGING(cim, vehicle_info, state_, cr, gains_, csim_); // FOR DEBUGGING !!!!

  /* IMPLEMENT THE SAFETY MECHANISM HERE */

  /*
  Compute:
    the Total Thrust: cim.U_control_inputs[0],
    the desired roll angle: cim.roll_desired,
    the desired pitch angle: cim.pitch_desired.
  */
  this->compute_U1_RollDes_PitchDes(cim);

  /*
    Compute the variables used for the differentiator of the desired roll and pitch angles.
  */
  this->computeFilterDifferentiatorVariables(cim, vehicle_info, state_);

  /*
    Compute INNER LOOP CONTROLLER
  */
  // this->computeInnerLoop(cim, vehicle_info, state_, cr, gains_, csim_);
  this->computeInnerLoopDEBUGGING(cim, vehicle_info, state_, cr, gains_, csim_); // FOR DEBUGGING !!!! 

  /*
    Compute the thrust in Newton that each motor needs to produce and convert it
    from Newton to the normalized value comprised between 0 and 1 to be sent to Pixhawk
  */
  // FOR X8COPTER
  this->computeNormalizedThrust(cim, vehicle_info);

  // FOR QUADCOPTER
  // this->computeNormalizedThrustQuadcopterMode(cim, vehicle_info);  

}

#endif //SELECTED_CONTROLLER == __MRAC__