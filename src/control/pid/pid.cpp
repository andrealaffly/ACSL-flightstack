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
 * File:        pid.cpp
 * Author:      Mattia Gramuglia
 * Date:        April 22, 2024
 * For info:    Andrea L'Afflitto 
 *              a.lafflitto@vt.edu
 * 
 * Description: Implementation of the PID controller.
 * 
 * GitHub:    https://github.com/andrealaffly/ACSL-flightstack.git
 **********************************************************************************************************************/

/**
 * @file pid.cpp
 * @brief Immplementation of the PID controller
 */
#include "multi_threaded_node.hpp"
#include "pid.hpp"
#include "logging_pid.hpp"
#include "json_parser.hpp"

// Constructor
PID::PID(MultiThreadedNode& node) : 
  Control(node)
{
  // Initialize controller gains
  this->readJSONfile("gains_pid.json");

  log_data_ = std::make_shared<LogData_PID>(node, *this);

  // Initialize logging
	log_data_->logInitializeLogging();
}

// Constructor initializes the full_state and sets the Eigen::Map references
PID::StateController::StateController() : 
  full_state(14, 0.0), // 10-element full_state vector
  state_roll_des_filter(full_state.data()), // Maps the elements [0:1]
  state_pitch_des_filter(full_state.data() + 2), // Maps the elements [2:3]
  state_roll_dot_des_filter(full_state.data() + 4), // Maps the elements [4:5]
  state_pitch_dot_des_filter(full_state.data() + 6), // Maps the elements [6:7]
  integral_translational_position_error(full_state.data() + 8), // Maps the elements [8:10]
  integral_angular_error(full_state.data() + 11) // Maps the elements [11:13]
{}

// Constructor
PID::ControllerSpecificInternalMembers::ControllerSpecificInternalMembers() : 
  outer_loop_P(Eigen::Vector3d::Zero()),
  outer_loop_I(Eigen::Vector3d::Zero()),
  outer_loop_D(Eigen::Vector3d::Zero()),
  outer_loop_dynamic_inversion(Eigen::Vector3d::Zero()),
  inner_loop_P(Eigen::Vector3d::Zero()),
  inner_loop_I(Eigen::Vector3d::Zero()),
  inner_loop_D(Eigen::Vector3d::Zero()),
  inner_loop_dynamic_inversion(Eigen::Vector3d::Zero())
{}

PID::StateController& PID::getStateController()
{
  return state_;
}

const double& PID::getTimeStepRK4() const
{
  return time_step_rk4_;
}

const GainsPID& PID::getGains() const
{
  return gains_;
}

// Getter function that returns a reference to the current getControllerSpecificInternalMembers object
const PID::ControllerSpecificInternalMembers& PID::getControllerSpecificInternalMembers() const
{
  return csim_;
}

/*
  Getter function that returns a pointer to the log_data_ instance
*/
std::shared_ptr<LogData_PID> PID::getLogData() const
{
  return log_data_;
}

const std::string& PID::getControllerName()
{
  return controller_name_;
}

/*
  Function to read the tuning gains coming from the .json file and assign it to the
  gains_ struct instantiated in the PID class
*/
void PID::readJSONfile(const std::string& fileName)
{
  // Define the path where the PID gains JSON files are located
  const std::string path = "./src/flightstack/params/control/pid/";

  // Concatenate the path with the file name
  std::string jsonFile = path + fileName;

  std::ifstream file(jsonFile);
  if (!file.is_open()) {
    throw std::runtime_error("Could not open file: " + jsonFile);
  }

	nlohmann::json j;
	file >> j;
	

  gains_.KP_translational = extractMatrixFromJSON<double, 3, 3>(j["KP_translational"]);
	gains_.KD_translational = extractMatrixFromJSON<double, 3, 3>(j["KD_translational"]);
  gains_.KI_translational = extractMatrixFromJSON<double, 3, 3>(j["KI_translational"]);
	gains_.KP_rotational = extractMatrixFromJSON<double, 3, 3>(j["KP_rotational"]);
  gains_.KD_rotational = extractMatrixFromJSON<double, 3, 3>(j["KD_rotational"]);
  gains_.KI_rotational = extractMatrixFromJSON<double, 3, 3>(j["KI_rotational"]);

}

/*
  Assigning the right-hand side of the differential equations to the first time derivative of the full_state.
*/
void PID::assignSystemToDxdt(state_type /* &x */, state_type &dxdt, const double /* t */) 
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

}

/*
  Function to compute the variables used for the differentiator of the desired roll and pitch angles.
  This function calculates the internal states and derivatives (first and second) of the desired roll and pitch
  angles using the provided filter coefficients and the current state variables.
*/
void PID::computeFilterDifferentiatorVariables(ControlInternalMembers& cim, 
                                                VehicleInfo& vehicle_info, 
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
void PID::computeOuterLoop(ControlInternalMembers& cim,
                            VehicleInfo& vehicle_info,
                            StateController& state_, 
                            ControlReferences& cr,
                            GainsPID& gains_,
                            ControllerSpecificInternalMembers& csim_)
{
  csim_.outer_loop_dynamic_inversion = - vehicle_info.mass * GRAVITATIONAL_ACCELERATION * this->e3_basis;
  csim_.outer_loop_P = - gains_.KP_translational * cim.translational_position_error;
  csim_.outer_loop_D = - gains_.KD_translational * (cr.velocity - cr.user_defined_velocity);
  csim_.outer_loop_I = - gains_.KI_translational * state_.integral_translational_position_error;

  cim.mu_translational_raw = csim_.outer_loop_dynamic_inversion + 
    vehicle_info.mass * (
      csim_.outer_loop_P
    + csim_.outer_loop_D
    + csim_.outer_loop_I
    + cr.user_defined_acceleration
  );
}

/*
  INNER LOOP CONTROLLER
*/
void PID::computeInnerLoop(ControlInternalMembers& cim,
                            VehicleInfo& vehicle_info,
                            StateController& state_, 
                            ControlReferences& cr,
                            GainsPID& gains_,
                            ControllerSpecificInternalMembers& csim_)
{
  csim_.inner_loop_dynamic_inversion = cr.angular_velocity.cross(vehicle_info.inertia_matrix * cr.angular_velocity);
  csim_.inner_loop_P = - gains_.KP_rotational * cim.angular_error;
  csim_.inner_loop_D = - gains_.KD_rotational * (cim.euler_angles_rpy_dot - cim.angular_position_desired_dot);
  csim_.inner_loop_I = - gains_.KI_rotational * state_.integral_angular_error;

  cim.U2_U3_U4 = csim_.inner_loop_dynamic_inversion + 
    vehicle_info.inertia_matrix * (
      csim_.inner_loop_P 
    + csim_.inner_loop_D
    + csim_.inner_loop_I
    + cim.angular_position_desired_dot_dot
  );
}


/*
  PID Control algorithm
*/
void PID::computeControlAlgorithm() 
{

  // Compute the translational position error
  this->computeTranslationalPositionError(cim, cr.position, cr.user_defined_position);
  
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

  /* IMPLEMENT THE SAFETY MECHANISM HERE */

  /*
  Compute:
    the Total Thrust: cim.U_control_inputs[0],
    the desired roll angle: cim.roll_desired,
    the desired pitch angle: cim.pitch_desired.
  */
  this->compute_U1_RollDes_PitchDes(cim);

  // Compute angular error
  this->computeAngularError(cim, cr.euler_angles_rpy, cim.angular_position_desired);

  /*
    Compute the variables used for the differentiator of the desired roll and pitch angles.
  */
  this->computeFilterDifferentiatorVariables(cim, vehicle_info, state_);

  /*
    Compute INNER LOOP CONTROLLER
  */
  this->computeInnerLoop(cim, vehicle_info, state_, cr, gains_, csim_);

  /*
    Compute the thrust in Newton that each motor needs to produce and convert it
    from Newton to the normalized value comprised between 0 and 1 to be sent to Pixhawk
  */
  // FOR X8COPTER
  this->computeNormalizedThrust(cim, vehicle_info);

  // FOR QUADCOPTER
  // this->computeNormalizedThrustQuadcopterMode(cim, vehicle_info);

}
