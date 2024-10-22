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
 * File:        mrac.hpp
 * Author:      Mattia Gramuglia
 * Date:        June 13, 2024
 * For info:    Andrea L'Afflitto 
 *              a.lafflitto@vt.edu
 * 
 * Description: Implementation of the MRAC controller.
 * 
 * GitHub:    https://github.com/andrealaffly/ACSL-flightstack.git
 **********************************************************************************************************************/

/**
 * @file mrac.hpp
 * @brief Implementation of MRAC controller
 */
#ifndef MRAC_HPP
#define MRAC_HPP

#include <cmath>
#include <algorithm>
#include <string> 

#include <Eigen/Dense>

#include "control.hpp"
#include "mrac_gains.hpp"
#include "continuous_lyapunov_equation.hpp"
#include "projection_operator.hpp"

/**
 * @class LogData_MRAC
 * Forward decleration of LogData_MRAC class
 */
class LogData_MRAC;

/**
 * @class MRAC
 */
class MRAC : public Control
{
public:

  /**
  * @brief This struct represents the state of the MRAC controller differential equations that will be integrated at each RK4 iteration.
  * IT IS NOT THE VEHICLE STATE (e.g. POSITION, VELOCITY, etc.).
  * @struct StateController
  */
  struct StateController 
  {
    // Constructor

    StateController();

    std::vector<double> full_state; 

    /*
      Those are references to portions of the full_state vector that behave and can be used just like any other
      Eigen vector. Modifying those will directly act on the memory locations where they are stored, hence changes will
      automatically be refelected in full_state
    */
    Eigen::Map<Eigen::Matrix<double, 2, 1>> state_roll_des_filter; 
    Eigen::Map<Eigen::Matrix<double, 2, 1>> state_pitch_des_filter; 
    Eigen::Map<Eigen::Matrix<double, 2, 1>> state_roll_dot_des_filter; 
    Eigen::Map<Eigen::Matrix<double, 2, 1>> state_pitch_dot_des_filter;
    Eigen::Map<Eigen::Matrix<double, 3, 1>> integral_translational_position_error; // [position] actual - refmod
    Eigen::Map<Eigen::Matrix<double, 3, 1>> integral_angular_error; // [attitude] actual - desired
    Eigen::Map<Eigen::Matrix<double, 6, 1>> x_ref_translational;
    Eigen::Map<Eigen::Matrix<double, 3, 1>> integral_translational_position_error_ref; // [position] refmod - user
    Eigen::Map<Eigen::Matrix<double, 6, 3>> K_hat_x_translational;
    Eigen::Map<Eigen::Matrix<double, 3, 3>> K_hat_r_translational;
    Eigen::Map<Eigen::Matrix<double, 6, 3>> Theta_hat_translational; 
    Eigen::Map<Eigen::Matrix<double, 3, 1>> omega_ref_rotational;
    Eigen::Map<Eigen::Matrix<double, 3, 3>> K_hat_x_rotational;
    Eigen::Map<Eigen::Matrix<double, 3, 3>> K_hat_r_rotational;
    Eigen::Map<Eigen::Matrix<double, 6, 3>> Theta_hat_rotational; 
    Eigen::Map<Eigen::Matrix<double, 3, 1>> integral_angular_velocity_error_ref; // [omega] refmod - cmd
  };

 /**
  * @struct ControllerSpecificInternalMembers
  * @brief This struct contains the variables that are used in this particular controller
  */
  struct ControllerSpecificInternalMembers 
  {
    // Constructor
    /**
     * @brief Construct a new Controller Specific Internal Members object
     * @param None
     */
    ControllerSpecificInternalMembers();

    Eigen::Matrix<double, 3, 1> translational_position_error_ref; // [position] refmod - user
    Eigen::Matrix<double, 3, 1> r_cmd_translational; // Reference command input
    Eigen::Matrix<double, 6, 1> x_ref_dot_translational; // Translational reference model
    Eigen::Matrix<double, 3, 1> mu_PID_baseline_translational;
    Eigen::Matrix<double, 6, 1> augmented_regressor_vector_translational;
    Eigen::Matrix<double, 6, 3> K_hat_x_dot_translational;
    Eigen::Matrix<double, 3, 3> K_hat_r_dot_translational; 
    Eigen::Matrix<double, 6, 3> Theta_hat_dot_translational; 
    Eigen::Matrix<double, 3, 1> mu_adaptive_translational;
    double dead_zone_value_translational;

    Eigen::Vector3d outer_loop_P; // Proportional part of the baseline control input of the outer loop
    Eigen::Vector3d outer_loop_I; // Integral part of the baseline control input of the outer loop
    Eigen::Vector3d outer_loop_D; // Derivative part of the baseline control input of the outer loop
    Eigen::Vector3d outer_loop_dynamic_inversion; // Dynamic inversion term of the baseline control input of the outer loop

    Eigen::Matrix3d jacobian_matrix;
    Eigen::Matrix3d jacobian_matrix_dot;
    Eigen::Vector3d omega_cmd_rotational; // Command angular velocity
    Eigen::Vector3d omega_cmd_dot_rotational; // Time derivative of command angular velocity
    Eigen::Vector3d angular_velocity_error_ref; // [omega] refmod - cmd
    Eigen::Vector3d omega_ref_dot_rotational; // Reference model
    Eigen::Vector3d r_cmd_rotational; // Reference command input
    Eigen::Vector3d tau_PID_baseline_rotational;
    Eigen::Matrix<double, 6, 1> augmented_regressor_vector_rotational;
    Eigen::Matrix<double, 3, 3> K_hat_x_dot_rotational; 
    Eigen::Matrix<double, 3, 3> K_hat_r_dot_rotational; 
    Eigen::Matrix<double, 6, 3> Theta_hat_dot_rotational; 
    Eigen::Vector3d tau_adaptive_rotational;
    double dead_zone_value_rotational;
    
    Eigen::Vector3d inner_loop_P; // Proportional part of the baseline control input of the inner loop
    Eigen::Vector3d inner_loop_I; // Integral part of the baseline control input of the inner loop
    Eigen::Vector3d inner_loop_D; // Derivative part of the baseline control input of the inner loop
    Eigen::Vector3d inner_loop_dynamic_inversion; // Dynamic inversion term of the baseline control input of the inner loop

  };

  // Constructor
  /**
   * @class MRAC
   * @param node 
   */
  MRAC(MultiThreadedNode& node);

  // Getter functions
  /**
   * @brief Getter functions
   * 
   * @return StateController& 
   */
  StateController& getStateController();
  const double& getTimeStepRK4() const; 
  const GainsMRAC& getGains() const;
  const ControllerSpecificInternalMembers& getControllerSpecificInternalMembers() const;
  std::shared_ptr<LogData_MRAC> getLogData() const;
  static const std::string& getControllerName();

  /**
   * @brief Function to read the tuning gains coming from the .json file and assign it to the gains_ struct instantiated in the MRAC class
   * @param const std::string& fileName
   */
  void readJSONfile(const std::string& fileName);

  /**
   * @brief Function that given the gains read from the JSON file, initializes the rest of the parameters accordingly
   * @param vehicle_info 
   * @param gains_ 
   */
  void initializeControllerParameters(VehicleInfo& vehicle_info, GainsMRAC& gains_);

  /**
   * @brief Assigning the right-hand side of the differential equations to the first time derivative of the full_state.
   */
  void assignSystemToDxdt(state_type /* &x */, state_type &dxdt, const double /* t */);

  /**
   * @brief   Function to compute the variables used for the differentiator of the desired roll and pitch angles.
   * This function calculates the internal states and derivatives (first and second) of the roll and pitch desired angles using the provided filter coefficients and the current state variables.
   * @param cim 
   * @param vehicle_info 
   */
  void computeFilterDifferentiatorVariables(ControlInternalMembers& cim, 
                                            VehicleInfo& vehicle_info, 
                                            StateController& state_);

  /**
   * @brief OUTER LOOP CONTROLLER
   * @param cim 
   * @param vehicle_info 
   * @param state_ 
   * @param cr 
   * @param gains_ 
   * @param csim_ 
   */
  void computeOuterLoop(ControlInternalMembers& cim,
                        VehicleInfo& vehicle_info,
                        StateController& state_, 
                        ControlReferences& cr,
                        GainsMRAC& gains_,
                        ControllerSpecificInternalMembers& csim_);
  \
  /**
   * @brief OUTER LOOP CONTROLLER USED FOR DEBUGGING, using 'user' instead of 'reference-model' in the baseline
   * @param cim 
   * @param vehicle_info 
   * @param state_ 
   * @param cr 
   * @param gains_ 
   * @param csim_ 
   */
  void computeOuterLoopDEBUGGING(ControlInternalMembers& cim,
                                  VehicleInfo& vehicle_info,
                                  StateController& state_, 
                                  ControlReferences& cr,
                                  GainsMRAC& gains_,
                                  ControllerSpecificInternalMembers& csim_);
  
  /**
   * @brief INNER LOOP CONTROLLER
   * @param cim 
   * @param vehicle_info 
   * @param state_ 
   * @param cr 
   * @param gains_ 
   * @param csim_ 
   */
  void computeInnerLoop(ControlInternalMembers& cim,
                        VehicleInfo& vehicle_info,
                        StateController& state_, 
                        ControlReferences& cr,
                        GainsMRAC& gains_,
                        ControllerSpecificInternalMembers& csim_);

  /**
   * @brief INNER LOOP CONTROLLER USED FOR DEBUGGING, using 'desired' instead of 'reference-model' in the baseline
   * @param cim 
   * @param vehicle_info 
   * @param state_ 
   * @param cr 
   * @param gains_ 
   * @param csim_ 
   */
  void computeInnerLoopDEBUGGING(ControlInternalMembers& cim,
                                  VehicleInfo& vehicle_info,
                                  StateController& state_, 
                                  ControlReferences& cr,
                                  GainsMRAC& gains_,
                                  ControllerSpecificInternalMembers& csim_);

  /**
   * @brief MRAC Control algorithm
   * Function to compute various rotational parameters.
     This function sets the desired angle, angular rate and angular acceleration for yaw,
     computes the inverse of the Jacobian matrix, the derivative of the Euler angles,
     and the rotation matrix to go from global to local coordinates.
   * @param None
   */
  void computeControlAlgorithm();

private:
  
  StateController state_;
  const double time_step_rk4_ = 0.01; // [s] time step for integrating using RK4

  GainsMRAC gains_;

  ControllerSpecificInternalMembers csim_;

  // Create a pointer to the LogData instance
  std::shared_ptr<LogData_MRAC> log_data_;

  // Controller's name. Keep this equal to the class name to avoid ambiguity
  static inline const std::string controller_name_ = "MRAC";

};



#endif // MRAC_HPP