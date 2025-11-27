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
 * File:        funnel_two_layer_mrac.hpp
 * Author:      Mattia Gramuglia
 * Date:        December 04, 2024
 * For info:    Andrea L'Afflitto 
 *              a.lafflitto@vt.edu
 * 
 * Description: Implementation of the Funnel Two-Layer MRAC controller.
 * 
 * GitHub:    https://github.com/andrealaffly/ACSL-flightstack.git
 **********************************************************************************************************************/

#pragma once

#include <cmath>
#include <algorithm>
#include <string> 

#include <Eigen/Dense>

#include "control.hpp"
#include "funnel_two_layer_mrac_gains.hpp"
#include "continuous_lyapunov_equation.hpp"
#include "projection_operator.hpp"
#include "helpers.hpp"
#include "base_mrac.hpp"
#include "flightstack/utils/low_pass_filter.hpp"

// Forward declaration of LogData_FunnelTwoLayerMRAC class
class LogData_FunnelTwoLayerMRAC;

class FunnelTwoLayerMRAC : public Control
{
public:

  /*
    Integration error estimated by the boost integrator
  */
  struct ErrorIntegrator 
    {
      // Constructor
      ErrorIntegrator();

      std::vector<double> xerr; 
    };

  /*
    This struct represents the state of the FunnelTwoLayerMRAC controller differential equations that will be integrated
    at each RK4 iteration.
    IT IS NOT THE VEHICLE STATE (e.g. POSITION, VELOCITY, etc.).
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
    Eigen::Map<Eigen::Matrix<double, 6, 3>> K_hat_g_translational; // Two-Layer
    Eigen::Map<Eigen::Matrix<double, 3, 3>> K_hat_g_rotational; // Two-Layer
    Eigen::Map<Eigen::Matrix<double, 1, 1>> eta_funnel_translational; // Funnel

    /*
      Function used to set the initial conditions for the eta_funnel_translational variable of the StateController
    */
    void setInitialCondition_eta_funnel_translational(double value)
    {
      eta_funnel_translational << value;
    }
  };

  /*
    This struct contains the variables that are used in this particular controller
  */
  struct ControllerSpecificInternalMembers 
  {
    // Constructor
    ControllerSpecificInternalMembers();

    Eigen::Matrix<double, 3, 1> translational_position_error_ref; // [position] refmod - user
    Eigen::Matrix<double, 6, 1> e_previous_translational; // Translational tracking error at previous timestep
    Eigen::Matrix<double, 6, 1> e_dot_translational; // Translational tracking error derivative
    Eigen::Matrix<double, 3, 1> r_cmd_translational; // Reference command input
    Eigen::Matrix<double, 6, 1> x_ref_dot_translational; // Translational reference model
    Eigen::Matrix<double, 3, 1> mu_PID_baseline_translational;
    Eigen::Matrix<double, 6, 1> augmented_regressor_vector_translational;
    Eigen::Matrix<double, 6, 3> K_hat_x_dot_translational;
    Eigen::Matrix<double, 3, 3> K_hat_r_dot_translational; 
    Eigen::Matrix<double, 6, 3> Theta_hat_dot_translational;
    Eigen::Matrix<double, 6, 3> K_hat_g_dot_translational; // Two-Layer
    Eigen::Matrix<double, 3, 1> mu_adaptive_translational;
    double dead_zone_value_translational;
    bool proj_op_activated_K_hat_x_translational;
    bool proj_op_activated_K_hat_r_translational;
    bool proj_op_activated_Theta_hat_translational;
    bool proj_op_activated_K_hat_g_translational;
    Eigen::Matrix<double, 1, 1> eta_dot_funnel_translational; // ODE used to compute the dynamics of the translational funnel diameter
    double H_function_funnel_translational;
    double Ve_funnel_translational;
    double xi_funnel_translational;
    double lambda_sat_funnel_translational;
    double sigma_ideal_funnel_translational;
    double sigma_nom_funnel_translational;
    int case_eta_dot_funnel_translational;

    Eigen::Vector3d outer_loop_P; // Proportional part of the baseline control input of the outer loop
    Eigen::Vector3d outer_loop_I; // Integral part of the baseline control input of the outer loop
    Eigen::Vector3d outer_loop_D; // Derivative part of the baseline control input of the outer loop
    Eigen::Vector3d outer_loop_D_filtered; // Derivative part of the baseline control input of the outer loop, filtered
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
    Eigen::Matrix<double, 3, 3> K_hat_g_dot_rotational; // Two-Layer
    Eigen::Vector3d tau_adaptive_rotational;
    double dead_zone_value_rotational;
    bool proj_op_activated_K_hat_x_rotational;
    bool proj_op_activated_K_hat_r_rotational;
    bool proj_op_activated_Theta_hat_rotational;
    bool proj_op_activated_K_hat_g_rotational;
    
    Eigen::Vector3d inner_loop_P; // Proportional part of the baseline control input of the inner loop
    Eigen::Vector3d inner_loop_I; // Integral part of the baseline control input of the inner loop
    Eigen::Vector3d inner_loop_D; // Derivative part of the baseline control input of the inner loop
    Eigen::Vector3d inner_loop_dynamic_inversion; // Dynamic inversion term of the baseline control input of the inner loop

  };

  // Constructor
  FunnelTwoLayerMRAC(MultiThreadedNode& node);

  // Getter functions
  ErrorIntegrator& getErrorIntegrator();
  StateController& getStateController();
  const double& getTimeStepRK4() const; 
  const GainsFunnelTwoLayerMRAC& getGains() const;
  const ControllerSpecificInternalMembers& getControllerSpecificInternalMembers() const;
  std::shared_ptr<LogData_FunnelTwoLayerMRAC> getLogData() const;
  static const std::string& getControllerName();

  void readJSONfile(const std::string& fileName);
  void readFilterJSONfile(const std::string& fileName);

  void initializeControllerParameters(VehicleInfo& vehicle_info, GainsFunnelTwoLayerMRAC& gains_);

  void checkParametersFunnelTranslational(GainsFunnelTwoLayerMRAC& gains);

  void writeDerivedGainsToJsonFile();

  void assignSystemToDxdt(const state_type /* &x */, state_type &dxdt, const double /* t */);

  void computeFilterDifferentiatorVariables(ControlInternalMembers& cim, 
                                            VehicleInfo& vehicle_info, 
                                            StateController& state_);

  void computeOuterLoop(ControlInternalMembers& cim,
                        VehicleInfo& vehicle_info,
                        StateController& state_, 
                        ControlReferences& cr,
                        GainsFunnelTwoLayerMRAC& gains_,
                        ControllerSpecificInternalMembers& csim_);

  void computeInnerLoop(ControlInternalMembers& cim,
                        VehicleInfo& vehicle_info,
                        StateController& state_, 
                        ControlReferences& cr,
                        GainsFunnelTwoLayerMRAC& gains_,
                        ControllerSpecificInternalMembers& csim_);

  void computeControlAlgorithm();

private:

  ErrorIntegrator error_integrator_;

  StateController state_;
  const double time_step_rk4_ = 0.01; // [s] time step for integrating using RK4

  GainsFunnelTwoLayerMRAC gains_;

  ControllerSpecificInternalMembers csim_;

  // LowPassFilter::FilterVector3d<LowPassFilter::FirstOrder> outer_loop_D_filter_;
  LowPassFilter::FilterVector3d<LowPassFilter::SecondOrder> outer_loop_D_filter_;

  // Create a pointer to the LogData instance
  std::shared_ptr<LogData_FunnelTwoLayerMRAC> log_data_;

  // Controller's name. Keep this equal to the class name to avoid ambiguity
  static inline const std::string controller_name_ = "FunnelTwoLayerMRAC";

};