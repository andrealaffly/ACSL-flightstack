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
 * File:        pid.hpp
 * Author:      Mattia Gramuglia
 * Date:        April 22, 2024
 * For info:    Andrea L'Afflitto 
 *              a.lafflitto@vt.edu
 * 
 * Description: Implementation of the PID controller.
 * 
 * GitHub:    https://github.com/andrealaffly/ACSL-flightstack.git
 **********************************************************************************************************************/

#ifndef PID_HPP
#define PID_HPP

#include <cmath>
#include <algorithm>
#include <string> 

#include <Eigen/Dense>

#include "control.hpp"
#include "pid_gains.hpp"

// Forward declaration of LogData_PID class
class LogData_PID;


class PID : public Control
{
public:

  /*
  This struct represents the state of the PID controller differential equations that will be integrated
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
    Eigen::Map<Eigen::Vector2d> state_roll_des_filter; 
    Eigen::Map<Eigen::Vector2d> state_pitch_des_filter; 
    Eigen::Map<Eigen::Vector2d> state_roll_dot_des_filter; 
    Eigen::Map<Eigen::Vector2d> state_pitch_dot_des_filter;
    Eigen::Map<Eigen::Vector3d> integral_translational_position_error; 
    Eigen::Map<Eigen::Vector3d> integral_angular_error; 

  };

  /*
    This struct contains the variables that are used in this particular controller
  */
  struct ControllerSpecificInternalMembers 
  {
    // Constructor
    ControllerSpecificInternalMembers();

    Eigen::Vector3d outer_loop_P; // Proportional part of the control input of the outer loop
    Eigen::Vector3d outer_loop_I; // Integral part of the control input of the outer loop
    Eigen::Vector3d outer_loop_D; // Derivative part of the control input of the outer loop
    Eigen::Vector3d outer_loop_dynamic_inversion; // Dynamic inversion term of the control input of the outer loop

    Eigen::Vector3d inner_loop_P; // Proportional part of the control input of the inner loop
    Eigen::Vector3d inner_loop_I; // Integral part of the control input of the inner loop
    Eigen::Vector3d inner_loop_D; // Derivative part of the control input of the inner loop
    Eigen::Vector3d inner_loop_dynamic_inversion; // Dynamic inversion term of the control input of the inner loop

  };

  // Constructor
  PID(MultiThreadedNode& node);

  // Getter functions
  StateController& getStateController();
  const double& getTimeStepRK4() const; 
  const GainsPID& getGains() const;
  const ControllerSpecificInternalMembers& getControllerSpecificInternalMembers() const;
  std::shared_ptr<LogData_PID> getLogData() const;
  static const std::string& getControllerName();

  void readJSONfile(const std::string& fileName);

  void assignSystemToDxdt(state_type /* &x */, state_type &dxdt, const double /* t */);

  void computeFilterDifferentiatorVariables(ControlInternalMembers& cim, 
                                            VehicleInfo& vehicle_info, 
                                            StateController& state_);

  void computeOuterLoop(ControlInternalMembers& cim,
                        VehicleInfo& vehicle_info,
                        StateController& state_, 
                        ControlReferences& cr,
                        GainsPID& gains_,
                        ControllerSpecificInternalMembers& csim_);

  void computeInnerLoop(ControlInternalMembers& cim,
                        VehicleInfo& vehicle_info,
                        StateController& state_, 
                        ControlReferences& cr,
                        GainsPID& gains_,
                        ControllerSpecificInternalMembers& csim_);

  void computeControlAlgorithm();

private:

  StateController state_;
  const double time_step_rk4_ = 0.01; // [s] time step for integrating using RK4

  GainsPID gains_;

  ControllerSpecificInternalMembers csim_;

  // Create a pointer to the LogData instance
  std::shared_ptr<LogData_PID> log_data_;

  // Controller's name. Keep this equal to the class name to avoid ambiguity
  static inline const std::string controller_name_ = "PID";

};



#endif // PID_HPP