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
 * File:        control_callback.cpp
 * Author:      Mattia Gramuglia
 * Date:        April 19, 2024
 * For info:    Andrea L'Afflitto 
 *              a.lafflitto@vt.edu
 * 
 * Description: Callback that is executed at a fixed specified rate defined by the "timer_controller_".
 *              Here are the actions that need to be performed at every controller iteration.
 * 
 * GitHub:    https://github.com/andrealaffly/ACSL-flightstack.git
 **********************************************************************************************************************/
/**
 * @file control_callback.cpp
 * @brief Callback that is executed at a fixed specified rate defined by the "timer_controller_".
 */
#include "control.hpp"
#include "multi_threaded_node.hpp"


/**
 * @class MultiThreadedNode
 */
void MultiThreadedNode::controller_callback()
{
  /* // Extract current thread
  auto thread_string = "THREAD " + string_thread_id();
  std::cout << "\n\n";
  std::cout << "CONTROLLER " << thread_string << std::endl;
 */
  this->updateCurrentTimeInSeconds();

  // Print current time to the console
  helper::FLIGHTSTACK_INFO_STREAM_NO_TAG(this->time_current_, 100);
  
  user_defined_trajectory_->updateUserDefinedTrajectory(this->time_current_);
	user_defined_trajectory_->updateUserDefinedYaw();

  // Using std::bind to create a function object
  auto controllerBind = std::bind(&ControlType::assignSystemToDxdt,
                                    control_,
                                    std::placeholders::_1,
                                    std::placeholders::_2,
                                    std::placeholders::_3);

  if (time_current_ > global_params_.TAKEOFF_START_TIME_SECONDS)
  {
  // std::cout << "Starting DO_STEP " << std::endl;
  // Start time of the RK4 integration/Control algorithm
  auto start_time_rk4_integration = std::chrono::high_resolution_clock::now();

  control_->computeControlAlgorithm();
  
  control_->stepper.do_step(controllerBind,
                            control_->getStateController().full_state,
                            this->time_current_,
                            control_->getTimeStepRK4());

  // End time of the RK4 integration/Control algorithm
  auto end_time_rk4_integration = std::chrono::high_resolution_clock::now();
  // Updating the execution time of the RK4 integration/Control algorithm
  auto duration_time_rk4_integration = std::chrono::duration_cast<std::chrono::microseconds>(
    end_time_rk4_integration - start_time_rk4_integration
  );
  control_->setAlgorithmExecutionTimeMicroseconds(duration_time_rk4_integration);
  // std::cout << "algorithm_execution_time_microseconds_ " << control_->getAlgorithmExecutionTimeMicroseconds().count() << std::endl;
  // std::cout << "Ending DO_STEP " << std::endl;
  }


  // DEBUGGINGGGGGGGG
  // std::cout << "K_hat_r_translational " << control_->getStateController().K_hat_r_translational << std::endl;
  // std::cout << "thrust_vector_quadcopter " << control_->getControlInternalMembers().thrust_vector_quadcopter << std::endl;
  // std::cout << "thrust_vector_quadcopter_normalized " << control_->getControlInternalMembers().thrust_vector_quadcopter_normalized << std::endl;
  // std::cout << "mass " << control_->getVehicleInfo().mass << std::endl;
  // std::cout << "inertia_matrix " << control_->getVehicleInfo().inertia_matrix << std::endl;
  // std::cout << "drag_coefficient_matrix " << control_->getVehicleInfo().drag_coefficient_matrix << std::endl;
  // std::cout << "r_cmd_translational " << control_->getControllerSpecificInternalMembers().r_cmd_translational << std::endl;
  // std::cout << "A_filter_roll_des " << control_->getVehicleInfo().A_filter_roll_des << std::endl;
  // std::cout << "B_filter_roll_des " << control_->getVehicleInfo().B_filter_roll_des << std::endl;
  // std::cout << "C_filter_roll_des " << control_->getVehicleInfo().C_filter_roll_des << std::endl;
  // std::cout << "D_filter_roll_des " << control_->getVehicleInfo().D_filter_roll_des << std::endl;

  // std::cout << "state_roll_des_filter " << control_->getStateController().state_roll_des_filter << std::endl;
  // std::cout << "KP_translational " << control_->getGains().KP_translational << std::endl;
  // std::cout << "KD_translational " << control_->getGains().KD_translational << std::endl;
  // std::cout << "KI_translational " << control_->getGains().KI_translational << std::endl;
  // std::cout << "KP_rotational " << control_->getGains().KP_rotational << std::endl;
  // std::cout << "KD_rotational " << control_->getGains().KD_rotational << std::endl;
  // std::cout << "KI_rotational " << control_->getGains().KI_rotational << std::endl;
  // std::cout << "KP_refmod_translational " << control_->getGains().KP_refmod_translational << std::endl;
  // std::cout << "KD_refmod_translational " << control_->getGains().KD_refmod_translational << std::endl;
  // std::cout << "KI_refmod_translational " << control_->getGains().KI_refmod_translational << std::endl;
  // std::cout << "KP_omega_cmd_rotational " << control_->getGains().KP_omega_cmd_rotational << std::endl;
  // std::cout << "KP_omega_ref_rotational " << control_->getGains().KP_omega_ref_rotational << std::endl;
  // std::cout << "Gamma_x_translational " << control_->getGains().Gamma_x_translational << std::endl;
  // std::cout << "Gamma_r_translational " << control_->getGains().Gamma_r_translational << std::endl;
  // std::cout << "Gamma_Theta_translational " << control_->getGains().Gamma_Theta_translational << std::endl;
  // std::cout << "Q_translational " << control_->getGains().Q_translational << std::endl;
  // std::cout << "P_translational " << control_->getGains().P_translational << std::endl;
  // std::cout << "Gamma_x_rotational " << control_->getGains().Gamma_x_rotational << std::endl;
  // std::cout << "Gamma_r_rotational " << control_->getGains().Gamma_r_rotational << std::endl;
  // std::cout << "Gamma_Theta_rotational " << control_->getGains().Gamma_Theta_rotational << std::endl;
  // std::cout << "Q_rotational " << control_->getGains().Q_rotational << std::endl;
  // std::cout << "P_rotational " << control_->getGains().P_rotational << std::endl;
  // std::cout << "A_translational " << control_->getGains().A_translational << std::endl;
  // std::cout << "B_translational " << control_->getGains().B_translational << std::endl;
  // std::cout << "A_ref_translational " << control_->getGains().A_ref_translational << std::endl;
  // std::cout << "B_ref_translational " << control_->getGains().B_ref_translational << std::endl;
  // std::cout << "A_rotational " << control_->getGains().A_rotational << std::endl;
  // std::cout << "B_rotational " << control_->getGains().B_rotational << std::endl;
  // std::cout << "A_ref_rotational " << control_->getGains().A_ref_rotational << std::endl;
  // std::cout << "B_ref_rotational " << control_->getGains().B_ref_rotational << std::endl;

  // Logging data
  control_->getLogData()->logLogData();

}
