/* control_callback.cpp

	Mattia Gramuglia
	April 19, 2024
*/


#include "control.hpp"
#include "multi_threaded_node.hpp"


/*
  Callback function that is executed at a fixed specified rate defined by the "timer_controller_"
*/
void MultiThreadedNode::controller_callback()
{
  // Extract current thread
  auto thread_string = "THREAD " + string_thread_id();
  std::cout << "\n\n";
  std::cout << "CONTROLLER " << thread_string << std::endl;

  updateCurrentTimeInSeconds();
  std::cout << "Current Time: " << this->time_current_ << " s" << std::endl;
  
  user_defined_trajectory_->updateUserDefinedTrajectory(this->time_current_);
	user_defined_trajectory_->updateUserDefinedYaw();

  // Using std::bind to create a function object
  auto controllerBind = std::bind(&PID::computeControlAlgorithm,
                                    control_,
                                    std::placeholders::_1,
                                    std::placeholders::_2,
                                    std::placeholders::_3);

  if (time_current_ > global_params_.TAKEOFF_START_TIME_SECONDS)
  {
  // std::cout << "Starting DO_STEP " << std::endl;
  // Start time of the RK4 integration/Control algorithm
  auto start_time_rk4_integration = std::chrono::high_resolution_clock::now();
  
  control_->stepper.do_step(controllerBind,
                            control_->getStatePID().full_state,
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
  // std::cout << "thrust_vector_quadcopter " << control_->getControlInternalMembers().thrust_vector_quadcopter << std::endl;
  // std::cout << "thrust_vector_quadcopter_normalized " << control_->getControlInternalMembers().thrust_vector_quadcopter_normalized << std::endl;
  // std::cout << "mass " << control_->getVehicleInfo().mass << std::endl;
  // std::cout << "inertia_matrix " << control_->getVehicleInfo().inertia_matrix << std::endl;
  // std::cout << "drag_coefficient_matrix " << control_->getVehicleInfo().drag_coefficient_matrix << std::endl;

  // std::cout << "state_roll_ref_filter " << control_->getStatePID().state_roll_ref_filter << std::endl;
  // std::cout << "KP_translational " << control_->getGainsPID().KP_translational << std::endl;
  // std::cout << "KD_translational " << control_->getGainsPID().KD_translational << std::endl;
  // std::cout << "KI_translational " << control_->getGainsPID().KI_translational << std::endl;
  // std::cout << "KP_rotational " << control_->getGainsPID().KP_rotational << std::endl;
  // std::cout << "KD_rotational " << control_->getGainsPID().KD_rotational << std::endl;
  // std::cout << "KI_rotational " << control_->getGainsPID().KI_rotational << std::endl;


  control_->getLogData()->logLogData(*(control_->getLogData()));

}
