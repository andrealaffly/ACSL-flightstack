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
 * File:        pixhawk_actuator_motors.cpp
 * Author:      Mattia Gramuglia
 * Date:        April 9, 2024
 * For info:    Andrea L'Afflitto 
 *              a.lafflitto@vt.edu
 * 
 * Description: Callback that is executed at a fixed specified rate defined by the "timer_controller_".
 * 							Here are the functions needed to publish to Pixhawk the actuators control inputs.
 * 
 * GitHub:    https://github.com/andrealaffly/ACSL-flightstack.git
 **********************************************************************************************************************/

#include "multi_threaded_node.hpp"

/*
  Callback function that is executed at a fixed specified rate defined by the "timer_actuator_motors_"
*/
void MultiThreadedNode::publisher_actuator_motors_callback()
{
  /* // Extract current thread
  auto thread_string = "THREAD " + string_thread_id();
	std::cout << "\n\n";
  std::cout << "PUBLISHER " << thread_string << std::endl;
	*/

  // Arm after ARM_START_TIME_SECONDS 
  if (time_current_ > global_params_.ARM_START_TIME_SECONDS && offboard_flag_ == 0)
	{
    // Change to Offboard mode
    this->goIntoOffboardMode();

    // Arm the vehicle
    this->arm();
    
    offboard_flag_ = 1;
  }

	// Publish control input to the motors
	if constexpr (config_param::PUBLISH_ACTUATOR_MOTORS_FLAG)
	{
		// offboard_control_mode needs to be paired with actuator_motors commands
		publish_offboard_control_mode();
		publish_actuator_motors();
	}
    
  // Disarm after DISARM_START_TIME_SECONDS
  if (time_current_ > global_params_.DISARM_START_TIME_SECONDS && offboard_flag_ == 1)
	{
		// Disarm the vehicle
    this->disarm();
    
    offboard_flag_ = 2;
  }

}

/*
  Send a command to Arm the vehicle
*/
void MultiThreadedNode::arm()
{
	publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);

	RCLCPP_INFO(this->get_logger(), "Arm command send");
}

/*
  Send a command to Disarm the vehicle
*/
void MultiThreadedNode::disarm()
{
	publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0);

	RCLCPP_INFO(this->get_logger(), "Disarm command send");
}

/*
  Publish the offboard control mode
*/
void MultiThreadedNode::publish_offboard_control_mode()
{
	px4_msgs::msg::OffboardControlMode msg{};
	msg.position = false;
	msg.velocity = false;
	msg.acceleration = false;
	msg.attitude = false;
	msg.body_rate = false;
	msg.thrust_and_torque = false;
	msg.direct_actuator = true;
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	
	publisher_offboard_control_mode_->publish(msg);
}

/*
  Publish a actuator_motors command
*/
void MultiThreadedNode::publish_actuator_motors()
{

	px4_msgs::msg::ActuatorMotors msg{};

	constexpr int thrust_vector_normalized_rows = 
			decltype(Control::ControlInternalMembers::thrust_vector_normalized)::RowsAtCompileTime;

	if (time_current_ < global_params_.TAKEOFF_START_TIME_SECONDS || 
		  time_current_ > global_params_.DISARM_START_TIME_SECONDS) 
	{
		for (int i = 0; i < thrust_vector_normalized_rows; i++)
		{
			// X8-COPTER
			msg.control[i] = MINIMUM_VALUE_PUBLISH_MOTORS;
		}
	}
	else
	{
		if constexpr (config_param::ENABLE_MOTOR_FAILURE)
		{
			Eigen::Matrix<double, 8, 1> motor_efficiency;

			if (time_current_ > global_params_.MOTOR_FAILURE_START_TIME_SECONDS &&
					time_current_ < global_params_.MOTOR_FAILURE_END_TIME_SECONDS)
			{
				motor_efficiency = config_.motor_failure_efficiencies;
			}
			else
			{
				motor_efficiency = Eigen::Matrix<double, 8, 1>::Ones();
			}

			for (int i = 0; i < thrust_vector_normalized_rows; i++)
			{
				// X8-COPTER
				msg.control[i] = this->getControl()->getControlInternalMembers().thrust_vector_normalized[i] * motor_efficiency[i];
			}
		}
		else
		{
			for (int i = 0; i < thrust_vector_normalized_rows; i++)
			{
				// X8-COPTER
				msg.control[i] = this->getControl()->getControlInternalMembers().thrust_vector_normalized[i];
			}
		}

		/* // QUADCOPTER
		msg.control[0] = this->getControl()->getControlInternalMembers().thrust_vector_quadcopter_normalized[0];
		msg.control[1] = this->getControl()->getControlInternalMembers().thrust_vector_quadcopter_normalized[1];
		msg.control[2] = this->getControl()->getControlInternalMembers().thrust_vector_quadcopter_normalized[2];
		msg.control[3] = this->getControl()->getControlInternalMembers().thrust_vector_quadcopter_normalized[3];
		msg.control[4] = 0.0;
		msg.control[5] = 0.0;
		msg.control[6] = 0.0;
		msg.control[7] = 0.0;
		*/
	}
	
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;

	msg.reversible_flags = 0;
	
	publisher_actuator_motors_->publish(msg);
}

/*
  Publish vehicle commands
  Parameters:
    command   Command code (matches VehicleCommand and MAVLink MAV_CMD codes)
    param1    Command parameter 1
    param2    Command parameter 2
*/
void MultiThreadedNode::publish_vehicle_command(uint16_t command, float param1, float param2)
{
	px4_msgs::msg::VehicleCommand msg{};
	msg.param1 = param1;
	msg.param2 = param2;
	msg.command = command;
	msg.target_system = 1;
	msg.target_component = 1;
	msg.source_system = 1;
	msg.source_component = 1;
	msg.from_external = true;
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	publisher_vehicle_command_->publish(msg);
}

// Change to Offboard mode
void MultiThreadedNode::goIntoOffboardMode()
{
	// Change to Offboard mode
	this->publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);

	RCLCPP_INFO(this->get_logger(), "Go into Offboard mode command send");
}




