/* pixhawk_actuator_motors.cpp

Mattia Gramuglia
April 9, 2024
*/

#include "multi_threaded_node.hpp"

/*
  Callback function that is executed at a fixed specified rate defined by the "timer_actuator_motors_"
*/
void MultiThreadedNode::publisher_actuator_motors_callback()
{
  // Extract current thread
  auto thread_string = "THREAD " + string_thread_id();
	std::cout << "\n\n";
  std::cout << "PUBLISHER " << thread_string << std::endl;

  // Arm after ARM_START_TIME_SECONDS 
  if (time_current_ > ARM_START_TIME_SECONDS && offboard_flag_ == 0)
	{
    // Change to Offboard mode
    this->publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);

    // Arm the vehicle
    this->arm();
    
    offboard_flag_ = 1;
  }

	// Publish control input to the motors after TAKEOFF_START_TIME_SECONDS 
//  if (time_current_ > TAKEOFF_START_TIME_SECONDS && offboard_flag_ == 1)
//	{
		if (PUBLISH_ACTUATOR_MOTORS_FLAG)
		{
			// offboard_control_mode needs to be paired with actuator_motors commands
			publish_offboard_control_mode();
			publish_actuator_motors();
		}
    
//  }
  
  // Disarm after DISARM_START_TIME_SECONDS
  if (time_current_ > DISARM_START_TIME_SECONDS && offboard_flag_ == 1)
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
	
	msg.control[0] = this->getControl()->getControlInternalMembers().thrust_vector_quadcopter_normalized[0];
	msg.control[1] = this->getControl()->getControlInternalMembers().thrust_vector_quadcopter_normalized[1];
	msg.control[2] = this->getControl()->getControlInternalMembers().thrust_vector_quadcopter_normalized[2];
	msg.control[3] = this->getControl()->getControlInternalMembers().thrust_vector_quadcopter_normalized[3];
	msg.control[4] = 0.0;
	msg.control[5] = 0.0;
	msg.control[6] = 0.0;
	msg.control[7] = 0.0;
	
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;

	msg.reversible_flags = 0;
	
	/*
  std::cout << "\n\n";
  std::cout << "PUBLISHED ACTUATOR MOTORS"   << std::endl;
  std::cout << "=========================="  << std::endl;
	std::cout << "publish_actuator_motors() time_current_: " << time_current_ << " s" << std::endl;
	std::cout << "publish_actuator_motors() motor_1      : " << msg.control[0] << std::endl;
	std::cout << "publish_actuator_motors() motor_2      : " << msg.control[1] << std::endl;
	std::cout << "publish_actuator_motors() motor_3      : " << msg.control[2] << std::endl;
	std::cout << "publish_actuator_motors() motor_4      : " << msg.control[3] << "\n\n";
	*/
	
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




