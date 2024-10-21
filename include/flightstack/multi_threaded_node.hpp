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
 * File:        multi_threaded_node.hpp
 * Author:      Mattia Gramuglia
 * Date:        April 8, 2024
 * For info:    Andrea L'Afflitto 
 *              a.lafflitto@vt.edu
 * 
 * Description: Multithreaded ROS2 node that joins together all the flight stack components
 * 
 * GitHub:    https://github.com/andrealaffly/ACSL-flightstack.git
 **********************************************************************************************************************/
/**
 * @file multi_threaded_node.hpp
 * @brief Multithreaded ROS2 node that joins together all the flight stack components
 */
#ifndef MULTI_THREADED_NODE_HPP
#define MULTI_THREADED_NODE_HPP

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <thread>
#include <stdint.h>
#include <fstream>
#include <iostream>
#include <vector>
#include <typeinfo>

#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <px4_msgs/msg/actuator_motors.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>

#include "config.hpp"
#include "json_parser.hpp"
#include "vehicle_state.hpp"
#include "user_defined_trajectory.hpp"
#include "piecewise_polynomial_trajectory.hpp"
#include "control.hpp"
#include "mocap.hpp"
#include "print_helpers.hpp"


using namespace std::chrono_literals;

/**
 * @class MultiThreadedNode
 * Multithreaded Node Class
 */
class MultiThreadedNode : public rclcpp::Node
{
public:
/**
 * @param options 
 */
  MultiThreadedNode(rclcpp::NodeOptions options);
  /**
  * @brief Sends a commnand to arm the vehicle
  * @param None
  */
  void arm();

  /**
   * @brief Sends a command to disarm the vehicle
   * @param None
   */
	void disarm();

  /**
   * @brief Publish the offboard control mode
   * 
   */
  void updateCurrentTimeInSeconds();

  const std::atomic<uint64_t>& getInitialTimestamp() const;
  const std::atomic<double>& getCurrentTime() const;
  std::shared_ptr<PiecewisePolynomialTrajectory> getUserDefinedTrajectory() const;
  std::shared_ptr<VehicleState> getVehicleState() const;
  std::shared_ptr<ControlType> getControl() const;
  ConfigurationParameters getConfigurationParameters() const;
  GlobalParameters getGlobalParameters() const;

private:

  // Instance of the ConfigurationParameters struct that contains the configuration parameters coming from config.json
  ConfigurationParameters config_;

  /*********************************************************************************************************************
    CALLBACK GROUP 1
   *********************************************************************************************************************
  */
  // Pixhawk vehicle odometry subscription callback/thread
  rclcpp::CallbackGroup::SharedPtr callback_group_sub_pixhawk_odometry_;
  rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr subscription_pixhawk_odometry_;

  /**
   * @brief Callback function that is executed everytime a VehicleOdometry message is made available by Pixhawk
   * @param msg
   */
  void subscriber_pixhawk_odometry_callback(const px4_msgs::msg::VehicleOdometry::UniquePtr msg);

  // Create a VehicleState instance
  std::shared_ptr<VehicleState>  vehicle_state_;

  /**
   * @brief Callback Group 2
   * @param None
   */
  void setupPixhawkOdometrySubscriber();

  /*********************************************************************************************************************
    CALLBACK GROUP 2
   *********************************************************************************************************************
  */
  // Pixhawk actuator motors publisher callback/thread
  rclcpp::CallbackGroup::SharedPtr callback_group_pub_actuator_motors_;
  rclcpp::Publisher<px4_msgs::msg::ActuatorMotors>::SharedPtr publisher_actuator_motors_;
  rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr publisher_vehicle_command_;
  rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr publisher_offboard_control_mode_;
  rclcpp::TimerBase::SharedPtr timer_actuator_motors_;

  /**
  * @brief Callback function that is executed at a fixed specified rate defined by the "timer_actuator_motors_"
  * @param None
  * @see void MultiThreadedNode::publisher_actuator_motors_callback()
  */
  void publisher_actuator_motors_callback();

  /**
   * @brief Publish the offboard control mode
   * @param None
   */
  void publish_offboard_control_mode();

  /**
   * @brief Publish a actuator_motors command
   * @param NOne
   */
	void publish_actuator_motors();

  /**
   * @param command 
   * @param param1 
   * @param param2 
   */
	void publish_vehicle_command(uint16_t command, float param1 = 0.0, float param2 = 0.0);

  /**
   * @brief Change into OffboardMode
   * @param NOne
   */
  inline void goIntoOffboardMode();

  std::atomic<uint64_t> timestamp_initial_;   //common synced initial timestamp

	std::atomic<double> time_current_; // current time

  /*
    flag for the arm/disarm logic:
    0 = flight not started yet
    1 = flight in progress
    2 = flight terminated
  */
  int offboard_flag_; 

  // Create a pointer to the PiecewisePolynomialTrajectory instance
  std::shared_ptr<PiecewisePolynomialTrajectory> user_defined_trajectory_;

  /**
   * @param None
   */
  void setupPixhawkActuatorMotorsPublisher();

  /**
   * @brief Callback function that is executed at a fixed specified rate defined by the "timer_controller_"
   * @param None
   */
  void controller_callback();

  // Create a pointer to the Control instance
  std::shared_ptr<ControlType> control_;

  void controller_and_publisher_actuator_motors_callback();

  /*********************************************************************************************************************
  OTHER
  *********************************************************************************************************************
  */
  // Instance of the GlobalParameters struct that contains the global parameters of the flightstack
  GlobalParameters global_params_;

};
/**
 * @class MultiThreadedNode multi_threaded_node.h "inc/multi_threaded_node.h"
 * @brief Multithreaded Node Class
 */

std::string string_thread_id();

#endif // MULTI_THREADED_NODE_HPP
