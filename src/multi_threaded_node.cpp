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
 * File:        multi_threaded_node.cpp
 * Author:      Mattia Gramuglia
 * Date:        April 9, 2024
 * For info:    Andrea L'Afflitto 
 *              a.lafflitto@vt.edu
 * 
 * Description: Multithreaded ROS2 node that joins together all the flight stack components
 * 
 * GitHub:    https://github.com/andrealaffly/ACSL-flightstack.git
 **********************************************************************************************************************/

#include "multi_threaded_node.hpp"

MultiThreadedNode::MultiThreadedNode(rclcpp::NodeOptions options)
: Node("MultiThreadedNode", options),
  config_(ConfigurationParameters::readConfigurationParametersFile("config.json")),
  vehicle_state_(std::make_shared<VehicleState>(*this)),
  timestamp_initial_(this->get_clock()->now().nanoseconds() / 1000),
  counter_time_current_(0),
  offboard_flag_(0),
  user_defined_trajectory_(std::make_shared<PiecewisePolynomialTrajectory>(*this)),
  control_(std::make_shared<ControlType>(*this)),
  global_params_(GlobalParameters(*this))
{
  std::cout << "SELECTED CONTROLLER: " << ControlType::getControllerName() << std::endl;
  
  std::cout << "timestamp_initial_: " << this->timestamp_initial_ << std::endl;

  // Perform the setup of the Pixhawk vehicle odometry subscription callback/thread
  setupPixhawkOdometrySubscriber();

  // Perform the setup of the Pixhawk actuator motors publisher callback/thread
  setupPixhawkActuatorMotorsPublisher();
}

/*
 A small convenience function for converting a thread ID to a string
*/
std::string string_thread_id()
{
  auto hashed = std::hash<std::thread::id>()(std::this_thread::get_id());
  return std::to_string(hashed);
}

/*
  Function that when called updates the PRIVATE variable "time_current_" to the current time
*/
void MultiThreadedNode::updateCurrentTimeInSeconds()
{
  time_current_ = ((this->get_clock()->now().nanoseconds() / 1000) - timestamp_initial_) / 1e6;
}

// Getter for timestamp_initial
const std::atomic<uint64_t>& MultiThreadedNode::getInitialTimestamp() const {
  return timestamp_initial_;
}

// Getter for time_current_
const std::atomic<double>& MultiThreadedNode::getCurrentTime() const {
  return time_current_;
}

// Getter function for user_defined_trajectory_ (FOR PiecewisePolynomialTrajectory)!!!
std::shared_ptr<PiecewisePolynomialTrajectory> MultiThreadedNode::getUserDefinedTrajectory() const {
  return user_defined_trajectory_;
}

// Getter function for vehicle_state_ 
std::shared_ptr<VehicleState> MultiThreadedNode::getVehicleState() const {
  return vehicle_state_;
}

// Getter function for control_ 
std::shared_ptr<ControlType> MultiThreadedNode::getControl() const {
  return control_;
}

// Getter function for config_
ConfigurationParameters MultiThreadedNode::getConfigurationParameters() const {
  return config_;
}

// Getter function for global_params_
GlobalParameters MultiThreadedNode::getGlobalParameters() const {
  return global_params_;
}

// Function that performs the setup of the Pixhawk vehicle odometry subscription callback/thread
void MultiThreadedNode::setupPixhawkOdometrySubscriber()
{
  // Pixhawk vehicle odometry subscription callback group
  callback_group_sub_pixhawk_odometry_ = create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive);

  // Each of the callback groups is basically a thread
  // Everything assigned to one of them gets bundled into the same thread
  auto sub_pixhawk_odometry_opt = rclcpp::SubscriptionOptions();
  sub_pixhawk_odometry_opt.callback_group = callback_group_sub_pixhawk_odometry_;

  // Quality Of Service initialization for "subscription_pixhawk_odometry_"
  rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
  auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);

  // Creation of the Pixhawk vehicle odometry subscription
  subscription_pixhawk_odometry_ = create_subscription<px4_msgs::msg::VehicleOdometry>(
    "/fmu/out/vehicle_odometry",
    qos,
    std::bind(
      &MultiThreadedNode::subscriber_pixhawk_odometry_callback, // First parameter is a reference to the function
      this,                                               // What the function should be bound to
      std::placeholders::_1),                             // At this point we're not positive of all the
                                                          // parameters being passed
                                                          // So we just put a generic placeholder
                                                          // into the binder
                                                          // (since we know we need ONE parameter)
    sub_pixhawk_odometry_opt);                            // This is where we set the callback group.
                                                          // This subscription will run with callback group subscriber1

}

// Function that performs the setup of the Pixhawk actuator motors publisher callback/thread
void MultiThreadedNode::setupPixhawkActuatorMotorsPublisher()
{
  // Actuator motors publisher callback group
  callback_group_pub_actuator_motors_ = create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive);

  // Each of the callback groups is basically a thread
  // Everything assigned to one of them gets bundled into the same thread
  auto pub_actuator_motors_opt = rclcpp::PublisherOptions();
  pub_actuator_motors_opt.callback_group = callback_group_pub_actuator_motors_;

  // Creation of the actuator motors publisher
  publisher_actuator_motors_ = create_publisher<px4_msgs::msg::ActuatorMotors>(
    "/fmu/in/actuator_motors",
    10);

  // Creation of the vehicle command publisher
  publisher_vehicle_command_ = create_publisher<px4_msgs::msg::VehicleCommand>(
    "/fmu/in/vehicle_command",
    10);

  // Creation of the offboard control mode publisher
  publisher_offboard_control_mode_ = create_publisher<px4_msgs::msg::OffboardControlMode>(
    "/fmu/in/offboard_control_mode",
    10);
  
  // Creation of the timer that periodically calls the execution of the "publisher_actuator_motors_callback"
  timer_actuator_motors_ = create_wall_timer(
    10ms,
    std::bind(
      &MultiThreadedNode::controller_and_publisher_actuator_motors_callback,
      this));

}

/*
  Function that sequentially calls the controller and then publishes the control input to the motors
*/
void MultiThreadedNode::controller_and_publisher_actuator_motors_callback()
{
  MultiThreadedNode::controller_callback();
  MultiThreadedNode::publisher_actuator_motors_callback();
}


