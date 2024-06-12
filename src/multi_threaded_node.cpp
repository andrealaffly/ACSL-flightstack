/* multi_threaded_node.cpp

Mattia Gramuglia
April 9, 2024
*/

#include "multi_threaded_node.hpp"

MultiThreadedNode::MultiThreadedNode(rclcpp::NodeOptions options)
: Node("MultiThreadedNode", options),
  offboard_flag_(0),
  config_(readConfigurationParametersFile("config.json"))
{

  vehicle_state_ = std::make_shared<VehicleState>(*this);
  user_defined_trajectory_ = std::make_shared<PiecewisePolynomialTrajectory>(*this);
  control_ = std::make_shared<PID>(*this);

  // Initialize the Global Parameters
  this->global_params_ = GlobalParameters(*this);

  timestamp_initial_ = this->get_clock()->now().nanoseconds() / 1000; 
  std::cout << "timestamp_initial_ :" << timestamp_initial_ << std::endl;

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

// Getter function for control_ (FOR PID) 
std::shared_ptr<PID> MultiThreadedNode::getControl() const {
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


