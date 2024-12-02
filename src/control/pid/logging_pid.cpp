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
 * File:        logging_pid.cpp
 * Author:      Mattia Gramuglia
 * Date:        April 18, 2024
 * For info:    Andrea L'Afflitto 
 *              a.lafflitto@vt.edu
 * 
 * Description: Logger for the PID controller.
 * 
 * GitHub:    https://github.com/andrealaffly/ACSL-flightstack.git
 **********************************************************************************************************************/

#include "pid.hpp"
#include "logging_pid.hpp"
#include "multi_threaded_node.hpp"

// Define the logger for LogData
src::logger LogData_PID::logger_logdata;

// Constructor
LogData_PID::LogData_PID(MultiThreadedNode& node, PID& controller) :
  node_(node),
  controller_(controller)
{}

// Function to print headers
void LogData_PID::logInitializeHeaders()
{
	std::ostringstream oss;

	oss << "Initial timestamp, "
	    << "Current time [s], "
      << "User-defined position x [m], "
      << "User-defined position y [m], "
      << "User-defined position z [m], "
      << "User-defined velocity x [m/s], "
      << "User-defined velocity y [m/s], "
      << "User-defined velocity z [m/s], "
      << "User-defined acceleration x [m/s^2], "
      << "User-defined acceleration y [m/s^2], "
      << "User-defined acceleration z [m/s^2], "
      << "User-defined yaw [rad], "
      << "User-defined yaw_dot [rad/s], "
      << "User-defined yaw_dot_dot [rad/s^2], "
      << "Odometry time [s], "
      << "Position x [m], "
      << "Position y [m], "
      << "Position z [m], "
      << "Quaternion q0 [-], "
      << "Quaternion q1 [-], "
      << "Quaternion q2 [-], "
      << "Quaternion q3 [-], "
      << "Velocity x [m/s], "
      << "Velocity y [m/s], "
      << "Velocity z [m/s], "
      << "Angular velocity x [rad/s], "
      << "Angular velocity y [rad/s], "
      << "Angular velocity z [rad/s], "
      << "Roll [rad], "
      << "Pitch [rad], "
      << "Yaw [rad], "
      << "Algorithm execution time [us], "
      << "Mu translational raw x [N], "
      << "Mu translational raw y [N], "
      << "Mu translational raw z [N], "
      << "Mu translational x [N], "
      << "Mu translational y [N], "
      << "Mu translational z [N], "
      << "U control input U1 [N], "
      << "U control input U2 [Nm], "
      << "U control input U3 [Nm], "
      << "U control input U4 [Nm], "
      << "Roll desired [rad], "
      << "Pitch desired [rad], "
      << "Roll_dot desired [rad/s], "
      << "Pitch_dot desired [rad/s], "
      << "Roll_dot_dot desired [rad/s^2], "
      << "Pitch_dot_dot desired [rad/s^2], "
      << "Roll_dot [rad/s], "
      << "Pitch_dot [rad/s], "
      << "Yaw_dot [rad/s], "
      << "Thrust motors QUADCOPTER T1 [N], "
      << "Thrust motors QUADCOPTER T2 [N], "
      << "Thrust motors QUADCOPTER T3 [N], "
      << "Thrust motors QUADCOPTER T4 [N], "
      << "Thrust motors normalized QUADCOPTER T1 [-], "
      << "Thrust motors normalized QUADCOPTER T2 [-], "
      << "Thrust motors normalized QUADCOPTER T3 [-], "
      << "Thrust motors normalized QUADCOPTER T4 [-], "
      << "Thrust motors X8COPTER T1 [N], "
      << "Thrust motors X8COPTER T2 [N], "
      << "Thrust motors X8COPTER T3 [N], "
      << "Thrust motors X8COPTER T4 [N], "
      << "Thrust motors X8COPTER T5 [N], "
      << "Thrust motors X8COPTER T6 [N], "
      << "Thrust motors X8COPTER T7 [N], "
      << "Thrust motors X8COPTER T8 [N], "
      << "Thrust motors normalized X8COPTER T1 [-], "
      << "Thrust motors normalized X8COPTER T2 [-], "
      << "Thrust motors normalized X8COPTER T3 [-], "
      << "Thrust motors normalized X8COPTER T4 [-], "
      << "Thrust motors normalized X8COPTER T5 [-], "
      << "Thrust motors normalized X8COPTER T6 [-], "
      << "Thrust motors normalized X8COPTER T7 [-], "
      << "Thrust motors normalized X8COPTER T8 [-], "
      << "Roll error [rad], "
      << "Pitch error [rad], "
      << "Yaw error [rad], "
      << "Outer loop Proportional term x [-], "
      << "Outer loop Proportional term y [-], "
      << "Outer loop Proportional term z [-], "
      << "Outer loop Integral term x [-], "
      << "Outer loop Integral term y [-], "
      << "Outer loop Integral term z [-], "
      << "Outer loop Derivative term x [-], "
      << "Outer loop Derivative term y [-], "
      << "Outer loop Derivative term z [-], "
      << "Outer loop Dynamic Inversion term x [-], "
      << "Outer loop Dynamic Inversion term y [-], "
      << "Outer loop Dynamic Inversion term z [-], "
      << "Inner loop Proportional term x [-], "
      << "Inner loop Proportional term y [-], "
      << "Inner loop Proportional term z [-], "
      << "Inner loop Integral term x [-], "
      << "Inner loop Integral term y [-], "
      << "Inner loop Integral term z [-], "
      << "Inner loop Derivative term x [-], "
      << "Inner loop Derivative term y [-], "
      << "Inner loop Derivative term z [-], "
      << "Inner loop Dynamic Inversion term x [-], "
      << "Inner loop Dynamic Inversion term y [-], "
      << "Inner loop Dynamic Inversion term z [-], "
      
  ;
      
	BOOST_LOG(LogData_PID::logger_logdata) << oss.str();
}

// Function to initialize the logging system
void LogData_PID::logInitializeLogging()
{
  // Get the current date and time
  auto now = std::chrono::system_clock::now();
  std::time_t now_c = std::chrono::system_clock::to_time_t(now);

  // Get the current date in the desired format
  std::stringstream date_ss;
  date_ss << std::put_time(std::localtime(&now_c), "%Y%m%d"); // Current date
  std::string current_date = date_ss.str();

  // Create the directory path for the logs
  std::string log_base_directory = "./src/flightstack/log/" + current_date  + "/" + ControlType::getControllerName();

  // Check if the directory exists, and create it if it doesn't
  if (!std::filesystem::exists(log_base_directory)) {
    std::filesystem::create_directories(log_base_directory);
  }

  // Check if the "logs" subdirectory exists within the log_base_directory, create it if needed
  std::string logs_directory = log_base_directory + "/logs";
  if (!std::filesystem::exists(logs_directory)) {
      std::filesystem::create_directories(logs_directory);
  }

  // Check if the "gains" subdirectory exists within the log_base_directory, create it if needed
  std::string gains_directory = log_base_directory + "/gains";
  if (!std::filesystem::exists(gains_directory)) {
      std::filesystem::create_directories(gains_directory);
  }

  // Generate the log file name with a timestamp
  std::stringstream log_ss;
  log_ss << logs_directory << "/log_" << std::put_time(std::localtime(&now_c), "%Y%m%d_%H%M%S") << ".log";
  std::string log_filename = log_ss.str();

  // Add the "Tag" attribute with a constant value of "LogDataTag" to the logger
  LogData_PID::logger_logdata.add_attribute("Tag", attrs::constant<std::string>("LogDataTag"));

  // Define a synchronous sink with a text ostream backend
  typedef sinks::synchronous_sink<sinks::text_ostream_backend> text_sink;
  boost::shared_ptr<text_sink> logdata_sink = boost::make_shared<text_sink>();

  // Add a stream to the backend (in this case, a file stream)
  logdata_sink->locked_backend()->add_stream(boost::make_shared<std::ofstream>(log_filename));

  // Set the formatter for the sink
  logdata_sink->set_formatter(
    expr::stream
    << "[" << expr::format_date_time<boost::posix_time::ptime>("TimeStamp", "%Y-%m-%d %H:%M:%S.%f") << "] " // Format date and time
    << "[" << expr::attr<boost::log::attributes::current_thread_id::value_type>("ThreadID") << "] " // Current thread ID
    << "[" << expr::attr<std::string>("Tag") << "] " // Tag attribute value
    << "[" << expr::attr<boost::log::attributes::current_process_id::value_type>("ProcessID") << "] " // Current process ID
    << "[" << expr::attr<unsigned int>("LineID") << "] " // Line ID
    << expr::smessage // Log message
  );

  // Add the sink to the logging core
  logging::core::get()->add_sink(logdata_sink);

  // Set a filter for the sink
  logdata_sink->set_filter(expr::has_attr("Tag") && expr::attr<std::string>("Tag") == "LogDataTag");

  logging::add_common_attributes(); // Add attributes like timestamp

  // Copy the gains file to the gains_directory with a new name containing the time info
  std::string source_file = "./src/flightstack/params/control/pid/gains_pid.json";
  std::stringstream target_ss;
  target_ss << gains_directory << "/gains_pid_" << std::put_time(std::localtime(&now_c), "%Y%m%d_%H%M%S") << ".json";
  std::string target_file = target_ss.str();

  if (std::filesystem::exists(source_file)) {
    std::cout << "GAINS FILE PRESENT" << std::endl;
    std::filesystem::copy(source_file, target_file);
  } else {
    std::cout << "GAINS FILE NOT PRESENT" << std::endl;
  }

  logInitializeHeaders();
}

// Function to log the data
void LogData_PID::logLogData()
{
	std::ostringstream oss;

	oss << node_.getInitialTimestamp() << ", "
	    << node_.getCurrentTime() << ", "
      << node_.getUserDefinedTrajectory()->getUserDefinedPosition()(0) << ", "
      << node_.getUserDefinedTrajectory()->getUserDefinedPosition()(1) << ", "
      << node_.getUserDefinedTrajectory()->getUserDefinedPosition()(2) << ", "
      << node_.getUserDefinedTrajectory()->getUserDefinedVelocity()(0) << ", "
      << node_.getUserDefinedTrajectory()->getUserDefinedVelocity()(1) << ", "
      << node_.getUserDefinedTrajectory()->getUserDefinedVelocity()(2) << ", "
      << node_.getUserDefinedTrajectory()->getUserDefinedAcceleration()(0) << ", "
      << node_.getUserDefinedTrajectory()->getUserDefinedAcceleration()(1) << ", "
      << node_.getUserDefinedTrajectory()->getUserDefinedAcceleration()(2) << ", "
      << node_.getUserDefinedTrajectory()->getUserDefinedYaw() << ", "
      << node_.getUserDefinedTrajectory()->getUserDefinedYawDot() << ", "
      << node_.getUserDefinedTrajectory()->getUserDefinedYawDotDot() << ", "
      << node_.getVehicleState()->getTimeOdometryInSeconds() << ", "
      << node_.getVehicleState()->getPosition()(0) << ", "
      << node_.getVehicleState()->getPosition()(1) << ", "
      << node_.getVehicleState()->getPosition()(2) << ", "
      << node_.getVehicleState()->getQuaternion().w() << ", "
      << node_.getVehicleState()->getQuaternion().x() << ", "
      << node_.getVehicleState()->getQuaternion().y() << ", "
      << node_.getVehicleState()->getQuaternion().z() << ", "
      << node_.getVehicleState()->getVelocity()(0) << ", "
      << node_.getVehicleState()->getVelocity()(1) << ", "
      << node_.getVehicleState()->getVelocity()(2) << ", "
      << node_.getVehicleState()->getAngularVelocity()(0) << ", "
      << node_.getVehicleState()->getAngularVelocity()(1) << ", "
      << node_.getVehicleState()->getAngularVelocity()(2) << ", "
      << node_.getVehicleState()->getEulerAnglesRPY()(0) << ", "
      << node_.getVehicleState()->getEulerAnglesRPY()(1) << ", "
      << node_.getVehicleState()->getEulerAnglesRPY()(2) << ", "
      << controller_.getAlgorithmExecutionTimeMicroseconds().count() << ", "
      << controller_.getControlInternalMembers().mu_translational_raw(0) << ", "
      << controller_.getControlInternalMembers().mu_translational_raw(1) << ", "
      << controller_.getControlInternalMembers().mu_translational_raw(2) << ", "
      << controller_.getControlInternalMembers().mu_translational(0) << ", "
      << controller_.getControlInternalMembers().mu_translational(1) << ", "
      << controller_.getControlInternalMembers().mu_translational(2) << ", "
      << controller_.getControlInternalMembers().U_control_inputs(0) << ", "
      << controller_.getControlInternalMembers().U_control_inputs(1) << ", "
      << controller_.getControlInternalMembers().U_control_inputs(2) << ", "
      << controller_.getControlInternalMembers().U_control_inputs(3) << ", "
      << controller_.getControlInternalMembers().roll_desired << ", "
      << controller_.getControlInternalMembers().pitch_desired << ", "
      << controller_.getControlInternalMembers().roll_desired_dot << ", "
      << controller_.getControlInternalMembers().pitch_desired_dot << ", "
      << controller_.getControlInternalMembers().roll_desired_dot_dot << ", "
      << controller_.getControlInternalMembers().pitch_desired_dot_dot << ", "
      << controller_.getControlInternalMembers().euler_angles_rpy_dot(0) << ", "
      << controller_.getControlInternalMembers().euler_angles_rpy_dot(1) << ", "
      << controller_.getControlInternalMembers().euler_angles_rpy_dot(2) << ", "
      << controller_.getControlInternalMembers().thrust_vector_quadcopter(0) << ", "
      << controller_.getControlInternalMembers().thrust_vector_quadcopter(1) << ", "
      << controller_.getControlInternalMembers().thrust_vector_quadcopter(2) << ", "
      << controller_.getControlInternalMembers().thrust_vector_quadcopter(3) << ", "
      << controller_.getControlInternalMembers().thrust_vector_quadcopter_normalized(0) << ", "
      << controller_.getControlInternalMembers().thrust_vector_quadcopter_normalized(1) << ", "
      << controller_.getControlInternalMembers().thrust_vector_quadcopter_normalized(2) << ", "
      << controller_.getControlInternalMembers().thrust_vector_quadcopter_normalized(3) << ", "
      << controller_.getControlInternalMembers().thrust_vector(0) << ", "
      << controller_.getControlInternalMembers().thrust_vector(1) << ", "
      << controller_.getControlInternalMembers().thrust_vector(2) << ", "
      << controller_.getControlInternalMembers().thrust_vector(3) << ", "
      << controller_.getControlInternalMembers().thrust_vector(4) << ", "
      << controller_.getControlInternalMembers().thrust_vector(5) << ", "
      << controller_.getControlInternalMembers().thrust_vector(6) << ", "
      << controller_.getControlInternalMembers().thrust_vector(7) << ", "
      << controller_.getControlInternalMembers().thrust_vector_normalized(0) << ", "
      << controller_.getControlInternalMembers().thrust_vector_normalized(1) << ", "
      << controller_.getControlInternalMembers().thrust_vector_normalized(2) << ", "
      << controller_.getControlInternalMembers().thrust_vector_normalized(3) << ", "
      << controller_.getControlInternalMembers().thrust_vector_normalized(4) << ", "
      << controller_.getControlInternalMembers().thrust_vector_normalized(5) << ", "
      << controller_.getControlInternalMembers().thrust_vector_normalized(6) << ", "
      << controller_.getControlInternalMembers().thrust_vector_normalized(7) << ", "
      << controller_.getControlInternalMembers().angular_error(0) << ", "
      << controller_.getControlInternalMembers().angular_error(1) << ", "
      << controller_.getControlInternalMembers().angular_error(2) << ", "
      << controller_.getControllerSpecificInternalMembers().outer_loop_P(0) << ", "
      << controller_.getControllerSpecificInternalMembers().outer_loop_P(1) << ", "
      << controller_.getControllerSpecificInternalMembers().outer_loop_P(2) << ", "
      << controller_.getControllerSpecificInternalMembers().outer_loop_I(0) << ", "
      << controller_.getControllerSpecificInternalMembers().outer_loop_I(1) << ", "
      << controller_.getControllerSpecificInternalMembers().outer_loop_I(2) << ", "
      << controller_.getControllerSpecificInternalMembers().outer_loop_D(0) << ", "
      << controller_.getControllerSpecificInternalMembers().outer_loop_D(1) << ", "
      << controller_.getControllerSpecificInternalMembers().outer_loop_D(2) << ", "
      << controller_.getControllerSpecificInternalMembers().outer_loop_dynamic_inversion(0) << ", "
      << controller_.getControllerSpecificInternalMembers().outer_loop_dynamic_inversion(1) << ", "
      << controller_.getControllerSpecificInternalMembers().outer_loop_dynamic_inversion(2) << ", "
      << controller_.getControllerSpecificInternalMembers().inner_loop_P(0) << ", "
      << controller_.getControllerSpecificInternalMembers().inner_loop_P(1) << ", "
      << controller_.getControllerSpecificInternalMembers().inner_loop_P(2) << ", "
      << controller_.getControllerSpecificInternalMembers().inner_loop_I(0) << ", "
      << controller_.getControllerSpecificInternalMembers().inner_loop_I(1) << ", "
      << controller_.getControllerSpecificInternalMembers().inner_loop_I(2) << ", "
      << controller_.getControllerSpecificInternalMembers().inner_loop_D(0) << ", "
      << controller_.getControllerSpecificInternalMembers().inner_loop_D(1) << ", "
      << controller_.getControllerSpecificInternalMembers().inner_loop_D(2) << ", "
      << controller_.getControllerSpecificInternalMembers().inner_loop_dynamic_inversion(0) << ", "
      << controller_.getControllerSpecificInternalMembers().inner_loop_dynamic_inversion(1) << ", "
      << controller_.getControllerSpecificInternalMembers().inner_loop_dynamic_inversion(2) << ", "
      ;

	BOOST_LOG(LogData_PID::logger_logdata) << oss.str();
}

