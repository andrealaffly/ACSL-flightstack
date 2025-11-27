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
 * File:        logging_two_layer_mrac.cpp
 * Author:      Mattia Gramuglia
 * Date:        September 23, 2024
 * For info:    Andrea L'Afflitto 
 *              a.lafflitto@vt.edu
 * 
 * Description: Logger for the TwoLayerMRAC controller.
 * 
 * GitHub:    https://github.com/andrealaffly/ACSL-flightstack.git
 **********************************************************************************************************************/

#include "two_layer_mrac.hpp"
#include "logging_two_layer_mrac.hpp"
#include "multi_threaded_node.hpp"


// Define the logger for LogData
src::logger LogData_TwoLayerMRAC::logger_logdata;

// Constructor
LogData_TwoLayerMRAC::LogData_TwoLayerMRAC(MultiThreadedNode& node, TwoLayerMRAC& controller) :
  node_(node),
  controller_(controller)
{}

// Function to print headers
void LogData_TwoLayerMRAC::logInitializeHeaders()
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
      << "Mu translational raw global x [N], "
      << "Mu translational raw global y [N], "
      << "Mu translational raw global z [N], "
      << "Mu translational local x [N], "
      << "Mu translational local y [N], "
      << "Mu translational local z [N], "
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
      << "Translational reference model position x [m], "
      << "Translational reference model position y [m], "
      << "Translational reference model position z [m], "
      << "Translational reference model velocity x [m/s], "
      << "Translational reference model velocity y [m/s], "
      << "Translational reference model velocity z [m/s], "
      << "K_hat_x_translational index-00 [-], "
      << "K_hat_x_translational index-10 [-], "
      << "K_hat_x_translational index-20 [-], "
      << "K_hat_x_translational index-30 [-], "
      << "K_hat_x_translational index-40 [-], "
      << "K_hat_x_translational index-50 [-], "
      << "K_hat_x_translational index-01 [-], "
      << "K_hat_x_translational index-11 [-], "
      << "K_hat_x_translational index-21 [-], "
      << "K_hat_x_translational index-31 [-], "
      << "K_hat_x_translational index-41 [-], "
      << "K_hat_x_translational index-51 [-], "
      << "K_hat_x_translational index-02 [-], "
      << "K_hat_x_translational index-12 [-], "
      << "K_hat_x_translational index-22 [-], "
      << "K_hat_x_translational index-32 [-], "
      << "K_hat_x_translational index-42 [-], "
      << "K_hat_x_translational index-52 [-], "
      << "K_hat_r_translational index-00 [-], "
      << "K_hat_r_translational index-10 [-], "
      << "K_hat_r_translational index-20 [-], "
      << "K_hat_r_translational index-01 [-], "
      << "K_hat_r_translational index-11 [-], "
      << "K_hat_r_translational index-21 [-], "
      << "K_hat_r_translational index-02 [-], "
      << "K_hat_r_translational index-12 [-], "
      << "K_hat_r_translational index-22 [-], "
      << "Theta_hat_translational index-00 [-], "
      << "Theta_hat_translational index-10 [-], "
      << "Theta_hat_translational index-20 [-], "
      << "Theta_hat_translational index-30 [-], "
      << "Theta_hat_translational index-40 [-], "
      << "Theta_hat_translational index-50 [-], "
      << "Theta_hat_translational index-01 [-], "
      << "Theta_hat_translational index-11 [-], "
      << "Theta_hat_translational index-21 [-], "
      << "Theta_hat_translational index-31 [-], "
      << "Theta_hat_translational index-41 [-], "
      << "Theta_hat_translational index-51 [-], "
      << "Theta_hat_translational index-02 [-], "
      << "Theta_hat_translational index-12 [-], "
      << "Theta_hat_translational index-22 [-], "
      << "Theta_hat_translational index-32 [-], "
      << "Theta_hat_translational index-42 [-], "
      << "Theta_hat_translational index-52 [-], "
      << "Rotational reference model omega x [rad/s], "
      << "Rotational reference model omega y [rad/s], "
      << "Rotational reference model omega z [rad/s], "
      << "K_hat_x_rotational index-00 [-], "
      << "K_hat_x_rotational index-10 [-], "
      << "K_hat_x_rotational index-20 [-], "
      << "K_hat_x_rotational index-01 [-], "
      << "K_hat_x_rotational index-11 [-], "
      << "K_hat_x_rotational index-21 [-], "
      << "K_hat_x_rotational index-02 [-], "
      << "K_hat_x_rotational index-12 [-], "
      << "K_hat_x_rotational index-22 [-], "
      << "K_hat_r_rotational index-00 [-], "
      << "K_hat_r_rotational index-10 [-], "
      << "K_hat_r_rotational index-20 [-], "
      << "K_hat_r_rotational index-01 [-], "
      << "K_hat_r_rotational index-11 [-], "
      << "K_hat_r_rotational index-21 [-], "
      << "K_hat_r_rotational index-02 [-], "
      << "K_hat_r_rotational index-12 [-], "
      << "K_hat_r_rotational index-22 [-], "
      << "Theta_hat_rotational index-00 [-], "
      << "Theta_hat_rotational index-10 [-], "
      << "Theta_hat_rotational index-20 [-], "
      << "Theta_hat_rotational index-30 [-], "
      << "Theta_hat_rotational index-40 [-], "
      << "Theta_hat_rotational index-50 [-], "
      << "Theta_hat_rotational index-01 [-], "
      << "Theta_hat_rotational index-11 [-], "
      << "Theta_hat_rotational index-21 [-], "
      << "Theta_hat_rotational index-31 [-], "
      << "Theta_hat_rotational index-41 [-], "
      << "Theta_hat_rotational index-51 [-], "
      << "Theta_hat_rotational index-02 [-], "
      << "Theta_hat_rotational index-12 [-], "
      << "Theta_hat_rotational index-22 [-], "
      << "Theta_hat_rotational index-32 [-], "
      << "Theta_hat_rotational index-42 [-], "
      << "Theta_hat_rotational index-52 [-], "
      << "r_cmd_translational x [N], "
      << "r_cmd_translational y [N], "
      << "r_cmd_translational z [N], "
      << "mu_PID_baseline_translational x [N], "
      << "mu_PID_baseline_translational y [N], "
      << "mu_PID_baseline_translational z [N], "
      << "mu_adaptive_translational x [N], "
      << "mu_adaptive_translational y [N], "
      << "mu_adaptive_translational z [N], "
      << "omega_cmd_rotational x [rad/s], "
      << "omega_cmd_rotational y [rad/s], "
      << "omega_cmd_rotational z [rad/s], "
      << "r_cmd_rotational x [rad/s^2], "
      << "r_cmd_rotational y [rad/s^2], "
      << "r_cmd_rotational z [rad/s^2], "
      << "tau_PID_baseline_rotational x [Nm], "
      << "tau_PID_baseline_rotational y [Nm], "
      << "tau_PID_baseline_rotational z [Nm], "
      << "tau_adaptive_rotational x [Nm], "
      << "tau_adaptive_rotational y [Nm], "
      << "tau_adaptive_rotational z [Nm], "
      << "dead_zone_value_translational [-], "
      << "dead_zone_value_rotational [-], "
      << "K_hat_g_translational index-00 [-], "
      << "K_hat_g_translational index-10 [-], "
      << "K_hat_g_translational index-20 [-], "
      << "K_hat_g_translational index-30 [-], "
      << "K_hat_g_translational index-40 [-], "
      << "K_hat_g_translational index-50 [-], "
      << "K_hat_g_translational index-01 [-], "
      << "K_hat_g_translational index-11 [-], "
      << "K_hat_g_translational index-21 [-], "
      << "K_hat_g_translational index-31 [-], "
      << "K_hat_g_translational index-41 [-], "
      << "K_hat_g_translational index-51 [-], "
      << "K_hat_g_translational index-02 [-], "
      << "K_hat_g_translational index-12 [-], "
      << "K_hat_g_translational index-22 [-], "
      << "K_hat_g_translational index-32 [-], "
      << "K_hat_g_translational index-42 [-], "
      << "K_hat_g_translational index-52 [-], "
      << "K_hat_g_rotational index-00 [-], "
      << "K_hat_g_rotational index-10 [-], "
      << "K_hat_g_rotational index-20 [-], "
      << "K_hat_g_rotational index-01 [-], "
      << "K_hat_g_rotational index-11 [-], "
      << "K_hat_g_rotational index-21 [-], "
      << "K_hat_g_rotational index-02 [-], "
      << "K_hat_g_rotational index-12 [-], "
      << "K_hat_g_rotational index-22 [-], "   
      << "proj_op_activated_K_hat_x_translational, "
      << "proj_op_activated_K_hat_r_translational, "
      << "proj_op_activated_Theta_hat_translational, "      
      << "proj_op_activated_K_hat_g_translational, "
      << "proj_op_activated_K_hat_x_rotational, "
      << "proj_op_activated_K_hat_r_rotational, "
      << "proj_op_activated_Theta_hat_rotational, "
      << "proj_op_activated_K_hat_g_rotational, "
      << "Mu translational raw local x [N], "
      << "Mu translational raw local y [N], "
      << "Mu translational raw local z [N], "
      << "Safety Mechanism tSphere [-], "
      << "Safety Mechanism tEllipticCone [-], "
      << "Safety Mechanism tPlane [-], "
      << "Safety Mechanism tPrime [-], "
      << "Safety Mechanism safe_mech_activated [-], "
  ;
      
  {
    BOOST_LOG_SCOPED_LOGGER_ATTR(LogData_TwoLayerMRAC::logger_logdata,
      "Tag", attrs::constant<std::string>("LogDataTag"));
    BOOST_LOG(LogData_TwoLayerMRAC::logger_logdata) << oss.str();
  }
}

// Function to initialize the logging system
void LogData_TwoLayerMRAC::logInitializeLogging()
{
  // Get the current date and time
  auto now = std::chrono::system_clock::now();
  std::time_t now_c = std::chrono::system_clock::to_time_t(now);

  // Get the current date in the desired format
  std::stringstream date_ss;
  date_ss << std::put_time(std::localtime(&now_c), "%Y%m%d"); // Current date
  std::string current_date = date_ss.str();

  // Create the directory path for the logs
  std::string log_base_directory = "./src/flightstack/log/" + current_date + "/" + ControlType::getControllerName();

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

  // Check if the "info" subdirectory exists within the log_base_directory, create it if needed
  std::string info_directory = log_base_directory + "/info";
  if (!std::filesystem::exists(info_directory)) {
    std::filesystem::create_directories(info_directory);
  }

  // ========== Main Log File Setup ===========

  // Generate the log file name with a timestamp
  std::stringstream log_ss;
  log_ss << logs_directory << "/log_" << std::put_time(std::localtime(&now_c), "%Y%m%d_%H%M%S") << ".log";
  std::string log_filename = log_ss.str();

  // Define a synchronous sink with a text ostream backend
  typedef sinks::synchronous_sink<sinks::text_ostream_backend> text_sink;
  boost::shared_ptr<text_sink> logdata_sink = boost::make_shared<text_sink>();

  // Add a stream to the interr sink (for the new log file)
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

  // Set a filter for the sink
  logdata_sink->set_filter(expr::has_attr("Tag") && expr::attr<std::string>("Tag") == "LogDataTag");

  // Add the interr sink to the logging core
  logging::core::get()->add_sink(logdata_sink);

  // =========================================

  logging::add_common_attributes(); // Add attributes like timestamp

  // Copy the gains file to the gains_directory with a new name containing the time info
  std::string gains_source_file = "./src/flightstack/params/control/two_layer_mrac/gains_two_layer_mrac.json";
  std::stringstream gains_target_ss;
  gains_target_ss << gains_directory << "/gains_two_layer_mrac_" <<
    std::put_time(std::localtime(&now_c), "%Y%m%d_%H%M%S") << ".json";
  std::string gains_target_file = gains_target_ss.str();
  if (std::filesystem::exists(gains_source_file)) {
    // std::cout << "GAINS FILE PRESENT" << std::endl;
    std::filesystem::copy(gains_source_file, gains_target_file);
  } else {
    std::cout << "GAINS FILE NOT PRESENT" << std::endl;
  }

  // Copy the outer_loop_safety_mechanism file to the info_directory with a new name containing the time info
  std::string safe_mech_source_file = "./src/flightstack/params/control/outer_loop_safety_mechanism.json";
  std::stringstream safe_mech_target_ss;
  safe_mech_target_ss << info_directory << "/safe_mech_two_layer_mrac_" <<
    std::put_time(std::localtime(&now_c), "%Y%m%d_%H%M%S") << ".json";
  std::string safe_mech_target_file = safe_mech_target_ss.str();
  if (std::filesystem::exists(safe_mech_source_file)) {
    // std::cout << "SAFETY MECHANISM PARAMETERS FILE PRESENT" << std::endl;
    std::filesystem::copy(safe_mech_source_file, safe_mech_target_file);
  } else {
    std::cout << "SAFETY MECHANISM PARAMETERS FILE NOT PRESENT" << std::endl;
  }

  logInitializeHeaders();

  // ====== DEBUG Log File Setup ======
  if constexpr (config_param::USE_DEBUG_LOGGER){

    // Check if the "logs_debug" subdirectory exists within the log_base_directory, create it if needed
    std::string logs_debug_directory = log_base_directory + "/logs_debug";
    if (!std::filesystem::exists(logs_debug_directory)) {
      std::filesystem::create_directories(logs_debug_directory);
    }

    // Generate the debug log file name with a timestamp
    std::stringstream debug_log_ss;
    debug_log_ss << logs_debug_directory << "/log_debug_" << std::put_time(std::localtime(&now_c), "%Y%m%d_%H%M%S") << ".log";
    std::string debug_log_filename = debug_log_ss.str();

    // Define a synchronous sink for the debug log file
    boost::shared_ptr<text_sink> debug_sink = boost::make_shared<text_sink>();

    // Add a stream to the debug sink (for the debug log file)
    debug_sink->locked_backend()->add_stream(boost::make_shared<std::ofstream>(debug_log_filename));

    // Set the formatter for the debug log file
    debug_sink->set_formatter(
      expr::stream
      << "[" << expr::format_date_time<boost::posix_time::ptime>("TimeStamp", "%Y-%m-%d %H:%M:%S.%f") << "] "
      << "[" << expr::attr<std::string>("Tag") << "] "
      << expr::smessage // Log message
    );

    // Add a filter to log only messages with the "DebugLogTag"
    debug_sink->set_filter(expr::has_attr("Tag") && expr::attr<std::string>("Tag") == "DebugLogTag");

    // Add the debug sink to the logging core
    logging::core::get()->add_sink(debug_sink);
  }
}

// Function to log the data
void LogData_TwoLayerMRAC::logLogData()
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
      << controller_.getControlInternalMembers().mu_translational_raw_global(0) << ", "
      << controller_.getControlInternalMembers().mu_translational_raw_global(1) << ", "
      << controller_.getControlInternalMembers().mu_translational_raw_global(2) << ", "
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
      << controller_.getStateController().x_ref_translational(0) << ", "
      << controller_.getStateController().x_ref_translational(1) << ", "
      << controller_.getStateController().x_ref_translational(2) << ", "
      << controller_.getStateController().x_ref_translational(3) << ", "
      << controller_.getStateController().x_ref_translational(4) << ", "
      << controller_.getStateController().x_ref_translational(5) << ", "
      << controller_.getStateController().K_hat_x_translational(0,0) << ", "
      << controller_.getStateController().K_hat_x_translational(1,0) << ", "
      << controller_.getStateController().K_hat_x_translational(2,0) << ", "
      << controller_.getStateController().K_hat_x_translational(3,0) << ", "
      << controller_.getStateController().K_hat_x_translational(4,0) << ", "
      << controller_.getStateController().K_hat_x_translational(5,0) << ", "
      << controller_.getStateController().K_hat_x_translational(0,1) << ", "
      << controller_.getStateController().K_hat_x_translational(1,1) << ", "
      << controller_.getStateController().K_hat_x_translational(2,1) << ", "
      << controller_.getStateController().K_hat_x_translational(3,1) << ", "
      << controller_.getStateController().K_hat_x_translational(4,1) << ", "
      << controller_.getStateController().K_hat_x_translational(5,1) << ", "
      << controller_.getStateController().K_hat_x_translational(0,2) << ", "
      << controller_.getStateController().K_hat_x_translational(1,2) << ", "
      << controller_.getStateController().K_hat_x_translational(2,2) << ", "
      << controller_.getStateController().K_hat_x_translational(3,2) << ", "
      << controller_.getStateController().K_hat_x_translational(4,2) << ", "
      << controller_.getStateController().K_hat_x_translational(5,2) << ", "
      << controller_.getStateController().K_hat_r_translational(0,0) << ", "
      << controller_.getStateController().K_hat_r_translational(1,0) << ", "
      << controller_.getStateController().K_hat_r_translational(2,0) << ", "
      << controller_.getStateController().K_hat_r_translational(0,1) << ", "
      << controller_.getStateController().K_hat_r_translational(1,1) << ", "
      << controller_.getStateController().K_hat_r_translational(2,1) << ", "
      << controller_.getStateController().K_hat_r_translational(0,2) << ", "
      << controller_.getStateController().K_hat_r_translational(1,2) << ", "
      << controller_.getStateController().K_hat_r_translational(2,2) << ", "
      << controller_.getStateController().Theta_hat_translational(0,0) << ", "
      << controller_.getStateController().Theta_hat_translational(1,0) << ", "
      << controller_.getStateController().Theta_hat_translational(2,0) << ", "
      << controller_.getStateController().Theta_hat_translational(3,0) << ", "
      << controller_.getStateController().Theta_hat_translational(4,0) << ", "
      << controller_.getStateController().Theta_hat_translational(5,0) << ", "
      << controller_.getStateController().Theta_hat_translational(0,1) << ", "
      << controller_.getStateController().Theta_hat_translational(1,1) << ", "
      << controller_.getStateController().Theta_hat_translational(2,1) << ", "
      << controller_.getStateController().Theta_hat_translational(3,1) << ", "
      << controller_.getStateController().Theta_hat_translational(4,1) << ", "
      << controller_.getStateController().Theta_hat_translational(5,1) << ", "
      << controller_.getStateController().Theta_hat_translational(0,2) << ", "
      << controller_.getStateController().Theta_hat_translational(1,2) << ", "
      << controller_.getStateController().Theta_hat_translational(2,2) << ", "
      << controller_.getStateController().Theta_hat_translational(3,2) << ", "
      << controller_.getStateController().Theta_hat_translational(4,2) << ", "
      << controller_.getStateController().Theta_hat_translational(5,2) << ", "
      << controller_.getStateController().omega_ref_rotational(0) << ", "
      << controller_.getStateController().omega_ref_rotational(1) << ", "
      << controller_.getStateController().omega_ref_rotational(2) << ", "
      << controller_.getStateController().K_hat_x_rotational(0,0) << ", "
      << controller_.getStateController().K_hat_x_rotational(1,0) << ", "
      << controller_.getStateController().K_hat_x_rotational(2,0) << ", "
      << controller_.getStateController().K_hat_x_rotational(0,1) << ", "
      << controller_.getStateController().K_hat_x_rotational(1,1) << ", "
      << controller_.getStateController().K_hat_x_rotational(2,1) << ", "
      << controller_.getStateController().K_hat_x_rotational(0,2) << ", "
      << controller_.getStateController().K_hat_x_rotational(1,2) << ", "
      << controller_.getStateController().K_hat_x_rotational(2,2) << ", "
      << controller_.getStateController().K_hat_r_rotational(0,0) << ", "
      << controller_.getStateController().K_hat_r_rotational(1,0) << ", "
      << controller_.getStateController().K_hat_r_rotational(2,0) << ", "
      << controller_.getStateController().K_hat_r_rotational(0,1) << ", "
      << controller_.getStateController().K_hat_r_rotational(1,1) << ", "
      << controller_.getStateController().K_hat_r_rotational(2,1) << ", "
      << controller_.getStateController().K_hat_r_rotational(0,2) << ", "
      << controller_.getStateController().K_hat_r_rotational(1,2) << ", "
      << controller_.getStateController().K_hat_r_rotational(2,2) << ", "
      << controller_.getStateController().Theta_hat_rotational(0,0) << ", "
      << controller_.getStateController().Theta_hat_rotational(1,0) << ", "
      << controller_.getStateController().Theta_hat_rotational(2,0) << ", "
      << controller_.getStateController().Theta_hat_rotational(3,0) << ", "
      << controller_.getStateController().Theta_hat_rotational(4,0) << ", "
      << controller_.getStateController().Theta_hat_rotational(5,0) << ", "
      << controller_.getStateController().Theta_hat_rotational(0,1) << ", "
      << controller_.getStateController().Theta_hat_rotational(1,1) << ", "
      << controller_.getStateController().Theta_hat_rotational(2,1) << ", "
      << controller_.getStateController().Theta_hat_rotational(3,1) << ", "
      << controller_.getStateController().Theta_hat_rotational(4,1) << ", "
      << controller_.getStateController().Theta_hat_rotational(5,1) << ", "
      << controller_.getStateController().Theta_hat_rotational(0,2) << ", "
      << controller_.getStateController().Theta_hat_rotational(1,2) << ", "
      << controller_.getStateController().Theta_hat_rotational(2,2) << ", "
      << controller_.getStateController().Theta_hat_rotational(3,2) << ", "
      << controller_.getStateController().Theta_hat_rotational(4,2) << ", "
      << controller_.getStateController().Theta_hat_rotational(5,2) << ", "
      << controller_.getControllerSpecificInternalMembers().r_cmd_translational(0) << ", "
      << controller_.getControllerSpecificInternalMembers().r_cmd_translational(1) << ", "
      << controller_.getControllerSpecificInternalMembers().r_cmd_translational(2) << ", "
      << controller_.getControllerSpecificInternalMembers().mu_PID_baseline_translational(0) << ", "
      << controller_.getControllerSpecificInternalMembers().mu_PID_baseline_translational(1) << ", "
      << controller_.getControllerSpecificInternalMembers().mu_PID_baseline_translational(2) << ", "
      << controller_.getControllerSpecificInternalMembers().mu_adaptive_translational(0) << ", "
      << controller_.getControllerSpecificInternalMembers().mu_adaptive_translational(1) << ", "
      << controller_.getControllerSpecificInternalMembers().mu_adaptive_translational(2) << ", "
      << controller_.getControllerSpecificInternalMembers().omega_cmd_rotational(0) << ", "
      << controller_.getControllerSpecificInternalMembers().omega_cmd_rotational(1) << ", "
      << controller_.getControllerSpecificInternalMembers().omega_cmd_rotational(2) << ", "
      << controller_.getControllerSpecificInternalMembers().r_cmd_rotational(0) << ", "
      << controller_.getControllerSpecificInternalMembers().r_cmd_rotational(1) << ", "
      << controller_.getControllerSpecificInternalMembers().r_cmd_rotational(2) << ", "
      << controller_.getControllerSpecificInternalMembers().tau_PID_baseline_rotational(0) << ", "
      << controller_.getControllerSpecificInternalMembers().tau_PID_baseline_rotational(1) << ", "
      << controller_.getControllerSpecificInternalMembers().tau_PID_baseline_rotational(2) << ", "
      << controller_.getControllerSpecificInternalMembers().tau_adaptive_rotational(0) << ", "
      << controller_.getControllerSpecificInternalMembers().tau_adaptive_rotational(1) << ", "
      << controller_.getControllerSpecificInternalMembers().tau_adaptive_rotational(2) << ", "
      << controller_.getControllerSpecificInternalMembers().dead_zone_value_translational << ", "
      << controller_.getControllerSpecificInternalMembers().dead_zone_value_rotational << ", "
      << controller_.getStateController().K_hat_g_translational(0,0) << ", "
      << controller_.getStateController().K_hat_g_translational(1,0) << ", "
      << controller_.getStateController().K_hat_g_translational(2,0) << ", "
      << controller_.getStateController().K_hat_g_translational(3,0) << ", "
      << controller_.getStateController().K_hat_g_translational(4,0) << ", "
      << controller_.getStateController().K_hat_g_translational(5,0) << ", "
      << controller_.getStateController().K_hat_g_translational(0,1) << ", "
      << controller_.getStateController().K_hat_g_translational(1,1) << ", "
      << controller_.getStateController().K_hat_g_translational(2,1) << ", "
      << controller_.getStateController().K_hat_g_translational(3,1) << ", "
      << controller_.getStateController().K_hat_g_translational(4,1) << ", "
      << controller_.getStateController().K_hat_g_translational(5,1) << ", "
      << controller_.getStateController().K_hat_g_translational(0,2) << ", "
      << controller_.getStateController().K_hat_g_translational(1,2) << ", "
      << controller_.getStateController().K_hat_g_translational(2,2) << ", "
      << controller_.getStateController().K_hat_g_translational(3,2) << ", "
      << controller_.getStateController().K_hat_g_translational(4,2) << ", "
      << controller_.getStateController().K_hat_g_translational(5,2) << ", "
      << controller_.getStateController().K_hat_g_rotational(0,0) << ", "
      << controller_.getStateController().K_hat_g_rotational(1,0) << ", "
      << controller_.getStateController().K_hat_g_rotational(2,0) << ", "
      << controller_.getStateController().K_hat_g_rotational(0,1) << ", "
      << controller_.getStateController().K_hat_g_rotational(1,1) << ", "
      << controller_.getStateController().K_hat_g_rotational(2,1) << ", "
      << controller_.getStateController().K_hat_g_rotational(0,2) << ", "
      << controller_.getStateController().K_hat_g_rotational(1,2) << ", "
      << controller_.getStateController().K_hat_g_rotational(2,2) << ", "
      << controller_.getControllerSpecificInternalMembers().proj_op_activated_K_hat_x_translational << ", "
      << controller_.getControllerSpecificInternalMembers().proj_op_activated_K_hat_r_translational << ", "
      << controller_.getControllerSpecificInternalMembers().proj_op_activated_Theta_hat_translational << ", "
      << controller_.getControllerSpecificInternalMembers().proj_op_activated_K_hat_g_translational << ", "
      << controller_.getControllerSpecificInternalMembers().proj_op_activated_K_hat_x_rotational << ", "
      << controller_.getControllerSpecificInternalMembers().proj_op_activated_K_hat_r_rotational << ", "
      << controller_.getControllerSpecificInternalMembers().proj_op_activated_Theta_hat_rotational << ", "
      << controller_.getControllerSpecificInternalMembers().proj_op_activated_K_hat_g_rotational << ", "
      << controller_.getControlInternalMembers().mu_translational_raw_local(0) << ", "
      << controller_.getControlInternalMembers().mu_translational_raw_local(1) << ", "
      << controller_.getControlInternalMembers().mu_translational_raw_local(2) << ", "
      << controller_.getOuterLoopSafetyMechanism().getSafetyMechanismMembers().tSphere << ", "
      << controller_.getOuterLoopSafetyMechanism().getSafetyMechanismMembers().tEllipticCone << ", "
      << controller_.getOuterLoopSafetyMechanism().getSafetyMechanismMembers().tPlane << ", "
      << controller_.getOuterLoopSafetyMechanism().getSafetyMechanismMembers().tPrime << ", "
      << controller_.getOuterLoopSafetyMechanism().getSafetyMechanismMembers().safe_mech_activated << ", "
    ;

  {
    BOOST_LOG_SCOPED_LOGGER_ATTR(LogData_TwoLayerMRAC::logger_logdata,
      "Tag", attrs::constant<std::string>("LogDataTag"));
    BOOST_LOG(LogData_TwoLayerMRAC::logger_logdata) << oss.str();
  }

  /* 
    Log to the DEBUG log file
  */
  if constexpr (config_param::USE_DEBUG_LOGGER){
    std::ostringstream oss_debug;

    oss_debug << ", "
      << node_.getCurrentTime() << ", "
    ;

    // Log all elements of xerr 
    for (uint8_t i = 0; i < controller_.getErrorIntegrator().xerr.size(); ++i) {
      oss_debug << controller_.getErrorIntegrator().xerr[i] << ", ";
    }

    logMatrixColumnMajor(oss_debug, controller_.getControllerSpecificInternalMembers().K_hat_x_dot_translational);
    logMatrixColumnMajor(oss_debug, controller_.getControllerSpecificInternalMembers().K_hat_r_dot_translational);
    logMatrixColumnMajor(oss_debug, controller_.getControllerSpecificInternalMembers().Theta_hat_dot_translational);
    logMatrixColumnMajor(oss_debug, controller_.getControllerSpecificInternalMembers().K_hat_g_dot_translational);
    logMatrixColumnMajor(oss_debug, controller_.getControllerSpecificInternalMembers().K_hat_x_dot_rotational);
    logMatrixColumnMajor(oss_debug, controller_.getControllerSpecificInternalMembers().K_hat_r_dot_rotational);
    logMatrixColumnMajor(oss_debug, controller_.getControllerSpecificInternalMembers().Theta_hat_dot_rotational);
    logMatrixColumnMajor(oss_debug, controller_.getControllerSpecificInternalMembers().K_hat_g_dot_rotational);

    {
      BOOST_LOG_SCOPED_LOGGER_ATTR(LogData_TwoLayerMRAC::logger_logdata,
        "Tag", attrs::constant<std::string>("DebugLogTag"));
      BOOST_LOG(LogData_TwoLayerMRAC::logger_logdata) << oss_debug.str();
    }
  }
}

