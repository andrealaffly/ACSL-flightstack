/* logging_pid.cpp

Mattia Gramuglia
April 18, 2024
*/

#include "logging_pid.hpp"
#include "multi_threaded_node.hpp"

// Define the logger for LogData
boost::log::sources::logger logger_logdata;

// Constructor
LogData_PID::LogData_PID(MultiThreadedNode& node, PID& controller) :
	timestamp_initial(node.getInitialTimestamp()), // Initialize timestamp_initial_ with the value from MultiThreadedNode
  time_current(node.getCurrentTime()),
	user_defined_position(node.getUserDefinedTrajectory()->getUserDefinedPosition()),
	user_defined_velocity(node.getUserDefinedTrajectory()->getUserDefinedVelocity()),
	user_defined_acceleration(node.getUserDefinedTrajectory()->getUserDefinedAcceleration()),
	user_defined_yaw(node.getUserDefinedTrajectory()->getUserDefinedYaw()),
	user_defined_yaw_dot(node.getUserDefinedTrajectory()->getUserDefinedYawDot()),
	user_defined_yaw_dot_dot(node.getUserDefinedTrajectory()->getUserDefinedYawDotDot()),
  time_odometry(node.getVehicleState()->getTimeOdometryInSeconds()),
  position(node.getVehicleState()->getPosition()),
  q(node.getVehicleState()->getQuaternion()),
  velocity(node.getVehicleState()->getVelocity()),
  angular_velocity(node.getVehicleState()->getAngularVelocity()),
  euler_angles_rpy(node.getVehicleState()->getEulerAnglesRPY()),
  algorithm_execution_time_microseconds(controller.getAlgorithmExecutionTimeMicroseconds()),
  mu_translational_raw(controller.getControlInternalMembers().mu_translational_raw),
  mu_translational(controller.getControlInternalMembers().mu_translational),
  U_control_inputs(controller.getControlInternalMembers().U_control_inputs),
  roll_reference(controller.getControlInternalMembers().roll_reference),
  pitch_reference(controller.getControlInternalMembers().pitch_reference),
  roll_reference_dot(controller.getControlInternalMembers().roll_reference_dot),
  pitch_reference_dot(controller.getControlInternalMembers().pitch_reference_dot),
  roll_reference_dot_dot(controller.getControlInternalMembers().roll_reference_dot_dot),
  pitch_reference_dot_dot(controller.getControlInternalMembers().pitch_reference_dot_dot),
  euler_angles_rpy_dot(controller.getControlInternalMembers().euler_angles_rpy_dot),
  thrust_vector_quadcopter(controller.getControlInternalMembers().thrust_vector_quadcopter),
  thrust_vector_quadcopter_normalized(controller.getControlInternalMembers().thrust_vector_quadcopter_normalized),
  thrust_vector(controller.getControlInternalMembers().thrust_vector),
  thrust_vector_normalized(controller.getControlInternalMembers().thrust_vector_normalized),
  angular_error(controller.getControlInternalMembers().angular_error),
  outer_loop_P(controller.getControllerSpecificInternalMembers().outer_loop_P),
  outer_loop_I(controller.getControllerSpecificInternalMembers().outer_loop_I),
  outer_loop_D(controller.getControllerSpecificInternalMembers().outer_loop_D),
  outer_loop_dynamic_inversion(controller.getControllerSpecificInternalMembers().outer_loop_dynamic_inversion),
  inner_loop_P(controller.getControllerSpecificInternalMembers().inner_loop_P),
  inner_loop_I(controller.getControllerSpecificInternalMembers().inner_loop_I),
  inner_loop_D(controller.getControllerSpecificInternalMembers().inner_loop_D),
  inner_loop_dynamic_inversion(controller.getControllerSpecificInternalMembers().inner_loop_dynamic_inversion),
  node_(node) // node must be the last in the list to be initialized
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
      << "Roll reference [rad], "
      << "Pitch reference [rad], "
      << "Roll_dot reference [rad/s], "
      << "Pitch_dot reference [rad/s], "
      << "Roll_dot_dot reference [rad/s^2], "
      << "Pitch_dot_dot reference [rad/s^2], "
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
      
	BOOST_LOG(logger_logdata) << oss.str();
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
  std::string log_base_directory = "./src/flightstack/log/" + current_date;

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
  logger_logdata.add_attribute("Tag", attrs::constant<std::string>("LogDataTag"));

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
void LogData_PID::logLogData(const LogData_PID& data)
{
	std::ostringstream oss;

	oss << data.timestamp_initial << ", "
	    << data.time_current << ", "
      << data.user_defined_position(0) << ", "
      << data.user_defined_position(1) << ", "
      << data.user_defined_position(2) << ", "
      << data.user_defined_velocity(0) << ", "
      << data.user_defined_velocity(1) << ", "
      << data.user_defined_velocity(2) << ", "
      << data.user_defined_acceleration(0) << ", "
      << data.user_defined_acceleration(1) << ", "
      << data.user_defined_acceleration(2) << ", "
      << data.user_defined_yaw << ", "
      << data.user_defined_yaw_dot << ", "
      << data.user_defined_yaw_dot_dot << ", "
      << data.time_odometry << ", "
      << data.position(0) << ", "
      << data.position(1) << ", "
      << data.position(2) << ", "
      << data.q.w() << ", "
      << data.q.x() << ", "
      << data.q.y() << ", "
      << data.q.z() << ", "
      << data.velocity(0) << ", "
      << data.velocity(1) << ", "
      << data.velocity(2) << ", "
      << data.angular_velocity(0) << ", "
      << data.angular_velocity(1) << ", "
      << data.angular_velocity(2) << ", "
      << data.euler_angles_rpy(0) << ", "
      << data.euler_angles_rpy(1) << ", "
      << data.euler_angles_rpy(2) << ", "
      << data.algorithm_execution_time_microseconds.count() << ", "
      << data.mu_translational_raw(0) << ", "
      << data.mu_translational_raw(1) << ", "
      << data.mu_translational_raw(2) << ", "
      << data.mu_translational(0) << ", "
      << data.mu_translational(1) << ", "
      << data.mu_translational(2) << ", "
      << data.U_control_inputs(0) << ", "
      << data.U_control_inputs(1) << ", "
      << data.U_control_inputs(2) << ", "
      << data.U_control_inputs(3) << ", "
      << data.roll_reference << ", "
      << data.pitch_reference << ", "
      << data.roll_reference_dot << ", "
      << data.pitch_reference_dot << ", "
      << data.roll_reference_dot_dot << ", "
      << data.pitch_reference_dot_dot << ", "
      << data.euler_angles_rpy_dot(0) << ", "
      << data.euler_angles_rpy_dot(1) << ", "
      << data.euler_angles_rpy_dot(2) << ", "
      << data.thrust_vector_quadcopter(0) << ", "
      << data.thrust_vector_quadcopter(1) << ", "
      << data.thrust_vector_quadcopter(2) << ", "
      << data.thrust_vector_quadcopter(3) << ", "
      << data.thrust_vector_quadcopter_normalized(0) << ", "
      << data.thrust_vector_quadcopter_normalized(1) << ", "
      << data.thrust_vector_quadcopter_normalized(2) << ", "
      << data.thrust_vector_quadcopter_normalized(3) << ", "
      << data.thrust_vector(0) << ", "
      << data.thrust_vector(1) << ", "
      << data.thrust_vector(2) << ", "
      << data.thrust_vector(3) << ", "
      << data.thrust_vector(4) << ", "
      << data.thrust_vector(5) << ", "
      << data.thrust_vector(6) << ", "
      << data.thrust_vector(7) << ", "
      << data.thrust_vector_normalized(0) << ", "
      << data.thrust_vector_normalized(1) << ", "
      << data.thrust_vector_normalized(2) << ", "
      << data.thrust_vector_normalized(3) << ", "
      << data.thrust_vector_normalized(4) << ", "
      << data.thrust_vector_normalized(5) << ", "
      << data.thrust_vector_normalized(6) << ", "
      << data.thrust_vector_normalized(7) << ", "
      << data.angular_error(0) << ", "
      << data.angular_error(1) << ", "
      << data.angular_error(2) << ", "
      << data.outer_loop_P(0) << ", "
      << data.outer_loop_P(1) << ", "
      << data.outer_loop_P(2) << ", "
      << data.outer_loop_I(0) << ", "
      << data.outer_loop_I(1) << ", "
      << data.outer_loop_I(2) << ", "
      << data.outer_loop_D(0) << ", "
      << data.outer_loop_D(1) << ", "
      << data.outer_loop_D(2) << ", "
      << data.outer_loop_dynamic_inversion(0) << ", "
      << data.outer_loop_dynamic_inversion(1) << ", "
      << data.outer_loop_dynamic_inversion(2) << ", "
      << data.inner_loop_P(0) << ", "
      << data.inner_loop_P(1) << ", "
      << data.inner_loop_P(2) << ", "
      << data.inner_loop_I(0) << ", "
      << data.inner_loop_I(1) << ", "
      << data.inner_loop_I(2) << ", "
      << data.inner_loop_D(0) << ", "
      << data.inner_loop_D(1) << ", "
      << data.inner_loop_D(2) << ", "
      << data.inner_loop_dynamic_inversion(0) << ", "
      << data.inner_loop_dynamic_inversion(1) << ", "
      << data.inner_loop_dynamic_inversion(2) << ", "
      ;

	BOOST_LOG(logger_logdata) << oss.str();
}

