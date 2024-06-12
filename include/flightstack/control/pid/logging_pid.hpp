/* logging_pid.hpp

	Mattia Gramuglia
	April 18, 2024
*/

#ifndef LOGGING_PID_HPP
#define LOGGING_PID_HPP

#include <atomic>
#include <cstddef>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <ostream>
#include <sstream>
#include <string>

#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/date_time/posix_time/posix_time_types.hpp>
#include <boost/log/attributes.hpp>
#include <boost/log/attributes/scoped_attribute.hpp>
#include <boost/log/core.hpp>
#include <boost/log/expressions.hpp>
#include <boost/log/sinks/sync_frontend.hpp>
#include <boost/log/sinks/text_ostream_backend.hpp>
#include <boost/log/sources/basic_logger.hpp>
#include <boost/log/sources/logger.hpp>
#include <boost/log/sources/record_ostream.hpp>
#include <boost/log/sources/severity_channel_logger.hpp>
#include <boost/log/sources/severity_logger.hpp>
#include <boost/log/support/date_time.hpp>
#include <boost/log/trivial.hpp>
#include <boost/log/utility/setup/common_attributes.hpp>
#include <boost/log/utility/setup/file.hpp>
#include <boost/log/utility/value_ref.hpp>
#include <boost/phoenix/bind.hpp>
#include <boost/smart_ptr/make_shared_object.hpp>
#include <boost/smart_ptr/shared_ptr.hpp>
#include <Eigen/Dense>

namespace logging = boost::log;
namespace sinks = boost::log::sinks;
namespace src = boost::log::sources;
namespace expr = boost::log::expressions;
namespace attrs = boost::log::attributes;
namespace keywords = boost::log::keywords;

// Define logger for LogData
extern src::logger logger_logdata;

// Forward declaration of MultiThreadedNode class
class MultiThreadedNode;

// Forward declaration of PID class
class PID;

class LogData_PID 
{
public:

	// Constructor
  LogData_PID(MultiThreadedNode& node, PID& controller);

  // References to variables that will be logged
	const std::atomic<uint64_t>& timestamp_initial;
	const std::atomic<double>& time_current;
  const Eigen::Vector3d& user_defined_position;
  const Eigen::Vector3d& user_defined_velocity;
  const Eigen::Vector3d& user_defined_acceleration;
  const double& user_defined_yaw;
  const double& user_defined_yaw_dot;
  const double& user_defined_yaw_dot_dot;

  std::atomic<double>& time_odometry;
  Eigen::Vector3d& position;           
  Eigen::Quaterniond& q;               
  Eigen::Vector3d& velocity;            
  Eigen::Vector3d& angular_velocity;   
  Eigen::Vector3d& euler_angles_rpy; 

  const std::chrono::duration<double, std::micro>& algorithm_execution_time_microseconds;
  const Eigen::Vector3d& mu_translational_raw;
  const Eigen::Vector3d& mu_translational;
  const Eigen::Vector4d& U_control_inputs;
  const double& roll_reference;
  const double& pitch_reference; 
  const double& roll_reference_dot;
  const double& pitch_reference_dot; 
  const double& roll_reference_dot_dot;
  const double& pitch_reference_dot_dot;
  const Eigen::Vector3d& euler_angles_rpy_dot;
  const Eigen::Vector4d& thrust_vector_quadcopter; 
  const Eigen::Vector4d& thrust_vector_quadcopter_normalized;
  const Eigen::Matrix<double, 8, 1>& thrust_vector;
  const Eigen::Matrix<double, 8, 1>& thrust_vector_normalized;
  const Eigen::Vector3d& angular_error;
  const Eigen::Vector3d& outer_loop_P; 
  const Eigen::Vector3d& outer_loop_I; 
  const Eigen::Vector3d& outer_loop_D; 
  const Eigen::Vector3d& outer_loop_dynamic_inversion; 
  const Eigen::Vector3d& inner_loop_P; 
  const Eigen::Vector3d& inner_loop_I; 
  const Eigen::Vector3d& inner_loop_D; 
  const Eigen::Vector3d& inner_loop_dynamic_inversion; 

  void logInitializeHeaders();
  void logInitializeLogging();
  void logLogData(const LogData_PID& data);
  

private:

	MultiThreadedNode& node_;
	

};


#endif // LOGGING_PID_HPP