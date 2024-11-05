///@cond
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
///@endcond
/***********************************************************************************************************************
 * File:        logging_mocap.cpp \n
 * Author:      Mattia Gramuglia \n
 * Date:        May 10, 2024 \n 
 * For info:    Andrea L'Afflitto 
 *              a.lafflitto@vt.edu
 * 
 * Description: Logger for the motion capture (MOCAP) system.
 * 
 * GitHub:    https://github.com/andrealaffly/ACSL-flightstack.git
 **********************************************************************************************************************/

/**
 * @file logging_mocap.cpp
 * @brief Logger for the motion capture (MOCAP) system. 
 */

#include "config.hpp"
#include "logging_mocap.hpp"
#include "mocap.hpp"


namespace _drivers_
{
namespace _udp_driver_
{

// Define the logger for MocapData
src::logger MocapData::logger_mocapdata;

// Constructor
/**
 * MocapData
 * @brief Construct a new Mocap Data:: Mocap Data object
 * @param node 
 */
MocapData::MocapData(UdpReceiverNode& node) :
	node_(node) // node must be the last in the list to be initialized
{}

/**
 * @brief MocapData Initializing headers
 * @param None
 * 
 */
void MocapData::logInitializeHeaders()
{
	std::ostringstream oss;

	oss << "Mocap timestamp, "
	    << "Mocap time [s], "
      << "Mocap position x [m], "
      << "Mocap position y [m], "
      << "Mocap position z [m], "
      << "Mocap Quaternion q0 [-], "
      << "Mocap Quaternion q1 [-], "
      << "Mocap Quaternion q2 [-], "
      << "Mocap Quaternion q3 [-], "
      << "Mocap Velocity x [m/s], "
      << "Mocap Velocity y [m/s], "
      << "Mocap Velocity z [m/s], "
      << "Mocap Angular velocity x [rad/s], "
      << "Mocap Angular velocity y [rad/s], "
      << "Mocap Angular velocity z [rad/s], "
  ;
      
	BOOST_LOG(MocapData::logger_mocapdata) << oss.str();
}

// Function to initialize the logging system

/*!
 * @brief Function to initialize the logging system
 * @param None
 */

void MocapData::logInitializeLogging()
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

  // Check if the "mocap" subdirectory exists within the log_base_directory, create it if needed
  std::string mocap_directory = log_base_directory + "/mocap";
  if (!std::filesystem::exists(mocap_directory)) {
      std::filesystem::create_directories(mocap_directory);
  }

  // Generate the mocap log file name with a timestamp
  std::stringstream mocap_ss;
  mocap_ss << mocap_directory << "/mocap_log_" << std::put_time(std::localtime(&now_c), "%Y%m%d_%H%M%S") << ".log";
  std::string mocap_log_filename = mocap_ss.str();

  MocapData::logger_mocapdata.add_attribute("Tag", attrs::constant< std::string >("MocapTag"));

  // Define a synchronous sink with a text ostream backend
  typedef sinks::synchronous_sink<sinks::text_ostream_backend> text_sink;
  boost::shared_ptr<text_sink> mocap_sink = boost::make_shared<text_sink>();

  // Add a stream to the backend (in this case, a file stream)
  mocap_sink->locked_backend()->add_stream(boost::make_shared<std::ofstream>(mocap_log_filename));

  // Set the formatter for the sink
  mocap_sink->set_formatter(
    expr::stream
    << "[" << expr::format_date_time<boost::posix_time::ptime>("TimeStamp", "%Y-%m-%d %H:%M:%S.%f") << "] " // Format date and time
    << "[" << expr::attr<boost::log::attributes::current_thread_id::value_type>("ThreadID") << "] " // Current thread ID
    << "[" << expr::attr<std::string>("Tag") << "] " // Tag attribute value
    << "[" << expr::attr<boost::log::attributes::current_process_id::value_type>("ProcessID") << "] " // Current process ID
    << "[" << expr::attr<unsigned int>("LineID") << "] " // Line ID
    << expr::smessage // Log message
  );

  // Add the sink to the logging core
  logging::core::get()->add_sink(mocap_sink);

  // Set a filter for the sink
  mocap_sink->set_filter(expr::has_attr("Tag") && expr::attr<std::string>("Tag") == "MocapTag");

  logging::add_common_attributes(); // Add attributes like timestamp

  logInitializeHeaders();
}

// Function to log the data
void MocapData::logMocapData()
{
	std::ostringstream oss;

	oss << node_.getTimestampMocap() << ", "
	    << node_.getTimeMocap() << ", "
      << node_.getMocapStates().x << ", "
      << node_.getMocapStates().y << ", "
      << node_.getMocapStates().z << ", "
      << node_.getMocapStates().q0 << ", "
      << node_.getMocapStates().q1 << ", "
      << node_.getMocapStates().q2 << ", "
      << node_.getMocapStates().q3 << ", "
      << node_.getMocapStates().vx << ", "
      << node_.getMocapStates().vy << ", "
      << node_.getMocapStates().vz << ", "
      << node_.getMocapStates().rollspeed << ", "
      << node_.getMocapStates().pitchspeed << ", "
      << node_.getMocapStates().yawspeed << ", "
      ;

	BOOST_LOG(MocapData::logger_mocapdata) << oss.str();
}

} // namespace _udp_driver_    
} // namespace _drivers_