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
 * File:        logging_funnel_two_layer_mrac.hpp
 * Author:      Mattia Gramuglia
 * Date:        December 04, 2024
 * For info:    Andrea L'Afflitto 
 *              a.lafflitto@vt.edu
 * 
 * Description: Logger for the FunnelTwoLayerMRAC controller.
 * 
 * GitHub:    https://github.com/andrealaffly/ACSL-flightstack.git
 **********************************************************************************************************************/

#pragma once

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

#include "logging_helpers.hpp"
#include "git_info_utils.hpp"


namespace logging = boost::log;
namespace sinks = boost::log::sinks;
namespace src = boost::log::sources;
namespace expr = boost::log::expressions;
namespace attrs = boost::log::attributes;
namespace keywords = boost::log::keywords;

// Forward declaration of MultiThreadedNode class
class MultiThreadedNode;

// Forward declaration of FunnelTwoLayerMRAC class
class FunnelTwoLayerMRAC;

// Struct that contains info related to the logger that can be accessed via a getter from the logger class.
struct LoggingInfo
{
  std::string timestamp;             // YYYYMMDD_HHMMSS
  std::string date;                  // YYYYMMDD

  std::string base_dir;              // ./src/flightstack/log/<date>/<controller>

  std::string logs_dir;
  std::string log_filename;

  std::string gains_dir;
  std::string gains_target_filename;

  std::string info_dir;
  std::string safe_mech_dir;
  std::string safe_mech_target_filename;
  std::string git_info_dir;
  std::string git_info_filename;
  std::string low_pass_filter_dir;
  std::string low_pass_filter_filename;

  std::string der_gains_dir;
  std::string der_gains_filename;

  std::string logs_debug_dir;
  std::string log_debug_filename;
};

class LogData_FunnelTwoLayerMRAC
{
public:

  // Define logger for LogData
  static src::logger logger_logdata;

	// Constructor
  LogData_FunnelTwoLayerMRAC(MultiThreadedNode& node, FunnelTwoLayerMRAC& controller);

  void logInitializeHeaders();
  void logInitializeLogging();

  void generateTimestampAndDate();
  void createLogDirectories();
  void generateLogFilenames();
  void setupMainLogSink();
  void copyParameterFiles();
  void setupDebugLogSink();
  void writeGitMetadata();

  void logLogData();

  // Getter for the logging info
  const LoggingInfo& getLoggingInfo() const { return logging_info_; }

private:

	MultiThreadedNode& node_;

  FunnelTwoLayerMRAC& controller_;

  LoggingInfo logging_info_;
	
};