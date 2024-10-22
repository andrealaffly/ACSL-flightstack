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
 * File:        config.cpp
 * Author:      Mattia Gramuglia
 * Date:        June 7, 2024
 * For info:    Andrea L'Afflitto 
 *              a.lafflitto@vt.edu
 * 
 * Description: Configuration file
 * 
 * GitHub:    https://github.com/andrealaffly/ACSL-flightstack.git
 **********************************************************************************************************************/

/**
 * @file config.cpp
 * @brief Contains the functions that reads the configuration parameters and assigns it to the members of the ConfigurationParameters struct .
 */
#include "config.hpp"
#include "multi_threaded_node.hpp"

/*
  Function that reads the configuration parameters and assigns it to the members of the 
  ConfigurationParameters struct
*/
/**
 * @class ConfigurationParameters
 */
ConfigurationParameters ConfigurationParameters::readConfigurationParametersFile(const std::string& configFileName)
{
  // Define the path where the user-defined trajectory JSON files are located
  const std::string path = "./src/flightstack/params/config/";

  // Concatenate the path with the file name
  std::string jsonFile = path + configFileName;

  std::ifstream config_file(jsonFile);
    if (!config_file.is_open()) {
      throw std::runtime_error("Could not open config file: " + jsonFile);
    }

  nlohmann::json config_json;
  config_file >> config_json;

  ConfigurationParameters config;

  if (config_json.contains("user_defined_trajectory_file")) {
    config.user_defined_trajectory_file = config_json["user_defined_trajectory_file"].get<std::string>();
  }

  if (config_json.contains("hover_after_trajectory_time_seconds")) {
    config.hover_after_trajectory_time_seconds = config_json["hover_after_trajectory_time_seconds"].get<double>();
  }

  return config;
}

/**
 * @struct Global Parameters
 */
// Constructor
GlobalParameters::GlobalParameters(MultiThreadedNode& node) :
  PUBLISH_ACTUATOR_MOTORS_FLAG(true),
  ARM_START_TIME_SECONDS(5.0),
  TAKEOFF_START_TIME_SECONDS(10.0),
  LANDING_START_TIME_SECONDS(
    this->TAKEOFF_START_TIME_SECONDS + 
    node.getUserDefinedTrajectory()->getWaypointTimes().back() + 
    node.getConfigurationParameters().hover_after_trajectory_time_seconds),
  LANDING_END_TIME_SECONDS(this->LANDING_START_TIME_SECONDS + 4.0),
  DISARM_START_TIME_SECONDS(this->LANDING_END_TIME_SECONDS + 1.0),
  OFFSET_ODOMETRY_TIME_SECONDS(this->TAKEOFF_START_TIME_SECONDS - 1.0)
{}