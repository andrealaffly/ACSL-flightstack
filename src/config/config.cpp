/* config.cpp

Mattia Gramuglia
June 7, 2024
*/

#include "config.hpp"
#include "multi_threaded_node.hpp"

/*
  Function that reads the configuration parameters and assigns it to the members of the 
  ConfigurationParameters struct
*/
ConfigurationParameters readConfigurationParametersFile(const std::string& configFileName)
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

  if (config_json.contains("controller_type")) {
    config.controller_type = config_json["controller_type"].get<std::string>();
  }

  if (config_json.contains("hover_after_trajectory_time_seconds")) {
    config.hover_after_trajectory_time_seconds = config_json["hover_after_trajectory_time_seconds"].get<double>();
  }

  // config.user_defined_trajectory_file = config_json["user_defined_trajectory_file"];
  // config.controller_type = config_json["controller_type"];

  return config;
}

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