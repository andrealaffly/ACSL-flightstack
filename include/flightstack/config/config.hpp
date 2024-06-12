/* config.hpp

Mattia Gramuglia
June 7, 2024
*/

#ifndef CONFIG_HPP
#define CONFIG_HPP

#include <fstream>
#include <nlohmann/json.hpp>

// Forward declaration of MultiThreadedNode class
class MultiThreadedNode;

/*********************************************************************************************************************
  Configurations Parameters set from config.json 
**********************************************************************************************************************
*/
struct ConfigurationParameters
{
  // Name of the piecewise polynomial user-defined trajectory file
  std::string user_defined_trajectory_file;

  // Controller type to run onboard
  std::string controller_type;

  // Time in seconds for which the UAV will perform hovering after having completed the user-defined trajectory
  double hover_after_trajectory_time_seconds;

};

/*********************************************************************************************************************
  Global Parameters shared among all the program
**********************************************************************************************************************
*/
struct GlobalParameters
{
  // Flag for publishing data to the motors
  bool PUBLISH_ACTUATOR_MOTORS_FLAG;

  // Time in seconds at which we arm the motors
  double ARM_START_TIME_SECONDS; 

  // Time in seconds at which we start the mission/takeoff
  double TAKEOFF_START_TIME_SECONDS; 

  // Time in seconds at which we start the landing
  double LANDING_START_TIME_SECONDS; 

  // Time in seconds at which we end the landing
  double LANDING_END_TIME_SECONDS; 

  // Time in seconds at which we disarm the motors
  double DISARM_START_TIME_SECONDS; 

  // Time in seconds at which we perform the offset of the VehicleState class members
  // This must always be performed before takeoff
  double OFFSET_ODOMETRY_TIME_SECONDS;

  // Default constructor
  GlobalParameters() = default;

  // Constructor
  GlobalParameters(MultiThreadedNode& node);

};

ConfigurationParameters readConfigurationParametersFile(const std::string& configFileName);



#endif // CONFIG_HPP
