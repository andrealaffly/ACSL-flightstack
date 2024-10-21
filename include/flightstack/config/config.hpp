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
 * File:        config.hpp
 * Author:      Mattia Gramuglia
 * Date:        June 7, 2024
 * For info:    Andrea L'Afflitto 
 *              a.lafflitto@vt.edu
 * 
 * Description: Configuration file. Here you specify which controller you want to run at compile time.
 * 
 * GitHub:    https://github.com/andrealaffly/ACSL-flightstack.git
 **********************************************************************************************************************/

/**
 * @file config.hpp
 * @brief Configuration file. Here you specify which cotroller you want to run at compile time.
 */
#ifndef CONFIG_HPP
#define CONFIG_HPP

#include <fstream>
#include <nlohmann/json.hpp>

// Forward declaration of MultiThreadedNode class

/**
 * @class MultiThreadedNode
 * 
 */
class MultiThreadedNode;

/*********************************************************************************************************************
  CONTROLLER selection
**********************************************************************************************************************
*/
// Define named constants for controller types
#define __PID__ 1
#define __MRAC__ 2

// SELECT here the CONTROLLER you want to run -----------------------------------------------------------------------
#define SELECTED_CONTROLLER __MRAC__
// ------------------------------------------------------------------------------------------------------------------

// Define ControlType based on SELECTED_CONTROLLER using type aliasing
#if SELECTED_CONTROLLER == __PID__ 

#include "pid.hpp"
#include "logging_pid.hpp"
using ControlType = PID;

#elif SELECTED_CONTROLLER == __MRAC__

#include "mrac.hpp"
#include "logging_mrac.hpp"
using ControlType = MRAC;

#else
#error "ERROR: Unsupported controller type selected."
#endif


/*********************************************************************************************************************
  Configurations Parameters set from config.json 
**********************************************************************************************************************
*/

/**
 * @struct ConfigurationParameters
 * @brief Configurations Parameters set from config.json
 * 
 */
struct ConfigurationParameters
{
  // Name of the piecewise polynomial user-defined trajectory file
  std::string user_defined_trajectory_file;

  // Time in seconds for which the UAV will perform hovering after having completed the user-defined trajectory
  double hover_after_trajectory_time_seconds;

  static ConfigurationParameters readConfigurationParametersFile(const std::string& configFileName);

};

/*********************************************************************************************************************
  Global Parameters shared among all the program
**********************************************************************************************************************
*/

/**
 * @struct GlobalParameters
 * @brief Global parameteres shared among all the program
 * 
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




#endif // CONFIG_HPP
