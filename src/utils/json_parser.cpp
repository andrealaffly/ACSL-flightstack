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
 * File:        json_parser.cpp \n
 * Author:      Mattia Gramuglia \n 
 * Date:        April 2, 2024 \n
 * For info:    Andrea L'Afflitto 
 *              a.lafflitto@vt.edu
 * 
 * Description: Utility functions to parse JSON files
 * 
 * GitHub:    https://github.com/andrealaffly/ACSL-flightstack.git
 **********************************************************************************************************************/

/**
 * @file json_parser.cpp
 * @brief Utility functions to parse JSON files
 */
#include <fstream>
#include <nlohmann/json.hpp>

#include "json_parser.hpp"

/**
 * @class MotorsCommands
 */
void MotorsCommands::readJSONConfig(const std::string& jsonFile) {
	std::ifstream file(jsonFile);
	nlohmann::json j;
	file >> j;
	
	this->motor_1 = j["MotorsCommands"]["motor_1"];
	this->motor_2 = j["MotorsCommands"]["motor_2"];
	this->motor_3 = j["MotorsCommands"]["motor_3"];
	this->motor_4 = j["MotorsCommands"]["motor_4"];
	this->motor_5 = j["MotorsCommands"]["motor_5"];
	this->motor_6 = j["MotorsCommands"]["motor_6"];
	this->motor_7 = j["MotorsCommands"]["motor_7"];
	this->motor_8 = j["MotorsCommands"]["motor_8"];
	
}


