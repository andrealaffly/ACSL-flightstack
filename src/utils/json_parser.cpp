/* json_parser.cpp

	Mattia Gramuglia
	April 2, 2024
*/

#include <fstream>
#include <nlohmann/json.hpp>

#include "json_parser.hpp"

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

// Function to extract a Eigen::Matrix3d from JSON and populate Eigen matrix
Eigen::Matrix3d extractMatrix3dFromJSON(const nlohmann::json& jsonMatrix) {
	Eigen::Matrix3d matrix;
	for (int i = 0; i < 3; ++i) {
		for (int j = 0; j < 3; ++j) {
			matrix(i, j) = jsonMatrix[i][j];
		}
	}
	return matrix;
}

