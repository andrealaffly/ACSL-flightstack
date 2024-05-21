/* json_parser.hpp

	Mattia Gramuglia
	April 2, 2024
*/

#ifndef JSON_PARSER_HPP
#define JSON_PARSER_HPP

#include <string>
#include <fstream>
#include <nlohmann/json.hpp>

#include <Eigen/Dense>

struct MotorsCommands {

	void readJSONConfig(const std::string& jsonFile);

	float motor_1;
	float motor_2;
	float motor_3;
	float motor_4;
	float motor_5;
	float motor_6;
	float motor_7;
	float motor_8;
};

Eigen::Matrix3d extractMatrix3dFromJSON(const nlohmann::json& jsonMatrix);



#endif // JSON_PARSER_HPP


