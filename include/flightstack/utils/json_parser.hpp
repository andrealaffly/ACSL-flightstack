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
 * File:        json_parser.hpp
 * Author:      Mattia Gramuglia
 * Date:        April 2, 2024
 * For info:    Andrea L'Afflitto 
 *              a.lafflitto@vt.edu
 * 
 * Description: Utility functions to parse JSON files
 * 
 * GitHub:    https://github.com/andrealaffly/ACSL-flightstack.git
 **********************************************************************************************************************/

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

/*
 *	Function to extract a Matrix from JSON and populate a Eigen matrix
 *	Examples of usage:
 *		Eigen::Matrix<double, 3, 3> matrix33d = extractMatrixFromJSON<double, 3, 3>(jsonMatrix);
 *		Eigen::Matrix<float, 4, 4> matrix44f = extractMatrixFromJSON<float, 4, 4>(jsonMatrix);
**/
template<typename Scalar, int Rows, int Cols>
inline Eigen::Matrix<Scalar, Rows, Cols> extractMatrixFromJSON(const nlohmann::json& jsonMatrix)
{
	// Extract the scaling coefficient
	Scalar scaling_coef = jsonMatrix["scaling_coef"];

	// Initialize the Eigen matrix
	Eigen::Matrix<Scalar, Rows, Cols> matrix;

	// Populate the Eigen matrix with the values from the JSON, scaled by the scaling coefficient
	for (int i = 0; i < Rows; ++i) {
		for (int j = 0; j < Cols; ++j) {
			matrix(i, j) = jsonMatrix["matrix"][i][j];
		}
	}

	matrix *= scaling_coef;

	return matrix;
}




#endif // JSON_PARSER_HPP


