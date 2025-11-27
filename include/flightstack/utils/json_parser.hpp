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
#pragma once

#include <cmath>
#include <string>
#include <fstream>
#include <array>
#include <type_traits>
#include <nlohmann/json.hpp>

#include <Eigen/Dense>

#include "flightstack/utils/low_pass_filter.hpp"

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
	// Initialize the Eigen matrix
	Eigen::Matrix<Scalar, Rows, Cols> matrix;

	// Check if the matrix has a scaling coefficient
	if (jsonMatrix.contains("scaling_coef")) {
		// Extract the scaling coefficient
		Scalar scaling_coef = jsonMatrix["scaling_coef"];

		// Populate the Eigen matrix with the values from the JSON, scaled by the scaling coefficient
		for (int i = 0; i < Rows; ++i) {
			for (int j = 0; j < Cols; ++j) {
				matrix(i, j) = jsonMatrix["matrix"][i][j];
			}
		}

		matrix *= scaling_coef; // Apply scaling
	}
	else
	{
		// No scaling coefficient, just populate the matrix
		for (int i = 0; i < Rows; ++i) {
			for (int j = 0; j < Cols; ++j) {
				matrix(i, j) = jsonMatrix[i][j];
			}
		}
	}

	return matrix;
}

// Make Eigen matrices serializable using nlohmann::json
namespace nlohmann {

template <typename Scalar, int Rows, int Cols>
struct adl_serializer<Eigen::Matrix<Scalar, Rows, Cols>>
{
	static void to_json(json& j, const Eigen::Matrix<Scalar, Rows, Cols>& mat)
	{
		j = json::array();
		for (int i = 0; i < mat.rows(); ++i) {
			json row = json::array();
			for (int k = 0; k < mat.cols(); ++k)
				row.push_back(mat(i, k));
			j.push_back(row);
		}
	}

	static void from_json(const json& j, Eigen::Matrix<Scalar, Rows, Cols>& mat)
	{
		mat.resize(j.size(), j.at(0).size());
		for (int i = 0; i < mat.rows(); ++i)
			for (int k = 0; k < mat.cols(); ++k)
				mat(i, k) = j.at(i).at(k);
	}
};

} // namespace nlohmann


template<typename T>
double DEG2RAD( const T deg )
{
	return deg * M_PI / 180.0;
}

template<typename T>
double RAD2DEG( const T rad )
{
	return rad * 180.0 / M_PI;
}


namespace LowPassFilterJSON {

template <typename FilterType>
inline std::array<typename LowPassFilter::FilterVector3d<FilterType>::Config, 3>
extractFilterConfigFromJSON(const nlohmann::json& j_filter)
{
	using Config = typename LowPassFilter::FilterVector3d<FilterType>::Config;
	std::array<Config, 3> configs;

	constexpr std::array<const char*, 3> axes = {"x", "y", "z"};

	for (size_t i = 0; i < axes.size(); ++i) {
		const auto& axis = axes[i];
		configs[i].cutoff_hz = j_filter.at(axis).at("cutoff_hz").get<double>();
		configs[i].sample_rate_hz = j_filter.at(axis).at("sample_rate_hz").get<double>();

		if constexpr (std::is_same_v<FilterType, LowPassFilter::SecondOrder>) {
			configs[i].q = j_filter.at(axis).value("q", 0.7071);
		}
	}

	return configs;
}

} // namespace LowPassFilterJSON



