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
 * File:        pcac_gains.hpp
 * Author:      Mattia Gramuglia
 * Date:        April 23, 2024
 * For info:    Andrea L'Afflitto 
 *              a.lafflitto@vt.edu
 * 
 * Description: Tuning gains of the PCAC controller.
 * 
 * GitHub:    https://github.com/andrealaffly/ACSL-flightstack.git
 **********************************************************************************************************************/

#pragma once

#include <cmath>
#include <algorithm>

#include <Eigen/Dense>
#include <nlohmann/json.hpp>

using Eigen::MatrixXd;
using std::string;
using nlohmann::json;

  /*
    PCAC coding style guide
    1. Variables are camelCase
    2. Variables must not include digits
    3. Matrices are PascalCase and may use numbers
    4. Instances of gains structure are all UPPERCASE
    5. 
  */
inline constexpr double GRAVITATIONAL_ACCELERATION = 9.80665; // [m/s^2]

// Struct containing the PCAC parameters coming from the .json file 
class ParamsVehicle
{
  public:
  double motorSaturationUpper;
  double motorSaturationLower;
  double minValuePublishMotors;
  double mass;

  Eigen::Matrix3d inertia_matrix;
  double air_density;
  double surface_area;
  Eigen::Matrix3d drag_coefficient_matrix;

  double distance_motors_centerline_x_dir;
  double distance_motors_centerline_y_dir;
  double propeller_drag_coefficient;

  // Filter/differentiator values to compute roll_desired_dot
  Eigen::Matrix2d A_filter_roll_des;
  Eigen::Vector2d B_filter_roll_des;
  Eigen::RowVector2d C_filter_roll_des;
  double D_filter_roll_des;

  // Filter/differentiator values to compute pitch_desired_dot
  Eigen::Matrix2d A_filter_pitch_des;
  Eigen::Vector2d B_filter_pitch_des;
  Eigen::RowVector2d C_filter_pitch_des;
  double D_filter_pitch_des;

  // Filter/differentiator values to compute roll_desired_dot_dot
  Eigen::Matrix2d A_filter_roll_dot_des;
  Eigen::Vector2d B_filter_roll_dot_des;
  Eigen::RowVector2d C_filter_roll_dot_des;
  double D_filter_roll_dot_des;

  // Filter/differentiator values to compute pitch_desired_dot_dot
  Eigen::Matrix2d A_filter_pitch_dot_des;
  Eigen::Vector2d B_filter_pitch_dot_des;
  Eigen::RowVector2d C_filter_pitch_dot_des;
  double D_filter_pitch_dot_des;

  // Declare matrix mixer_matrix using a lambda function
  Eigen::Matrix<double, 8, 4> mixer_matrix;

  // Declare matrix mixer_matrix_quadcopter using a lambda function
  Eigen::Matrix4d mixer_matrix_quadcopter;

  // Polynomial coefficients vector to evaluate the Commanded Thrust [-] based on the Thrust in Newton
  // S500 with Holybro props
  Eigen::VectorXd thrust_polynomial_coefficients_quadcopter;
  
  ParamsVehicle();
  ParamsVehicle(json j);
};
