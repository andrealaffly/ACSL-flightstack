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
 * File:        vehicle_info.hpp
 * Author:      Mattia Gramuglia
 * Date:        April 22, 2024
 * For info:    Andrea L'Afflitto 
 *              a.lafflitto@vt.edu
 * 
 * Description: Vehicle properties such as mass, inertia matrix, motor thrust curves, etc.
 * 
 * GitHub:    https://github.com/andrealaffly/ACSL-flightstack.git
 **********************************************************************************************************************/

#ifndef VEHICLE_INFO_HPP
#define VEHICLE_INFO_HPP

#include <cmath>

#include <Eigen/Dense>

inline constexpr double GRAVITATIONAL_ACCELERATION = 9.80665; // [m/s^2]

inline constexpr double UPPER_MOTOR_THRUST_SATURATION_LIMIT_IN_NEWTON = 9.5;

inline constexpr double LOWER_MOTOR_THRUST_SATURATION_LIMIT_IN_NEWTON = 0.3;

inline constexpr double MINIMUM_VALUE_PUBLISH_MOTORS = 0.05;


struct VehicleInfo 
{

  const double mass = 1.920; // [kg] vehicle mass

  const Eigen::Matrix3d inertia_matrix = (Eigen::Matrix3d() << 
     2.271990e-02, -6.557000e-06, -1.003498e-03,
    -6.557000e-06,  2.202047e-02,  5.658400e-06,
    -1.003498e-03,  5.658400e-06,  1.614693e-02
  ).finished(); // [kg*m^2] inertia matrix of the vehicle system (drone frame + box + propellers) expressed in
                // Pixhawk coordinate system (FRD - x-Front, y-Right, z-Down), computed at the vehicle center of mass
  
  const double air_density = 1.225; // [kg/m^3] air density

  const double surface_area = 0.07; // [m^2] surface area of the drone to account fro the drag

  const double drag_coefficient = 1.28; // [-] drag coefficient (equal to that of a plate)
  const Eigen::Matrix3d drag_coefficient_matrix = (Eigen::Matrix3d() << 
    drag_coefficient, 0,                0,
    0,                drag_coefficient, 0,
    0,                0,                0
  ).finished();

  const double distance_motors_cenetrline_x_dir = 0.0881269; // [m] distance between the centerline of the drone and the
                                                            // motors along x direction in local FRD frame
  const double distance_motors_cenetrline_y_dir = 0.1083450; // [m] distance between the centerline of the drone and the
                                                            // motors along y direction in local FRD frame
  const double propeller_drag_coefficient = 0.01; // [m] propellers drag coefficient

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

  // Declare const matrix mixer_matrix using a lambda function
  const Eigen::Matrix<double, 8, 4> mixer_matrix = [this]() {
    Eigen::Matrix<double, 8, 4> mat; // 8 rows, 4 columns

    // [1/8, -1/(8*l_y),  1/(8*l_x),  1/(8*c_t)]
    // [1/8,  1/(8*l_y),  1/(8*l_x), -1/(8*c_t)]
    // [1/8,  1/(8*l_y), -1/(8*l_x),  1/(8*c_t)]
    // [1/8, -1/(8*l_y), -1/(8*l_x), -1/(8*c_t)]
    // [1/8,  1/(8*l_y),  1/(8*l_x),  1/(8*c_t)]
    // [1/8, -1/(8*l_y),  1/(8*l_x), -1/(8*c_t)]
    // [1/8, -1/(8*l_y), -1/(8*l_x),  1/(8*c_t)]
    // [1/8,  1/(8*l_y), -1/(8*l_x), -1/(8*c_t)]
    
    // Assign values element-by-element
    mat(0, 0) =  1.0 / 8.0;
    mat(0, 1) = -1.0 / (8.0 * this->distance_motors_cenetrline_y_dir);
    mat(0, 2) =  1.0 / (8.0 * this->distance_motors_cenetrline_x_dir);
    mat(0, 3) =  1.0 / (8.0 * this->propeller_drag_coefficient);

    mat(1, 0) =  1.0 / 8.0;
    mat(1, 1) =  1.0 / (8.0 * this->distance_motors_cenetrline_y_dir);
    mat(1, 2) =  1.0 / (8.0 * this->distance_motors_cenetrline_x_dir);
    mat(1, 3) = -1.0 / (8.0 * this->propeller_drag_coefficient);

    mat(2, 0) =  1.0 / 8.0;
    mat(2, 1) =  1.0 / (8.0 * this->distance_motors_cenetrline_y_dir);
    mat(2, 2) = -1.0 / (8.0 * this->distance_motors_cenetrline_x_dir);
    mat(2, 3) =  1.0 / (8.0 * this->propeller_drag_coefficient);

    mat(3, 0) =  1.0 / 8.0;
    mat(3, 1) = -1.0 / (8.0 * this->distance_motors_cenetrline_y_dir);
    mat(3, 2) = -1.0 / (8.0 * this->distance_motors_cenetrline_x_dir);
    mat(3, 3) = -1.0 / (8.0 * this->propeller_drag_coefficient);

    mat(4, 0) =  1.0 / 8.0;
    mat(4, 1) =  1.0 / (8.0 * this->distance_motors_cenetrline_y_dir);
    mat(4, 2) =  1.0 / (8.0 * this->distance_motors_cenetrline_x_dir);
    mat(4, 3) =  1.0 / (8.0 * this->propeller_drag_coefficient);

    mat(5, 0) =  1.0 / 8.0;
    mat(5, 1) = -1.0 / (8.0 * this->distance_motors_cenetrline_y_dir);
    mat(5, 2) =  1.0 / (8.0 * this->distance_motors_cenetrline_x_dir);
    mat(5, 3) = -1.0 / (8.0 * this->propeller_drag_coefficient);

    mat(6, 0) =  1.0 / 8.0;
    mat(6, 1) = -1.0 / (8.0 * this->distance_motors_cenetrline_y_dir);
    mat(6, 2) = -1.0 / (8.0 * this->distance_motors_cenetrline_x_dir);
    mat(6, 3) =  1.0 / (8.0 * this->propeller_drag_coefficient);

    mat(7, 0) =  1.0 / 8.0;
    mat(7, 1) =  1.0 / (8.0 * this->distance_motors_cenetrline_y_dir);
    mat(7, 2) = -1.0 / (8.0 * this->distance_motors_cenetrline_x_dir);
    mat(7, 3) = -1.0 / (8.0 * this->propeller_drag_coefficient);

    return mat; 
  }();


  // Declare const matrix mixer_matrix_quadcopter using a lambda function
  const Eigen::Matrix4d mixer_matrix_quadcopter = [this]() {
    Eigen::Matrix4d mat; // 4 rows, 4 columns

    // [1/4, -1/(4*l_y),  1/(4*l_x),  1/(4*c_t)]
    // [1/4,  1/(4*l_y),  1/(4*l_x), -1/(4*c_t)]
    // [1/4,  1/(4*l_y), -1/(4*l_x),  1/(4*c_t)]
    // [1/4, -1/(4*l_y), -1/(4*l_x), -1/(4*c_t)]
    
    // Assign values element-by-element
    mat(0, 0) =  1.0 / 4.0;
    mat(0, 1) = -1.0 / (4.0 * this->distance_motors_cenetrline_y_dir);
    mat(0, 2) =  1.0 / (4.0 * this->distance_motors_cenetrline_x_dir);
    mat(0, 3) =  1.0 / (4.0 * this->propeller_drag_coefficient); 

    mat(1, 0) =  1.0 / 4.0;
    mat(1, 1) =  1.0 / (4.0 * this->distance_motors_cenetrline_y_dir);
    mat(1, 2) =  1.0 / (4.0 * this->distance_motors_cenetrline_x_dir);
    mat(1, 3) = -1.0 / (4.0 * this->propeller_drag_coefficient); 

    mat(2, 0) =  1.0 / 4.0;
    mat(2, 1) =  1.0 / (4.0 * this->distance_motors_cenetrline_y_dir);
    mat(2, 2) = -1.0 / (4.0 * this->distance_motors_cenetrline_x_dir);
    mat(2, 3) =  1.0 / (4.0 * this->propeller_drag_coefficient); 

    mat(3, 0) =  1.0 / 4.0;
    mat(3, 1) = -1.0 / (4.0 * this->distance_motors_cenetrline_y_dir);
    mat(3, 2) = -1.0 / (4.0 * this->distance_motors_cenetrline_x_dir);
    mat(3, 3) = -1.0 / (4.0 * this->propeller_drag_coefficient); 

    return mat; 
  }();

  // // Polynomial coefficients vector to evaluate the Commanded Thrust [-] based on the Thrust in Newton
  // // OLD ESCs
  // const Eigen::VectorXd thrust_polynomial_coefficients_quadcopter = (Eigen::VectorXd(8) << 
  //   0.0000105272829209078,
  //   -0.000353007963591775,
  //   0.00474469218478217,
  //   -0.0326989257250683,
  //   0.123233111657682,
  //   -0.256734365616374,
  //   0.376206271320848,
  //   -0.0492707360005048
  // ).finished();

  // Polynomial coefficients vector to evaluate the Commanded Thrust [-] based on the Thrust in Newton
  // NEW ESCs (TMotor)
  const Eigen::VectorXd thrust_polynomial_coefficients_quadcopter = (Eigen::VectorXd(8) << 
    0.00000318912344541255,
    -0.000107583270223678,
    0.00147671457913486,
    -0.0107666934546496,
    0.0459838527842087,
    -0.121504752465409,
    0.285725583084306,
    -0.0118110779377008
  ).finished();

};


#endif // VEHICLE_INFO_HPP
