
#include <cmath>
#include <algorithm>

#include <Eigen/Dense>
#include <json_parser.hpp>
#include "vehicleParams.hpp"

using namespace Eigen;
using namespace std;

ParamsVehicle::ParamsVehicle(){} //default constructor
ParamsVehicle::ParamsVehicle(json j) {

  motorSaturationUpper = j.at("motorSaturationUpper").get<double>();
  motorSaturationLower = j.at("motorSaturationLower").get<double>();
  minValuePublishMotors = j.at("minValuePublishMotors").get<double>();
  mass = j.at("mass").get<double>();
    inertia_matrix = extractMatrixFromJSON<double, 3, 3>(j["inertia_matrix"]);
  air_density = j.at("air_density").get<double>();
  surface_area = j.at("surface_area").get<double>();\
    drag_coefficient_matrix = extractMatrixFromJSON<double, 3, 3>(j["drag_coefficient_matrix"]);

  A_filter_roll_des = extractMatrixFromJSON<double, 2, 2>(j["A_filter_roll_des"]);
  B_filter_roll_des = extractMatrixFromJSON<double, 2, 1>(j["B_filter_roll_des"]);
  C_filter_roll_des = extractMatrixFromJSON<double, 1, 2>(j["C_filter_roll_des"]);
  D_filter_roll_des = j["D_filter_roll_des"];

  A_filter_pitch_des = extractMatrixFromJSON<double, 2, 2>(j["A_filter_pitch_des"]);
  B_filter_pitch_des = extractMatrixFromJSON<double, 2, 1>(j["B_filter_pitch_des"]);
  C_filter_pitch_des = extractMatrixFromJSON<double, 1, 2>(j["C_filter_pitch_des"]);
  D_filter_pitch_des = j["D_filter_pitch_des"];

  A_filter_roll_dot_des = extractMatrixFromJSON<double, 2, 2>(j["A_filter_roll_dot_des"]);
  B_filter_roll_dot_des = extractMatrixFromJSON<double, 2, 1>(j["B_filter_roll_dot_des"]);
  C_filter_roll_dot_des = extractMatrixFromJSON<double, 1, 2>(j["C_filter_roll_dot_des"]);
  D_filter_roll_dot_des = j["D_filter_roll_dot_des"];

  A_filter_pitch_dot_des = extractMatrixFromJSON<double, 2, 2>(j["A_filter_pitch_dot_des"]);
  B_filter_pitch_dot_des = extractMatrixFromJSON<double, 2, 1>(j["B_filter_pitch_dot_des"]);
  C_filter_pitch_dot_des = extractMatrixFromJSON<double, 1, 2>(j["C_filter_pitch_dot_des"]);
  
  distance_motors_centerline_x_dir = j.at("distance_motors_centerline_x_dir").get<double>();
  distance_motors_centerline_y_dir = j.at("distance_motors_centerline_y_dir").get<double>();
  propeller_drag_coefficient = j.at("propeller_drag_coefficient").get<double>();
  
    // Assign values element-by-element
    mixer_matrix(0, 0) =  1.0 / 8.0;
    mixer_matrix(0, 1) = -1.0 / (8.0 * distance_motors_centerline_y_dir);
    mixer_matrix(0, 2) =  1.0 / (8.0 * distance_motors_centerline_x_dir);
    mixer_matrix(0, 3) =  1.0 / (8.0 * propeller_drag_coefficient);

    mixer_matrix(1, 0) =  1.0 / 8.0;
    mixer_matrix(1, 1) =  1.0 / (8.0 * distance_motors_centerline_y_dir);
    mixer_matrix(1, 2) =  1.0 / (8.0 * distance_motors_centerline_x_dir);
    mixer_matrix(1, 3) = -1.0 / (8.0 * propeller_drag_coefficient);

    mixer_matrix(2, 0) =  1.0 / 8.0;
    mixer_matrix(2, 1) =  1.0 / (8.0 * distance_motors_centerline_y_dir);
    mixer_matrix(2, 2) = -1.0 / (8.0 * distance_motors_centerline_x_dir);
    mixer_matrix(2, 3) =  1.0 / (8.0 * propeller_drag_coefficient);

    mixer_matrix(3, 0) =  1.0 / 8.0;
    mixer_matrix(3, 1) = -1.0 / (8.0 * distance_motors_centerline_y_dir);
    mixer_matrix(3, 2) = -1.0 / (8.0 * distance_motors_centerline_x_dir);
    mixer_matrix(3, 3) = -1.0 / (8.0 * propeller_drag_coefficient);

    mixer_matrix(4, 0) =  1.0 / 8.0;
    mixer_matrix(4, 1) =  1.0 / (8.0 * distance_motors_centerline_y_dir);
    mixer_matrix(4, 2) =  1.0 / (8.0 * distance_motors_centerline_x_dir);
    mixer_matrix(4, 3) =  1.0 / (8.0 * propeller_drag_coefficient);

    mixer_matrix(5, 0) =  1.0 / 8.0;
    mixer_matrix(5, 1) = -1.0 / (8.0 * distance_motors_centerline_y_dir);
    mixer_matrix(5, 2) =  1.0 / (8.0 * distance_motors_centerline_x_dir);
    mixer_matrix(5, 3) = -1.0 / (8.0 * propeller_drag_coefficient);

    mixer_matrix(6, 0) =  1.0 / 8.0;
    mixer_matrix(6, 1) = -1.0 / (8.0 * distance_motors_centerline_y_dir);
    mixer_matrix(6, 2) = -1.0 / (8.0 * distance_motors_centerline_x_dir);
    mixer_matrix(6, 3) =  1.0 / (8.0 * propeller_drag_coefficient);

    mixer_matrix(7, 0) =  1.0 / 8.0;
    mixer_matrix(7, 1) =  1.0 / (8.0 * distance_motors_centerline_y_dir);
    mixer_matrix(7, 2) = -1.0 / (8.0 * distance_motors_centerline_x_dir);
    mixer_matrix(7, 3) = -1.0 / (8.0 * propeller_drag_coefficient);

    // Assign values element-by-element
    mixer_matrix_quadcopter(0, 0) =  1.0 / 4.0;
    mixer_matrix_quadcopter(0, 1) = -1.0 / (4.0 * distance_motors_centerline_y_dir);
    mixer_matrix_quadcopter(0, 2) =  1.0 / (4.0 * distance_motors_centerline_x_dir);
    mixer_matrix_quadcopter(0, 3) = -1.0 / (4.0 * propeller_drag_coefficient); 

    mixer_matrix_quadcopter(1, 0) =  1.0 / 4.0;
    mixer_matrix_quadcopter(1, 1) =  1.0 / (4.0 * distance_motors_centerline_y_dir);
    mixer_matrix_quadcopter(1, 2) = -1.0 / (4.0 * distance_motors_centerline_x_dir);
    mixer_matrix_quadcopter(1, 3) = -1.0 / (4.0 * propeller_drag_coefficient); 

    mixer_matrix_quadcopter(2, 0) =  1.0 / 4.0;
    mixer_matrix_quadcopter(2, 1) =  1.0 / (4.0 * distance_motors_centerline_y_dir);
    mixer_matrix_quadcopter(2, 2) =  1.0 / (4.0 * distance_motors_centerline_x_dir);
    mixer_matrix_quadcopter(2, 3) =  1.0 / (4.0 * propeller_drag_coefficient); 

    mixer_matrix_quadcopter(3, 0) =  1.0 / 4.0;
    mixer_matrix_quadcopter(3, 1) = -1.0 / (4.0 * distance_motors_centerline_y_dir);
    mixer_matrix_quadcopter(3, 2) = -1.0 / (4.0 * distance_motors_centerline_x_dir);
    mixer_matrix_quadcopter(3, 3) =  1.0 / (4.0 * propeller_drag_coefficient); 
  
    // Polynomial coefficients vector to evaluate the Commanded Thrust [-] based on the Thrust in Newton
  // S500 with Holybro props
  thrust_polynomial_coefficients_quadcopter = extractMatrixFromJSON<double, 1, 5>(j["thrust_polynomial_coefficients_quadcopter"]);
}