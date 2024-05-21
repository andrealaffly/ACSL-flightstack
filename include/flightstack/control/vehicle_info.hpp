/* vehicle_info.hpp

	Mattia Gramuglia
	April 22, 2024
*/

#ifndef VEHICLE_INFO_HPP
#define VEHICLE_INFO_HPP

#define GRAVITATIONAL_ACCELERATION 9.80665 // [m/s^2]

#include <cmath>

#include <Eigen/Dense>


struct VehicleInfo 
{

  const float mass = 1.920; // [kg] vehicle mass

  const Eigen::Matrix3d inertia_matrix = (Eigen::Matrix3d() << 
     2.271990e-02, -6.557000e-06, -1.003498e-03,
    -6.557000e-06,  2.202047e-02,  5.658400e-06,
    -1.003498e-03,  5.658400e-06,  1.614693e-02
  ).finished(); // [kg*m^2] inertia matrix of the vehicle system (drone frame + box + propellers) expressed in
                // Pixhawk coordinate system (FRD - x-Front, y-Right, z-Down), computed at the vehicle center of mass
  
  const float air_density = 1.225; // [kg/m^3] air density

  const float surface_area = 0.07; // [m^2] surface area of the drone to account fro the drag

  const float drag_coefficient = 1.28; // [-] drag coefficient (equal to that of a plate)
  const Eigen::Matrix3d drag_coefficient_matrix = (Eigen::Matrix3d() << 
    drag_coefficient, 0,                0,
    0,                drag_coefficient, 0,
    0,                0,                0
  ).finished();

  const float distance_motors_cenetrline_x_dir = 0.0881269; // [m] distance between the centerline of the drone and the
                                                            // motors along x direction in local FRD frame
  const float distance_motors_cenetrline_y_dir = 0.1083450; // [m] distance between the centerline of the drone and the
                                                            // motors along y direction in local FRD frame
  const float propeller_drag_coefficient = 0.01; // [m] propellers drag coefficient

  const Eigen::Matrix2d A_filter_roll_ref = (Eigen::Matrix2d() << 
    -15, -225, 
     1,   0 
  ).finished(); // A matrix of the roll_ref filter

  const Eigen::Vector2d B_filter_roll_ref = Eigen::Vector2d(1, 0); // B matrix of the roll_ref filter

  const Eigen::RowVector2d C_filter_roll_ref = Eigen::RowVector2d(225, 0); // C matrix of the roll_ref filter

  const int D_filter_roll_ref = 0; // D matrix of the roll_ref filter

  const Eigen::Matrix2d A_filter_pitch_ref = (Eigen::Matrix2d() << 
    -15, -225, 
      1,   0 
  ).finished(); // A matrix of the pitch_ref filter

  const Eigen::Vector2d B_filter_pitch_ref = Eigen::Vector2d(1, 0); // B matrix of the pitch_ref filter

  const Eigen::RowVector2d C_filter_pitch_ref = Eigen::RowVector2d(225, 0); // C matrix of the pitch_ref filter

  const int D_filter_pitch_ref = 0; // D matrix of the pitch_ref filter

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
