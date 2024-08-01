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
 * File:        piecewise_polynomial_trajectory.cpp
 * Author:      Mattia Gramuglia
 * Date:        April 15, 2024
 * For info:    Andrea L'Afflitto 
 *              a.lafflitto@vt.edu
 * 
 * Description: Piecewise polynomial minimu-jerk trajectory computed offline through a MATLAB script.
 *              The heading (yaw) angle is kept tangential to the trajectory in the X-Y plane and computed online
 *              along with its first and second derivative.
 * 
 * GitHub:    https://github.com/andrealaffly/ACSL_flightstack_X8.git
 **********************************************************************************************************************/

#include <fstream>
#include <nlohmann/json.hpp>

#include "piecewise_polynomial_trajectory.hpp"
#include "multi_threaded_node.hpp"

// Constructor
PiecewisePolynomialTrajectory::PiecewisePolynomialTrajectory(MultiThreadedNode& node) : 
  UserDefinedTrajectory(node),
  time_current_adjusted_(0),
  segment_(0),
  user_defined_position_previous_(Eigen::Vector3d::Zero()),
  user_defined_yaw_previous_(0),
  norm_velocity_XY_(0)
{
  // Initialize the user-defined trajectory
  this->initializePiecewisePolynomialTrajectory(
    this->getNode().getConfigurationParameters().user_defined_trajectory_file
  );
}

/*
  Function that performs the initialization of the PiecewisePolynomialTrajectory based on the name of the 
  file provided as input
*/
void PiecewisePolynomialTrajectory::initializePiecewisePolynomialTrajectory(const std::string& TrajectoryFileName)
{
  // Initialize the matrices of the piecewise polynomial user-defined trajectory
  this->readJSONfile(TrajectoryFileName);

  // Set the polynomial coefficients of the various matrices
  this->setPolynomialCoefficientMatrices();
}

/*
  Function to read the trajectory info coming from the .json file and assign it to the
  piecewise_polynomial_trajectory_info_ struct instantiated in the PiecewisePolynomialTrajectory class
*/
void PiecewisePolynomialTrajectory::readJSONfile(const std::string& fileName)
{
  // Define the path where the user-defined trajectory JSON files are located
  const std::string path = "./src/flightstack/params/user_defined_trajectory/";

  // Concatenate the path with the file name
  std::string jsonFile = path + fileName;

  std::ifstream file(jsonFile);
  if (!file.is_open()) {
    throw std::runtime_error("Could not open file: " + jsonFile);
  }

	nlohmann::json j;
	file >> j;
	
	piecewise_polynomial_trajectory_info_.waypoint_times_ = 
    j["waypoint_times"].get<std::vector<double>>();
	piecewise_polynomial_trajectory_info_.piecewise_polynomial_coefficients_ = 
    j["piecewise_polynomial_coefficients"].get<std::vector<std::vector<double>>>();

}

// Getter function for the waypoint_times_
const std::vector<double>& PiecewisePolynomialTrajectory::getWaypointTimes() const
{
  return piecewise_polynomial_trajectory_info_.waypoint_times_;
}

// Getter function for the piecewise_polynomial_coefficients_
const std::vector<std::vector<double>>& PiecewisePolynomialTrajectory::getPiecewisePolynomialCoefficients() const
{
  return piecewise_polynomial_trajectory_info_.piecewise_polynomial_coefficients_;
}

// Getter function for the time_current_adjusted_
const std::atomic<double>& PiecewisePolynomialTrajectory::getTimeCurrentAdjusted() const {
  return time_current_adjusted_;
}

// Getter function for the segment_
const int& PiecewisePolynomialTrajectory::getSegment() const {
  return segment_;
}

// Getter function for the user_defined_yaw_previous_
const double& PiecewisePolynomialTrajectory::getUserDefinedYawPrevious() const {
  return user_defined_yaw_previous_;
}

// Getter function for the norm_velocity_XY_
const double& PiecewisePolynomialTrajectory::getNormVelocityXY() const {
  return norm_velocity_XY_;
}

/*
  Function that uses the piecewise_polynomial_coefficients_ matrix to compute the derivatives and generate the 
  matrices of the coefficients for the x, y, and z polynomials of the position, velocity, acceleration, and jerk.
*/
void PiecewisePolynomialTrajectory::setPolynomialCoefficientMatrices() 
{
  const auto& pp_coefficients = piecewise_polynomial_trajectory_info_.piecewise_polynomial_coefficients_;

  // Call polyCoefAssigning to assign polynomial coefficients to designated variables
  auto position_coefs = polyCoefAssigning(pp_coefficients);

  // Extract coefficients for x, y, and z components
  this->position_coef_x_ = position_coefs[0];
  this->position_coef_y_ = position_coefs[1];
  this->position_coef_z_ = position_coefs[2];

  // Compute velocity coefficients
  this->velocity_coef_x_ = polyDerMatrix(this->position_coef_x_);
  this->velocity_coef_y_ = polyDerMatrix(this->position_coef_y_);
  this->velocity_coef_z_ = polyDerMatrix(this->position_coef_z_);

  // Compute acceleration coefficients
  this->acceleration_coef_x_ = polyDerMatrix(this->velocity_coef_x_);
  this->acceleration_coef_y_ = polyDerMatrix(this->velocity_coef_y_);
  this->acceleration_coef_z_ = polyDerMatrix(this->velocity_coef_z_);

  // Compute jerk coefficients
  this->jerk_coef_x_ = polyDerMatrix(this->acceleration_coef_x_);
  this->jerk_coef_y_ = polyDerMatrix(this->acceleration_coef_y_);
  this->jerk_coef_z_ = polyDerMatrix(this->acceleration_coef_z_);

  /* // DEBUUUUUUG
  std::cout << "Position Coefficient X:\n" << this->position_coef_x_ << std::endl;
  std::cout << "Velocity Coefficient X:\n" << this->velocity_coef_x_ << std::endl;
  std::cout << "Acceleration Coefficient X:\n" << this->acceleration_coef_x_ << std::endl;
  std::cout << "Jerk Coefficient X:\n" << this->jerk_coef_x_ << std::endl; */

}

/*
  Function that updates the user-defined trajectory based on the time at which it is requested to be evaluated
*/
void PiecewisePolynomialTrajectory::updateUserDefinedTrajectory(std::atomic<double>& time_current) 
{
  // Adjust the time_current so that the mission starts TAKEOFF_START_TIME_SECONDS after the program is run
  double temp_time_minus_takeoff = time_current - this->getNode().getGlobalParameters().TAKEOFF_START_TIME_SECONDS;
  std::atomic<double> time_current_minus_takeoff(temp_time_minus_takeoff);

  // Adjust time and identify segment
  std::tie(this->time_current_adjusted_, this->segment_) = polynomialTimeAdjusted(
    piecewise_polynomial_trajectory_info_.waypoint_times_, time_current_minus_takeoff);
  /* NOTE THAT
    If the current time has passed the last waypoint time, then segment_ = 0;
  */

  // If the current time has passed the last waypoint time but it is not greater than LANDING_START_TIME_SECONDS,
  // set the trajectory to perform HOVER in the last position
  if (time_current_minus_takeoff >= this->getWaypointTimes().back() && 
        time_current < this->getNode().getGlobalParameters().LANDING_START_TIME_SECONDS) 
  {
    // std::cout << "User-defined trajectory: PERFORM HOVER IN LAST GOOD POSITION" << std::endl;
    this->setUserDefinedPosition(this->user_defined_position_previous_);
    this->setUserDefinedVelocity(Eigen::Vector3d::Zero());
    this->setUserDefinedAcceleration(Eigen::Vector3d::Zero());

  // If the current time has passed the LANDING_START_TIME_SECONDS but not the LANDING_END_TIME_SECONDS,
  // perform the LANDING meneuvre
  } else if (time_current >= this->getNode().getGlobalParameters().LANDING_START_TIME_SECONDS && 
              time_current < this->getNode().getGlobalParameters().LANDING_END_TIME_SECONDS)
  {
    // std::cout << "User-defined trajectory: PERFORM THE LANDING MANEUVRE" << std::endl;
    // Evaluate and update user_defined_position_
    this->setUserDefinedPosition(Eigen::Vector3d(
      this->user_defined_position_previous_[0],
      this->user_defined_position_previous_[1],
      evaluatePolynomial(landing_position_coef_z_,
                         time_current - this->getNode().getGlobalParameters().LANDING_START_TIME_SECONDS)));

    // Evaluate and update user_defined_velocity_
    this->setUserDefinedVelocity(Eigen::Vector3d(
      0.0,
      0.0,
      evaluatePolynomial(landing_velocity_coef_z_,
                         time_current - this->getNode().getGlobalParameters().LANDING_START_TIME_SECONDS)));

    // Evaluate and update user_defined_acceleration_
    this->setUserDefinedAcceleration(Eigen::Vector3d(
      0.0,
      0.0,
      evaluatePolynomial(landing_acceleration_coef_z_,
                         time_current - this->getNode().getGlobalParameters().LANDING_START_TIME_SECONDS)));

  // If the current time has passed the LANDING_END_TIME_SECONDS,
  // stay at the last good location
  } else if (time_current >= this->getNode().getGlobalParameters().LANDING_END_TIME_SECONDS)
  {
    // std::cout << "User-defined trajectory: END OF LANDING MANEUVRE" << std::endl;
    this->setUserDefinedPosition(this->user_defined_position_previous_);
    this->setUserDefinedVelocity(Eigen::Vector3d::Zero());
    this->setUserDefinedAcceleration(Eigen::Vector3d::Zero());

  // Follow the piecewise polynomial trajectory
  } else {

    // std::cout << "User-defined trajectory: EXECUTE THE MAIN TRAJECTORY" << std::endl;
    // Evaluate and update user_defined_position_
    this->setUserDefinedPosition(Eigen::Vector3d(
      evaluatePolynomial(position_coef_x_.row(this->segment_), this->time_current_adjusted_),
      evaluatePolynomial(position_coef_y_.row(this->segment_), this->time_current_adjusted_),
      evaluatePolynomial(position_coef_z_.row(this->segment_), this->time_current_adjusted_)));

    // Evaluate and update user_defined_velocity_
    this->setUserDefinedVelocity(Eigen::Vector3d(
      evaluatePolynomial(velocity_coef_x_.row(this->segment_), this->time_current_adjusted_),
      evaluatePolynomial(velocity_coef_y_.row(this->segment_), this->time_current_adjusted_),
      evaluatePolynomial(velocity_coef_z_.row(this->segment_), this->time_current_adjusted_)));

    // Evaluate and update user_defined_acceleration_
    this->setUserDefinedAcceleration(Eigen::Vector3d(
      evaluatePolynomial(acceleration_coef_x_.row(this->segment_), this->time_current_adjusted_),
      evaluatePolynomial(acceleration_coef_y_.row(this->segment_), this->time_current_adjusted_),
      evaluatePolynomial(acceleration_coef_z_.row(this->segment_), this->time_current_adjusted_)));
  }

  // Update user_defined_position_previous_
  this->user_defined_position_previous_ = this->getUserDefinedPosition();

  // DEBUUUUUUG
  
  /* 
  std::cout << "time_current_minus_takeoff:\n" << time_current_minus_takeoff << std::endl;
  std::cout << "getUserDefinedPosition:\n" << this->getUserDefinedPosition() << std::endl;
  std::cout << "getUserDefinedVelocity:\n" << this->getUserDefinedVelocity() << std::endl;
  std::cout << "getUserDefinedAcceleration:\n" << this->getUserDefinedAcceleration() << std::endl;
  std::cout << "segment_:\n" << this->segment_ << std::endl;
  */
 
  

}

/*
  Function that updates the user-defined yaw, yaw_dot, and yaw_dot_dot
  based on the time at which it is requested to be evaluated
*/
void PiecewisePolynomialTrajectory::updateUserDefinedYaw()
{
  // Adjust the time_current so that the mission starts TAKEOFF_START_TIME_SECONDS after the program is run
  double temp_time_minus_takeoff = this->getNode().getCurrentTime() - 
                                   this->getNode().getGlobalParameters().TAKEOFF_START_TIME_SECONDS;
  std::atomic<double> time_current_minus_takeoff(temp_time_minus_takeoff);

  // Compute the 2D norm of the trajectory velocity in the XY plane
  this->norm_velocity_XY_ = norm2D(this->velocity_coef_x_.row(this->segment_),
                                   this->velocity_coef_y_.row(this->segment_),
                                   this->time_current_adjusted_);

  // If the norm of the velocity in the XY plane is close to zero OR If the current time has passed the
  // last waypoint time, THEN set the trajectory to keep the last yaw angle
  if (this->norm_velocity_XY_ < 1e-3 || (time_current_minus_takeoff >= this->getWaypointTimes().back()))
  {
    // std::cout << "User-defined Yaw: KEEP LAST GOOD YAW ANGLE" << std::endl;

    this->setUserDefinedYaw(this->user_defined_yaw_previous_);
    this->setUserDefinedYawDot(0.0);
    this->setUserDefinedYawDotDot(0.0);

    // this->setUserDefinedYaw(-((2.0 * M_PI) / 60.0) * time_current_minus_takeoff);
    // this->setUserDefinedYawDot(-(2.0 * M_PI) / 60.0);
    // this->setUserDefinedYawDotDot(0.0);

  } else {
    // std::cout << "User-defined Yaw: EXECUTE THE MAIN YAW TRAJECTORY" << std::endl;

    // Update the user-defined yaw angle
    this->setUserDefinedYaw(yawComputation(this->velocity_coef_x_.row(this->segment_),
                                           this->velocity_coef_y_.row(this->segment_),
                                           this->time_current_adjusted_)); 
    // this->setUserDefinedYaw(0.0); 

    // Update the first derivative of the user-defined yaw angle
    this->setUserDefinedYawDot(yawDotComputation(this->velocity_coef_x_.row(this->segment_),
                                                 this->velocity_coef_y_.row(this->segment_),
                                                 this->acceleration_coef_x_.row(this->segment_),
                                                 this->acceleration_coef_y_.row(this->segment_),
                                                 this->time_current_adjusted_));
    // this->setUserDefinedYawDot(0.0);


    // Update the second derivative of the user-defined yaw angle
    this->setUserDefinedYawDotDot(yawDotDotComputation(this->velocity_coef_x_.row(this->segment_),
                                                       this->velocity_coef_y_.row(this->segment_),
                                                       this->acceleration_coef_x_.row(this->segment_),
                                                       this->acceleration_coef_y_.row(this->segment_),
                                                       this->jerk_coef_x_.row(this->segment_),
                                                       this->jerk_coef_y_.row(this->segment_),
                                                       this->time_current_adjusted_));
    // this->setUserDefinedYawDotDot(0.0);

 }

  // Update user_defined_yaw_previous_
  this->user_defined_yaw_previous_ = this->getUserDefinedYaw();

  // DEBUUUUUUG
  /* 
  std::cout << "getUserDefinedYaw:\n" << this->getUserDefinedYaw() << std::endl;
  std::cout << "getUserDefinedYawDot:\n" << this->getUserDefinedYawDot() << std::endl;
  std::cout << "getUserDefinedYawDotDot:\n" << this->getUserDefinedYawDotDot() << std::endl;
  */
}

/***********************************************************************************************************************
 * Auxillary functions
************************************************************************************************************************
*/

// Function to assign polynomial coefficients to designated variables
std::vector<Eigen::MatrixXd> polyCoefAssigning(const std::vector<std::vector<double>>& poly_coeff_matrix_in) 
{
  // Get the size of the input matrix
  int rows = poly_coeff_matrix_in.size();
  int cols = poly_coeff_matrix_in[0].size();
  
  // Calculate the size of the output matrices
  int out_rows = rows / 3;
  
  // Initialize output matrices for x, y, and z variables
  Eigen::MatrixXd poly_coeff_matrix_out_x(out_rows, cols);
  Eigen::MatrixXd poly_coeff_matrix_out_y(out_rows, cols);
  Eigen::MatrixXd poly_coeff_matrix_out_z(out_rows, cols);
  
  // Assign polynomial coefficients to designated variables
  for (int i = 0; i < out_rows; ++i) {
    for (int j = 0; j < cols; ++j) {
      poly_coeff_matrix_out_x(i, j) = poly_coeff_matrix_in[3 * i][j];
      poly_coeff_matrix_out_y(i, j) = poly_coeff_matrix_in[3 * i + 1][j];
      poly_coeff_matrix_out_z(i, j) = poly_coeff_matrix_in[3 * i + 2][j];
    }
  }
  
  // Return the output matrices
  return {poly_coeff_matrix_out_x, poly_coeff_matrix_out_y, poly_coeff_matrix_out_z};
}

// Function to compute the derivative of the piecewise polynomial coefficient matrix
Eigen::MatrixXd polyDerMatrix(const Eigen::MatrixXd& poly_coeff_matrix_in)
{
  // Get the size of the input matrix
  int rows = poly_coeff_matrix_in.rows();
  int cols = poly_coeff_matrix_in.cols();
  
  // Initialize the output matrix for the derivative
  Eigen::MatrixXd poly_coeff_matrix_out(rows, cols - 1);
  
  // Compute the derivative for each row of the input matrix
  for (int i = 0; i < rows; ++i) {
    for (int j = 0; j < cols - 1; ++j) {
      poly_coeff_matrix_out(i, j) = (cols - 1 - j) * poly_coeff_matrix_in(i, j);
    }
  }
  
  return poly_coeff_matrix_out;
}

// Adjusts the time to be fed to the various polynomials and identifies the segment of the trajectory
std::pair<double, int> polynomialTimeAdjusted(const std::vector<double>& time_waypoint_vector, double time)
 {
  int num_waypoints = time_waypoint_vector.size();
  int segment = 0;
  double time_adjusted = 0.0;
  
  for (int i = 0; i < num_waypoints - 1; ++i) {
    if (time >= time_waypoint_vector[i] && time <= time_waypoint_vector[i + 1]) {
      segment = i;
      time_adjusted = time - time_waypoint_vector[segment];
      break;
    }
  }
  
  return {time_adjusted, segment};
}

// Function to evaluate a polynomial given its coefficients and a value at which to evaluate the polynomial
double evaluatePolynomial(const Eigen::VectorXd& coefficients, double value)
{
  double result = 0;
    for (int i = 0, deg = coefficients.size() - 1; deg >= 0; deg--, i++)
      {
        result += coefficients[i] * std::pow(value, deg);
      }
  return result;
}

// Compute the 2D norm using polynomial coefficients
double norm2D(const Eigen::VectorXd& poly_coef_x, const Eigen::VectorXd& poly_coef_y, double t)
{
  double norm_value = std::sqrt(std::pow(evaluatePolynomial(poly_coef_x, t), 2) +
                                std::pow(evaluatePolynomial(poly_coef_y, t), 2));
  return norm_value;
}

// Compute the derivative of the 2D norm using polynomial coefficients and their derivatives
double norm2Dderivative(const Eigen::VectorXd& poly_coef_x,
                        const Eigen::VectorXd& poly_coef_y,
                        const Eigen::VectorXd& poly_coef_x_prime,
                        const Eigen::VectorXd& poly_coef_y_prime,
                        double t)
{
  double norm_value = norm2D(poly_coef_x, poly_coef_y, t);
  double derivative_value = ((evaluatePolynomial(poly_coef_x, t) * evaluatePolynomial(poly_coef_x_prime, t) +
                              evaluatePolynomial(poly_coef_y, t) * evaluatePolynomial(poly_coef_y_prime, t)) /
                              norm_value);
  return derivative_value;
}

// Compute the user-defined yaw from the trajectory velocity in the XY plane
double yawComputation(const Eigen::VectorXd& Vx_coef, const Eigen::VectorXd& Vy_coef, double t)
{
  // Compute the velocity components Vx and Vy at time t using polynomial coefficients
  double Vx = evaluatePolynomial(Vx_coef, t);
  double Vy = evaluatePolynomial(Vy_coef, t);

  // Compute the yaw angle using atan2
  double yaw = std::atan2(Vy, Vx);
  
  return yaw;
}

// Compute the user-defined yaw_dot from the trajectory velocity in the XY plane
double yawDotComputation(const Eigen::VectorXd& Vx_coef, const Eigen::VectorXd& Vy_coef,
                         const Eigen::VectorXd& Ax_coef, const Eigen::VectorXd& Ay_coef, double t)
{
  // Compute the acceleration components Ax and Ay at time t using polynomial coefficients
  double Ax = evaluatePolynomial(Ax_coef, t);
  double Ay = evaluatePolynomial(Ay_coef, t);
  
  // Compute the acceleration angle in the complex plane
  double acceleration_angle = std::atan2(Ay, Ax);

  // Compute the velocity and acceleration norms
  double velocity_norm = norm2D(Vx_coef, Vy_coef, t);
  double acceleration_norm = norm2D(Ax_coef, Ay_coef, t);

  // Compute the yaw angle
  double yaw = yawComputation(Vx_coef, Vy_coef, t);

  // Compute the yaw rate
  double yaw_dot = (acceleration_norm / velocity_norm) * std::sin(acceleration_angle - yaw);
  
  return yaw_dot;
}

// Compute the user-defined yaw_dot_dot from the trajectory velocity in the XY plane
double yawDotDotComputation(const Eigen::VectorXd& Vx_coef, const Eigen::VectorXd& Vy_coef,
                            const Eigen::VectorXd& Ax_coef, const Eigen::VectorXd& Ay_coef,
                            const Eigen::VectorXd& Jx_coef, const Eigen::VectorXd& Jy_coef,
                            double t)
{
  // Compute the jerk components Jx and Jy at time t using polynomial coefficients
  double Jx = evaluatePolynomial(Jx_coef, t);
  double Jy = evaluatePolynomial(Jy_coef, t);
  
  // Compute the jerk angle in the complex plane
  double jerk_angle = std::atan2(Jy, Jx);

  // Compute the velocity and jerk norms
  double velocity_norm = norm2D(Vx_coef, Vy_coef, t);
  double jerk_norm = norm2D(Jx_coef, Jy_coef, t);

  // Compute the derivative of the velocity norm
  double velocity_norm_prime = norm2Dderivative(Vx_coef, Vy_coef, Ax_coef, Ay_coef, t);
  
  // Compute the yaw angle and its derivative
  double yaw = yawComputation(Vx_coef, Vy_coef, t);
  double yaw_dot = yawDotComputation(Vx_coef, Vy_coef, Ax_coef, Ay_coef, t);

  // Compute the second derivative of yaw angle
  double yaw_dot_dot = (jerk_norm * std::sin(jerk_angle - yaw) - 
                        2 * velocity_norm_prime * yaw_dot) / velocity_norm;
  
  return yaw_dot_dot;
}
