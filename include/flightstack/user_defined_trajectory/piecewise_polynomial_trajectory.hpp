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
 * File:        piecewise_polynomial_trajectory.hpp
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

#ifndef PIECEWISE_POLYNOMIAL_TRAJECTORY_HPP
#define PIECEWISE_POLYNOMIAL_TRAJECTORY_HPP

#include <vector>
#include <string>
#include <fstream>
#include <iostream>
#include <math.h>

#include <Eigen/Dense>
#include <nlohmann/json.hpp>

#include "user_defined_trajectory.hpp"


// Struct containing the trajectory info coming from the .json file 
struct PiecewisePolynomialTrajectoryInfo {
	std::vector<double> waypoint_times_; // times at which I want to reach the waypoints
	std::vector<std::vector<double>> piecewise_polynomial_coefficients_; // matrix containing the piecewise polynomial
                                                                       // coefficients
};

class PiecewisePolynomialTrajectory : public UserDefinedTrajectory
{
public:

  // Constructor
  PiecewisePolynomialTrajectory(MultiThreadedNode& node);

  void initializePiecewisePolynomialTrajectory(const std::string& TrajectoryFileName);

  void readJSONfile(const std::string& fileName);

  // Getter functions
  const std::vector<double>& getWaypointTimes() const;
  const std::vector<std::vector<double>>& getPiecewisePolynomialCoefficients() const;
  const std::atomic<double>& getTimeCurrentAdjusted() const;
  const int& getSegment() const;
  const double& getUserDefinedYawPrevious() const;
  const double& getNormVelocityXY() const;

  void setPolynomialCoefficientMatrices();
  void updateUserDefinedTrajectory(std::atomic<double>& time_current);
  void updateUserDefinedYaw();

private:

  std::atomic<double> time_current_adjusted_; // current time adjusted with respect to the trajectory segment
  int segment_; // segment of the trajectory in which I am. If the trajectory has 5 waypoints, it has 4 segments
  Eigen::Vector3d user_defined_position_previous_; // user_defined_position_ computed during the previous iteration
  double user_defined_yaw_previous_; // user_defined_yaw_ computed during the previous iteration
  double norm_velocity_XY_; // norm of the X and Y components of the user_defined_velocity_

  // Polynomial coefficients
  Eigen::MatrixXd position_coef_x_;
  Eigen::MatrixXd position_coef_y_;
  Eigen::MatrixXd position_coef_z_;
  Eigen::MatrixXd velocity_coef_x_;
  Eigen::MatrixXd velocity_coef_y_;
  Eigen::MatrixXd velocity_coef_z_;
  Eigen::MatrixXd acceleration_coef_x_;
  Eigen::MatrixXd acceleration_coef_y_;
  Eigen::MatrixXd acceleration_coef_z_;
  Eigen::MatrixXd jerk_coef_x_;
  Eigen::MatrixXd jerk_coef_y_;
  Eigen::MatrixXd jerk_coef_z_;

  PiecewisePolynomialTrajectoryInfo piecewise_polynomial_trajectory_info_;

  // Position polynomial coefficients for LANDING from (X, Y, -1) to (X, Y, 0) in 4 seconds
  const Eigen::VectorXd landing_position_coef_z_ = (Eigen::VectorXd(8) << 
    -0.001220703125,
    0.01708984375,
    -0.0820312499999999,
    0.13671875,
    0.0,
    0.0,
    0.0,
    -1.0
  ).finished();

  // Velocity polynomial coefficients for LANDING from (X, Y, -1) to (X, Y, 0) in 4 seconds
  const Eigen::VectorXd landing_velocity_coef_z_ = (Eigen::VectorXd(7) << 
    -0.00854492187499999,
    0.1025390625,
    -0.41015625,
    0.546874999999999,
    0.0,
    0.0,
    0.0
  ).finished();

  // Acceleration polynomial coefficients for LANDING from (X, Y, -1) to (X, Y, 0) in 4 seconds
  const Eigen::VectorXd landing_acceleration_coef_z_ = (Eigen::VectorXd(6) << 
    -0.0512695312499999,
    0.5126953125,
    -1.640625,
    1.640625,
    0.0,
    0.0
  ).finished();

};

// Auxillary functions
std::vector<Eigen::MatrixXd> polyCoefAssigning(const std::vector<std::vector<double>>& poly_coeff_matrix_in);
Eigen::MatrixXd polyDerMatrix(const Eigen::MatrixXd& poly_coeff_matrix_in);
std::pair<double, int> polynomialTimeAdjusted(const std::vector<double>& time_waypoint_vector, double time);
double evaluatePolynomial(const Eigen::VectorXd& coefficients, double value);
double norm2D(const Eigen::VectorXd& poly_coef_x, const Eigen::VectorXd& poly_coef_y, double t);
double norm2Dderivative(const Eigen::VectorXd& poly_coef_x,
                        const Eigen::VectorXd& poly_coef_y,
                        const Eigen::VectorXd& poly_coef_x_prime,
                        const Eigen::VectorXd& poly_coef_y_prime,
                        double t);
double yawComputation(const Eigen::VectorXd& Vx_coef, const Eigen::VectorXd& Vy_coef, double t);                        
double yawDotComputation(const Eigen::VectorXd& Vx_coef, const Eigen::VectorXd& Vy_coef,
                         const Eigen::VectorXd& Ax_coef, const Eigen::VectorXd& Ay_coef, double t);
double yawDotDotComputation(const Eigen::VectorXd& Vx_coef, const Eigen::VectorXd& Vy_coef,
                            const Eigen::VectorXd& Ax_coef, const Eigen::VectorXd& Ay_coef,
                            const Eigen::VectorXd& Jx_coef, const Eigen::VectorXd& Jy_coef,
                            double t);



#endif // PIECEWISE_POLYNOMIAL_TRAJECTORY_HPP