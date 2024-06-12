/* piecewise_polynomial_trajectory.hpp

	Mattia Gramuglia
	April 15, 2024
*/

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