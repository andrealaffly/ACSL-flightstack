/* control.hpp

	Mattia Gramuglia
	April 19, 2024
*/

#ifndef CONTROL_HPP
#define CONTROL_HPP

#include <cmath>
#include <vector>
#include <functional> 
#include <chrono>

#include <Eigen/Dense>
#include <boost/numeric/odeint.hpp>

#include "vehicle_info.hpp"
#include "piecewise_polynomial_trajectory.hpp"

using namespace boost::numeric::odeint;

// The type of container used to hold the state vector
typedef std::vector<double> state_type;

// Forward declaration of MultiThreadedNode class
class MultiThreadedNode;

/*
  This struct contains the variables that are common among all controllers as they are the foundations of the base
  control architecture framework.
*/
struct ControlInternalMembers
{
  // Constructor
  ControlInternalMembers();

  Eigen::Vector3d mu_translational_raw; // translational virtual resultant force before applying safety mechanism
  Eigen::Vector3d mu_translational; // translational virtual resultant force after applying safety mechanism
  Eigen::Vector4d U_control_inputs; // U1, U2, U3, U4
  Eigen::Ref<Eigen::Vector3d> U2_U3_U4; // reference vector to U_control_inputs which includes only U2, U3, U4
  Eigen::Vector3d angular_position_reference; // [roll_reference, pitch_reference, user_defined_yaw]
  Eigen::Vector3d angular_position_reference_dot; // [roll_reference_dot, pitch_reference_dot, user_defined_yaw_dot]
  Eigen::Vector3d angular_position_reference_dot_dot; // [roll_reference_dot_dot, pitch_reference_dot_dot, user_defined_yaw_dot_dot]
  double& roll_reference; // reference roll angle computed by outer loop and sent as command to the inner loop
  double& pitch_reference; // reference pitch angle computed by outer loop and sent as command to the inner loop
  double& roll_reference_dot; // first time derivative of roll_reference
  double& pitch_reference_dot; // first time derivative of pitch_reference
  double& roll_reference_dot_dot; // second time derivative of roll_reference
  double& pitch_reference_dot_dot; // second time derivative of pitch_reference
  Eigen::Vector2d internal_state_roll_ref_filter; // internal state for filter/differentiator for reference roll
  Eigen::Vector2d internal_state_pitch_ref_filter; // internal state for filter/differentiator for reference pitch
  Eigen::Matrix3d jacobian_matrix_inverse; // inverse of the jacobian matrix
  Eigen::Matrix3d rotation_matrix_321_global_to_local; // 321 rotation matrix that transforms a vector from global to local coordinates
  Eigen::Vector3d euler_angles_rpy_dot; // first time derivative of the euler angles
  Eigen::Vector3d angular_error; // angular error
  Eigen::Vector3d translational_position_error; // translational position error
  Eigen::Matrix<double, 8, 1> thrust_vector; // vector containing the thrust of the 8 motors in Newton
  Eigen::Matrix<double, 8, 1> thrust_vector_normalized; // vector containing the thrust of the 8 motors with [0,1] values
  Eigen::Vector4d thrust_vector_quadcopter; // vector containing the thrust of the 4 motors in Newton
  Eigen::Vector4d thrust_vector_quadcopter_normalized; // vector containing the thrust of the 4 motors with [0,1] values
};


/*
  Struct that encapsulates references to external variables to be used inside the computeControlAlgorithm function
  to increase readability
*/
struct ControlReferences
{
  //Constructor
  ControlReferences(MultiThreadedNode& node);

  Eigen::Vector3d& user_defined_position;
  Eigen::Vector3d& user_defined_velocity;
  Eigen::Vector3d& user_defined_acceleration;
  double& user_defined_yaw;
  double& user_defined_yaw_dot;
  double& user_defined_yaw_dot_dot;

  Eigen::Vector3d& position;
  Eigen::Vector3d& velocity;
  Eigen::Vector3d& euler_angles_rpy;
  Eigen::Vector3d& angular_velocity;
};


class Control 
{
public:

  // Constructor
  Control(MultiThreadedNode& node);

  // Getter functions
  MultiThreadedNode& getNode() const;
  const VehicleInfo& getVehicleInfo() const;
  const ControlInternalMembers& getControlInternalMembers() const;
  const std::chrono::duration<double, std::micro>& getAlgorithmExecutionTimeMicroseconds() const;

  // Setter functions
  void setAlgorithmExecutionTimeMicroseconds(std::chrono::duration<double, std::micro> duration);

  Eigen::Matrix3d jacobianMatrixInverse(const double& roll, const double& pitch);

  Eigen::Matrix3d rotationMatrix321GlobalToLocal(const double& roll, const double& pitch, const double& yaw);

  void computeNormalizedThrustQuadcopterMode(ControlInternalMembers& cim, VehicleInfo& vehicle_info);

  //define_const_stepper
  runge_kutta4< state_type > stepper;

protected:

  VehicleInfo vehicle_info;

  ControlInternalMembers cim;

  ControlReferences cr;

private:

  MultiThreadedNode& node_;

  std::chrono::duration<double, std::micro> algorithm_execution_time_microseconds_;

};


#endif // CONTROL_HPP