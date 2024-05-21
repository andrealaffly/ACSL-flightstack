/* control.cpp

	Mattia Gramuglia
	April 19, 2024
*/


#include "control.hpp"
#include "multi_threaded_node.hpp"

// Constructor
ControlInternalMembers::ControlInternalMembers() : 
  U2_U3_U4(U_control_inputs.segment<3>(1)),
  roll_reference(angular_position_reference[0]),
  pitch_reference(angular_position_reference[1]),
  roll_reference_dot(angular_position_reference_dot[0]),
  pitch_reference_dot(angular_position_reference_dot[1]),
  roll_reference_dot_dot(angular_position_reference_dot_dot[0]),
  pitch_reference_dot_dot(angular_position_reference_dot_dot[1])

{}

//Constructor
ControlReferences::ControlReferences(MultiThreadedNode& node) :
  user_defined_position(node.getUserDefinedTrajectory()->getUserDefinedPosition()),
  user_defined_velocity(node.getUserDefinedTrajectory()->getUserDefinedVelocity()),
  user_defined_acceleration(node.getUserDefinedTrajectory()->getUserDefinedAcceleration()),
  user_defined_yaw(node.getUserDefinedTrajectory()->getUserDefinedYaw()),
  user_defined_yaw_dot(node.getUserDefinedTrajectory()->getUserDefinedYawDot()),
  user_defined_yaw_dot_dot(node.getUserDefinedTrajectory()->getUserDefinedYawDotDot()),
  position(node.getVehicleState()->getPosition()),
  velocity(node.getVehicleState()->getVelocity()),
  euler_angles_rpy(node.getVehicleState()->getEulerAnglesRPY()),
  angular_velocity(node.getVehicleState()->getAngularVelocity())
{}

// Constructor
Control::Control(MultiThreadedNode& node) :
  cr(node),
  node_(node)
{}

// Getter function that returns a reference to the current MultiThreadedNode object
MultiThreadedNode& Control::getNode() const {
  return node_;
}

// Getter function that returns a reference to the current VehicleInfo object
const VehicleInfo& Control::getVehicleInfo() const 
{
  return vehicle_info;
}

// Getter function that returns a reference to the current ControlInternalMembers object
const ControlInternalMembers& Control::getControlInternalMembers() const
{
  return cim;
}

// Getter function 
const std::chrono::duration<double, std::micro>& Control::getAlgorithmExecutionTimeMicroseconds() const
{
  return algorithm_execution_time_microseconds_;
}

// Setter function
void Control::setAlgorithmExecutionTimeMicroseconds(std::chrono::duration<double, std::micro> duration)
{
  algorithm_execution_time_microseconds_ = duration;
}

// Function that returns a 3x3 Jacobian matrix inverse given roll and pitch angles.
Eigen::Matrix3d Control::jacobianMatrixInverse(const double& roll, const double& pitch)
{
  // Ensure pitch is not near +/- pi/2 to avoid division by zero
  if (std::abs(std::cos(pitch)) < 1e-6) {
    throw std::invalid_argument("Pitch value results in division by zero.");
  }

  Eigen::Matrix3d jacobian_inverse;

  jacobian_inverse(0, 0) = 1;
  jacobian_inverse(0, 1) = (std::sin(roll) * std::sin(pitch)) / std::cos(pitch);
  jacobian_inverse(0, 2) = (std::cos(roll) * std::sin(pitch)) / std::cos(pitch);

  jacobian_inverse(1, 0) = 0;
  jacobian_inverse(1, 1) = std::cos(roll);
  jacobian_inverse(1, 2) = -std::sin(roll);

  jacobian_inverse(2, 0) = 0;
  jacobian_inverse(2, 1) = std::sin(roll) / std::cos(pitch);
  jacobian_inverse(2, 2) = std::cos(roll) / std::cos(pitch);

  return jacobian_inverse;
}

/*
  Function that computes the 321 rotation matrix that transforms a vector from global to local coordinates.

  rotation_matrix_321_local_to_global = R3 * R2 * R1; // hence
  rotation_matrix_321_global_to_local = R1_transpose * R2_transpose * R3_transpose;
*/
Eigen::Matrix3d Control::rotationMatrix321GlobalToLocal(const double& roll, const double& pitch, const double& yaw)
{
  Eigen::Matrix3d rotation_matrix_321_global_to_local;
  Eigen::Matrix3d R1_transpose;
  Eigen::Matrix3d R2_transpose;
  Eigen::Matrix3d R3_transpose;

  R1_transpose = (Eigen::Matrix3d() << 
    1,  0,               0,
    0,  std::cos(roll),  std::sin(roll),
    0, -std::sin(roll),  std::cos(roll)
  ).finished();

  R2_transpose = (Eigen::Matrix3d() << 
    std::cos(pitch),  0,  -std::sin(pitch),
    0,                1,   0,
    std::sin(pitch),  0,   std::cos(pitch)
  ).finished();

  R3_transpose = (Eigen::Matrix3d() << 
     std::cos(yaw),  std::sin(yaw),  0,
    -std::sin(yaw),  std::cos(yaw),  0,
     0,              0,              1
  ).finished();

  rotation_matrix_321_global_to_local = R1_transpose * R2_transpose * R3_transpose;

  return rotation_matrix_321_global_to_local;
}


void Control::computeNormalizedThrustQuadcopterMode(ControlInternalMembers& cim, VehicleInfo& vehicle_info)
{
  // Apply upper saturation at 9.5 N and lower saturation at 0.3 N
  Eigen::Vector4d thrust_vector_quadcopter_saturated = cim.thrust_vector_quadcopter.cwiseMin(9.5).cwiseMax(0.3);

  cim.thrust_vector_quadcopter_normalized[0] = 
    evaluatePolynomial(vehicle_info.thrust_polynomial_coefficients_quadcopter,
                       thrust_vector_quadcopter_saturated[0]);

  cim.thrust_vector_quadcopter_normalized[1] = 
    evaluatePolynomial(vehicle_info.thrust_polynomial_coefficients_quadcopter,
                       thrust_vector_quadcopter_saturated[1]);

  cim.thrust_vector_quadcopter_normalized[2] = 
    evaluatePolynomial(vehicle_info.thrust_polynomial_coefficients_quadcopter,
                       thrust_vector_quadcopter_saturated[2]);

  cim.thrust_vector_quadcopter_normalized[3] = 
    evaluatePolynomial(vehicle_info.thrust_polynomial_coefficients_quadcopter,
                       thrust_vector_quadcopter_saturated[3]);

}
