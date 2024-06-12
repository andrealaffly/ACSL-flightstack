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

/*
  Function that takes as input the thrust that each motor needs to generate in Newton and converts it
  in the normalized value comprised between 0 and 1 according to the experimental thrust/motor curve 
  obtained experimentally using the thrust stand.
  For QUADCOPTER 
*/
void Control::computeNormalizedThrustQuadcopterMode(ControlInternalMembers& cim, VehicleInfo& vehicle_info)
{
  /* 
  Apply upper saturation at UPPER_MOTOR_THRUST_SATURATION_LIMIT_IN_NEWTON and
  lower saturation at LOWER_MOTOR_THRUST_SATURATION_LIMIT_IN_NEWTON
  */
  Eigen::Vector4d thrust_vector_quadcopter_saturated = (cim.thrust_vector_quadcopter.
    cwiseMin(UPPER_MOTOR_THRUST_SATURATION_LIMIT_IN_NEWTON).
    cwiseMax(LOWER_MOTOR_THRUST_SATURATION_LIMIT_IN_NEWTON));

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

/*
  Function that takes as input the thrust that each motor needs to generate in Newton and converts it
  in the normalized value comprised between 0 and 1 according to the experimental thrust/motor curve 
  obtained experimentally using the thrust stand.
  For X8-COPTER 
*/
void Control::computeNormalizedThrust(ControlInternalMembers& cim, VehicleInfo& vehicle_info)
{
  /* 
  Apply upper saturation at UPPER_MOTOR_THRUST_SATURATION_LIMIT_IN_NEWTON and
  lower saturation at LOWER_MOTOR_THRUST_SATURATION_LIMIT_IN_NEWTON
  */
  Eigen::Matrix<double, 8, 1> thrust_vector_saturated = (cim.thrust_vector.
    cwiseMin(UPPER_MOTOR_THRUST_SATURATION_LIMIT_IN_NEWTON).
    cwiseMax(LOWER_MOTOR_THRUST_SATURATION_LIMIT_IN_NEWTON));

  cim.thrust_vector_normalized[0] = 
    evaluatePolynomial(vehicle_info.thrust_polynomial_coefficients_quadcopter,
                       thrust_vector_saturated[0]);

  cim.thrust_vector_normalized[1] = 
    evaluatePolynomial(vehicle_info.thrust_polynomial_coefficients_quadcopter,
                       thrust_vector_saturated[1]);

  cim.thrust_vector_normalized[2] = 
    evaluatePolynomial(vehicle_info.thrust_polynomial_coefficients_quadcopter,
                       thrust_vector_saturated[2]);

  cim.thrust_vector_normalized[3] = 
    evaluatePolynomial(vehicle_info.thrust_polynomial_coefficients_quadcopter,
                       thrust_vector_saturated[3]);

  cim.thrust_vector_normalized[4] = 
    evaluatePolynomial(vehicle_info.thrust_polynomial_coefficients_quadcopter,
                       thrust_vector_saturated[4]);

  cim.thrust_vector_normalized[5] = 
    evaluatePolynomial(vehicle_info.thrust_polynomial_coefficients_quadcopter,
                       thrust_vector_saturated[5]);

  cim.thrust_vector_normalized[6] = 
    evaluatePolynomial(vehicle_info.thrust_polynomial_coefficients_quadcopter,
                       thrust_vector_saturated[6]);

  cim.thrust_vector_normalized[7] = 
    evaluatePolynomial(vehicle_info.thrust_polynomial_coefficients_quadcopter,
                       thrust_vector_saturated[7]);

}

/*
  Function that computes:
    the Total Thrust U1,
    the desired/reference roll angle RollRef,
    the desired/reference pitch angle PitchRef.
  U1 will be fed to mixer matrix, while RollRef and PitchRef will be fed to the inner loop.
*/
void Control::compute_U1_RollRef_PitchRef(ControlInternalMembers& cim)
{
  // Convert the mu_translational from global to local coordinates
  cim.mu_translational = cim.rotation_matrix_321_global_to_local * cim.mu_translational_raw; 

  cim.U_control_inputs[0] = std::sqrt(
      std::pow(cim.mu_translational[0], 2)
    + std::pow(cim.mu_translational[1], 2)
    + std::pow(cim.mu_translational[2], 2)
  );

  double temporaryvar_roll_reference_1 = cim.mu_translational[1] / cim.U_control_inputs[0];
  cim.roll_reference = std::atan2(
    temporaryvar_roll_reference_1,
    std::sqrt(1 - std::pow(temporaryvar_roll_reference_1, 2))
  );

  cim.pitch_reference = std::atan2(-cim.mu_translational[0], -cim.mu_translational[2]);
}

/*
  Function to compute the angular error for roll, pitch, and yaw angles.
  This function calculates the difference between the current euler angles
  (roll, pitch, yaw) and the reference angles, taking into account the
  wrapping of the yaw angle to ensure it remains within the range [-pi, pi]
  to handle the discontinuity at +/- pi.
*/
void Control::computeAngularError(ControlInternalMembers& cim, ControlReferences& cr)
{
  cim.angular_error[0] = cr.euler_angles_rpy[0] - cim.roll_reference;
  cim.angular_error[1] = cr.euler_angles_rpy[1] - cim.pitch_reference;
  cim.angular_error[2] = makeYawAngularErrorContinuous(cr.euler_angles_rpy[2], cr.user_defined_yaw);
}

/*
  Function to compute the translational position error and various rotational parameters.
  This function calculates the translational position error, sets the reference angular rate 
  and angular acceleration for yaw, computes the inverse of the Jacobian matrix, the derivative 
  of the Euler angles, and the rotation matrix to go from global to local coordinates.
*/
void Control::computeTranslationalAndRotationalParameters(ControlInternalMembers& cim, ControlReferences& cr)
{
  // translational_position_error computation
  cim.translational_position_error = cr.position - cr.user_defined_position;

  // Set angular_position_reference_dot (angular_position_reference_dot[0] and angular_position_reference_dot[1] are
  // automatically updated as they are referenced by roll_reference_dot and pitch_reference_dot)
  cim.angular_position_reference_dot[2] = cr.user_defined_yaw_dot; 

  // Set angular_position_reference_dot_dot (angular_position_reference_dot_dot[0] and 
  // angular_position_reference_dot_dot[1] are automatically updated as they are referenced by roll_reference_dot
  // and pitch_reference_dot)
  cim.angular_position_reference_dot_dot[2] = cr.user_defined_yaw_dot_dot;

  cim.jacobian_matrix_inverse = this->jacobianMatrixInverse(cr.euler_angles_rpy[0], cr.euler_angles_rpy[1]);

  cim.euler_angles_rpy_dot = cim.jacobian_matrix_inverse * cr.angular_velocity;

  cim.rotation_matrix_321_global_to_local = this->rotationMatrix321GlobalToLocal(cr.euler_angles_rpy[0],
                                                                                 cr.euler_angles_rpy[1],
                                                                                 cr.euler_angles_rpy[2]);
}

/*
  Function that takes as input an angle alpha and wraps it in the interval [-pi, pi]
*/
double Control::wrapAngleToMinusPiAndPi(double alpha)
{
  return alpha - 2 * M_PI * std::floor((alpha + M_PI) / (2 * M_PI));
}

/*
  Function that makes the yaw angular error continuos considering the discontinuity/wrapping that occurs
  at +/- pi for both the yaw and the user-defined yaw
*/
double Control::makeYawAngularErrorContinuous(double yaw, double user_defined_yaw)
{
  double continuos_error;
  
  user_defined_yaw = wrapAngleToMinusPiAndPi(user_defined_yaw);
  
  double raw_error = yaw - user_defined_yaw;  
  
  if ( std::abs(raw_error) >= M_PI && std::abs(raw_error) < 2*M_PI && raw_error < 0 )
  {
    continuos_error = std::fmod(raw_error, M_PI) + M_PI;
  }
  else if ( std::abs(raw_error) >= M_PI && std::abs(raw_error) < 2*M_PI && raw_error > 0 )
  {
    continuos_error = std::fmod(raw_error, M_PI) - M_PI;
  }
  else
  {
    continuos_error = std::fmod(raw_error, M_PI);   
  }
  
  return continuos_error;
}




