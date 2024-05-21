/* pid.cpp

	Mattia Gramuglia
	April 22, 2024
*/


#include "multi_threaded_node.hpp"
#include "pid.hpp"
#include "json_parser.hpp"

// Constructor
PID::PID(MultiThreadedNode& node) : 
  Control(node)
{}

// Constructor initializes the full_state and sets the Map references
StatePID::StatePID() : 
  full_state(10, 0.0), // 10-element full_state vector
  state_roll_ref_filter(full_state.data()), // Maps the elements [0:1]
  state_pitch_ref_filter(full_state.data() + 2), // Maps the elements [2:3]
  integral_translational_position_error(full_state.data() + 4), // Maps the elements [4:6]
  integral_angular_error(full_state.data() + 7) // Maps the elements [7:9]
{}

StatePID& PID::getStatePID()
{
  return state_;
}

const double& PID::getTimeStepRK4() const
{
  return time_step_rk4_;
}

const GainsPID& PID::getGainsPID() const
{
  return gains_;
}

/*
  Function to read the tuning gains coming from the .json file and assign it to the
  gains_ struct instantiated in the PID class
*/
void PID::readJSONfile(const std::string& jsonFile)
{
  std::ifstream file(jsonFile);
	nlohmann::json j;
	file >> j;
	
	gains_.KP_translational = extractMatrix3dFromJSON(j["KP_translational"]);
	gains_.KD_translational = extractMatrix3dFromJSON(j["KD_translational"]);
  gains_.KI_translational = extractMatrix3dFromJSON(j["KI_translational"]);
	gains_.KP_rotational = extractMatrix3dFromJSON(j["KP_rotational"]);
  gains_.KD_rotational = extractMatrix3dFromJSON(j["KD_rotational"]);
  gains_.KI_rotational = extractMatrix3dFromJSON(j["KI_rotational"]);

}

/*
  Assigning the input parameter 'x', which represents the full_state, to the member variables of the state.
*/
void PID::assignStateVariables(state_type &x, StatePID& state)
{
  int current_index = 0; // Starting index for assignments

  // Assign elements [0:1] of 'x' to 'state_roll_ref_filter'
  state.state_roll_ref_filter = Eigen::Map<Eigen::Vector2d>(x.data() + current_index,
                                                            state.state_roll_ref_filter.size());
  current_index += state.state_roll_ref_filter.size();

  // Assign elements [2:3] of 'x' to 'state_pitch_ref_filter'
  state.state_pitch_ref_filter = Eigen::Map<Eigen::Vector2d>(x.data() + current_index,
                                                             state.state_pitch_ref_filter.size());
  current_index += state.state_pitch_ref_filter.size();

  // Assign elements [4:6] of 'x' to 'integral_translational_position_error'
  state.integral_translational_position_error = Eigen::Map<Eigen::Vector3d>(
    x.data() + current_index,
    state.integral_translational_position_error.size()
    );
  current_index += state.integral_translational_position_error.size();
  
  // Assign elements [7:9] of 'x' to 'integral_angular_error'
  state.integral_angular_error = Eigen::Map<Eigen::Vector3d>(x.data() + current_index,
                                                             state.integral_angular_error.size());                                                        
}

/*
  Assigning the right-hand side of the differential equations to the first time derivative of the full_state.
*/
void PID::assignControlInternalMembersToDxdt(ControlInternalMembers& cim, state_type& dxdt, StatePID& state) 
{
  if (dxdt.size() < state.full_state.size()) {
    // Ensure the output vector has enough size
    throw std::invalid_argument("dxdt vector must have at least 10 elements");
  }

  int current_index = 0; // Starting index for assignments

  // Assign to elements [0:1] of 'dxdt' --> 'internal_state_roll_ref_filter'
  std::copy(cim.internal_state_roll_ref_filter.begin(),
            cim.internal_state_roll_ref_filter.end(),
            dxdt.begin() + current_index);
  current_index += cim.internal_state_roll_ref_filter.size(); // Update the index

  // Assign to elements [2:3] of 'dxdt' --> 'internal_state_pitch_ref_filter'
  std::copy(cim.internal_state_pitch_ref_filter.begin(),
            cim.internal_state_pitch_ref_filter.end(),
            dxdt.begin() + current_index);
  current_index += cim.internal_state_pitch_ref_filter.size(); // Update the index

  // Assign to elements [4:6] of 'dxdt' --> 'translational_position_error'
  std::copy(cim.translational_position_error.begin(),
            cim.translational_position_error.end(),
            dxdt.begin() + current_index);
  current_index += cim.translational_position_error.size(); // Update the index

  // Assign to elements [7:9] of 'dxdt' --> 'angular_error'
  std::copy(cim.angular_error.begin(),
            cim.angular_error.end(),
            dxdt.begin() + current_index);

}

/*
  PID Control algorithm
*/
void PID::computeControlAlgorithm(state_type &x, state_type &dxdt, const double /* t */) 
{
  // Assigning the input parameter 'x', which represents the full_state, to the member variables of the state
  assignStateVariables(x, state_);
  
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

  cim.rotation_matrix_321_global_to_local = this->rotationMatrix321GlobalToLocal(cr.euler_angles_rpy[0],
                                                                                 cr.euler_angles_rpy[1],
                                                                                 cr.euler_angles_rpy[2]);

  cim.mu_translational_raw = vehicle_info.mass * (
    - gains_.KP_translational * cim.translational_position_error
    - gains_.KD_translational * (cr.velocity - cr.user_defined_velocity)
    - gains_.KI_translational * state_.integral_translational_position_error
    + cr.user_defined_acceleration
  );

  // Convert the mu_translational from global to local coordinates
  cim.mu_translational = cim.rotation_matrix_321_global_to_local * cim.mu_translational_raw; // TEMPORARY UNTIL THE SAFETY MECHANISM IS IMPLEMENTED

  cim.U_control_inputs[0] = std::sqrt(
    std::pow(cim.mu_translational[0], 2)
    + std::pow(cim.mu_translational[1], 2)
    + std::pow((vehicle_info.mass * GRAVITATIONAL_ACCELERATION - cim.mu_translational[2]), 2)
  );

  double temporaryvar_roll_reference_1 = -(1/cim.U_control_inputs[0]) * (
    cim.mu_translational[0] * std::sin(cr.user_defined_yaw)
    - cim.mu_translational[1] * std::cos(cr.user_defined_yaw)
  );
  cim.roll_reference = std::atan2(
    temporaryvar_roll_reference_1,
    std::sqrt(1 - std::pow(temporaryvar_roll_reference_1, 2))
  );

  cim.pitch_reference = std::atan2(
    - (cim.mu_translational[0] * std::cos(cr.user_defined_yaw)
    +  cim.mu_translational[1] * std::sin(cr.user_defined_yaw)),
    vehicle_info.mass * GRAVITATIONAL_ACCELERATION - cim.mu_translational[2]
  );

  // Compute angular error
  cim.angular_error[0] = cr.euler_angles_rpy[0] - cim.roll_reference;
  cim.angular_error[1] = cr.euler_angles_rpy[1] - cim.pitch_reference;
  cim.angular_error[2] = std::fmod((cr.euler_angles_rpy[2] - cr.user_defined_yaw + M_PI), 2*M_PI) - M_PI;

  cim.internal_state_roll_ref_filter = vehicle_info.A_filter_roll_ref * state_.state_roll_ref_filter 
                                     + vehicle_info.B_filter_roll_ref * cim.roll_reference;

  cim.internal_state_pitch_ref_filter = vehicle_info.A_filter_pitch_ref * state_.state_pitch_ref_filter 
                                    + vehicle_info.B_filter_pitch_ref * cim.pitch_reference;

  cim.roll_reference_dot = vehicle_info.C_filter_roll_ref * state_.state_roll_ref_filter;
  cim.pitch_reference_dot = vehicle_info.C_filter_pitch_ref * state_.state_pitch_ref_filter;

  cim.roll_reference_dot_dot = vehicle_info.C_filter_roll_ref * cim.internal_state_roll_ref_filter;
  cim.pitch_reference_dot_dot = vehicle_info.C_filter_pitch_ref * cim.internal_state_pitch_ref_filter;

  cim.euler_angles_rpy_dot = cim.jacobian_matrix_inverse * cr.angular_velocity;

  cim.U2_U3_U4 = cr.angular_velocity.cross(vehicle_info.inertia_matrix * cr.angular_velocity) + 
    vehicle_info.inertia_matrix * (
    - gains_.KP_rotational * cim.angular_error 
    - gains_.KD_rotational * (cim.euler_angles_rpy_dot - cim.angular_position_reference_dot)
    - gains_.KI_rotational * state_.integral_angular_error
    + cim.angular_position_reference_dot_dot
  );

  // cim.thrust_vector = vehicle_info.mixer_matrix * cim.U_control_inputs; // FOR X8-COPTER

  cim.thrust_vector_quadcopter = vehicle_info.mixer_matrix_quadcopter * cim.U_control_inputs; // FOR QUADCOPTER

  computeNormalizedThrustQuadcopterMode(cim, vehicle_info);


  // *************** DEBUGGING ***************
  // std::cout << "ALIAS thrust_vector_quadcopter DEBUG inside controller: " << cim.thrust_vector_quadcopter << std::endl;
  // std::cout << "ALIAS thrust_vector_quadcopter_normalized DEBUG inside controller: " << cim.thrust_vector_quadcopter_normalized << std::endl;
  
  // std::cout << "ALIAS mixer_matrix DEBUG inside controller: " << vehicle_info.mixer_matrix << std::endl;
  // std::cout << "ALIAS jacobian_matrix_inverse DEBUG inside controller: " << cim.jacobian_matrix_inverse << std::endl;
  // std::cout << "ALIAS mass DEBUG inside controller: " << vehicle_info.mass << std::endl;
  // std::cout << "ALIAS state_roll_ref_filter DEBUG inside controller: " << state_.state_roll_ref_filter << std::endl;
  // std::cout << "ALIAS KP_translational DEBUG inside controller: " << gains_.KP_translational << std::endl;

  // std::cout << "ALIAS user_defined_position DEBUG inside controller: " << cr.user_defined_position << std::endl;
  // std::cout << "ALIAS user_defined_velocity DEBUG inside controller: " << cr.user_defined_velocity << std::endl;
  // std::cout << "ALIAS user_defined_acceleration DEBUG inside controller: " << cr.user_defined_acceleration << std::endl;
  // std::cout << "ALIAS user_defined_yaw_ DEBUG inside controller: " << cr.user_defined_yaw << std::endl;
  // std::cout << "ALIAS user_defined_yaw_dot DEBUG inside controller: " << cr.user_defined_yaw_dot << std::endl;
  // std::cout << "ALIAS user_defined_yaw_dot_dot DEBUG inside controller: " << cr.user_defined_yaw_dot_dot << std::endl;

  // std::cout << "ALIAS position DEBUG inside controller: " << cr.position << std::endl;
  // std::cout << "ALIAS velocity DEBUG inside controller: " << cr.velocity << std::endl;
  // std::cout << "ALIAS euler_angles_rpy DEBUG inside controller: " << cr.euler_angles_rpy << std::endl;
  // std::cout << "ALIAS angular_velocity DEBUG inside controller: " << cr.angular_velocity << std::endl;
  // ************* END DEBUGGING *************

  // Assigning the right-hand side of the differential equations to the first time derivative of the full_state
  assignControlInternalMembersToDxdt(cim, dxdt, state_);

}
