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
{
  // Initialize controller gains
  this->readJSONfile("gains_pid.json");

  log_data_ = std::make_shared<LogData_PID>(node, *this);

  // Initialize logging
	log_data_->logInitializeLogging();
}

// Constructor initializes the full_state and sets the Eigen::Map references
StatePID::StatePID() : 
  full_state(10, 0.0), // 10-element full_state vector
  state_roll_ref_filter(full_state.data()), // Maps the elements [0:1]
  state_pitch_ref_filter(full_state.data() + 2), // Maps the elements [2:3]
  integral_translational_position_error(full_state.data() + 4), // Maps the elements [4:6]
  integral_angular_error(full_state.data() + 7) // Maps the elements [7:9]
{}

// Constructor
ControllerSpecificInternalMembers::ControllerSpecificInternalMembers() : 
  outer_loop_P(Eigen::Vector3d::Zero()),
  outer_loop_I(Eigen::Vector3d::Zero()),
  outer_loop_D(Eigen::Vector3d::Zero()),
  outer_loop_dynamic_inversion(Eigen::Vector3d::Zero()),
  inner_loop_P(Eigen::Vector3d::Zero()),
  inner_loop_I(Eigen::Vector3d::Zero()),
  inner_loop_D(Eigen::Vector3d::Zero()),
  inner_loop_dynamic_inversion(Eigen::Vector3d::Zero())
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

// Getter function that returns a reference to the current getControllerSpecificInternalMembers object
const ControllerSpecificInternalMembers& PID::getControllerSpecificInternalMembers() const
{
  return csim_;
}

/*
  Getter function that returns a pointer to the log_data_ instance
*/
std::shared_ptr<LogData_PID> PID::getLogData() const
{
  return log_data_;
}

/*
  Function to read the tuning gains coming from the .json file and assign it to the
  gains_ struct instantiated in the PID class
*/
void PID::readJSONfile(const std::string& fileName)
{
  // Define the path where the PID gains JSON files are located
  const std::string path = "./src/flightstack/params/control/pid/";

  // Concatenate the path with the file name
  std::string jsonFile = path + fileName;

  std::ifstream file(jsonFile);
  if (!file.is_open()) {
    throw std::runtime_error("Could not open file: " + jsonFile);
  }

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
  Function to compute the variables used for the differentiator of the reference roll and pitch angles.
  This function calculates the internal states and derivatives (first and second) of the roll and pitch
  reference angles using the provided filter coefficients and the current state variables.
*/
void PID::computeFilterDifferentiatorVariables(ControlInternalMembers& cim, VehicleInfo& vehicle_info, StatePID& state_)
{
  cim.internal_state_roll_ref_filter = vehicle_info.A_filter_roll_ref * state_.state_roll_ref_filter 
                                     + vehicle_info.B_filter_roll_ref * cim.roll_reference;

  cim.internal_state_pitch_ref_filter = vehicle_info.A_filter_pitch_ref * state_.state_pitch_ref_filter 
                                      + vehicle_info.B_filter_pitch_ref * cim.pitch_reference;

  cim.roll_reference_dot = vehicle_info.C_filter_roll_ref * state_.state_roll_ref_filter;
  cim.pitch_reference_dot = vehicle_info.C_filter_pitch_ref * state_.state_pitch_ref_filter;

  cim.roll_reference_dot_dot = vehicle_info.C_filter_roll_ref * cim.internal_state_roll_ref_filter;
  cim.pitch_reference_dot_dot = vehicle_info.C_filter_pitch_ref * cim.internal_state_pitch_ref_filter;
}

/*
  OUTER LOOP CONTROLLER
*/
void PID::computeOuterLoop(ControlInternalMembers& cim,
                            VehicleInfo& vehicle_info,
                            StatePID& state_, 
                            ControlReferences& cr,
                            GainsPID& gains_,
                            ControllerSpecificInternalMembers& csim_)
{
  csim_.outer_loop_dynamic_inversion = - vehicle_info.mass * GRAVITATIONAL_ACCELERATION * this->e3_basis;
  csim_.outer_loop_P = - gains_.KP_translational * cim.translational_position_error;
  csim_.outer_loop_D = - gains_.KD_translational * (cr.velocity - cr.user_defined_velocity);
  csim_.outer_loop_I = - gains_.KI_translational * state_.integral_translational_position_error;

  cim.mu_translational_raw = csim_.outer_loop_dynamic_inversion + 
    vehicle_info.mass * (
      csim_.outer_loop_P
    + csim_.outer_loop_D
    + csim_.outer_loop_I
    + cr.user_defined_acceleration
  );
}

/*
  INNER LOOP CONTROLLER
*/
void PID::computeInnerLoop(ControlInternalMembers& cim,
                            VehicleInfo& vehicle_info,
                            StatePID& state_, 
                            ControlReferences& cr,
                            GainsPID& gains_,
                            ControllerSpecificInternalMembers& csim_)
{
  csim_.inner_loop_dynamic_inversion = cr.angular_velocity.cross(vehicle_info.inertia_matrix * cr.angular_velocity);
  csim_.inner_loop_P = - gains_.KP_rotational * cim.angular_error;
  csim_.inner_loop_D = - gains_.KD_rotational * (cim.euler_angles_rpy_dot - cim.angular_position_reference_dot);
  csim_.inner_loop_I = - gains_.KI_rotational * state_.integral_angular_error;

  cim.U2_U3_U4 = csim_.inner_loop_dynamic_inversion + 
    vehicle_info.inertia_matrix * (
      csim_.inner_loop_P 
    + csim_.inner_loop_D
    + csim_.inner_loop_I
    + cim.angular_position_reference_dot_dot
  );
}


/*
  PID Control algorithm
*/
void PID::computeControlAlgorithm(state_type &x, state_type &dxdt, const double /* t */) 
{
  // Assigning the input parameter 'x', which represents the full_state, to the member variables of the state
  this->assignStateVariables(x, state_);

  /*
    This function calculates the translational position error, sets the reference angular rate 
    and angular acceleration for yaw, computes the inverse of the Jacobian matrix, the derivative 
    of the Euler angles, and the rotation matrix to go from global to local coordinates.
  */
  this->computeTranslationalAndRotationalParameters(cim, cr);
  
  /*
    Compute OUTER LOOP CONTROLLER
  */
  this->computeOuterLoop(cim, vehicle_info, state_, cr, gains_, csim_);

  // IMPLEMENT THE SAFETY MECHANISM HERE

  /*
  Compute:
    the Total Thrust: cim.U_control_inputs[0],
    the desired/reference roll angle: cim.roll_reference,
    the desired/reference pitch angle: cim.pitch_reference.
  */
  this->compute_U1_RollRef_PitchRef(cim);

  // Compute angular error
  this->computeAngularError(cim, cr);

  /*
    Compute the variables used for the differentiator of the reference roll and pitch angles.
  */
  this->computeFilterDifferentiatorVariables(cim, vehicle_info, state_);

  /*
    Compute INNER LOOP CONTROLLER
  */
  this->computeInnerLoop(cim, vehicle_info, state_, cr, gains_, csim_);

  /*
    Compute the thrust in Newton that each motor needs to produce
  */
  // FOR X8-COPTER
  cim.thrust_vector = vehicle_info.mixer_matrix * cim.U_control_inputs; 

  // FOR QUADCOPTER
  // cim.thrust_vector_quadcopter = vehicle_info.mixer_matrix_quadcopter * cim.U_control_inputs; 

  /*
  Convert the thrust that each motor needs to generate from Newton to
  the normalized value comprised between 0 and 1 to be sent to Pixhawk
  */
  // FOR X8COPTER
  this->computeNormalizedThrust(cim, vehicle_info);

  // FOR QUADCOPTER
  // this->computeNormalizedThrustQuadcopterMode(cim, vehicle_info);

   


  // *************** DEBUGGING ***************
  /* 
  std::cout << "ALIAS thrust_vector_quadcopter DEBUG inside controller: " << cim.thrust_vector_quadcopter << std::endl;
  std::cout << "ALIAS thrust_vector_quadcopter_normalized DEBUG inside controller: " << cim.thrust_vector_quadcopter_normalized << std::endl;
  
  std::cout << "ALIAS mixer_matrix DEBUG inside controller: " << vehicle_info.mixer_matrix << std::endl;
  std::cout << "ALIAS jacobian_matrix_inverse DEBUG inside controller: " << cim.jacobian_matrix_inverse << std::endl;
  std::cout << "ALIAS mass DEBUG inside controller: " << vehicle_info.mass << std::endl;
  std::cout << "ALIAS state_roll_ref_filter DEBUG inside controller: " << state_.state_roll_ref_filter << std::endl;
  std::cout << "ALIAS KP_translational DEBUG inside controller: " << gains_.KP_translational << std::endl;

  std::cout << "ALIAS user_defined_position DEBUG inside controller: " << cr.user_defined_position << std::endl;
  std::cout << "ALIAS user_defined_velocity DEBUG inside controller: " << cr.user_defined_velocity << std::endl;
  std::cout << "ALIAS user_defined_acceleration DEBUG inside controller: " << cr.user_defined_acceleration << std::endl;
  std::cout << "ALIAS user_defined_yaw_ DEBUG inside controller: " << cr.user_defined_yaw << std::endl;
  std::cout << "ALIAS user_defined_yaw_dot DEBUG inside controller: " << cr.user_defined_yaw_dot << std::endl;
  std::cout << "ALIAS user_defined_yaw_dot_dot DEBUG inside controller: " << cr.user_defined_yaw_dot_dot << std::endl;

  std::cout << "ALIAS position DEBUG inside controller: " << cr.position << std::endl;
  std::cout << "ALIAS velocity DEBUG inside controller: " << cr.velocity << std::endl;
  std::cout << "ALIAS euler_angles_rpy DEBUG inside controller: " << cr.euler_angles_rpy << std::endl;
  std::cout << "ALIAS angular_velocity DEBUG inside controller: " << cr.angular_velocity << std::endl;
  */
  // ************* END DEBUGGING *************

  // Assigning the right-hand side of the differential equations to the first time derivative of the full_state
  this->assignControlInternalMembersToDxdt(cim, dxdt, state_);

}
