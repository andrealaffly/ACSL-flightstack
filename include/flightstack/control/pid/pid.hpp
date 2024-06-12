/* pid.hpp

	Mattia Gramuglia
	April 22, 2024
*/

#ifndef PID_HPP
#define PID_HPP

#include <cmath>
#include <algorithm>

#include <Eigen/Dense>

#include "control.hpp"
#include "pid_gains.hpp"
#include "logging_pid.hpp"


/*
  This struct represents the state of the PID controller differential equations that will be integrated
  at each RK4 iteration.
  IT IS NOT THE VEHICLE STATE (e.g. POSITION, VELOCITY, etc.).
*/
struct StatePID 
{
  // Constructor
  StatePID();

  std::vector<double> full_state; 

  // Those are references to portions of the full_state vector that behave and can be used just like any other
  // Eigen vector. Modifying those will directly act on the memory locations where they are stored, hence changes will
  // automatically be refelected in full_state
  Eigen::Map<Eigen::Vector2d> state_roll_ref_filter; 
  Eigen::Map<Eigen::Vector2d> state_pitch_ref_filter; 
  Eigen::Map<Eigen::Vector3d> integral_translational_position_error; 
  Eigen::Map<Eigen::Vector3d> integral_angular_error; 

};

/*
  Description
*/
struct ControllerSpecificInternalMembers 
{
  // Constructor
  ControllerSpecificInternalMembers();

  Eigen::Vector3d outer_loop_P; // Proportional part of the control input of the outer loop
  Eigen::Vector3d outer_loop_I; // Integral part of the control input of the outer loop
  Eigen::Vector3d outer_loop_D; // Derivative part of the control input of the outer loop
  Eigen::Vector3d outer_loop_dynamic_inversion; // Dynamic inversion term of the control input of the outer loop

  Eigen::Vector3d inner_loop_P; // Proportional part of the control input of the inner loop
  Eigen::Vector3d inner_loop_I; // Integral part of the control input of the inner loop
  Eigen::Vector3d inner_loop_D; // Derivative part of the control input of the inner loop
  Eigen::Vector3d inner_loop_dynamic_inversion; // Dynamic inversion term of the control input of the inner loop

};


class PID : public Control
{
public:

  // Constructor
  PID(MultiThreadedNode& node);

  // Getter functions
  StatePID& getStatePID();
  const double& getTimeStepRK4() const; 
  const GainsPID& getGainsPID() const;
  const ControllerSpecificInternalMembers& getControllerSpecificInternalMembers() const;
  std::shared_ptr<LogData_PID> getLogData() const;

  void readJSONfile(const std::string& fileName);

  void assignStateVariables(state_type &x, StatePID& state);

  void assignControlInternalMembersToDxdt(ControlInternalMembers& cim, state_type& dxdt, StatePID& state);

  void computeFilterDifferentiatorVariables(ControlInternalMembers& cim, VehicleInfo& vehicle_info, StatePID& state_);

  void computeOuterLoop(ControlInternalMembers& cim,
                        VehicleInfo& vehicle_info,
                        StatePID& state_, 
                        ControlReferences& cr,
                        GainsPID& gains_,
                        ControllerSpecificInternalMembers& csim_);

  void computeInnerLoop(ControlInternalMembers& cim,
                        VehicleInfo& vehicle_info,
                        StatePID& state_, 
                        ControlReferences& cr,
                        GainsPID& gains_,
                        ControllerSpecificInternalMembers& csim_);

  void computeControlAlgorithm(state_type &x, state_type &dxdt, const double /* t */);

private:

  StatePID state_;
  const double time_step_rk4_ = 0.01; // [s] time step for integrating using RK4

  GainsPID gains_;

  ControllerSpecificInternalMembers csim_;

  // Create a pointer to the LogData instance
  std::shared_ptr<LogData_PID> log_data_;

};



#endif // PID_HPP