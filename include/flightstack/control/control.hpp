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
 * File:        control.hpp
 * Author:      Mattia Gramuglia
 * Date:        April 19, 2024
 * For info:    Andrea L'Afflitto 
 *              a.lafflitto@vt.edu
 * 
 * Description: General class that contains common members used by most of the control algorithms.
 *              Control algorithm classes inherit from this class.
 * 
 * GitHub:    https://github.com/andrealaffly/ACSL-flightstack.git
 **********************************************************************************************************************/

#ifndef CONTROL_HPP
#define CONTROL_HPP

#include <cmath>
#include <vector>
#include <functional> 
#include <chrono>

#include <Eigen/Dense>
// #include <boost/numeric/odeint.hpp>
#include <boost/numeric/odeint/stepper/runge_kutta_fehlberg78.hpp>
#include <boost/numeric/odeint/stepper/runge_kutta_cash_karp54.hpp>
#include <boost/numeric/odeint/stepper/runge_kutta_dopri5.hpp>
#include <boost/numeric/odeint/stepper/runge_kutta4.hpp>

#include "vehicle_info.hpp"
#include "piecewise_polynomial_trajectory.hpp"
#include "outer_loop_safety_mechanism.hpp"

using namespace boost::numeric::odeint;

// The type of container used to hold the state vector
typedef std::vector<double> state_type;

// Forward declaration of MultiThreadedNode class
class MultiThreadedNode;

class Control 
{
public:

  /*
    This struct contains the variables that are common among all controllers as they are the foundations of the base
    control architecture framework.
  */
  struct ControlInternalMembers
  {
    // Constructor
    ControlInternalMembers();

    Eigen::Vector3d mu_translational_raw_global; // translational virtual force in global coordinates before applying safety mechanism
    Eigen::Vector3d mu_translational_raw_local; // translational virtual force in local coordinates before applying safety mechanism
    Eigen::Vector3d mu_translational; // translational virtual force in local coordinates after applying safety mechanism
    Eigen::Vector4d U_control_inputs; // U1, U2, U3, U4
    Eigen::Ref<Eigen::Vector3d> U2_U3_U4; // reference vector to U_control_inputs which includes only U2, U3, U4
    Eigen::Vector3d angular_position_desired; // [roll_desired, pitch_desired, user_defined_yaw]
    Eigen::Vector3d angular_position_desired_dot; // [roll_desired_dot, pitch_desired_dot, user_defined_yaw_dot]
    Eigen::Vector3d angular_position_desired_dot_dot; // [roll_desired_dot_dot, pitch_desired_dot_dot, user_defined_yaw_dot_dot]
    double& roll_desired; // desired roll angle computed by outer loop and sent as command to the inner loop
    double& pitch_desired; // desired pitch angle computed by outer loop and sent as command to the inner loop
    double& roll_desired_dot; // first time derivative of roll_desired
    double& pitch_desired_dot; // first time derivative of pitch_desired
    double& roll_desired_dot_dot; // second time derivative of roll_desired
    double& pitch_desired_dot_dot; // second time derivative of pitch_desired
    Eigen::Vector2d internal_state_roll_des_filter; // internal state for filter/differentiator for desired roll
    Eigen::Vector2d internal_state_pitch_des_filter; // internal state for filter/differentiator for desired pitch
    Eigen::Vector2d internal_state_roll_dot_des_filter; // internal state for filter/differentiator for desired roll_dot
    Eigen::Vector2d internal_state_pitch_dot_des_filter; // internal state for filter/differentiator for desired pitch_dot
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

  // Constructor
  Control(MultiThreadedNode& node);

  void readJSONdifferentiatorFile();

  // Getter functions
  MultiThreadedNode& getNode() const;
  const VehicleInfo& getVehicleInfo() const;
  const ControlInternalMembers& getControlInternalMembers() const;
  const outer_loop_safety_mechanism::OuterLoopSafetyMechanism& getOuterLoopSafetyMechanism() const;
  const std::chrono::duration<double, std::micro>& getAlgorithmExecutionTimeMicroseconds() const;

  // Setter function
  void setAlgorithmExecutionTimeMicroseconds(std::chrono::duration<double, std::micro> duration);

  Eigen::Matrix3d jacobianMatrix(const double roll, const double pitch);

  Eigen::Matrix3d jacobianMatrixDot(const double roll,
                                    const double pitch,
                                    const double roll_dot, 
                                    const double pitch_dot);

  Eigen::Matrix3d jacobianMatrixInverse(const double roll, const double pitch);

  Eigen::Matrix3d rotationMatrix321GlobalToLocal(const double roll, const double pitch, const double yaw);

  void computeNormalizedThrustQuadcopterMode(ControlInternalMembers& cim, VehicleInfo& vehicle_info);

  void computeNormalizedThrust(ControlInternalMembers& cim, VehicleInfo& vehicle_info);

  void compute_U1_RollDes_PitchDes(ControlInternalMembers& cim);

  void computeTranslationalPositionError(ControlInternalMembers& cim,
                                         const Eigen::Vector3d& position,
                                         const Eigen::Vector3d& desired_position);

  void computeAngularError(ControlInternalMembers& cim,
                           const Eigen::Vector3d& euler_angles_rpy,
                           const Eigen::Vector3d& angular_position_desired);

  void computeRotationalParameters(ControlInternalMembers& cim, ControlReferences& cr);

  double wrapAngleToMinusPiAndPi(double alpha);

  double makeYawAngularErrorContinuous(double yaw, double user_defined_yaw);

  /*********************************************************************************************************************
   * Choose INTEGRATOR
  *********************************************************************************************************************/
   
  // // Runge-Kutta 4 integrator
  // runge_kutta4< state_type > stepper;

  // // Dormand-Prince 5 integrator
  // runge_kutta_dopri5< state_type > stepper;

  // // Fehlberg 78 integrator
  // runge_kutta_fehlberg78< state_type > stepper;

  // // Cash-Karp integrator
  runge_kutta_cash_karp54< state_type > stepper;

  /********************************************************************************************************************/

  template <typename T>
  inline void assignElementsToDxdt(T& entity, state_type &dxdt, int& current_index)
  {
    // constexpr int Rows = T::RowsAtCompileTime; // Get number of rows at compile time
    // constexpr int Cols = T::ColsAtCompileTime; // Get number of columns at compile time
    // Eigen::Reshaped<T, Rows * Cols, 1, 1> reshaped_entity = entity.reshaped();

    auto reshaped_entity = entity.reshaped();
    std::copy(reshaped_entity.begin(),
              reshaped_entity.end(),
              dxdt.begin() + current_index);

    current_index += entity.size();
  }

  /*
    Implementation of the smooth version of the dead-zone modification of MRAC through a 
    modulation function
    For reference: E. Lavretsky, K. Wise, "Robust and Adaptive Control", Springer 2013, Sec. 11.2.1
    PARAMETERS:
      - delta: constant that must be 0 < delta < 1. delta characterizes the slope of the modulation function.
      - e_0: constant that must be e_0 > 0. The dead-zone modification stops the adaptation process when 
             the norm of the tracking error becomes smaller than the prescribed value e_0.
  */
  template <typename T>
  inline double deadZoneModulationFunction(const T& e_vector,
                                           const double delta,
                                           const double e_0)
  {
    return std::max(0.0, std::min(1.0, (e_vector.norm() - delta * e_0)/((1.0 - delta) * e_0)));
  }

  /*
    Adaptive law formula equipped with the dead-zone modification and e-modification
  */
  template <typename Der1, typename Der2, typename Der3, typename Der4>
  inline auto adaptiveLawDeadzoneEmodification(const Der1& Gamma_gain,
                                               const double dead_zone_value,
                                               const Der2& pi_vector,
                                               const Der3& eTranspose_P_B,
                                               const double sigma_gain,
                                               const double eTranspose_P_B_norm,
                                               const Der4& K_hat_state)
  {
    auto K_hat_state_dot = Gamma_gain * dead_zone_value * (
                            pi_vector * eTranspose_P_B 
                            - sigma_gain * eTranspose_P_B_norm * K_hat_state);

    return K_hat_state_dot;
  }

protected:

  VehicleInfo vehicle_info;

  ControlInternalMembers cim;

  ControlReferences cr;

  outer_loop_safety_mechanism::OuterLoopSafetyMechanism safe_mech;

  // Third standard basis vector e_3
  static inline const Eigen::Vector3d e3_basis = Eigen::Vector3d(0.0, 0.0, 1.0);

private:

  MultiThreadedNode& node_;

  std::chrono::duration<double, std::micro> algorithm_execution_time_microseconds_;

};


#endif // CONTROL_HPP
