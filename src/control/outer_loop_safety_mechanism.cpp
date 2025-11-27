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
 * File:        outer_loop_safety_mechanism.cpp
 * Author:      Mattia Gramuglia
 * Date:        October 28, 2024
 * For info:    Andrea L'Afflitto 
 *              a.lafflitto@vt.edu
 * 
 * Description: Safety mechanism that acts between the outer and inner loop. It allows to constrain the maximum desired
 *              pitch and roll angles, the maximum total thrust, and to avoid free-fall maneuvers.
 * 
 * GitHub:    https://github.com/andrealaffly/ACSL-flightstack.git
 **********************************************************************************************************************/

#include "outer_loop_safety_mechanism.hpp"

namespace outer_loop_safety_mechanism
{

const SafetyMechanismTuningParams& OuterLoopSafetyMechanism::getSafetyMechanismTuningParams() const
{
  return params_;
}

const SafetyMechanismMembers& OuterLoopSafetyMechanism::getSafetyMechanismMembers() const
{
  return mems_;
}

/*
  Function to read the tuning parameters coming from the .json file for the OuterLoopSafetyMechanism class
*/
void OuterLoopSafetyMechanism::readJSONfile()
{
  // Define the tuning parameters JSON file
  const std::string jsonFile = "./src/flightstack/params/control/outer_loop_safety_mechanism.json";

  std::ifstream file(jsonFile);
  if (!file.is_open()) {
    throw std::runtime_error("Could not open file: " + jsonFile);
  }

	nlohmann::json j;
	file >> j;
	
  this->params_.maximum_total_thrust = j["maximum_total_thrust"];
  this->params_.maximum_roll_angle = j["maximum_roll_angle"];
  this->params_.maximum_pitch_angle = j["maximum_pitch_angle"];
  this->params_.alpha_plane = j["alpha_plane"];

  this->params_.maximum_roll_angle = DEG2RAD(this->params_.maximum_roll_angle);
  this->params_.maximum_pitch_angle = DEG2RAD(this->params_.maximum_pitch_angle);

}

/*
  Function that implements the safety mechanism that acts between the outer and inner loop.
  It allows to constrain the maximum desired pitch and roll angles, the maximum total thrust,
  and to avoid free-fall maneuvers.
*/
Eigen::Vector3d OuterLoopSafetyMechanism::applySafetyMechansim(
  const Eigen::Vector3d& mu_virtual_control,
  const Eigen::Vector3d& gravitational_force_vector)
{

  Eigen::Vector2d tSphereVector(NAN, NAN);
  Eigen::Vector2d tEllipticConeVector(NAN, NAN);
  mems_.tPlane = NAN;

  Eigen::Vector3d mu_tilde = mu_virtual_control + gravitational_force_vector;
  double gravitational_force = gravitational_force_vector(2);

  double mu_tilde_norm = mu_tilde.norm();

  // Sphere intersection
  if (mu_tilde_norm >= sphere_epsilon) {
    double a = mu_tilde(2) * gravitational_force;
    double mu_tilde_norm_squared = mu_tilde_norm * mu_tilde_norm;
    double b = std::sqrt(a * a +
      mu_tilde_norm_squared * 
      (params_.maximum_total_thrust * params_.maximum_total_thrust - gravitational_force * gravitational_force));
    
    tSphereVector(0) = (a + b) / mu_tilde_norm_squared;
    tSphereVector(1) = (a - b) / mu_tilde_norm_squared;
  }

  // Elliptic Cone intersection
  double sqrtTerm = std::sqrt(std::pow(mu_tilde(0) / std::tan(params_.maximum_pitch_angle), 2) + 
                              std::pow(mu_tilde(1) / std::tan(params_.maximum_roll_angle), 2));
  if (std::abs(mu_tilde(2) + sqrtTerm) >= elliptic_cone_epsilon) {
    tEllipticConeVector(0) = gravitational_force / (mu_tilde(2) + sqrtTerm);
  }
  if (std::abs(-mu_tilde(2) + sqrtTerm) >= elliptic_cone_epsilon) {
    tEllipticConeVector(1) = -gravitational_force / (-mu_tilde(2) + sqrtTerm);
  }

  // Plane intersection
  if (std::abs(mu_tilde(2)) >= plane_epsilon) {
    mems_.tPlane = (1.0 - params_.alpha_plane) * gravitational_force / mu_tilde(2);
  }

  // Filter out negative t values
  for (int i = 0; i < tSphereVector.size(); ++i) {
    if (tSphereVector(i) < 0) tSphereVector(i) = NAN;
  }
  for (int i = 0; i < tEllipticConeVector.size(); ++i) {
    if (tEllipticConeVector(i) < 0) tEllipticConeVector(i) = NAN;
  }
  if (mems_.tPlane < 0) mems_.tPlane = NAN;

  // Determine smallest t values for each surface
  mems_.tSphere = tSphereVector.minCoeff();
  mems_.tEllipticCone = tEllipticConeVector.minCoeff();

  // Select most restrictive requirement
  mems_.tPrime = std::min({mems_.tSphere, mems_.tEllipticCone, mems_.tPlane});
  if (mems_.tPrime > 1.0 || std::isnan(mems_.tPrime)) { // if tPrime is NAN it means mu_tilde = [0;0;0], we are just hovering
    mems_.tPrime = 1.0;
    mems_.safe_mech_activated = false;
  }
  else 
  {
    mems_.safe_mech_activated = true;
  }

  // Compute the final transformed vectors
  Eigen::Vector3d mu_tilde_prime = mu_tilde * mems_.tPrime;
  Eigen::Vector3d mu_virtual_control_prime = mu_tilde_prime - gravitational_force_vector;

  return mu_virtual_control_prime;
}

} // namespace outer_loop_safety_mechanism