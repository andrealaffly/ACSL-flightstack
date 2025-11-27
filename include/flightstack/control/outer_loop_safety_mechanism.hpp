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
 * File:        outer_loop_safety_mechanism.hpp
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

#pragma once 

#include <iostream>
#include <cmath>
#include <Eigen/Dense>
#include "json_parser.hpp"

namespace outer_loop_safety_mechanism
{

constexpr double sphere_epsilon = 1e-2;
constexpr double elliptic_cone_epsilon = 1e-2;
constexpr double plane_epsilon = 1e-2;

struct SafetyMechanismTuningParams
{
  double maximum_total_thrust; // [N]
  double maximum_roll_angle; // [rad]
  double maximum_pitch_angle; // [rad]
  double alpha_plane; // [-] coefficient for setting the 'height' of the bottom plane. Must be >0 and <1.
};

struct SafetyMechanismMembers
{
  double tSphere;
  double tEllipticCone;
  double tPlane;
  double tPrime;
  bool safe_mech_activated;

  // Constructor to initialize member variables with default values
  SafetyMechanismMembers()
    : tSphere(NAN),
      tEllipticCone(NAN),
      tPlane(NAN),
      tPrime(1.0),
      safe_mech_activated(false)
  {}
};

class OuterLoopSafetyMechanism
{
public:
  const SafetyMechanismTuningParams& getSafetyMechanismTuningParams() const;
  const SafetyMechanismMembers& getSafetyMechanismMembers() const;

  void readJSONfile();

  Eigen::Vector3d applySafetyMechansim(const Eigen::Vector3d& mu_virtual_control,
                                       const Eigen::Vector3d& gravitational_force_vector);

private:
  SafetyMechanismTuningParams params_;
  SafetyMechanismMembers mems_;

};

} // namespace outer_loop_safety_mechanism