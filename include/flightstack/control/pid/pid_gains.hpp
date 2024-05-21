/* pid_gains.hpp

	Mattia Gramuglia
	April 23, 2024
*/

#ifndef PID_GAINS_HPP
#define PID_GAINS_HPP

#include <cmath>
#include <algorithm>

#include <Eigen/Dense>


// Struct containing the PID tuning gains coming from the .json file 
struct GainsPID 
{
  Eigen::Matrix3d KP_translational;
  Eigen::Matrix3d KD_translational;
  Eigen::Matrix3d KI_translational;

  Eigen::Matrix3d KP_rotational;
  Eigen::Matrix3d KD_rotational;
  Eigen::Matrix3d KI_rotational;

};


#endif // PID_GAINS_HPP