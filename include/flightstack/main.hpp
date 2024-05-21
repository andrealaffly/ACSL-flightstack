/* main.hpp

Mattia Gramuglia
May 3, 2024
*/

#ifndef MAIN_HPP
#define MAIN_HPP

#include "multi_threaded_node.hpp"

/***********************************************************************************************************************
  DATA FUSION using GPS or MOCAP
***********************************************************************************************************************/
// Enum class for the possible modes to fuse Pixhawk IMU data
enum class EKF2FusionMode {
    GPS,
    MOCAP // VICON
};

/*
  Perform data fusion using GPS
*/
// inline constexpr EKF2FusionMode CURRENT_EKF2_FUSION_MODE = EKF2FusionMode::GPS;

/*
  Perform data fusion using MOCAP
*/
inline constexpr EKF2FusionMode CURRENT_EKF2_FUSION_MODE = EKF2FusionMode::MOCAP;

/***********************************************************************************************************************
  
***********************************************************************************************************************/



#endif // MAIN_HPP