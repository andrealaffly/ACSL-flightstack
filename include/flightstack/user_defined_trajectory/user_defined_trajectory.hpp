/* user_defined_trajectory.hpp

	Mattia Gramuglia
	April 12, 2024
*/

#ifndef USER_DEFINED_TRAJECTORY_HPP
#define USER_DEFINED_TRAJECTORY_HPP

#include <cmath>

#include <Eigen/Dense>

// Forward declaration of MultiThreadedNode class
class MultiThreadedNode;

class UserDefinedTrajectory
{
public:

  // Constructor
  UserDefinedTrajectory(MultiThreadedNode& node);

  // Getter functions
  MultiThreadedNode& getNode() const;
  const std::atomic<uint64_t>& getInitialTimestamp() const;
  Eigen::Vector3d& getUserDefinedPosition();
  Eigen::Vector3d& getUserDefinedVelocity();
  Eigen::Vector3d& getUserDefinedAcceleration();
  double& getUserDefinedYaw();
  double& getUserDefinedYawDot();
  double& getUserDefinedYawDotDot();

  // Setter functions
  void setUserDefinedPosition(const Eigen::Vector3d& position);
  void setUserDefinedVelocity(const Eigen::Vector3d& user_defined_velocity);
  void setUserDefinedAcceleration(const Eigen::Vector3d& user_defined_acceleration);
  void setUserDefinedYaw(const double& user_defined_yaw);
  void setUserDefinedYawDot(const double& user_defined_yaw_dot);
  void setUserDefinedYawDotDot(const double& user_defined_yaw_dot_dot);

private:

  MultiThreadedNode& node_;

  const std::atomic<uint64_t>& timestamp_initial_;  // Reference to the initial timestamp
  // const std::atomic<double>& time_current_;  // Reference to the current time

  Eigen::Vector3d user_defined_position_;
  Eigen::Vector3d user_defined_velocity_;
  Eigen::Vector3d user_defined_acceleration_;
  double user_defined_yaw_;
  double user_defined_yaw_dot_;
  double user_defined_yaw_dot_dot_;

};


#endif // USER_DEFINED_TRAJECTORY_HPP