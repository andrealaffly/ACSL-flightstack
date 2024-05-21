/* user_defined_trajectory.cpp

	Mattia Gramuglia
	April 12, 2024
*/

#include "user_defined_trajectory.hpp"
#include "multi_threaded_node.hpp"

// Constructor
UserDefinedTrajectory::UserDefinedTrajectory(MultiThreadedNode& node) :
	node_(node),
	timestamp_initial_(node.getInitialTimestamp()), // Initialize timestamp_initial_ with the value from MultiThreadedNode
	user_defined_position_(Eigen::Vector3d::Zero()),
	user_defined_velocity_(Eigen::Vector3d::Zero()),
	user_defined_acceleration_(Eigen::Vector3d::Zero()),
	user_defined_yaw_(0),
	user_defined_yaw_dot_(0),
	user_defined_yaw_dot_dot_(0)
{}

// Getter function that returns a reference to the current MultiThreadedNode object
MultiThreadedNode& UserDefinedTrajectory::getNode() const {
  return node_;
}

// Getter function for the initial timestamp generated when the MultiThreadedNode is started
const std::atomic<uint64_t>& UserDefinedTrajectory::getInitialTimestamp() const {
  return timestamp_initial_;
}

// Getter function for the user_defined_position_
Eigen::Vector3d& UserDefinedTrajectory::getUserDefinedPosition() {
  return user_defined_position_;
}

// Getter function for the user_defined_velocity_
Eigen::Vector3d& UserDefinedTrajectory::getUserDefinedVelocity() {
  return user_defined_velocity_;
}

// Getter function for the user_defined_acceleration_
Eigen::Vector3d& UserDefinedTrajectory::getUserDefinedAcceleration() {
  return user_defined_acceleration_;
}

// Getter function for the user_defined_yaw_
double& UserDefinedTrajectory::getUserDefinedYaw() {
  return user_defined_yaw_;
}

// Getter function for the user_defined_yaw_dot_
double& UserDefinedTrajectory::getUserDefinedYawDot() {
  return user_defined_yaw_dot_;
}

// Getter function for the user_defined_yaw_dot_dot_
double& UserDefinedTrajectory::getUserDefinedYawDotDot() {
  return user_defined_yaw_dot_dot_;
}

// Setter function for the user_defined_position_
void UserDefinedTrajectory::setUserDefinedPosition(const Eigen::Vector3d& user_defined_position) {
  user_defined_position_ = user_defined_position;
}

// Setter function for the user_defined_velocity_
void UserDefinedTrajectory::setUserDefinedVelocity(const Eigen::Vector3d& user_defined_velocity) {
  user_defined_velocity_ = user_defined_velocity;
}

// Setter function for the user_defined_acceleration_
void UserDefinedTrajectory::setUserDefinedAcceleration(const Eigen::Vector3d& user_defined_acceleration) {
  user_defined_acceleration_ = user_defined_acceleration;
}

// Setter function for the user_defined_yaw_
void UserDefinedTrajectory::setUserDefinedYaw(const double& user_defined_yaw) {
  user_defined_yaw_ = user_defined_yaw;
}

// Setter function for the user_defined_yaw_dot_
void UserDefinedTrajectory::setUserDefinedYawDot(const double& user_defined_yaw_dot) {
  user_defined_yaw_dot_ = user_defined_yaw_dot;
}

// Setter function for the user_defined_yaw_dot_dot_
void UserDefinedTrajectory::setUserDefinedYawDotDot(const double& user_defined_yaw_dot_dot) {
  user_defined_yaw_dot_dot_ = user_defined_yaw_dot_dot;
}