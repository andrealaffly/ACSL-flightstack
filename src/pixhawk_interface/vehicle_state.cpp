/* vehicle_state.cpp

	Mattia Gramuglia
	April 9, 2024
*/

#include "vehicle_state.hpp"
#include "multi_threaded_node.hpp"

// Constructor
VehicleState::VehicleState(MultiThreadedNode& node) :
  node_(node),
  timestamp_initial_(node.getInitialTimestamp()), // Initialize timestamp_initial_ with the value from MultiThreadedNode
  timestamp_(0),
  position_(Eigen::Vector3d::Zero()),
  q_(Eigen::Quaterniond::Identity()),
  velocity_(Eigen::Vector3d::Zero()),
  angular_velocity_(Eigen::Vector3d::Zero()),
  euler_angles_rpy_(Eigen::Vector3d::Zero()),
  is_offset_odometry_msg_(true),
  timestamp_offset_(0),
  position_offset_(Eigen::Vector3d::Zero()),
  q_offset_(Eigen::Quaterniond::Identity()),
  euler_angles_rpy_offset_(Eigen::Vector3d::Zero()),
  yaw_quaternion_inverse_offset_(Eigen::Quaterniond::Identity())
{}

// Getter function for the timestamp
uint64_t VehicleState::getTimestamp() const {
  return timestamp_;
}

// Getter function for the position
Eigen::Vector3d& VehicleState::getPosition() {
  return position_;
}

// Getter function for the quaternion
Eigen::Quaterniond& VehicleState::getQuaternion() {
  return q_;
}

// Getter function for the velocity
Eigen::Vector3d& VehicleState::getVelocity() {
  return velocity_;
}

// Getter function for the angular velocity
Eigen::Vector3d& VehicleState::getAngularVelocity() {
  return angular_velocity_;
}

// Getter function for the euler angles
Eigen::Vector3d& VehicleState::getEulerAnglesRPY() {
  return euler_angles_rpy_;
}

// Getter function for the initial timestamp generated when the MultiThreadedNode is started
const std::atomic<uint64_t>& VehicleState::getInitialTimestamp() const {
  return timestamp_initial_;
}

// Getter function for the time in seconds that has passed from the initial timestamp to the latest received message
std::atomic<double>& VehicleState::getTimeOdometryInSeconds()
{
  time_odometry_ = (timestamp_ - timestamp_initial_) / 1e6;
  return time_odometry_;
}

// Getter function for the is_offset_odometry_msg_
bool VehicleState::getIsOffsetOdometryMsg() const {
  return is_offset_odometry_msg_;
}

// Getter function for the timestamp_offset_
uint64_t VehicleState::getTimestampOffset() const {
  return timestamp_offset_;
}

// Getter function for the position_offset_
Eigen::Vector3d& VehicleState::getPositionOffset() {
  return position_offset_;
}

// Getter function for the q_offset_
Eigen::Quaterniond& VehicleState::getQuaternionOffset() {
  return q_offset_;
}

// Getter function for the euler_angles_rpy_offset_
Eigen::Vector3d& VehicleState::getEulerAnglesRPYOffset() {
  return euler_angles_rpy_offset_;
}

// Getter function for the yaw_quaternion_inverse_offset_
Eigen::Quaterniond& VehicleState::getYawQuaternionInverseOffset() { 
  return yaw_quaternion_inverse_offset_;
}


// Setter function for the timestamp
void VehicleState::setTimestamp(uint64_t timestamp) {
  timestamp_ = timestamp;
}

// Setter function for the position
void VehicleState::setPosition(const Eigen::Vector3d& position) {
  position_ = position;
}

// Setter function for the quaternion
void VehicleState::setQuaternion(const Eigen::Quaterniond& q) {
  q_ = q;
}

// Setter function for the velocity
void VehicleState::setVelocity(const Eigen::Vector3d& velocity) {
  velocity_ = velocity;
}

// Setter function for the angular velocity
void VehicleState::setAngularVelocity(const Eigen::Vector3d& angular_velocity) {
  angular_velocity_ = angular_velocity;
}

// Setter function for the euler angles
void VehicleState::setEulerAnglesRPY(const Eigen::Quaterniond& q) {
  Eigen::Vector3d euler_angles_rpy = quaternionToEulerAnglesRPY(q);
  euler_angles_rpy_ = euler_angles_rpy;
}

// Setter function for the is_offset_odometry_msg_
void VehicleState::setIsOffsetOdometryMsg(bool value) {
  is_offset_odometry_msg_ = value;
}

// Setter function for the timestamp_offset_
void VehicleState::setTimestampOffset(uint64_t timestamp_offset) {
  timestamp_offset_ = timestamp_offset;
}

// Setter function for the position_offset_
void VehicleState::setPositionOffset(const Eigen::Vector3d& position_offset) {
  position_offset_ = position_offset;
}

// Setter function for the quaternion
void VehicleState::setQuaternionOffset(const Eigen::Quaterniond& q_offset) {
  q_offset_ = q_offset;
}

// Setter function for the euler_angles_rpy_offset_
void VehicleState::setEulerAnglesRPYOffset(const Eigen::Quaterniond& q_offset) {
  Eigen::Vector3d euler_angles_rpy_offset = quaternionToEulerAnglesRPY(q_offset);
  euler_angles_rpy_offset_ = euler_angles_rpy_offset;
}

/*
  Setter function for the yaw_quaternion_inverse_offset_
*/
void VehicleState::setYawQuaternionInverseOffset()
{
  Eigen::Quaterniond yaw_quaternion = Eigen::Quaterniond(cos(euler_angles_rpy_offset_(2)/2),
                                                         0,
                                                         0,
                                                         sin(euler_angles_rpy_offset_(2)/2));

  yaw_quaternion_inverse_offset_ = yaw_quaternion.inverse();

}

/*
  Computing Euler Angles from the unit quaternion following a 3-2-1 rotation sequence
  The Euler Angles are ordered as: roll, pitch, yaw
  For reference:
    - https://github.com/PX4/PX4-Matrix/blob/master/matrix/Euler.hpp
    - https://github.com/PX4/PX4-Matrix/blob/master/matrix/Dcm.hpp
*/
Eigen::Vector3d VehicleState::quaternionToEulerAnglesRPY(const Eigen::Quaterniond& q) {
  Eigen::Vector3d euler_angles_rpy;

  // Compute elements of Direction Cosine Matrix (DCM) from quaternion
  double a = q.w();
  double b = q.x();
  double c = q.y();
  double d = q.z();
  double aa = a * a;
  double ab = a * b;
  double ac = a * c;
  double ad = a * d;
  double bb = b * b;
  double bc = b * c;
  double bd = b * d;
  double cc = c * c;
  double cd = c * d;
  double dd = d * d;

  // Compute DCM elements
  double dcm00 = aa + bb - cc - dd;
  // double dcm01 = 2 * (bc - ad); // not needed by the algorithm
  double dcm02 = 2 * (ac + bd);
  double dcm10 = 2 * (bc + ad);
  // double dcm11 = aa - bb + cc - dd; // not needed by the algorithm
  double dcm12 = 2 * (cd - ab);
  double dcm20 = 2 * (bd - ac);
  double dcm21 = 2 * (ab + cd);
  double dcm22 = aa - bb - cc + dd;

  // Compute Euler angles from DCM
  euler_angles_rpy.y() = asin(-dcm20);

  if (fabs(euler_angles_rpy.y() - M_PI / 2) < 1.0e-3) {
    euler_angles_rpy.x() = 0;
    euler_angles_rpy.z() = atan2(dcm12, dcm02);
  } else if (fabs(euler_angles_rpy.y() + M_PI / 2) < 1.0e-3) {
    euler_angles_rpy.x() = 0;
    euler_angles_rpy.z() = atan2(-dcm12, -dcm02);
  } else {
    euler_angles_rpy.x() = atan2(dcm21, dcm22);
    euler_angles_rpy.z() = atan2(dcm10, dcm00);
  }

  return euler_angles_rpy;
}
