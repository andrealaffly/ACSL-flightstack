/* vehicle_state.hpp

	Mattia Gramuglia
	April 9, 2024
*/

#ifndef VEHICLE_STATE_HPP
#define VEHICLE_STATE_HPP

#include <string>
#include <cmath>
#include <atomic>

#include <Eigen/Dense>

// Forward declaration of MultiThreadedNode class
class MultiThreadedNode;

class VehicleState 
{
public:

  // Constructor
  VehicleState(MultiThreadedNode& node);

  // Getter functions
  uint64_t getTimestamp() const;
  Eigen::Vector3d& getPosition();
  Eigen::Quaterniond& getQuaternion();
  Eigen::Vector3d& getVelocity();
  Eigen::Vector3d& getAngularVelocity();
  Eigen::Vector3d& getEulerAnglesRPY();
  const std::atomic<uint64_t>& getInitialTimestamp() const;
  std::atomic<double>& getTimeOdometryInSeconds();
  bool getIsOffsetOdometryMsg() const;
  uint64_t getTimestampOffset() const;
  Eigen::Vector3d& getPositionOffset();
  Eigen::Quaterniond& getQuaternionOffset();
  Eigen::Vector3d& getEulerAnglesRPYOffset();
  Eigen::Quaterniond& getYawQuaternionInverseOffset();

  // Setter functions
  void setTimestamp(uint64_t timestamp);
  void setPosition(const Eigen::Vector3d& position);
  void setQuaternion(const Eigen::Quaterniond& q);
  void setVelocity(const Eigen::Vector3d& velocity);
  void setAngularVelocity(const Eigen::Vector3d& angular_velocity);
  void setEulerAnglesRPY(const Eigen::Quaterniond& q);
  void setIsOffsetOdometryMsg(bool value);
  void setTimestampOffset(uint64_t timestamp_offset);
  void setPositionOffset(const Eigen::Vector3d& position_offset);
  void setQuaternionOffset(const Eigen::Quaterniond& q_offset);
  void setEulerAnglesRPYOffset(const Eigen::Quaterniond& q_offset);
  void setYawQuaternionInverseOffset();

  // Transform quaternion to euler angles
  Eigen::Vector3d quaternionToEulerAnglesRPY(const Eigen::Quaterniond& q);

private:

  MultiThreadedNode& node_;
  const std::atomic<uint64_t>& timestamp_initial_;  // Reference to the initial timestamp

  uint64_t timestamp_;                 // [us]
  Eigen::Vector3d position_;           // [m] By default expressed in NED earth-fixed frame
  Eigen::Quaterniond q_;               // [-] By default expressed in NED earth-fixed frame
  Eigen::Vector3d velocity_;           // [m/s] By default expressed in NED earth-fixed frame 
  Eigen::Vector3d angular_velocity_;   // [rad/s] By default expressed in FRD body-fixed frame

  Eigen::Vector3d euler_angles_rpy_;   // [rad]

  std::atomic<double> time_odometry_;  // [s]

  // Offset variables used to position Pixhawk in (0, 0, 0) with 0 yaw angle 
  // after the "offset" odometry message is received
  bool is_offset_odometry_msg_;
  uint64_t timestamp_offset_;
  Eigen::Vector3d position_offset_;
  Eigen::Quaterniond q_offset_;
  Eigen::Vector3d euler_angles_rpy_offset_;
  Eigen::Quaterniond yaw_quaternion_inverse_offset_;

};


#endif // VEHICLE_STATE_HPP
