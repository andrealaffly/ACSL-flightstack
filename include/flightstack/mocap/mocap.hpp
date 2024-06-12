/**************************************************************************
 * File:        mocap.hpp
 * Author:      girimugundankumar
 * Email:       girimugundan@vt.edu
 * Date:        April 20, 2024
 * 
 * Description: Class declaration for UDP socket as a lifecycle node.
 * 
 * GitHub:      
 * Referenced:  https://github.com/ros-drivers/transport_drivers/tree/main
 **************************************************************************/

#ifndef MOCAP_HPP_
#define MOCAP_HPP_

#include <chrono>
#include <memory>
#include <string>
#include <vector>
#include <sstream>
#include <bit>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include "lifecycle_msgs/msg/transition.hpp"
#include <lifecycle_msgs/msg/state.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>

#include "udp_driver.hpp"
#include "logging_mocap.hpp"

using lifecycle_msgs::msg::State;
using lifecycle_msgs::msg::Transition;

// -> IP of the Odroid M1s
inline constexpr const char* GROUND_STATION_IP = "127.0.0.1"; // For testing 
inline constexpr const char* ODROID_M1S_IP = "192.168.12.1";
// -> PORT of the Odroid M1s
inline constexpr uint16_t ODROID_M1S_PORT = 52000;


/// ------------- USER DEFINED ------------- ///
// -> Pixhawk vehicle odometry pose frame message package definitions
inline constexpr uint8_t POSE_FRAME_UNKNOWN = 0; // Pose frame unkown
inline constexpr uint8_t POSE_FRAME_NED     = 1; // NED earth-fixed frame
inline constexpr uint8_t POSE_FRAME_FRD     = 2; // FRD world-fixed frame, arbitrary heading reference
// -> Pixhawk vehicle odometry velocity framemessage pacakge definitions
inline constexpr uint8_t VELOCITY_FRAME_UNKNOWN = 0;  // Velocity frame unkown
inline constexpr uint8_t VELOCITY_FRAME_NED      = 1; // NED earth-fixed frame
inline constexpr uint8_t VELOCITY_FRAME_FRD      = 2; // FRD world-fixed frame, arbitrary heading reference
inline constexpr uint8_t VELOCITY_FRAME_BODY_FRD = 3; // FRD body-fixed frame

namespace lc = rclcpp_lifecycle;
using LNI = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface;

namespace _drivers_
{
namespace _udp_driver_
{

/// Define struct variables to store parsed data
  struct mocap_states
  {
    double x; 
    double y;
    double z;
    double q0;
    double q1;
    double q2;
    double q3;
    double vx;
    double vy;
    double vz;
    double rollspeed;
    double pitchspeed;
    double yawspeed;
  };

/// \brief UdpReceiverNode class which can receive UDP datagrams
class UdpReceiverNode final
  : public lc::LifecycleNode
{
public:
  /// \brief Constructor which accepts IoContext
  /// \param[in] ctx A shared IoContext
  /// \param[in] pointer to vehicle states in flight_bridge
  UdpReceiverNode(const IoContext & ctx, const std::atomic<uint64_t>& timestamp_initial);

  /// \brief Destructor - required to manage owned IoContext
  ~UdpReceiverNode();

  void checkLifecycleNodeStarted();

  /// \brief Callback from transition to "configuring" state.
  /// \param[in] state The current state that the node is in.
  LNI::CallbackReturn on_configure(const lc::State & state) override;

  /// \brief Callback from transition to "activating" state.
  /// \param[in] state The current state that the node is in.
  LNI::CallbackReturn on_activate(const lc::State & state) override;

  /// \brief Callback from transition to "deactivating" state.
  /// \param[in] state The current state that the node is in.
  LNI::CallbackReturn on_deactivate(const lc::State & state) override;

  /// \brief Callback from transition to "unconfigured" state.
  /// \param[in] state The current state that the node is in.
  LNI::CallbackReturn on_cleanup(const lc::State & state) override;

  /// \brief Callback from transition to "shutdown" state.
  /// \param[in] state The current state that the node is in.
  LNI::CallbackReturn on_shutdown(const lc::State & state) override;

  /// \brief Callback for receiving a UDP datagram
  void receiver_callback(const std::vector<uint8_t> & buffer);

  // Getter function for the mocap_states
  mocap_states& getMocapStates();

  // Getter function for the timestamp_mocap_
  uint64_t& getTimestampMocap();

  // Getter function for the time_mocap_
  double& getTimeMocap();



private:

  /// \brief Get the parameters for the ip and port to ping
  void get_params();

  /// Pointer to the asio context owned by this node for async communication
  std::unique_ptr<IoContext> m_owned_ctx{};
  
  /// String for the ip of the odrioid
  std::string m_ip{};
      
  /// String for the port of the odroid
  uint16_t m_port{};
  
  /// Pointer for the udp driver which wraps the udp socket
  std::unique_ptr<UdpDriver> m_udp_driver;
  
  /// Publisher for publishing the mocap message
  lc::LifecyclePublisher<px4_msgs::msg::VehicleOdometry>::SharedPtr mocap_publisher_;

  // Reference to the initial timestamp determined in multi_threaded_node
  const std::atomic<uint64_t>& timestamp_initial_;

  // Timestamp at which the mocap data is received
  uint64_t timestamp_mocap_;

  // Time in seconds at which the mocap data is received wrt the timestamp_initial_
  double time_mocap_;

  /// Instantiate mocap_states struct
  struct mocap_states mc;

  /// \brief Debugger function to output the mocap data.
  void debugMocapData2screen();

  // Create a pointer to the MocapData instance
  std::shared_ptr<MocapData> mocap_data_;

};  

// DUPLICATE - already present in multi_threaded_node
std::string string_thread_id();

}   // namespace _udp_driver_
}   // namesapce _drivers_

#endif  // MOCAP_HPP_