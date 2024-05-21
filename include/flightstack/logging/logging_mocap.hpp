/* logging_mocap.hpp

	Mattia Gramuglia
	May 10, 2024
*/

#ifndef LOGGING_MOCAP_HPP
#define LOGGING_MOCAP_HPP


#include <atomic>
#include <cstddef>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <ostream>
#include <sstream>
#include <string>

#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/date_time/posix_time/posix_time_types.hpp>
#include <boost/log/attributes.hpp>
#include <boost/log/attributes/scoped_attribute.hpp>
#include <boost/log/core.hpp>
#include <boost/log/expressions.hpp>
#include <boost/log/sinks/sync_frontend.hpp>
#include <boost/log/sinks/text_ostream_backend.hpp>
#include <boost/log/sources/basic_logger.hpp>
#include <boost/log/sources/logger.hpp>
#include <boost/log/sources/record_ostream.hpp>
#include <boost/log/sources/severity_channel_logger.hpp>
#include <boost/log/sources/severity_logger.hpp>
#include <boost/log/support/date_time.hpp>
#include <boost/log/trivial.hpp>
#include <boost/log/utility/setup/common_attributes.hpp>
#include <boost/log/utility/setup/file.hpp>
#include <boost/log/utility/value_ref.hpp>
#include <boost/phoenix/bind.hpp>
#include <boost/smart_ptr/make_shared_object.hpp>
#include <boost/smart_ptr/shared_ptr.hpp>
#include <Eigen/Dense>






namespace logging = boost::log;
namespace sinks = boost::log::sinks;
namespace src = boost::log::sources;
namespace expr = boost::log::expressions;
namespace attrs = boost::log::attributes;
namespace keywords = boost::log::keywords;

// Define a logger for MocapData
extern src::logger logger_mocapdata;

namespace _drivers_
{
namespace _udp_driver_
{

// Forward declaration of UdpReceiverNode class
class UdpReceiverNode;


class MocapData 
{
public:

	// Constructor
  MocapData(UdpReceiverNode& node);

  // References to variables that will be logged
  uint64_t& timestamp_mocap;
  double& time_mocap;
  double& x; 
  double& y;
  double& z;
  double& q0;
  double& q1;
  double& q2;
  double& q3;
  double& vx;
  double& vy;
  double& vz;
  double& rollspeed;
  double& pitchspeed;
  double& yawspeed;
  

private:

	UdpReceiverNode& node_;

};

void logInitializeHeaders();
void logInitializeLogging();
void logMocapData(const MocapData& data);

} // namespace _udp_driver_    
} // namespace _drivers_


#endif // LOGGING_MOCAP_HPP