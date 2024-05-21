/* logging_mocap.cpp

Mattia Gramuglia
May 10, 2024
*/

#include "logging_mocap.hpp"
#include "mocap.hpp"

// Define the logger for MocapData
src::logger logger_mocapdata;


namespace _drivers_
{
namespace _udp_driver_
{

// Constructor
MocapData::MocapData(UdpReceiverNode& node) :
	timestamp_mocap(node.getTimestampMocap()), // Initialize timestamp_initial_ with the value from MultiThreadedNode
  time_mocap(node.getTimeMocap()),
	x(node.getMocapStates().x),
  y(node.getMocapStates().y),
  z(node.getMocapStates().z),
  q0(node.getMocapStates().q0),
  q1(node.getMocapStates().q1),
  q2(node.getMocapStates().q2),
  q3(node.getMocapStates().q3),
  vx(node.getMocapStates().vx),
  vy(node.getMocapStates().vy),
  vz(node.getMocapStates().vz),
  rollspeed(node.getMocapStates().rollspeed),
  pitchspeed(node.getMocapStates().pitchspeed),
  yawspeed(node.getMocapStates().yawspeed),
	node_(node) // node must be the last in the list to be initialized
{}

// Function to print headers
void logInitializeHeaders()
{
	std::ostringstream oss;

	oss << "Mocap timestamp, "
	    << "Mocap time [s], "
      << "Mocap position x [m], "
      << "Mocap position y [m], "
      << "Mocap position z [m], "
      << "Mocap Quaternion q0 [-], "
      << "Mocap Quaternion q1 [-], "
      << "Mocap Quaternion q2 [-], "
      << "Mocap Quaternion q3 [-], "
      << "Mocap Velocity x [m/s], "
      << "Mocap Velocity y [m/s], "
      << "Mocap Velocity z [m/s], "
      << "Mocap Angular velocity x [rad/s], "
      << "Mocap Angular velocity y [rad/s], "
      << "Mocap Angular velocity z [rad/s], "
  ;
      
	BOOST_LOG(logger_mocapdata) << oss.str();
}

// Function to initialize the logging system
void logInitializeLogging()
{
  // Get the current date and time
  auto now = std::chrono::system_clock::now();
  std::time_t now_c = std::chrono::system_clock::to_time_t(now);

  // Get the current date in the desired format
  std::stringstream date_ss;
  date_ss << std::put_time(std::localtime(&now_c), "%Y%m%d"); // Current date
  std::string current_date = date_ss.str();

  // Create the directory path for the logs
  std::string log_base_directory = "./src/flightstack/log/" + current_date;

  // Check if the directory exists, and create it if it doesn't
  if (!std::filesystem::exists(log_base_directory)) {
    std::filesystem::create_directories(log_base_directory);
  }

  // Check if the "mocap" subdirectory exists within the log_base_directory, create it if needed
  std::string mocap_directory = log_base_directory + "/mocap";
  if (!std::filesystem::exists(mocap_directory)) {
      std::filesystem::create_directories(mocap_directory);
  }

  // Generate the mocap log file name with a timestamp
  std::stringstream mocap_ss;
  mocap_ss << mocap_directory << "/mocap_log_" << std::put_time(std::localtime(&now_c), "%Y%m%d_%H%M%S") << ".log";
  std::string mocap_log_filename = mocap_ss.str();

  logger_mocapdata.add_attribute("Tag", attrs::constant< std::string >("MocapTag"));

  // Define a synchronous sink with a text ostream backend
  typedef sinks::synchronous_sink<sinks::text_ostream_backend> text_sink;
  boost::shared_ptr<text_sink> mocap_sink = boost::make_shared<text_sink>();

  // Add a stream to the backend (in this case, a file stream)
  mocap_sink->locked_backend()->add_stream(boost::make_shared<std::ofstream>(mocap_log_filename));

  // Set the formatter for the sink
  mocap_sink->set_formatter(
    expr::stream
    << "[" << expr::format_date_time<boost::posix_time::ptime>("TimeStamp", "%Y-%m-%d %H:%M:%S.%f") << "] " // Format date and time
    << "[" << expr::attr<boost::log::attributes::current_thread_id::value_type>("ThreadID") << "] " // Current thread ID
    << "[" << expr::attr<std::string>("Tag") << "] " // Tag attribute value
    << "[" << expr::attr<boost::log::attributes::current_process_id::value_type>("ProcessID") << "] " // Current process ID
    << "[" << expr::attr<unsigned int>("LineID") << "] " // Line ID
    << expr::smessage // Log message
  );

  // Add the sink to the logging core
  logging::core::get()->add_sink(mocap_sink);

  // Set a filter for the sink
  mocap_sink->set_filter(expr::has_attr("Tag") && expr::attr<std::string>("Tag") == "MocapTag");

  // // Initialize logging with the dynamically generated mocap log file name
  // logging::register_simple_formatter_factory<logging::trivial::severity_level, char>("Severity");
  // logging::add_file_log(
  //   keywords::file_name = mocap_log_filename, // Dynamic mocap log file name
  //   keywords::format = "[%TimeStamp%] [%ThreadID%] [%Tag%] [%ProcessID%] [%LineID%] %Message%"
  // );


  logging::add_common_attributes(); // Add attributes like timestamp

  logInitializeHeaders();
}

// Function to log the data
void logMocapData(const MocapData& data)
{
	std::ostringstream oss;

	oss << data.timestamp_mocap << ", "
	    << data.time_mocap << ", "
      << data.x << ", "
      << data.y << ", "
      << data.z << ", "
      << data.q0 << ", "
      << data.q1 << ", "
      << data.q2 << ", "
      << data.q3 << ", "
      << data.vx << ", "
      << data.vy << ", "
      << data.vz << ", "
      << data.rollspeed << ", "
      << data.pitchspeed << ", "
      << data.yawspeed << ", "
      ;

	BOOST_LOG(logger_mocapdata) << oss.str();
}

} // namespace _udp_driver_    
} // namespace _drivers_