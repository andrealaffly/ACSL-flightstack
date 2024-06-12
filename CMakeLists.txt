cmake_minimum_required(VERSION 3.5)
project(flightstack)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Set the path to the Boost root directory
set(BOOST_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/include/boost")

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(Boost REQUIRED COMPONENTS log log_setup)
find_package(lifecycle_msgs REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

# Include directories
include_directories(${Boost_INCLUDE_DIRS})
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include/json/include)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include/eigen)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include/flightstack)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include/flightstack/utils)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include/flightstack/pixhawk_interface)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include/flightstack/user_defined_trajectory)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include/flightstack/logging)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include/flightstack/control)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include/flightstack/control/pid)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include/flightstack/mocap)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include/flightstack/config)



# Add source files
set(SOURCES
	src/main.cpp
	src/multi_threaded_node.cpp
	src/pixhawk_interface/pixhawk_vehicle_odometry.cpp
	src/pixhawk_interface/pixhawk_actuator_motors.cpp
	src/pixhawk_interface/vehicle_state.cpp
  src/user_defined_trajectory/user_defined_trajectory.cpp
  src/user_defined_trajectory/piecewise_polynomial_trajectory.cpp
  src/logging/logging_mocap.cpp
  src/control/control.cpp
  src/control/control_callback.cpp
  src/control/pid/pid.cpp
  src/control/pid/logging_pid.cpp
	src/utils/json_parser.cpp
  src/mocap/io_context.cpp
  src/mocap/udp_driver.cpp
  src/mocap/udp_socket.cpp
  src/mocap/mocap.cpp
  src/config/config.cpp
)

# Add executable
add_executable(flightstack ${SOURCES})

# Add header files to the target
target_include_directories(flightstack PUBLIC)

# Link against Boost libraries
target_link_libraries(flightstack ${Boost_LIBRARIES})

ament_target_dependencies(flightstack rclcpp px4_msgs lifecycle_msgs rclcpp_lifecycle)

install(TARGETS
  flightstack
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
