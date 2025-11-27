########################################################################################################################
# Copyright (c) 2024 Mattia Gramuglia, Giri M. Kumar, Andrea L'Afflitto. All rights reserved.
#
# Redistribution and use in source and binary forms, with or without modification, are permitted provided that the
# following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following
#    disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the
#    following disclaimer in the documentation and/or other materials provided with the distribution.
#
# 3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote
#    products derived from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS “AS IS” AND ANY EXPRESS OR IMPLIED WARRANTIES,
# INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
# SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
# WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
######################################################################################################################## 

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

include_directories(${Boost_INCLUDE_DIRS})
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include/eigen)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include/flightstack)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include/flightstack/config)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include/flightstack/control)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include/flightstack/control/funnel_two_layer_mrac)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include/flightstack/control/mrac)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include/flightstack/control/pid)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include/flightstack/control/two_layer_mrac)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include/flightstack/logging)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include/flightstack/mocap)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include/flightstack/pixhawk_interface)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include/flightstack/user_defined_trajectory)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include/flightstack/utils)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include/json/include)


set(SOURCES
  src/config/config.cpp
  src/control/control.cpp
  src/control/control_callback.cpp
  src/control/outer_loop_safety_mechanism.cpp
  src/control/funnel_two_layer_mrac/funnel_two_layer_mrac.cpp
  src/control/funnel_two_layer_mrac/logging_funnel_two_layer_mrac.cpp
  src/control/mrac/logging_mrac.cpp
  src/control/mrac/mrac.cpp
  src/control/pid/logging_pid.cpp
  src/control/pid/pid.cpp
  src/control/two_layer_mrac/two_layer_mrac.cpp
  src/control/two_layer_mrac/logging_two_layer_mrac.cpp
  src/logging/logging_mocap.cpp
  src/main.cpp
  src/mocap/io_context.cpp
  src/mocap/mocap.cpp
  src/mocap/udp_driver.cpp
  src/mocap/udp_socket.cpp
  src/multi_threaded_node.cpp
  src/pixhawk_interface/pixhawk_actuator_motors.cpp
  src/pixhawk_interface/pixhawk_vehicle_odometry.cpp
  src/pixhawk_interface/vehicle_state.cpp
  src/user_defined_trajectory/piecewise_polynomial_trajectory.cpp
  src/user_defined_trajectory/user_defined_trajectory.cpp
  src/utils/continuous_lyapunov_equation.cpp
  src/utils/json_parser.cpp
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

  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_low_pass_filter
    test/utils/low_pass_filter_test.cpp
  )

  if(TARGET test_low_pass_filter)
    target_include_directories(test_low_pass_filter PUBLIC
      ${CMAKE_CURRENT_SOURCE_DIR}/include
    )
    # Optional: link extra libraries if needed
    # target_link_libraries(test_low_pass_filter some_library)

    # Correct way to label the test for CTest
    set_tests_properties(test_low_pass_filter PROPERTIES LABELS "flightstack_core")
  endif()
endif()

ament_package()
