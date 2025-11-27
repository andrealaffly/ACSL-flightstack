/***********************************************************************************************************************
 * Copyright (c) 2024 Mattia Gramuglia, Giri M. Kumar, Andrea L'Afflitto. All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without modification, are permitted provided that the
 * following conditions are met:
 * 
 * 1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following
 *    disclaimer.
 * 
 * 2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the
 *    following disclaimer in the documentation and/or other materials provided with the distribution.
 * 
 * 3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote
 *    products derived from this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS “AS IS” AND ANY EXPRESS OR IMPLIED WARRANTIES,
 * INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 **********************************************************************************************************************/

/***********************************************************************************************************************
 * File:        low_pass_filter_test.cpp
 * Author:      Mattia Gramuglia
 * Date:        September 25, 2025
 * For info:    Andrea L'Afflitto 
 *              a.lafflitto@vt.edu
 * 
 * Description: Test of the implementation of first and second order recursive low pass filters
 *              found at "include/flightstack/utils/low_pass_filter.hpp"
 *              
 * GitHub:    https://github.com/andrealaffly/ACSL-flightstack.git
 **********************************************************************************************************************/

#include <gtest/gtest.h>
#include <Eigen/Dense>
#include "flightstack/utils/low_pass_filter.hpp"

using namespace LowPassFilter;

// Helper template to run the same tests for any filter type
template <typename FilterType>
void testFilterVector3d() {
  FilterVector3d<FilterType> filter;

  // Configure filter for all channels
  std::array<typename FilterVector3d<FilterType>::Config, 3> configs = {{
      {10.0, 100.0, 0.7071},
      {10.0, 100.0, 0.7071},
      {10.0, 100.0, 0.7071}
  }};
  filter.configure(configs);

  // Initial input
  Eigen::Vector3d first_input(10.0, 20.0, 30.0);
  Eigen::Vector3d first_output = filter.process(first_input);

  // Step input
  Eigen::Vector3d second_input(20.0, 30.0, 100.0);
  Eigen::Vector3d second_output = filter.process(second_input);

  std::cout << "first_input: " << first_input.transpose() << "\n";
  std::cout << "first_output: " << first_output.transpose() << "\n";
  std::cout << "second_input: " << second_input.transpose() << "\n";
  std::cout << "second_output: " << second_output.transpose() << "\n";

  // The first output should equal the initial input
  EXPECT_DOUBLE_EQ(first_output.x(), first_input.x());
  EXPECT_DOUBLE_EQ(first_output.y(), first_input.y());
  EXPECT_DOUBLE_EQ(first_output.z(), first_input.z());

  // The output should move toward the new input
  EXPECT_GT(second_output.x(), first_output.x());
  EXPECT_LT(second_output.x(), second_input.x());
  EXPECT_GT(second_output.y(), first_output.y());
  EXPECT_LT(second_output.y(), second_input.y());
  EXPECT_GT(second_output.z(), first_output.z());
  EXPECT_LT(second_output.z(), second_input.z());
}

// Now define separate Google Test instances for each filter type
TEST(FilterVector3dTest, FirstOrder) {
  testFilterVector3d<FirstOrder>();
}

TEST(FilterVector3dTest, SecondOrder) {
  testFilterVector3d<SecondOrder>();
}

