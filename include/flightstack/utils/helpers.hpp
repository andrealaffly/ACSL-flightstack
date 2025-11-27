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
 * File:        helpers.hpp
 * Author:      Mattia Gramuglia
 * Date:        June 17, 2025
 * For info:    Andrea L'Afflitto 
 *              a.lafflitto@vt.edu
 * 
 * Description: Helper functions.
 * 
 * GitHub:    https://github.com/andrealaffly/ACSL-flightstack.git
 **********************************************************************************************************************/

#pragma once

#include <stdexcept>

#include <Eigen/Dense>

namespace acsl_helpers {

// Helper function to compute the maximum eigenvalue of a symmetric matrix
template <typename MatrixType>
double computeMaxEigenvalueSymmetricMatrix(const MatrixType& symmetric_matrix) {
  Eigen::SelfAdjointEigenSolver<MatrixType> solver(symmetric_matrix);
  if (solver.info() != Eigen::Success) {
    throw std::runtime_error("Eigenvalue computation failed!");
  }
  return solver.eigenvalues().maxCoeff();
}

// Helper function to compute the minimum eigenvalue of a symmetric matrix
template <typename MatrixType>
double computeMinEigenvalueSymmetricMatrix(const MatrixType& symmetric_matrix) {
  Eigen::SelfAdjointEigenSolver<MatrixType> solver(symmetric_matrix);
  if (solver.info() != Eigen::Success) {
    throw std::runtime_error("Eigenvalue computation failed!");
  }
  return solver.eigenvalues().minCoeff();
}

} // namespace acsl_helpers