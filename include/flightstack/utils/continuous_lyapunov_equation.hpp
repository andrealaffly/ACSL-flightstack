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
 * Part of the code in this file leverages the following material.
 *
 * Referenced:  https://github.com/RobotLocomotion/drake/pull/10885/files
 *              All components of Drake are licensed under the BSD 3-Clause License
 *              shown below. Where noted in the source code, some portions may 
 *              be subject to other permissive, non-viral licenses.
 *
 *              Copyright 2012-2022 Robot Locomotion Group @ CSAIL
 *              All rights reserved.
 *
 *              Redistribution and use in source and binary forms, with or without
 *              modification, are permitted provided that the following conditions are
 *              met:
 *
 *              Redistributions of source code must retain the above copyright notice,
 *              this list of conditions and the following disclaimer.  Redistributions
 *              in binary form must reproduce the above copyright notice, this list of
 *              conditions and the following disclaimer in the documentation and/or
 *              other materials provided with the distribution.  Neither the name of
 *              the Massachusetts Institute of Technology nor the names of its
 *              contributors may be used to endorse or promote products derived from
 *              this software without specific prior written permission.
 *
 *              THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *              "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *              LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *              A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *              HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *              SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *              LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *              DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *              THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *              (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *              OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 **********************************************************************************************************************/

/***********************************************************************************************************************
 * File:        continuous_lyapunov_equation.hpp
 * Author:      Mattia Gramuglia
 * Date:        June 14, 2024
 * For info:    Andrea L'Afflitto 
 *              a.lafflitto@vt.edu
 * 
 * Description: Computes the solution of the continuos Lyapunov equation
 *              WARNING !!!
 *              The following code uses a different convention than MATLAB's.
 *              This code solves the equation: `AᵀX + XA + Q = 0`
 *              MATLAB command "lyap(A, Q)" solves the equation: `AX + XAᵀ + Q = 0`
 *              So this code provides the same result as obtained in MATLAB using 
 *              the command: lyap(A', Q)
 *              !!!!!!!!!!!
 * 
 * GitHub:    https://github.com/andrealaffly/ACSL-flightstack.git
 **********************************************************************************************************************/

/**
 * @file ccontinuous_lyapunov_equation.hpp
 * @brief Computes the solution of the continuos Lyapunov equation
 */
#ifndef CONTINUOUS_LYAPUNOV_EQUATION_HPP
#define CONTINUOUS_LYAPUNOV_EQUATION_HPP

#include <Eigen/Dense>
#include <Eigen/QR>

/**
 * WARNING !!!
 * The following code uses a different convention than MATLAB's.
 * This code solves the equation: `AᵀX + XA + Q = 0`
   MATLAB command "lyap(A, Q)" solves the equation: `AX + XAᵀ + Q = 0`
   So this code provides the same result as obtained in MATLAB using 
   the command: lyap(A', Q)
   !!!!!!!!!!!
 */
// Alias for a 1-dimensional vector
using Vector1d = Eigen::Matrix<double, 1, 1>;

/**
 * @param A A user defined real square matrix.
 * @param Q A user defined real symmetric matrix.
 *
 * @pre Q is a symmetric matrix.
 *
 * Computes a unique solution X to the continuous Lyapunov equation: `AᵀX + XA +
 * Q = 0`, where A is real and square, and Q is real, symmetric and of equal
 * size as A.
 * @throws std::runtime_error if A or Q are not square matrices or do not
 * have the same size.
 *
 * Limitations: Given the Eigenvalues of A as λ₁, ..., λₙ, there exists
 * a unique solution if and only if λᵢ + λ̅ⱼ ≠ 0 ∀ i,j, where λ̅ⱼ is
 * the complex conjugate of λⱼ.
 * @throws std::runtime_error if the solution is not unique.
 *
 * There are no further limitations on the eigenvalues of A.
 * Further, if all λᵢ have negative real parts, and if Q is positive
 * semi-definite, then X is also positive semi-definite [1]. Therefore, if one
 * searches for a Lyapunov function V(z) = zᵀXz for the stable linear system ż =
 * Az, then the solution of the Lyapunov Equation `AᵀX + XA + Q = 0` only
 * returns a valid Lyapunov function if Q is positive semi-definite.
 *
 * The implementation is based on SLICOT routine SB03MD [2]. Note the
 * transformation Q = -C. The complexity of this routine is O(n³).
 * If A is larger than 2-by-2, then a Schur factorization is performed.
 * @throw std::runtime_error if Schur factorization failed.
 *
 * A tolerance of ε is used to check if a double variable is equal to zero,
 * where the default value for ε is 1e-10. It has been used to check (1) if λᵢ +
 * λ̅ⱼ = 0, ∀ i,j; (2) if A is a 1-by-1 zero matrix; (3) if A's trace or
 * determinant is 0 when A is a 2-by-2 matrix.
 *
 * [1] Bartels, R.H. and G.W. Stewart, "Solution of the Matrix Equation AX + XB
 * = C," Comm. of the ACM, Vol. 15, No. 9, 1972.
 *
 * [2] http://slicot.org/objects/software/shared/doc/SB03MD.html
 *
 */

Eigen::MatrixXd RealContinuousLyapunovEquation(
  const Eigen::Ref<const Eigen::MatrixXd>& A,
  const Eigen::Ref<const Eigen::MatrixXd>& Q);

namespace internal {

// Subroutines which help special cases. These cases are also called within
// SolveReducedRealContinuousLyapunovFunction.

Vector1d Solve1By1RealContinuousLyapunovEquation(
  const Eigen::Ref<const Vector1d>& A, const Eigen::Ref<const Vector1d>& Q);

Eigen::Matrix2d Solve2By2RealContinuousLyapunovEquation(
  const Eigen::Ref<const Eigen::Matrix2d>& A,
  const Eigen::Ref<const Eigen::Matrix2d>& Q);

// If the problem size is larger than 2-by-2, then it is reduced into a form
// which can be recursively solved by smaller problems.

Eigen::MatrixXd SolveReducedRealContinuousLyapunovEquation(
  const Eigen::Ref<const Eigen::MatrixXd>& A,
  const Eigen::Ref<const Eigen::MatrixXd>& Q);

}  // namespace internal

#endif // CONTINUOUS_LYAPUNOV_EQUATION_HPP