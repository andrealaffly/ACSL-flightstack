///@cond
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
///@endcond
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
 * File:        continuous_lyapunov_equation.cpp
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
 * @file continuous_lyapunov_equation.cpp
 * @brief Computes the solution of the continuos Lyapunov equation
 * 
 * WARNING !!!
 * The following code uses a different convention than MATLAB's.
 * This code solves the equation: `AᵀX + XA + Q = 0`
 * MATLAB command "lyap(A, Q)" solves the equation: `AX + XAᵀ + Q = 0`
 * So this code provides the same result as obtained in MATLAB using 
 * the command: lyap(A', Q)!!!!!!!!!!!
 */
#include <cmath>
#include <complex>
#include <limits>
#include <stdexcept>

#include "continuous_lyapunov_equation.hpp"

using Eigen::Matrix2d;
using Eigen::MatrixXd;
using Eigen::VectorXcd;

namespace {
const double kTolerance = 5 * std::numeric_limits<double>::epsilon();

bool is_zero(double x, double eps = 1e-10) {
    return std::fabs(x) < eps;
}

bool is_approx_equal_abstol(const Eigen::MatrixXd& A, const Eigen::MatrixXd& B, double tol) {
    return (A - B).cwiseAbs().maxCoeff() < tol;
}
}  // namespace

/**
 * @brief RealContinuousLyapunovEquation
 * 
 * @param A 
 * @param Q 
 * @return MatrixXd 
 */
MatrixXd RealContinuousLyapunovEquation(const Eigen::Ref<const MatrixXd>& A,
                                        const Eigen::Ref<const MatrixXd>& Q) {
  if (A.rows() != A.cols() || Q.rows() != Q.cols() || A.rows() != Q.rows()) {
    throw std::runtime_error(
        "RealContinuousLyapunovEquation(): A and Q must be square and of equal "
        "size!");
  }
  
  if (!is_approx_equal_abstol(Q, Q.transpose(), 1e-10)) {
    throw std::runtime_error("Q is not symmetric.");
  }

  // The cases where A is 1-by-1 or 2-by-2 are caught as we have an efficient
  // methods at hand to solve these without the use of Schur factorization.
  if (static_cast<int>(A.cols()) == 1) {
    if (is_zero(A(0, 0))) {
      throw std::runtime_error(
          "RealContinuousLyapunovEquation(): Solution is not unique!");
    }
    return internal::Solve1By1RealContinuousLyapunovEquation(A, Q);
  }
  if (static_cast<int>(A.cols()) == 2) {
    // If A has two real eigenvalues λ₁ and λ₂, then we check (1) if λ₁ or λ₂ is
    // 0, i.e. if the determinant of A is 0; (2) if λ₁ + λ₂ = 0, i.e., if the
    // trace of A is 0. If A has two complex eigenvalues λ₁ and λ̅₁, then we
    // check if λ₁ + λ̅₁ = 0, i.e., if the trace of A is 0.
    if (is_zero(A(0, 0) + A(1, 1)) ||
        is_zero(A(0, 0) * A(1, 1) - A(1, 0) * A(0, 1))) {
      throw std::runtime_error(
          "RealContinuousLyapunovEquation(): Solution is not unique!");
    }
    // Reducing Q to upper triangular form.
    MatrixXd Q_upper{Q};
    Q_upper(1, 0) = NAN;
    return internal::Solve2By2RealContinuousLyapunovEquation(A, Q_upper);
  }
  VectorXcd eig_val{A.eigenvalues()};
  for (int i = 0; i < eig_val.size(); ++i) {
    for (int j = i; j < eig_val.size(); ++j) {
      std::complex<double> eig_sum = eig_val(i) + std::conj(eig_val(j));
      if (is_zero(eig_sum.real()) && is_zero(eig_sum.imag())) {
        throw std::runtime_error(
            "RealContinuousLyapunovEquation(): Solution is not unique!");
      }
    }
  }
  // Transform into reduced form S'*X̅  + X̅ *S = -Q̅, where A = U*S*U', X̅ =
  // U'*X*U, Q̅ = U'*Q*U, U is the unitary matrix, and S the reduced form. Note
  // that the expression for X̅ in [2] is incorrect.
  Eigen::RealSchur<MatrixXd> schur(A);
  const MatrixXd U{schur.matrixU()};
  const MatrixXd S{schur.matrixT()};
  if (schur.info() != Eigen::Success) {
    throw std::runtime_error(
        "RealContinuousLyapunovEquation(): Schur factorization failed.");
  }
  // Reduce the symmetric Q̅ to its upper triangular form Q̅_ᵤₚₚₑᵣ.
  MatrixXd Q_bar_upper{MatrixXd::Constant(Q.rows(), Q.cols(), NAN)};
  Q_bar_upper.triangularView<Eigen::Upper>() = U.transpose() * Q * U;
  return (U *
          internal::SolveReducedRealContinuousLyapunovEquation(S, Q_bar_upper) *
          U.transpose());
}

namespace internal {

using Eigen::Matrix3d;
using Eigen::Vector3d;
using Eigen::VectorXd;

Vector1d Solve1By1RealContinuousLyapunovEquation(
    const Eigen::Ref<const Vector1d>& A, const Eigen::Ref<const Vector1d>& Q) {
  return Vector1d(-Q(0) / (2 * A(0)));
}

Matrix2d Solve2By2RealContinuousLyapunovEquation(
    const Eigen::Ref<const Matrix2d>& A, const Eigen::Ref<const Matrix2d>& Q) {
  // Rewrite AᵀX + XA + Q = 0 (case where A,Q are 2-by-2) into linear set A_vec
  // *vec(X) = -vec(Q) and solve for X.
  // Only the upper triangular part of Q is used, thus it is treated to be
  // symmetric.
  Matrix3d A_vec;
  A_vec << 2 * A(0, 0), 2 * A(1, 0), 0, A(0, 1), A(0, 0) + A(1, 1), A(1, 0), 0,
      2 * A(0, 1), 2 * A(1, 1);
  const Vector3d Q_vec(-Q(0, 0), -Q(0, 1), -Q(1, 1));

  // See https://eigen.tuxfamily.org/dox/group__TutorialLinearAlgebra.html for
  // quick overview of possible solver algorithms.
  // ColPivHouseholderQR is accurate and fast for small systems.
  Vector3d x{A_vec.colPivHouseholderQr().solve(Q_vec)};
  Matrix2d X;
  X << x(0), x(1), x(1), x(2);
  return X;
}

MatrixXd SolveReducedRealContinuousLyapunovEquation(
    const Eigen::Ref<const MatrixXd>& S,
    const Eigen::Ref<const MatrixXd>& Q_bar) {
  const int m{static_cast<int>(S.rows())};
  if (m == 1) return Solve1By1RealContinuousLyapunovEquation(S, Q_bar);
  if (m == 2) return Solve2By2RealContinuousLyapunovEquation(S, Q_bar);
  // Notation & partition adapted from SB03MD:
  //
  //
  //    S = [s₁₁    s']   Q̅ = [q̅₁₁   q̅']   X̅ = [x̅₁₁    x̅']
  //        [0      S₁] ,     [q̅     Q₁] ,     [x̅      X₁]
  //
  // where s₁₁ is a 1-by-1 or 2-by-2 matrix.
  //
  //    s₁₁'x̅₁₁ + x̅₁₁s₁₁ = -q̅₁₁                                      (3.1)
  //
  //    S₁'x̅ + x̅s₁₁      = -q̅₁₁ - sx̅₁₁                               (3.2)
  //
  //    S₁'X₁ + X₁S₁     = -Q₁ - sx̅' - x̅s'                           (3.3)

  int n{1};  // n equals the size of the n-by-n block in the left-upper
             // corner of S.
  // Check if the top block of S is 1-by-1 or 2-by-2.
  if (!(std::abs(S(1, 0)) < kTolerance * S.norm())) {
    n = 2;
  }


  const MatrixXd s_11{S.topLeftCorner(n, n)};
  const MatrixXd s{S.transpose().bottomLeftCorner(m - n, n)};
  const MatrixXd S_1{S.bottomRightCorner(m - n, m - n)};
  const MatrixXd q_bar_11{Q_bar.topLeftCorner(n, n)};
  const MatrixXd q_bar{Q_bar.transpose().bottomLeftCorner(m - n, n)};
  const MatrixXd Q_1{Q_bar.bottomRightCorner(m - n, m - n)};

  MatrixXd x_bar_11(n, n), x_bar(m - n, n);
  if (n == 1) {
    // solving equation 3.1
    x_bar_11 << Solve1By1RealContinuousLyapunovEquation(s_11, q_bar_11);
    // solving equation 3.2
    const MatrixXd lhs{S_1.transpose() +
                       MatrixXd::Identity(S_1.cols(), S_1.rows()) * s_11(0)};
    const VectorXd rhs{-q_bar - s * x_bar_11(0)};
    x_bar << lhs.colPivHouseholderQr().solve(rhs);
  } else {
    x_bar_11 << Solve2By2RealContinuousLyapunovEquation(s_11, q_bar_11);
    // solving equation 3.2
    // The equation reads as S₁'x̅ + x̅s₁₁ = -q̅₁₁ - sx̅₁₁,
    // where S₁ ∈ ℝᵐ⁻²ˣᵐ⁻²; x̅, q̅, s ∈ ℝᵐ⁻²ˣ² and s₁₁, x̅₁₁ ∈ ℝ²ˣ².
    // We solve the linear equation by vectorization and feeding it into
    // colPivHouseHolderQr().
    //
    //  Notation:
    //
    //  The elements in s₁₁ are names as the following:
    //  s₁₁ = [s₁₁(0,0) s₁₁(0,1); s₁₁(1,0) s₁₁(1,1)].
    //  Note that eigen starts counting at 0, and not as above at 1.
    //
    //  The ith column of a matrix is accessed by [i], i.e. the first column
    //  of x̅ is x̅[0].
    //
    //  Define:
    //
    //  S_1_vec = [S₁' 0; 0 S₁'] with 0 \in ∈ℝᵐˣᵐ
    //
    //  S_11_vec = [s₁₁(0,0)*I s₁₁(1,0)*I; s₁₁(0,1)*I s₁₁(1,1)*I] with I ∈ ℝᵐˣᵐ.
    //
    //  Now define lhs = S_1_vec + S_11_vec.
    //
    //  x_vec = [x̅[0]' x̅[1]']' where [i] is the ith column
    //
    //  rhs = - [(q̅+sx̅₁₁)[0]' (q̅+s*x̅₁₁)[1]']'
    //
    //  Now S₁'x̅ + x̅s₁₁ = -q̅₁₁ - sx̅₁₁  can be rewritten into:
    //
    //  lhs*x_vec = rhs.
    //
    //  This expression can be fed into colPivHouseHolderQr() and solved.
    //  Finally, construct x_bar from x_vec.
    //

    MatrixXd S_1_vec(2 * (m - 2), 2 * (m - 2));
    S_1_vec << S_1.transpose(), MatrixXd::Zero(m - 2, m - 2),
        MatrixXd::Zero(m - 2, m - 2), S_1.transpose();

    MatrixXd s_11_vec(2 * (m - 2), 2 * (m - 2));
    s_11_vec << s_11(0, 0) * MatrixXd::Identity(m - 2, m - 2),
        s_11(1, 0) * MatrixXd::Identity(m - 2, m - 2),
        s_11(0, 1) * MatrixXd::Identity(m - 2, m - 2),
        s_11(1, 1) * MatrixXd::Identity(m - 2, m - 2);
    MatrixXd lhs{S_1_vec + s_11_vec};

    VectorXd rhs(2 * (m - 2), 1);
    rhs << (-q_bar - s * x_bar_11).col(0), (-q_bar - s * x_bar_11).col(1);

    VectorXd x_bar_vec{lhs.colPivHouseholderQr().solve(rhs)};
    x_bar.col(0) = x_bar_vec.segment(0, m - 2);
    x_bar.col(1) = x_bar_vec.segment(m - 2, m - 2);
  }

  // Set up equation 3.3
  const Eigen::MatrixXd temp_summand{s * x_bar.transpose()};
  // Since Q₁ is only an upper triangular matrix, with NAN in the lower part,
  // Q_NEW_ᵤₚₚₑᵣ is so as well.
  const Eigen::MatrixXd Q_new_upper{
      (Q_1 + temp_summand + temp_summand.transpose())};
  // The solution is found recursively.
  MatrixXd X_bar(S.cols(), S.rows());
  X_bar << x_bar_11, x_bar.transpose(), x_bar,
      SolveReducedRealContinuousLyapunovEquation(S_1, Q_new_upper);

  return X_bar;
}

}  // namespace internal
