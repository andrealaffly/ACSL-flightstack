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
 * File:        projection_operator.hpp
 * Author:      Mattia Gramuglia
 * Date:        September 4, 2024
 * For info:    Andrea L'Afflitto 
 *              a.lafflitto@vt.edu
 * 
 * Description: Implementation of the projection operator to be used in the MRAC controllers.
 *              For reference: A. L'Afflitto, "Notes on Adaptive Control and Estimation", Springer, Sec. 3.5. or
 *              E. Lavretsky, K. Wise, "Robust and Adaptive Control", Springer 2013, Sec. 11.4
 * 
 * GitHub:    https://github.com/andrealaffly/ACSL-flightstack.git
 **********************************************************************************************************************/
#ifndef PROJECTION_OPERATOR_HPP
#define PROJECTION_OPERATOR_HPP

#include <Eigen/Dense>

namespace projection_operator
{

/*
  Generate the matrix S from its diagonal terms contained in the column vector S_diagonal
*/
template<typename Derived>
auto generateMatrixFromDiagonal(const Eigen::MatrixBase<Derived>& S_diagonal)
  -> Eigen::Matrix<typename Derived::Scalar, Derived::RowsAtCompileTime, Derived::RowsAtCompileTime>
{
  static_assert(Derived::ColsAtCompileTime == 1, "Input must be a column vector");

  Eigen::Matrix<typename Derived::Scalar, Derived::RowsAtCompileTime, Derived::RowsAtCompileTime> S = 
    Eigen::Matrix<typename Derived::Scalar, Derived::RowsAtCompileTime, Derived::RowsAtCompileTime>::Zero();
  for (int i = 0; i < Derived::RowsAtCompileTime; ++i) {
    S(i, i) = S_diagonal(i);
  }
  return S;
}

/*
  Generate the matrix S from the ellipsoid semi-axis length terms contained in the column vector S_diagonal.
  If S = diag(1/a^2, 1/b^2, 1/c^2) then the semi-axis lengths are [a, b, c].
*/
template<typename Derived>
auto generateEllipsoidMatrixFromDiagonal(const Eigen::MatrixBase<Derived>& S_diagonal)
  -> Eigen::Matrix<typename Derived::Scalar, Derived::RowsAtCompileTime, Derived::RowsAtCompileTime>
{
  static_assert(Derived::ColsAtCompileTime == 1, "Input must be a column vector");

  Eigen::Matrix<typename Derived::Scalar, Derived::RowsAtCompileTime, Derived::RowsAtCompileTime> S = 
    Eigen::Matrix<typename Derived::Scalar, Derived::RowsAtCompileTime, Derived::RowsAtCompileTime>::Zero();
  for (int i = 0; i < Derived::RowsAtCompileTime; ++i) {
    S(i, i) = 1 / std::pow(S_diagonal(i), 2);
  }
  return S;
}

/* 
  Compute the epsilon parameter from the scaling coefficient alpha
*/
inline double computeEpsilonFromAlpha(const double alpha)
{
  double epsilon = (1.0 / (alpha*alpha) - 1.0);
  return epsilon;
}

/*
  Struct to hold the output of the convex function
  - h_function: convex set function
  - dh_dx_jacobian: derivative of h_function with respect to x as a row vector (Jacobian)
*/
template <int N>
struct ConvexFunctionOutput
{
  double h_function;                           // Scalar 
  Eigen::Matrix<double, 1, N> dh_dx_jacobian;  // Row vector 
};

namespace ball
{

/* 
  Convex function to compute h and dh_dx. 
  The inner convex set is a ball of radius sqrt(x_max)
  The outer convex set is a ball of radius sqrt(x_max + epsilon)
*/
template <int N>
ConvexFunctionOutput<N> convexFunction(const Eigen::Matrix<double, N, 1>& x,
                                       const double x_max, const double epsilon)
{
  // Compute h_function = (x'*x - x_max) / epsilon
  double h_function = (x.dot(x) - x_max) / epsilon;

  // Compute dh_dx_jacobian = 2/epsilon * x' (row vector)
  Eigen::Matrix<double, 1, N> dh_dx_jacobian = (2.0 / epsilon) * x.transpose();

  // Return a struct containing both h_function and dh_dx_jacobian
  return ConvexFunctionOutput<N>{h_function, dh_dx_jacobian};
}

/* 
  Function to project the vector x_d based on the projection_operator::ball::convexFunction()
  x_d: derivative wrt time of x (RHS of its dynamics equation)
*/
template <int N>
Eigen::Matrix<double, N, 1> projectionVector(const Eigen::Matrix<double, N, 1>& x,
                                             const Eigen::Matrix<double, N, 1>& x_d,
                                             const double x_max, const double epsilon)
{
  // Call the convex_function to get h_function and dh_dx_jacobian
  ConvexFunctionOutput<N> cfo = convexFunction<N>(x, x_max, epsilon);
  double h_function = cfo.h_function;
  Eigen::Matrix<double, 1, N> dh_dx_jacobian = cfo.dh_dx_jacobian;

  // Check if h_function > 0 and dh_dx_jacobian * x_d > 0
  if (h_function > 0 && dh_dx_jacobian.dot(x_d) > 0) 
  {
    // Precompute the transpose of dh_dx_jacobian
    Eigen::Matrix<double, N, 1> dh_dx_jacobian_transpose = dh_dx_jacobian.transpose();

    // Modify x_d based on the projection formula
    Eigen::Matrix<double, N, 1> x_d_modified = 
      x_d - h_function * dh_dx_jacobian_transpose * dh_dx_jacobian * x_d / 
      dh_dx_jacobian.dot(dh_dx_jacobian_transpose);

    return x_d_modified;
  }
  else 
  {
    // If the conditions are not met, return x_d unchanged
    return x_d;
  }
}

/*
  Function to apply projectionVector to a matrix by reshaping it into a vector.
*/ 
template <typename Derived1, typename Derived2>
auto projectionMatrix(const Eigen::MatrixBase<Derived1>& matrix,
                      const Eigen::MatrixBase<Derived2>& matrix_d,
                      const double x_max, const double epsilon)
    -> Eigen::Matrix<typename Derived1::Scalar, Derived1::RowsAtCompileTime, Derived1::ColsAtCompileTime>
{
  // Use Eigen::internal::traits to get uniform access to row/col dimensions
  // Two template types are used because the first input is a Eigen::Map while the
  // second input is a regular Eigen::Matrix.
  constexpr int Rows1 = Eigen::internal::traits<Derived1>::RowsAtCompileTime;
  constexpr int Cols1 = Eigen::internal::traits<Derived1>::ColsAtCompileTime;
  constexpr int Rows2 = Eigen::internal::traits<Derived2>::RowsAtCompileTime;
  constexpr int Cols2 = Eigen::internal::traits<Derived2>::ColsAtCompileTime;

  // Ensure the dimensions of both matrices are the same
  static_assert(Rows1 == Rows2 && Cols1 == Cols2, "The matrices must have the same dimensions.");

  // Total number of elements in the matrix
  constexpr int TotalElements = Rows1 * Cols1;

  // Reshape the matrix into a column vector (row-major order)
  Eigen::Map<const Eigen::Matrix<typename Derived1::Scalar, TotalElements, 1>>
    reshaped_matrix(matrix.derived().data());
  Eigen::Map<const Eigen::Matrix<typename Derived2::Scalar, TotalElements, 1>>
    reshaped_matrix_d(matrix_d.derived().data());

  // Apply the projectionVector on the reshaped vector
  Eigen::Matrix<double, TotalElements, 1> projected_vector = 
    projectionVector<TotalElements>(reshaped_matrix, reshaped_matrix_d, x_max, epsilon);

  // Reshape the projected vector back into a matrix of the same size
  Eigen::Map<Eigen::Matrix<double, Rows1, Cols1>> projected_matrix(projected_vector.data());

  return projected_matrix;
}
} // namespace ball

namespace ellipsoid
{

/* 
  Convex function to compute h and dh_dx. 
  The outer convex set is an ellipsoid centered in x_e with semi-axis length described by the diagonal elements
    that populate the S matrix. If S = diag(1/a^2, 1/b^2, 1/c^2) then the semi-axis lengths are [a, b, c].
  The inner convex set is the outer convex set ellipsoid scaled by the coefficient alpha = 1/sqrt(1 + epsilon).
*/
template <int N>
ConvexFunctionOutput<N> convexFunction(const Eigen::Matrix<double, N, 1>& x,
                                       const Eigen::Matrix<double, N, 1>& x_e,
                                       const Eigen::Matrix<double, N, N>& S,
                                       const double epsilon)
{
  // Compute the difference vector (x - x_e)
  Eigen::Matrix<double, N, 1> x_diff = x - x_e;
  
  // Compute (x - x_e)^T * S * (x - x_e)
  double quadratic_term = x_diff.transpose() * S * x_diff;
  
  // Compute h(x) = ((1 + epsilon) * quadratic_term - 1) / epsilon
  double h_function = ((1.0 + epsilon) * quadratic_term - 1.0) / epsilon;

  // Compute dh_dx_jacobian
  Eigen::Matrix<double, 1, N> dh_dx_jacobian = (2.0 * (1 + epsilon) / epsilon) * x_diff.transpose() * S;

  // Return a struct containing both h_function and dh_dx_jacobian
  return ConvexFunctionOutput<N>{h_function, dh_dx_jacobian};
}

/* 
  Function to project the vector x_d based on the projection_operator::ellipsoid::convexFunction()
  x_d: derivative wrt time of x (RHS of its dynamics equation)
*/
template <int N>
Eigen::Matrix<double, N, 1> projectionVector(const Eigen::Matrix<double, N, 1>& x,
                                             const Eigen::Matrix<double, N, 1>& x_d,
                                             const Eigen::Matrix<double, N, 1>& x_e,
                                             const Eigen::Matrix<double, N, N>& S,
                                             const double epsilon)
{
  // Call the convex_function to get h_function and dh_dx_jacobian
  ConvexFunctionOutput<N> cfo = convexFunction<N>(x, x_e, S, epsilon);
  double h_function = cfo.h_function;
  Eigen::Matrix<double, 1, N> dh_dx_jacobian = cfo.dh_dx_jacobian;

  // Check if h_function > 0 and dh_dx_jacobian * x_d > 0
  if (h_function > 0 && dh_dx_jacobian.dot(x_d) > 0) 
  {
    // Precompute the transpose of dh_dx_jacobian
    Eigen::Matrix<double, N, 1> dh_dx_jacobian_transpose = dh_dx_jacobian.transpose();

    // Modify x_d based on the projection formula
    Eigen::Matrix<double, N, 1> x_d_modified = 
      x_d - h_function * dh_dx_jacobian_transpose * dh_dx_jacobian * x_d / 
      dh_dx_jacobian.dot(dh_dx_jacobian_transpose);

    return x_d_modified;
  }
  else 
  {
    // If the conditions are not met, return x_d unchanged
    return x_d;
  }
}

/*
  Function to apply projectionVector to a matrix by reshaping it into a vector.
*/ 
template <typename Derived1, typename Derived2>
auto projectionMatrix(const Eigen::MatrixBase<Derived1>& matrix,
                      const Eigen::MatrixBase<Derived2>& matrix_d,
                      const Eigen::Matrix<double, Derived1::RowsAtCompileTime * Derived1::ColsAtCompileTime, 1>& x_e,
                      const Eigen::Matrix<double,
                                          Derived1::RowsAtCompileTime * Derived1::ColsAtCompileTime,
                                          Derived1::RowsAtCompileTime * Derived1::ColsAtCompileTime>& S,
                      const double epsilon)
    -> Eigen::Matrix<typename Derived1::Scalar, Derived1::RowsAtCompileTime, Derived1::ColsAtCompileTime>
{
  // Use Eigen::internal::traits to get uniform access to row/col dimensions
  // Two template types are used because the first input is a Eigen::Map while the
  // second input is a regular Eigen::Matrix.
  constexpr int Rows1 = Eigen::internal::traits<Derived1>::RowsAtCompileTime;
  constexpr int Cols1 = Eigen::internal::traits<Derived1>::ColsAtCompileTime;
  constexpr int Rows2 = Eigen::internal::traits<Derived2>::RowsAtCompileTime;
  constexpr int Cols2 = Eigen::internal::traits<Derived2>::ColsAtCompileTime;

  // Ensure the dimensions of both matrices are the same
  static_assert(Rows1 == Rows2 && Cols1 == Cols2, "The matrices must have the same dimensions.");

  // Total number of elements in the matrix
  constexpr int TotalElements = Rows1 * Cols1;

  // Reshape the matrix into a column vector (row-major order)
  Eigen::Map<const Eigen::Matrix<typename Derived1::Scalar, TotalElements, 1>>
    reshaped_matrix(matrix.derived().data());
  Eigen::Map<const Eigen::Matrix<typename Derived2::Scalar, TotalElements, 1>>
    reshaped_matrix_d(matrix_d.derived().data());

  // Apply the projectionVector on the reshaped vector
  Eigen::Matrix<double, TotalElements, 1> projected_vector = 
    projectionVector<TotalElements>(reshaped_matrix, reshaped_matrix_d, x_e, S, epsilon);

  // Reshape the projected vector back into a matrix of the same size
  Eigen::Map<Eigen::Matrix<double, Rows1, Cols1>> projected_matrix(projected_vector.data());

  return projected_matrix;
}
} // namespace ellipsoid

} // namespace projection_operator

#endif // PROJECTION_OPERATOR_HPP