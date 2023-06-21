/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file ParameterMatrix.h
 * @brief Define ParameterMatrix class which is used to store values at
 * interpolation points.
 * @author Varun Agrawal, Frank Dellaert
 * @date September 21, 2020
 */

#pragma once

#include <gtsam/base/Matrix.h>
#include <gtsam/base/Testable.h>
#include <gtsam/base/VectorSpace.h>

#include <iostream>

namespace gtsam {

/**
 * A matrix abstraction of MxN values at the Basis points.
 * This class serves as a wrapper over an Eigen matrix.
 * @param N: the number of Basis points (e.g. Chebyshev points of the second
 * kind).
 */
class ParameterMatrix {
 private:
  Matrix matrix_;

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  enum { dimension = Eigen::Dynamic };

  /**
   * Create ParameterMatrix using the number of basis points.
   * @param M: The dimension size of the type you wish to evaluate.
   * @param N: The number of basis points (the columns).
   */
  ParameterMatrix(const size_t M, const size_t N) : matrix_(M, N) {
    matrix_.setZero();
  }

  /**
   * Create ParameterMatrix from an MxN Eigen Matrix.
   * @param matrix: An Eigen matrix used to initialze the ParameterMatrix.
   */
  ParameterMatrix(const Matrix& matrix) : matrix_(matrix) {}

  /// Get the number of rows.
  size_t rows() const { return matrix_.rows(); }

  /// Get the number of columns.
  size_t cols() const { return matrix_.cols(); }

  /// Get the underlying matrix.
  Matrix matrix() const { return matrix_; }

  /// Return the tranpose of the underlying matrix.
  Matrix transpose() const { return matrix_.transpose(); }

  /**
   * Get the matrix row specified by `index`.
   * @param index: The row index to retrieve.
   */
  Eigen::Matrix<double, 1, -1> row(size_t index) const {
    return matrix_.row(index);
  }

  /**
   * Set the matrix row specified by `index`.
   * @param index: The row index to set.
   */
  auto row(size_t index) -> Eigen::Block<Matrix, 1, -1, false> {
    return matrix_.row(index);
  }

  /**
   * Get the matrix column specified by `index`.
   * @param index: The column index to retrieve.
   */
  Eigen::Matrix<double, -1, 1> col(size_t index) const {
    return matrix_.col(index);
  }

  /**
   * Set the matrix column specified by `index`.
   * @param index: The column index to set.
   */
  auto col(size_t index) -> Eigen::Block<Matrix, -1, 1, true> {
    return matrix_.col(index);
  }

  /**
   * Set all matrix coefficients to zero.
   */
  void setZero() { matrix_.setZero(); }

  /**
   * Add a ParameterMatrix to another.
   * @param other: ParameterMatrix to add.
   */
  ParameterMatrix operator+(const ParameterMatrix& other) const {
    return ParameterMatrix(matrix_ + other.matrix());
  }

  /**
   * Add a MxN-sized vector to the ParameterMatrix.
   * @param other: Vector which is reshaped and added.
   */
  ParameterMatrix operator+(const Eigen::Matrix<double, -1, 1>& other) const {
    // This form avoids a deep copy and instead typecasts `other`.
    Eigen::Map<const Matrix> other_(other.data(), rows(), cols());
    return ParameterMatrix(matrix_ + other_);
  }

  /**
   * Subtract a ParameterMatrix from another.
   * @param other: ParameterMatrix to subtract.
   */
  ParameterMatrix operator-(const ParameterMatrix& other) const {
    return ParameterMatrix(matrix_ - other.matrix());
  }

  /**
   * Subtract a MxN-sized vector from the ParameterMatrix.
   * @param other: Vector which is reshaped and subracted.
   */
  ParameterMatrix operator-(const Eigen::Matrix<double, -1, 1>& other) const {
    Eigen::Map<const Matrix> other_(other.data(), rows(), cols());
    return ParameterMatrix(matrix_ - other_);
  }

  /**
   * Multiply ParameterMatrix with an Eigen matrix.
   * @param other: Eigen matrix which should be multiplication compatible with
   * the ParameterMatrix.
   */
  Matrix operator*(const Matrix& other) const { return matrix_ * other; }

  /// @name Vector Space requirements
  /// @{

  /**
   * Print the ParameterMatrix.
   * @param s: The prepend string to add more contextual info.
   */
  void print(const std::string& s = "") const {
    std::cout << (s == "" ? s : s + " ") << matrix_ << std::endl;
  }

  /**
   * Check for equality up to absolute tolerance.
   * @param other: The ParameterMatrix to check equality with.
   * @param tol: The absolute tolerance threshold.
   */
  bool equals(const ParameterMatrix& other, double tol = 1e-8) const {
    return gtsam::equal_with_abs_tol(matrix_, other.matrix(), tol);
  }

  /// Returns dimensionality of the tangent space
  inline size_t dim() const { return matrix_.size(); }

  /// Convert to vector form, is done row-wise
  inline Vector vector() const {
    using RowMajor = Eigen::Matrix<double, -1, -1, Eigen::RowMajor>;
    Vector result(matrix_.size());
    Eigen::Map<RowMajor>(&result(0), rows(), cols()) = matrix_;
    return result;
  }

  /** Identity function to satisfy VectorSpace traits.
   *
   * NOTE: The size at compile time is unknown so this identity is zero
   * length and thus not valid.
   */
  inline static ParameterMatrix Identity(size_t M = 0, size_t N = 0) {
    return ParameterMatrix(M, N);
  }

  /// @}
};

/// traits for ParameterMatrix
template <>
struct traits<ParameterMatrix> : public internal::VectorSpace<ParameterMatrix> {
};

/* ************************************************************************* */
// Stream operator that takes a ParameterMatrix. Used for printing.
inline std::ostream& operator<<(std::ostream& os,
                                const ParameterMatrix& parameterMatrix) {
  os << parameterMatrix.matrix();
  return os;
}

}  // namespace gtsam
