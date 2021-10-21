/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file ParamaterMatrix.h
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
 * @tparam M: The dimension of the type you wish to evaluate.
 * @param N: the number of Basis points (e.g. Chebyshev points of the second
 * kind).
 */
template <int M>
class ParameterMatrix {
  using MatrixType = Eigen::Matrix<double, M, -1>;

 private:
  MatrixType matrix_;

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  enum { dimension = Eigen::Dynamic };

  /**
   * Create ParameterMatrix using the number of basis points.
   * @param N: The number of basis points (the columns).
   */
  ParameterMatrix(const size_t N) : matrix_(M, N) { matrix_.setZero(); }

  /**
   * Create ParameterMatrix from an MxN Eigen Matrix.
   * @param matrix: An Eigen matrix used to initialze the ParameterMatrix.
   */
  ParameterMatrix(const MatrixType& matrix) : matrix_(matrix) {}

  /// Get the number of rows.
  size_t rows() const { return matrix_.rows(); }

  /// Get the number of columns.
  size_t cols() const { return matrix_.cols(); }

  /// Get the underlying matrix.
  MatrixType matrix() const { return matrix_; }

  /// Return the tranpose of the underlying matrix.
  Eigen::Matrix<double, -1, M> transpose() const { return matrix_.transpose(); }

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
  auto row(size_t index) -> Eigen::Block<MatrixType, 1, -1, false> {
    return matrix_.row(index);
  }

  /**
   * Get the matrix column specified by `index`.
   * @param index: The column index to retrieve.
   */
  Eigen::Matrix<double, M, 1> col(size_t index) const {
    return matrix_.col(index);
  }

  /**
   * Set the matrix column specified by `index`.
   * @param index: The column index to set.
   */
  auto col(size_t index) -> Eigen::Block<MatrixType, M, 1, true> {
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
  ParameterMatrix<M> operator+(const ParameterMatrix<M>& other) const {
    return ParameterMatrix<M>(matrix_ + other.matrix());
  }

  /**
   * Add a MxN-sized vector to the ParameterMatrix.
   * @param other: Vector which is reshaped and added.
   */
  ParameterMatrix<M> operator+(
      const Eigen::Matrix<double, -1, 1>& other) const {
    // This form avoids a deep copy and instead typecasts `other`.
    Eigen::Map<const MatrixType> other_(other.data(), M, cols());
    return ParameterMatrix<M>(matrix_ + other_);
  }

  /**
   * Subtract a ParameterMatrix from another.
   * @param other: ParameterMatrix to subtract.
   */
  ParameterMatrix<M> operator-(const ParameterMatrix<M>& other) const {
    return ParameterMatrix<M>(matrix_ - other.matrix());
  }

  /**
   * Subtract a MxN-sized vector from the ParameterMatrix.
   * @param other: Vector which is reshaped and subracted.
   */
  ParameterMatrix<M> operator-(
      const Eigen::Matrix<double, -1, 1>& other) const {
    Eigen::Map<const MatrixType> other_(other.data(), M, cols());
    return ParameterMatrix<M>(matrix_ - other_);
  }

  /**
   * Multiply ParameterMatrix with an Eigen matrix.
   * @param other: Eigen matrix which should be multiplication compatible with
   * the ParameterMatrix.
   */
  MatrixType operator*(const Eigen::Matrix<double, -1, -1>& other) const {
    return matrix_ * other;
  }

  /// @name Vector Space requirements, following LieMatrix
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
  bool equals(const ParameterMatrix<M>& other, double tol = 1e-8) const {
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
  inline static ParameterMatrix identity() {
    // throw std::runtime_error(
    //     "ParameterMatrix::identity(): Don't use this function");
    return ParameterMatrix(0);
  }

  /// @}
};

// traits for ParameterMatrix
template <int M>
struct traits<ParameterMatrix<M>>
    : public internal::VectorSpace<ParameterMatrix<M>> {};

/* ************************************************************************* */
// Stream operator that takes a ParameterMatrix. Used for printing.
template <int M>
inline std::ostream& operator<<(std::ostream& os,
                                const ParameterMatrix<M>& parameterMatrix) {
  os << parameterMatrix.matrix();
  return os;
}

}  // namespace gtsam