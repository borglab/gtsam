/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file   RegularHessianFactor.h
 * @brief  HessianFactor class with constant sized blocks
 * @author Sungtae An
 * @date   March 2014
 */

#pragma once

#include <gtsam/linear/HessianFactor.h>
#include <gtsam/linear/RegularJacobianFactor.h>
#include <vector>

namespace gtsam {

template<size_t D>
class RegularHessianFactor: public HessianFactor {

public:

  typedef Eigen::Matrix<double, D, 1> VectorD;
  typedef Eigen::Matrix<double, D, D> MatrixD;

  /** Construct an n-way factor.  Gs contains the upper-triangle blocks of the
   * quadratic term (the Hessian matrix) provided in row-order, gs the pieces
   * of the linear vector term, and f the constant term.
   */
  RegularHessianFactor(const KeyVector& js,
      const std::vector<Matrix>& Gs, const std::vector<Vector>& gs, double f) :
      HessianFactor(js, Gs, gs, f) {
    checkInvariants();
  }

  /** Construct a binary factor.  Gxx are the upper-triangle blocks of the
   * quadratic term (the Hessian matrix), gx the pieces of the linear vector
   * term, and f the constant term.
   */
  RegularHessianFactor(Key j1, Key j2, const MatrixD& G11, const MatrixD& G12,
      const VectorD& g1, const MatrixD& G22, const VectorD& g2, double f) :
      HessianFactor(j1, j2, G11, G12, g1, G22, g2, f) {
  }

  /** Construct a ternary factor.  Gxx are the upper-triangle blocks of the
   * quadratic term (the Hessian matrix), gx the pieces of the linear vector
   * term, and f the constant term.
   */
  RegularHessianFactor(Key j1, Key j2, Key j3,
      const MatrixD& G11, const MatrixD& G12, const MatrixD& G13, const VectorD& g1,
      const MatrixD& G22, const MatrixD& G23, const VectorD& g2,
      const MatrixD& G33, const VectorD& g3, double f) :
    HessianFactor(j1, j2, j3, G11, G12, G13, g1, G22, G23, g2, G33, g3, f) {
  }

  /** Constructor with an arbitrary number of keys and with the augmented information matrix
   *   specified as a block matrix. */
  template<typename KEYS>
  RegularHessianFactor(const KEYS& keys,
      const SymmetricBlockMatrix& augmentedInformation) :
      HessianFactor(keys, augmentedInformation) {
    checkInvariants();
  }

  /// Construct from RegularJacobianFactor
  RegularHessianFactor(const RegularJacobianFactor<D>& jf)
      : HessianFactor(jf) {}

  /// Construct from a GaussianFactorGraph
  RegularHessianFactor(const GaussianFactorGraph& factors,
                       const Scatter& scatter)
      : HessianFactor(factors, scatter) {
    checkInvariants();
  }

  /// Construct from a GaussianFactorGraph
  RegularHessianFactor(const GaussianFactorGraph& factors)
      : HessianFactor(factors) {
    checkInvariants();
  }

private:

  /// Check invariants after construction
  void checkInvariants() {
    if (info_.cols() != 1 + (info_.nBlocks()-1) * (DenseIndex)D)
      throw std::invalid_argument(
           "RegularHessianFactor constructor was given non-regular factors");
  }

  // Use Eigen magic to access raw memory
  typedef Eigen::Map<VectorD> DMap;
  typedef Eigen::Map<const VectorD> ConstDMap;

  // Scratch space for multiplyHessianAdd
  // According to link below this is thread-safe.
  // http://stackoverflow.com/questions/11160964/multiple-copies-of-the-same-object-c-thread-safe
  mutable std::vector<VectorD> y_;

public:

  /** y += alpha * A'*A*x */
  void multiplyHessianAdd(double alpha, const VectorValues& x,
      VectorValues& y) const override {
    HessianFactor::multiplyHessianAdd(alpha, x, y);
  }

  /** y += alpha * A'*A*x */
  void multiplyHessianAdd(double alpha, const double* x,
      double* yvalues) const {
    // Create a vector of temporary y_ values, corresponding to rows i
    y_.resize(size());
    for(VectorD & yi: y_)
      yi.setZero();

    // Accessing the VectorValues one by one is expensive
    // So we will loop over columns to access x only once per column
    // And fill the above temporary y_ values, to be added into yvalues after
    VectorD xj(D);
    for (DenseIndex j = 0; j < (DenseIndex) size(); ++j) {
      Key key = keys_[j];
      const double* xj = x + key * D;
      DenseIndex i = 0;
      for (; i < j; ++i)
        y_[i] += info_.aboveDiagonalBlock(i, j) * ConstDMap(xj);
      // blocks on the diagonal are only half
      y_[i] += info_.diagonalBlock(j) * ConstDMap(xj);
      // for below diagonal, we take transpose block from upper triangular part
      for (i = j + 1; i < (DenseIndex) size(); ++i)
        y_[i] += info_.aboveDiagonalBlock(j, i).transpose() * ConstDMap(xj);
    }

    // copy to yvalues
    for (DenseIndex i = 0; i < (DenseIndex) size(); ++i) {
      Key key = keys_[i];
      DMap(yvalues + key * D) += alpha * y_[i];
    }
  }

  /// Raw memory version, with offsets TODO document reasoning
  void multiplyHessianAdd(double alpha, const double* x, double* yvalues,
      std::vector<size_t> offsets) const {

    // Create a vector of temporary y_ values, corresponding to rows i
    y_.resize(size());
    for(VectorD & yi: y_)
      yi.setZero();

    // Accessing the VectorValues one by one is expensive
    // So we will loop over columns to access x only once per column
    // And fill the above temporary y_ values, to be added into yvalues after
    for (DenseIndex j = 0; j < (DenseIndex) size(); ++j) {
      DenseIndex i = 0;
      for (; i < j; ++i)
        y_[i] += info_.aboveDiagonalBlock(i, j)
            * ConstDMap(x + offsets[keys_[j]],
                offsets[keys_[j] + 1] - offsets[keys_[j]]);
      // blocks on the diagonal are only half
      y_[i] += info_.diagonalBlock(j)
          * ConstDMap(x + offsets[keys_[j]],
              offsets[keys_[j] + 1] - offsets[keys_[j]]);
      // for below diagonal, we take transpose block from upper triangular part
      for (i = j + 1; i < (DenseIndex) size(); ++i)
        y_[i] += info_.aboveDiagonalBlock(j, i).transpose()
            * ConstDMap(x + offsets[keys_[j]],
                offsets[keys_[j] + 1] - offsets[keys_[j]]);
    }

    // copy to yvalues
    for (DenseIndex i = 0; i < (DenseIndex) size(); ++i)
      DMap(yvalues + offsets[keys_[i]],
          offsets[keys_[i] + 1] - offsets[keys_[i]]) += alpha * y_[i];
  }

  /** Return the diagonal of the Hessian for this factor (raw memory version) */
  void hessianDiagonal(double* d) const override {

    // Loop over all variables in the factor
    for (DenseIndex pos = 0; pos < (DenseIndex) size(); ++pos) {
      Key j = keys_[pos];
      // Get the diagonal block, and insert its diagonal
      DMap(d + D * j) += info_.diagonal(pos);
    }
  }

  /// Add gradient at zero to d TODO: is it really the goal to add ??
  void gradientAtZero(double* d) const override {

    // Loop over all variables in the factor
    for (DenseIndex pos = 0; pos < (DenseIndex) size(); ++pos) {
      Key j = keys_[pos];
      // Get the diagonal block, and insert its diagonal
      DMap(d + D * j) -= info_.aboveDiagonalBlock(pos, size());;
    }
  }

  /* ************************************************************************* */

};
// end class RegularHessianFactor

// traits
template<size_t D> struct traits<RegularHessianFactor<D> > : public Testable<
    RegularHessianFactor<D> > {
};

}

