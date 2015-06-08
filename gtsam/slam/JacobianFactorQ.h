/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/*
 * @file  JacobianFactorQ.h
 * @date  Oct 27, 2013
 * @uthor Frank Dellaert
 */

#pragma once

#include "RegularJacobianFactor.h"

namespace gtsam {
/**
 * JacobianFactor for Schur complement that uses Q noise model
 */
template<size_t D, size_t ZDim>
class JacobianFactorQ: public RegularJacobianFactor<D> {

  typedef RegularJacobianFactor<D> Base;
  typedef Eigen::Matrix<double, ZDim, D> MatrixZD;
  typedef std::pair<Key, MatrixZD> KeyMatrixZD;

public:

  /// Default constructor
  JacobianFactorQ() {
  }

  /// Empty constructor with keys
  JacobianFactorQ(const FastVector<Key>& keys, //
      const SharedDiagonal& model = SharedDiagonal()) :
      Base() {
    Matrix zeroMatrix = Matrix::Zero(0, D);
    Vector zeroVector = Vector::Zero(0);
    typedef std::pair<Key, Matrix> KeyMatrix;
    std::vector<KeyMatrix> QF;
    QF.reserve(keys.size());
    BOOST_FOREACH(const Key& key, keys)
      QF.push_back(KeyMatrix(key, zeroMatrix));
    JacobianFactor::fillTerms(QF, zeroVector, model);
  }

  /// Constructor
  JacobianFactorQ(const std::vector<KeyMatrixZD>& Fblocks, const Matrix& E,
      const Matrix3& P, const Vector& b, const SharedDiagonal& model =
          SharedDiagonal()) :
      Base() {
    size_t j = 0, m2 = E.rows(), m = m2 / ZDim;
    // Calculate projector Q
    Matrix Q = eye(m2) - E * P * E.transpose();
    // Calculate pre-computed Jacobian matrices
    // TODO: can we do better ?
    typedef std::pair<Key, Matrix> KeyMatrix;
    std::vector<KeyMatrix> QF;
    QF.reserve(m);
    // Below, we compute each mZDim*D block A_j = Q_j * F_j = (mZDim*ZDim) * (Zdim*D)
    BOOST_FOREACH(const KeyMatrixZD& it, Fblocks)
      QF.push_back(
          KeyMatrix(it.first, Q.block(0, ZDim * j++, m2, ZDim) * it.second));
    // Which is then passed to the normal JacobianFactor constructor
    JacobianFactor::fillTerms(QF, Q * b, model);
  }
};
// end class JacobianFactorQ

// traits
template<size_t D, size_t ZDim> struct traits<JacobianFactorQ<D, ZDim> > : public Testable<
    JacobianFactorQ<D, ZDim> > {
};

}
