/*
 * @file  JacobianFactorQ.h
 * @date  Oct 27, 2013
 * @uthor Frank Dellaert
 */

#pragma once

#include "JacobianSchurFactor.h"

namespace gtsam {
/**
 * JacobianFactor for Schur complement that uses Q noise model
 */
template<size_t D>
class JacobianFactorQ: public JacobianSchurFactor<D> {

  typedef JacobianSchurFactor<D> Base;

public:

  /// Default constructor
  JacobianFactorQ() {
  }

  /// Empty constructor with keys
  JacobianFactorQ(const FastVector<Key>& keys,
      const SharedDiagonal& model =  SharedDiagonal()) : JacobianSchurFactor<D>() {
    Matrix zeroMatrix = Matrix::Zero(0,D);
    Vector zeroVector = Vector::Zero(0);
    typedef std::pair<Key, Matrix> KeyMatrix;
    std::vector<KeyMatrix> QF;
    QF.reserve(keys.size());
    BOOST_FOREACH(const Key& key, keys)
            QF.push_back(KeyMatrix(key, zeroMatrix));
    JacobianFactor::fillTerms(QF, zeroVector, model);
  }

  /// Constructor
  JacobianFactorQ(const std::vector<typename Base::KeyMatrix2D>& Fblocks,
      const Matrix& E, const Matrix3& P, const Vector& b,
      const SharedDiagonal& model = SharedDiagonal()) :
      JacobianSchurFactor<D>() {
    size_t j = 0, m2 = E.rows(), m = m2 / 2;
    // Calculate projector Q
    Matrix Q = eye(m2) - E * P * E.transpose();
    // Calculate pre-computed Jacobian matrices
    // TODO: can we do better ?
    typedef std::pair<Key, Matrix> KeyMatrix;
    std::vector < KeyMatrix > QF;
    QF.reserve(m);
    // Below, we compute each 2m*D block A_j = Q_j * F_j = (2m*2) * (2*D)
    BOOST_FOREACH(const typename Base::KeyMatrix2D& it, Fblocks)
      QF.push_back(KeyMatrix(it.first, Q.block(0, 2 * j++, m2, 2) * it.second));
    // Which is then passed to the normal JacobianFactor constructor
    JacobianFactor::fillTerms(QF, Q * b, model);
  }
};

}
