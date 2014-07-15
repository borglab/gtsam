/*
 * @file  JacobianFactorSVD.h
 * @date  Oct 27, 2013
 * @uthor Frank Dellaert
 */

#pragma once
#include "gtsam/slam/JacobianSchurFactor.h"

namespace gtsam {
/**
 * JacobianFactor for Schur complement that uses Q noise model
 */
template<size_t D, class Z>
class JacobianFactorSVD: public JacobianSchurFactor<D> {

public:

  typedef Eigen::Matrix<double, Z::dimension, D> Matrix2D;   // e.g 2 x 6 with Z=Point2
  typedef std::pair<Key, Matrix2D> KeyMatrix2D;
  typedef std::pair<Key, Matrix> KeyMatrix;

  /// Default constructor
  JacobianFactorSVD() {}

  /// Empty constructor with keys
  JacobianFactorSVD(const FastVector<Key>& keys,
      const SharedDiagonal& model =  SharedDiagonal()) : JacobianSchurFactor<D>() {
    Matrix zeroMatrix = Matrix::Zero(0,D);
    Vector zeroVector = Vector::Zero(0);
    std::vector<KeyMatrix> QF;
    QF.reserve(keys.size());
    BOOST_FOREACH(const Key& key, keys)
            QF.push_back(KeyMatrix(key, zeroMatrix));
    JacobianFactor::fillTerms(QF, zeroVector, model);
  }

  /// Constructor
  JacobianFactorSVD(const std::vector<KeyMatrix2D,Eigen::aligned_allocator<KeyMatrix2D> >& Fblocks, const Matrix& Enull, const Vector& b,
      const SharedDiagonal& model =  SharedDiagonal()) : JacobianSchurFactor<D>() {
    size_t numKeys = Enull.rows() / Z::Dim();
    size_t j = 0, m2 = Z::Dim()*numKeys-3;
    // PLAIN NULL SPACE TRICK
    // Matrix Q = Enull * Enull.transpose();
    // BOOST_FOREACH(const KeyMatrix2D& it, Fblocks)
    //   QF.push_back(KeyMatrix(it.first, Q.block(0, 2 * j++, m2, 2) * it.second));
    // JacobianFactor factor(QF, Q * b);
    std::vector<KeyMatrix> QF;
    QF.reserve(numKeys);
    BOOST_FOREACH(const KeyMatrix2D& it, Fblocks)
    QF.push_back(KeyMatrix(it.first, (Enull.transpose()).block(0, Z::Dim() * j++, m2, Z::Dim()) * it.second));
    JacobianFactor::fillTerms(QF, Enull.transpose() * b, model);
  }
};

}
