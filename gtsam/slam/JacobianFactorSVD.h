/*
 * @file  JacobianFactorSVD.h
 * @date  Oct 27, 2013
 * @uthor Frank Dellaert
 */

#pragma once
#include <gtsam/linear/RegularJacobianFactor.h>

namespace gtsam {
/**
 * JacobianFactor for Schur complement that uses the "Nullspace Trick" by Mourikis
 *
 * This trick is equivalent to the Schur complement, but can be faster.
 * In essence, the linear factor |E*dp + F*dX - b|, where p is point and X are poses,
 * is multiplied by Enull, a matrix that spans the left nullspace of E, i.e.,
 * The mx3 matrix is analyzed with SVD as E = [Erange Enull]*S*V (mxm * mx3 * 3x3)
 * where Enull is an m x (m-3) matrix
 * Then Enull'*E*dp = 0, and
 *  |Enull'*E*dp + Enull'*F*dX - Enull'*b| == |Enull'*F*dX - Enull'*b|
 * Normally F is m x 6*numKeys, and Enull'*F yields an (m-3) x 6*numKeys matrix.
 *
 * The code below assumes that F is block diagonal and is given as a vector of ZDim*D blocks.
 * Example: m = 4 (2 measurements), Enull = 4*1, F = 4*12 (for D=6)
 * Then Enull'*F = 1*4 * 4*12 = 1*12, but each 1*6 piece can be computed as a 1x2 * 2x6 mult
 */
template<size_t D, size_t ZDim>
class JacobianFactorSVD: public RegularJacobianFactor<D> {

  typedef RegularJacobianFactor<D> Base;
  typedef Eigen::Matrix<double, ZDim, D> MatrixZD; // e.g 2 x 6 with Z=Point2
  typedef std::pair<Key, Matrix> KeyMatrix;

public:

  /// Default constructor
  JacobianFactorSVD() {
  }

  /// Empty constructor with keys
  JacobianFactorSVD(const KeyVector& keys, //
      const SharedDiagonal& model = SharedDiagonal()) :
      Base() {
    Matrix zeroMatrix = Matrix::Zero(0, D);
    Vector zeroVector = Vector::Zero(0);
    std::vector<KeyMatrix> QF;
    QF.reserve(keys.size());
    for(const Key& key: keys)
      QF.push_back(KeyMatrix(key, zeroMatrix));
    JacobianFactor::fillTerms(QF, zeroVector, model);
  }

  /**
   * @brief Constructor
   * Takes the CameraSet derivatives (as ZDim*D blocks of block-diagonal F)
   * and a reduced point derivative, Enull
   * and creates a reduced-rank Jacobian factor on the CameraSet
   *
   * @Fblocks:
   */
  JacobianFactorSVD(const KeyVector& keys,
      const std::vector<MatrixZD, Eigen::aligned_allocator<MatrixZD> >& Fblocks, const Matrix& Enull,
      const Vector& b, //
      const SharedDiagonal& model = SharedDiagonal()) :
      Base() {
    size_t numKeys = Enull.rows() / ZDim;
    size_t m2 = ZDim * numKeys - 3; // TODO: is this not just Enull.rows()?
    // PLAIN nullptr SPACE TRICK
    // Matrix Q = Enull * Enull.transpose();
    // for(const KeyMatrixZD& it: Fblocks)
    //   QF.push_back(KeyMatrix(it.first, Q.block(0, 2 * j++, m2, 2) * it.second));
    // JacobianFactor factor(QF, Q * b);
    std::vector<KeyMatrix> QF;
    QF.reserve(numKeys);
    for (size_t k = 0; k < Fblocks.size(); ++k) {
      Key key = keys[k];
      QF.push_back(
          KeyMatrix(key,
              (Enull.transpose()).block(0, ZDim * k, m2, ZDim) * Fblocks[k]));
    }
    JacobianFactor::fillTerms(QF, Enull.transpose() * b, model);
  }
};

}
