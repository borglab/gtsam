/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file BinaryJacobianFactor.h
 *
 * @brief A binary JacobianFactor specialization that uses fixed matrix math for speed
 *
 * @date June 2015
 * @author Frank Dellaert
 */

#pragma once

#include <gtsam/linear/JacobianFactor.h>
#include <gtsam/base/SymmetricBlockMatrix.h>
#include <gtsam/base/timing.h>

namespace gtsam {

/**
 * A binary JacobianFactor specialization that uses fixed matrix math for speed
 */
template<int M, int N1, int N2>
struct BinaryJacobianFactor: JacobianFactor {

  /// Constructor
  BinaryJacobianFactor(Key key1, const Eigen::Matrix<double, M, N1>& A1,
      Key key2, const Eigen::Matrix<double, M, N2>& A2,
      const Eigen::Matrix<double, M, 1>& b, //
      const SharedDiagonal& model = SharedDiagonal()) :
      JacobianFactor(key1, A1, key2, A2, b, model) {
  }

  inline Key key1() const {
    return keys_[0];
  }
  inline Key key2() const {
    return keys_[1];
  }

  // Fixed-size matrix update
  void updateHessian(const KeyVector& infoKeys,
      SymmetricBlockMatrix* info) const override {
    gttic(updateHessian_BinaryJacobianFactor);
    // Whiten the factor if it has a noise model
    const SharedDiagonal& model = get_model();
    if (model && !model->isUnit()) {
      if (model->isConstrained())
        throw std::invalid_argument(
            "BinaryJacobianFactor::updateHessian: cannot update information with "
                "constrained noise model");
      BinaryJacobianFactor whitenedFactor(key1(), model->Whiten(getA(begin())),
          key2(), model->Whiten(getA(end())), model->whiten(getb()));
      whitenedFactor.updateHessian(infoKeys, info);
    } else {
      // First build an array of slots
      DenseIndex slot1 = Slot(infoKeys, key1());
      DenseIndex slot2 = Slot(infoKeys, key2());
      DenseIndex slotB = info->nBlocks() - 1;

      const Matrix& Ab = Ab_.matrix();
      Eigen::Block<const Matrix, M, N1> A1(Ab, 0, 0);
      Eigen::Block<const Matrix, M, N2> A2(Ab, 0, N1);
      Eigen::Block<const Matrix, M, 1> b(Ab, 0, N1 + N2);

      // We perform I += A'*A to the upper triangle
      info->diagonalBlock(slot1).rankUpdate(A1.transpose());
      info->updateOffDiagonalBlock(slot1, slot2, A1.transpose() * A2);
      info->updateOffDiagonalBlock(slot1, slotB, A1.transpose() * b);
      info->diagonalBlock(slot2).rankUpdate(A2.transpose());
      info->updateOffDiagonalBlock(slot2, slotB, A2.transpose() * b);
      info->updateDiagonalBlock(slotB, b.transpose() * b);
    }
  }
};

template<int M, int N1, int N2>
struct traits<BinaryJacobianFactor<M, N1, N2> > : Testable<
    BinaryJacobianFactor<M, N1, N2> > {
};

} //namespace gtsam
