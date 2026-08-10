/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010-2026, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file BetweenJacobianFactor.h
 * @brief Fixed-size Jacobian factor exploiting BetweenFactor structure.
 */

#pragma once

#include <gtsam/linear/BinaryJacobianFactor.h>

#include <algorithm>

namespace gtsam {

/**
 * A fixed-size binary Jacobian factor whose second Jacobian originated as the
 * identity in a BetweenFactor. After whitening, that block remains identity for
 * unit noise, diagonal for diagonal noise, and upper triangular for a full
 * Gaussian square-root information matrix. Those cases admit structured
 * Hessian updates; unrecognized structure falls back to BinaryJacobianFactor.
 */
template <int N>
struct BetweenJacobianFactor : BinaryJacobianFactor<N, N, N> {
  static_assert(N > 0, "BetweenJacobianFactor requires a fixed dimension");

  using Base = BinaryJacobianFactor<N, N, N>;
  using MatrixN = Eigen::Matrix<double, N, N>;
  using VectorN = Eigen::Matrix<double, N, 1>;

  BetweenJacobianFactor(Key key1, const MatrixN& A1, Key key2,
                        const MatrixN& A2, const VectorN& b,
                        const SharedDiagonal& model = SharedDiagonal())
      : Base(key1, A1, key2, A2, b, model),
        secondBlockStructure_(classify(A2)) {}

  GaussianFactor::shared_ptr clone() const override {
    return std::make_shared<BetweenJacobianFactor>(*this);
  }

  void updateHessian(const KeyVector& infoKeys,
                     SymmetricBlockMatrix* info) const override {
    gttic(updateHessian_BetweenJacobianFactor);
    if (!canUseStructuredUpdate()) {
      Base::updateHessian(infoKeys, info);
      return;
    }
    updateStructuredHessian(infoKeys, info, 0, info->nBlocks());
  }

  void updateHessian(const KeyVector& infoKeys, SymmetricBlockMatrix* info,
                     DenseIndex beginCol, DenseIndex endCol) const override {
    gttic(updateHessianRange_BetweenJacobianFactor);
    if (!canUseStructuredUpdate()) {
      Base::updateHessian(infoKeys, info, beginCol, endCol);
      return;
    }
    updateStructuredHessian(infoKeys, info, beginCol, endCol);
  }

 private:
  enum class SecondBlockStructure {
    Identity,
    Diagonal,
    UpperTriangular,
    General
  };

  SecondBlockStructure secondBlockStructure_;

  static SecondBlockStructure classify(const MatrixN& A2) {
    if (A2.isIdentity(0.0)) return SecondBlockStructure::Identity;
    if (A2.isDiagonal(0.0)) return SecondBlockStructure::Diagonal;
    bool upperTriangular = true;
    for (DenseIndex column = 0; column < N && upperTriangular; ++column) {
      for (DenseIndex row = column + 1; row < N; ++row) {
        if (A2(row, column) != 0.0) {
          upperTriangular = false;
          break;
        }
      }
    }
    if (upperTriangular) return SecondBlockStructure::UpperTriangular;
    return SecondBlockStructure::General;
  }

  bool canUseStructuredUpdate() const {
    const SharedDiagonal& model = this->get_model();
    return (!model || model->isUnit()) &&
           secondBlockStructure_ != SecondBlockStructure::General;
  }

  void updateStructuredHessian(const KeyVector& infoKeys,
                               SymmetricBlockMatrix* info, DenseIndex beginCol,
                               DenseIndex endCol) const {
    const DenseIndex slot1 = this->Slot(infoKeys, this->key1());
    const DenseIndex slot2 = this->Slot(infoKeys, this->key2());
    const DenseIndex slotB = info->nBlocks() - 1;
    const auto ownsColumn = [beginCol, endCol](DenseIndex column) {
      return column >= beginCol && column < endCol;
    };

    const Matrix& Ab = this->Ab_.matrix();
    const Eigen::Block<const Matrix, N, N> A1(Ab, 0, 0);
    const Eigen::Block<const Matrix, N, N> A2(Ab, 0, N);
    const Eigen::Block<const Matrix, N, 1> b(Ab, 0, 2 * N);

    if (ownsColumn(slot1)) {
      if constexpr (N == 1) {
        info->updateDiagonalBlock(slot1, A1.transpose() * A1);
      } else {
        info->diagonalBlock(slot1).rankUpdate(A1.transpose());
      }
    }
    if (ownsColumn(std::max(slot1, slot2))) {
      if constexpr (N == 1) {
        info->updateOffDiagonalBlock(slot1, slot2, A1.transpose() * A2);
      } else if (secondBlockStructure_ == SecondBlockStructure::Identity) {
        info->updateOffDiagonalBlock(slot1, slot2, A1.transpose());
      } else if (secondBlockStructure_ == SecondBlockStructure::Diagonal) {
        info->updateOffDiagonalBlock(
            slot1, slot2, A1.transpose() * A2.diagonal().asDiagonal());
      } else {
        const MatrixN contribution =
            A1.transpose() * A2.template triangularView<Eigen::Upper>();
        info->updateOffDiagonalBlock(slot1, slot2, contribution);
      }
    }
    if (ownsColumn(std::max(slot1, slotB))) {
      info->updateOffDiagonalBlock(slot1, slotB, A1.transpose() * b);
    }
    if (ownsColumn(slot2)) {
      if constexpr (N == 1) {
        info->updateDiagonalBlock(slot2, A2.transpose() * A2);
      } else if (secondBlockStructure_ == SecondBlockStructure::Identity) {
        info->addScaledIdentity(slot2, 1.0);
      } else if (secondBlockStructure_ == SecondBlockStructure::Diagonal) {
        const MatrixN contribution =
            A2.diagonal().cwiseAbs2().matrix().asDiagonal();
        info->updateDiagonalBlock(slot2, contribution);
      } else {
        info->diagonalBlock(slot2).rankUpdate(A2.transpose());
      }
    }
    if (ownsColumn(std::max(slot2, slotB))) {
      if constexpr (N == 1) {
        info->updateOffDiagonalBlock(slot2, slotB, A2.transpose() * b);
      } else if (secondBlockStructure_ == SecondBlockStructure::Identity) {
        info->updateOffDiagonalBlock(slot2, slotB, b);
      } else if (secondBlockStructure_ == SecondBlockStructure::Diagonal) {
        info->updateOffDiagonalBlock(slot2, slotB,
                                     A2.diagonal().cwiseProduct(b));
      } else {
        const VectorN contribution =
            A2.transpose().template triangularView<Eigen::Lower>() * b;
        info->updateOffDiagonalBlock(slot2, slotB, contribution);
      }
    }
    if (ownsColumn(slotB)) {
      info->updateDiagonalBlock(slotB, b.transpose() * b);
    }
  }
};

template <int N>
struct traits<BetweenJacobianFactor<N>> : Testable<BetweenJacobianFactor<N>> {};

}  // namespace gtsam
