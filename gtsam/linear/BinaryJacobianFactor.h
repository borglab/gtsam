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
#include <gtsam/linear/VectorValues.h>
#include <gtsam/base/SymmetricBlockMatrix.h>
#include <gtsam/base/timing.h>

#include <algorithm>

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

  /** Add the Hessian diagonal using fixed-size matrix operations. */
  void hessianDiagonalAdd(VectorValues& diagonal) const override {
    const SharedDiagonal& model = get_model();
    if (model && !model->isUnit()) {
      JacobianFactor::hessianDiagonalAdd(diagonal);
      return;
    }

    const Matrix& Ab = Ab_.matrix();
    const Eigen::Block<const Matrix, M, N1> A1(Ab, 0, 0);
    const Eigen::Block<const Matrix, M, N2> A2(Ab, 0, N1);
    const Eigen::Matrix<double, N1, 1> diagonal1 =
        A1.colwise().squaredNorm().transpose();
    const Eigen::Matrix<double, N2, 1> diagonal2 =
        A2.colwise().squaredNorm().transpose();

    const auto addDiagonal = [&diagonal](Key key, const auto& contribution) {
      auto result = diagonal.emplace(key, contribution.rows());
      if (result.second) {
        result.first->second = contribution;
      } else {
        result.first->second += contribution;
      }
    };
    addDiagonal(key1(), diagonal1);
    addDiagonal(key2(), diagonal2);
  }

  /** Compute the error change using a fixed-size residual. */
  double deltaError(const VectorValues& values, double* oldError = nullptr,
                    double* newError = nullptr) const override {
    const SharedDiagonal& model = get_model();
    if (model && !model->isUnit()) {
      return JacobianFactor::deltaError(values, oldError, newError);
    }

    const Matrix& Ab = Ab_.matrix();
    const Eigen::Block<const Matrix, M, N1> A1(Ab, 0, 0);
    const Eigen::Block<const Matrix, M, N2> A2(Ab, 0, N1);
    const Eigen::Block<const Matrix, M, 1> b(Ab, 0, N1 + N2);

    Eigen::Matrix<double, M, 1> error = -b;
    error.noalias() += A1 * values.at(key1());
    error.noalias() += A2 * values.at(key2());

    const double oldValue = 0.5 * b.squaredNorm();
    const double newValue = 0.5 * error.squaredNorm();
    if (oldError) *oldError = oldValue;
    if (newError) *newError = newValue;
    return oldValue - newValue;
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
      BinaryJacobianFactor whitenedFactor(
          key1(), model->Whiten(getA(begin())), key2(),
          model->Whiten(getA(begin() + 1)), model->whiten(getb()));
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
      if constexpr (N1 == 1) {
        info->updateDiagonalBlock(slot1, A1.transpose() * A1);
      } else {
        info->diagonalBlock(slot1).rankUpdate(A1.transpose());
      }
      info->updateOffDiagonalBlock(slot1, slot2, A1.transpose() * A2);
      info->updateOffDiagonalBlock(slot1, slotB, A1.transpose() * b);
      if constexpr (N2 == 1) {
        info->updateDiagonalBlock(slot2, A2.transpose() * A2);
      } else {
        info->diagonalBlock(slot2).rankUpdate(A2.transpose());
      }
      info->updateOffDiagonalBlock(slot2, slotB, A2.transpose() * b);
      info->updateDiagonalBlock(slotB, b.transpose() * b);
    }
  }

  /** Update a range of Hessian block columns using fixed-size operations. */
  void updateHessian(const KeyVector& infoKeys, SymmetricBlockMatrix* info,
                     DenseIndex beginCol, DenseIndex endCol) const override {
    gttic(updateHessianRange_BinaryJacobianFactor);
    const SharedDiagonal& model = get_model();
    if (model && !model->isUnit()) {
      JacobianFactor::updateHessian(infoKeys, info, beginCol, endCol);
      return;
    }

    const DenseIndex slot1 = Slot(infoKeys, key1());
    const DenseIndex slot2 = Slot(infoKeys, key2());
    const DenseIndex slotB = info->nBlocks() - 1;
    const auto ownsColumn = [beginCol, endCol](DenseIndex column) {
      return column >= beginCol && column < endCol;
    };

    const Matrix& Ab = Ab_.matrix();
    const Eigen::Block<const Matrix, M, N1> A1(Ab, 0, 0);
    const Eigen::Block<const Matrix, M, N2> A2(Ab, 0, N1);
    const Eigen::Block<const Matrix, M, 1> b(Ab, 0, N1 + N2);

    if (ownsColumn(slot1)) {
      if constexpr (N1 == 1) {
        info->updateDiagonalBlock(slot1, A1.transpose() * A1);
      } else {
        info->diagonalBlock(slot1).rankUpdate(A1.transpose());
      }
    }
    if (ownsColumn(std::max(slot1, slot2))) {
      info->updateOffDiagonalBlock(slot1, slot2, A1.transpose() * A2);
    }
    if (ownsColumn(std::max(slot1, slotB))) {
      info->updateOffDiagonalBlock(slot1, slotB, A1.transpose() * b);
    }
    if (ownsColumn(slot2)) {
      if constexpr (N2 == 1) {
        info->updateDiagonalBlock(slot2, A2.transpose() * A2);
      } else {
        info->diagonalBlock(slot2).rankUpdate(A2.transpose());
      }
    }
    if (ownsColumn(std::max(slot2, slotB))) {
      info->updateOffDiagonalBlock(slot2, slotB, A2.transpose() * b);
    }
    if (ownsColumn(slotB)) {
      info->updateDiagonalBlock(slotB, b.transpose() * b);
    }
  }
};

template<int M, int N1, int N2>
struct traits<BinaryJacobianFactor<M, N1, N2> > : Testable<
    BinaryJacobianFactor<M, N1, N2> > {
};

} //namespace gtsam
