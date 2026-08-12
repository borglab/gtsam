/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010-2026, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file TernaryJacobianFactor.h
 * @brief Fixed-size specialization of a three-key JacobianFactor.
 * @author Frank Dellaert
 */

#pragma once

#include <gtsam/base/timing.h>
#include <gtsam/linear/JacobianFactor.h>
#include <gtsam/linear/internal/FixedJacobianFactorOps.h>

#include <algorithm>

namespace gtsam {

/** A ternary JacobianFactor specialization using fixed-size matrix products. */
template <int M, int N1, int N2, int N3>
struct TernaryJacobianFactor : JacobianFactor {
  /// Construct from three fixed-size Jacobian blocks and a right-hand side.
  TernaryJacobianFactor(Key key1, const Eigen::Matrix<double, M, N1>& A1,
                        Key key2, const Eigen::Matrix<double, M, N2>& A2,
                        Key key3, const Eigen::Matrix<double, M, N3>& A3,
                        const Eigen::Matrix<double, M, 1>& b,
                        const SharedDiagonal& model = SharedDiagonal())
      : JacobianFactor(key1, A1, key2, A2, key3, A3, b, model) {}

  Key key1() const { return keys_[0]; }
  Key key2() const { return keys_[1]; }
  Key key3() const { return keys_[2]; }

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
    const Eigen::Block<const Matrix, M, N3> A3(Ab, 0, N1 + N2);
    internal::accumulateHessianDiagonal<M, N1>(key1(), A1, diagonal);
    internal::accumulateHessianDiagonal<M, N2>(key2(), A2, diagonal);
    internal::accumulateHessianDiagonal<M, N3>(key3(), A3, diagonal);
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
    const Eigen::Block<const Matrix, M, N3> A3(Ab, 0, N1 + N2);
    const Eigen::Block<const Matrix, M, 1> b(Ab, 0, N1 + N2 + N3);

    Eigen::Matrix<double, M, 1> error = -b;
    internal::accumulateResidual<M, N1>(A1, values.at(key1()), error);
    internal::accumulateResidual<M, N2>(A2, values.at(key2()), error);
    internal::accumulateResidual<M, N3>(A3, values.at(key3()), error);

    const double oldValue = 0.5 * b.squaredNorm();
    const double newValue = 0.5 * error.squaredNorm();
    if (oldError) *oldError = oldValue;
    if (newError) *newError = newValue;
    return oldValue - newValue;
  }

  /** Update the complete augmented Hessian using fixed-size products. */
  void updateHessian(const KeyVector& infoKeys,
                     SymmetricBlockMatrix* info) const override {
    gttic(updateHessian_TernaryJacobianFactor);
    const SharedDiagonal& model = get_model();
    if (model && !model->isUnit()) {
      if (model->isConstrained()) {
        throw std::invalid_argument(
            "TernaryJacobianFactor::updateHessian: cannot update information "
            "with constrained noise model");
      }
      TernaryJacobianFactor whitenedFactor(
          key1(), model->Whiten(getA(begin())), key2(),
          model->Whiten(getA(begin() + 1)), key3(),
          model->Whiten(getA(begin() + 2)), model->whiten(getb()));
      whitenedFactor.updateHessian(infoKeys, info);
      return;
    }

    const DenseIndex slot1 = Slot(infoKeys, key1());
    const DenseIndex slot2 = Slot(infoKeys, key2());
    const DenseIndex slot3 = Slot(infoKeys, key3());
    const DenseIndex slotB = info->nBlocks() - 1;
    const Matrix& Ab = Ab_.matrix();
    const Eigen::Block<const Matrix, M, N1> A1(Ab, 0, 0);
    const Eigen::Block<const Matrix, M, N2> A2(Ab, 0, N1);
    const Eigen::Block<const Matrix, M, N3> A3(Ab, 0, N1 + N2);
    const Eigen::Block<const Matrix, M, 1> b(Ab, 0, N1 + N2 + N3);

    internal::updateJacobianHessian<M, N1>(slot1, A1, slotB, b, info);
    internal::updateCrossHessian<M, N1, N2>(slot1, A1, slot2, A2, info);
    internal::updateCrossHessian<M, N1, N3>(slot1, A1, slot3, A3, info);
    internal::updateJacobianHessian<M, N2>(slot2, A2, slotB, b, info);
    internal::updateCrossHessian<M, N2, N3>(slot2, A2, slot3, A3, info);
    internal::updateJacobianHessian<M, N3>(slot3, A3, slotB, b, info);
    internal::updateSelfHessian<M, 1>(slotB, b, info);
  }

  /** Update selected augmented-Hessian block columns. */
  void updateHessian(const KeyVector& infoKeys, SymmetricBlockMatrix* info,
                     DenseIndex beginCol, DenseIndex endCol) const override {
    gttic(updateHessianRange_TernaryJacobianFactor);
    const SharedDiagonal& model = get_model();
    if (model && !model->isUnit()) {
      JacobianFactor::updateHessian(infoKeys, info, beginCol, endCol);
      return;
    }

    const DenseIndex slot1 = Slot(infoKeys, key1());
    const DenseIndex slot2 = Slot(infoKeys, key2());
    const DenseIndex slot3 = Slot(infoKeys, key3());
    const DenseIndex slotB = info->nBlocks() - 1;
    const auto ownsColumn = [beginCol, endCol](DenseIndex column) {
      return column >= beginCol && column < endCol;
    };
    const Matrix& Ab = Ab_.matrix();
    const Eigen::Block<const Matrix, M, N1> A1(Ab, 0, 0);
    const Eigen::Block<const Matrix, M, N2> A2(Ab, 0, N1);
    const Eigen::Block<const Matrix, M, N3> A3(Ab, 0, N1 + N2);
    const Eigen::Block<const Matrix, M, 1> b(Ab, 0, N1 + N2 + N3);

    if (ownsColumn(slot1)) {
      internal::updateSelfHessian<M, N1>(slot1, A1, info);
    }
    if (ownsColumn(std::max(slot1, slot2))) {
      internal::updateCrossHessian<M, N1, N2>(slot1, A1, slot2, A2, info);
    }
    if (ownsColumn(std::max(slot1, slot3))) {
      internal::updateCrossHessian<M, N1, N3>(slot1, A1, slot3, A3, info);
    }
    if (ownsColumn(std::max(slot1, slotB))) {
      internal::updateCrossHessian<M, N1, 1>(slot1, A1, slotB, b, info);
    }
    if (ownsColumn(slot2)) {
      internal::updateSelfHessian<M, N2>(slot2, A2, info);
    }
    if (ownsColumn(std::max(slot2, slot3))) {
      internal::updateCrossHessian<M, N2, N3>(slot2, A2, slot3, A3, info);
    }
    if (ownsColumn(std::max(slot2, slotB))) {
      internal::updateCrossHessian<M, N2, 1>(slot2, A2, slotB, b, info);
    }
    if (ownsColumn(slot3)) {
      internal::updateSelfHessian<M, N3>(slot3, A3, info);
    }
    if (ownsColumn(std::max(slot3, slotB))) {
      internal::updateCrossHessian<M, N3, 1>(slot3, A3, slotB, b, info);
    }
    if (ownsColumn(slotB)) {
      internal::updateSelfHessian<M, 1>(slotB, b, info);
    }
  }
};

template <int M, int N1, int N2, int N3>
struct traits<TernaryJacobianFactor<M, N1, N2, N3>>
    : Testable<TernaryJacobianFactor<M, N1, N2, N3>> {};

}  // namespace gtsam
