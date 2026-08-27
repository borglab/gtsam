/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010-2026, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file WahbaFactor.h
 * @brief Chordal Wahba factor with exact QCQP conversion.
 * @author Avinash Subramanian
 * @author Frederike Dümbgen
 * @author Frank Dellaert
 */

#pragma once

#include <gtsam/constrained/QcqpProblem.h>
#include <gtsam/constrained/QpCost.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/Unit3.h>
#include <gtsam/nonlinear/NoiseModelFactorN.h>

#include <memory>
#include <stdexcept>
#include <vector>

namespace gtsam {

/**
 * A chordal Wahba factor relating two directions through an unknown rotation.
 *
 * For a rotation aRb that maps frame-b coordinates into frame a, the
 * unwhitened residual is
 *
 *   e(aRb) = aRb * bDirection - measured_aDirection.
 *
 * The directions are stored as Unit3 values, while the residual is the
 * three-dimensional difference between their unit vectors. This is the
 * classical chordal Wahba objective and is affine in the entries of aRb, so it
 * supports an exact D=1 QCQP conversion. RotateDirectionsFactor instead uses a
 * two-dimensional Unit3 tangent-space error and a different nonlinear model.
 *
 * See `gtsam/slam/doc/WahbaFactor.ipynb` for a comparison of the two residuals
 * and a runnable QCQP objective check.
 */
class WahbaFactor : public NoiseModelFactorN<Rot3> {
 public:
  using Base = NoiseModelFactorN<Rot3>;
  using This = WahbaFactor;

 private:
  Unit3 bDirection_;           ///< Known direction expressed in frame b.
  Unit3 measured_aDirection_;  ///< Corresponding direction measured in frame a.

 public:
  using Base::evaluateError;

  /// Construct from a key, two directions, and a three-dimensional noise model.
  WahbaFactor(Key key, const Unit3& bDirection,
              const Unit3& measured_aDirection, const SharedNoiseModel& model)
      : Base(model, key),
        bDirection_(bDirection),
        measured_aDirection_(measured_aDirection) {}

  NonlinearFactor::shared_ptr clone() const override {
    return std::make_shared<This>(*this);
  }

  bool equals(const NonlinearFactor& expected,
              double tol = 1e-9) const override {
    const auto* e = dynamic_cast<const This*>(&expected);
    return e != nullptr && Base::equals(*e, tol) &&
           bDirection_.equals(e->bDirection_, tol) &&
           measured_aDirection_.equals(e->measured_aDirection_, tol);
  }

  /// Evaluate the rotated frame-b direction minus measured_aDirection.
  Vector evaluateError(const Rot3& aRb, OptionalMatrixType H) const override {
    const Point3 predicted_aDirection = aRb.rotate(bDirection_.unitVector(), H);
    return predicted_aDirection - measured_aDirection_.unitVector();
  }

  /** Add the exact D=1 chordal Wahba cost to a QCQP. */
  void qcqpFactors(NonlinearFactorGraph* costs,
                   NonlinearEqualityConstraints* constraints,
                   size_t columnDimension = 1) const override {
    if (columnDimension != 1) {
      throw std::invalid_argument(
          "WahbaFactor::qcqpFactors only supports column dimension 1");
    }
    if (!costs) {
      throw std::invalid_argument("WahbaFactor::qcqpFactors costs is null");
    }
    if (!this->noiseModel_ ||
        std::dynamic_pointer_cast<noiseModel::Robust>(this->noiseModel_) ||
        this->noiseModel_->isConstrained()) {
      throw std::runtime_error(
          "WahbaFactor::qcqpFactors requires a non-null, non-robust/non-hard "
          "quadratic noise model");
    }

    constexpr int PointDim = 3;
    constexpr int N = 3;
    constexpr int LiftedDim = 1 + PointDim * N;

    Matrix B = Matrix::Zero(PointDim, LiftedDim);
    B.col(0) = -measured_aDirection_.unitVector();
    const Point3 bDirection = bDirection_.unitVector();
    for (int column = 0; column < N; ++column) {
      B.block(0, 1 + column * PointDim, PointDim, PointDim)
          .diagonal()
          .setConstant(bDirection(column));
    }

    const Matrix whitenedB = this->noiseModel_->Whiten(B);
    const Matrix Q = whitenedB.transpose() * whitenedB;

    InsertQcqpConstraints<Rot3, 1>(this->key(), constraints);
    const SymmetricBlockMatrix blockQ(std::vector<DenseIndex>{LiftedDim}, Q);
    costs->push_back(std::make_shared<QpCost>(KeyVector{this->key()}, blockQ));
  }
};

}  // namespace gtsam
