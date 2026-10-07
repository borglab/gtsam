/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010-2026, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file KnownLandmarkSquaredRangeFactor.h
 * @brief Factors for range-only observations of a knownlandmark from an unknown position.
 * @author Avinash Subramanian
 * @author Frederike Dümbgen
 * @author Frank Dellaert
 */


#pragma once

#include <gtsam/nonlinear/NoiseModelFactorN.h>

#include <Eigen/Core>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <vector>

namespace gtsam {

/**
 * A unary known-landmark factor using the conventional GTSAM state direction.
 *
 * For a position wX, the residual is
 *
 *   e(wX) = (measuredRange_)^2 - (wX - wL)^2
 *
 * The known world landmark is fixed data, and the supplied noise model
 * represents the measurement weight. 
 *
 * See `gtsam/slam/doc/KnownLandmarkSquaredRangeFactor.ipynb` for more details regarding usage.
 */
template <int N>
class KnownLandmarkSquaredRangeFactor : public NoiseModelFactorN<Eigen::Matrix<double, N, 1>> {
 public:
  static_assert(N > 0, "KnownLandmarkSquaredRangeFactor requires N > 0");
  using Point = Eigen::Matrix<double, N, 1>;
  using Base = NoiseModelFactorN<Point>;
  using This = KnownLandmarkSquaredRangeFactor<N>;


 private:
  Point wL_;              ///< Known landmark expressed in the world frame.
  double measuredRange_;  ///< Corresponding range measurement of landmark position wL_

 public:
  using Base::evaluateError;

  /// Construct from a key, world landmark, measurement, and noise model.
  KnownLandmarkSquaredRangeFactor(Key key, const Point& wL, double measuredRange,
                      const SharedNoiseModel& model)
      : Base(model, key), wL_(wL), measuredRange_(measuredRange) {}

  NonlinearFactor::shared_ptr clone() const override {
    return std::make_shared<This>(*this);
  }

  bool equals(const NonlinearFactor& expected,
              double tol = 1e-9) const override {
    const auto* e = dynamic_cast<const This*>(&expected);
    return e != nullptr && Base::equals(*e, tol) &&
           traits<Point>::Equals(wL_, e->wL_, tol) &&
           traits<double>::Equals(measuredRange_, e->measuredRange_, tol);
  }

  /** Evaluate squared predicted range minus squared measured range */
  Vector evaluateError(const Point& wX, OptionalMatrixType H) const override {
    const Point difference = wX - wL_;
    const double predictedSquaredRange = difference.squaredNorm();

    cost double error = predictedSquaredRange - measuredRange_ * measuredRange_;
    if (H) {
      *H = 2.0 * difference.transpose();
    }
    return Vector1(error);
  }
};


}  // namespace gtsam
