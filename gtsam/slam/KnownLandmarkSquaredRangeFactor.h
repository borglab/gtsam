/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010-2026, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file KnownLandmarkSquaredRangeFactor.h
 * @brief Factor for range-only observation of a known landmark from an unknown position.
 * @author Avinash Subramanian
 * @author Frederike Dümbgen
 * @author Frank Dellaert
 */


#pragma once

#include <gtsam/nonlinear/NoiseModelFactorN.h>

#include <stdexcept>
#include <Eigen/Core>
#include <memory>

namespace gtsam {

/**
 * A unary known-landmark factor using a squared-range residual.
 *
 * For a position wX, known world landmark wL, and measured range d,
 * the residual is
 *
 *   e(wX) = ||wX - wL||^2 - d^2.
 *
 * The known world landmark is fixed data. The supplied noise model weights
 * the scalar squared-range residual.
 *
 * See `gtsam/slam/doc/KnownLandmarkSquaredRangeFactor.ipynb` for more details 
 * regarding usage.
 */

template <int N>
class KnownLandmarkSquaredRangeFactor 
    : public NoiseModelFactorN<Eigen::Matrix<double, N, 1>> {
 public:
  static_assert(N > 0, "KnownLandmarkSquaredRangeFactor requires N > 0");

  using Point = Eigen::Matrix<double, N, 1>;
  using Base = NoiseModelFactorN<Point>;
  using This = KnownLandmarkSquaredRangeFactor<N>;

 private:
  Point wL_;                    ///< Known landmark expressed in the world frame.
  double measuredRangeSquared_; ///< Corresponding squared range measurement of landmark position wL_.

 public:
  using Base::evaluateError;

/** 
* Construct from a key, world landmark, range measurement, and noise model.
* 
* @param key Key associated with the unknown position.
* @param wL Known landmark expressed in the world frame.
* @param measuredRange Measured range to the landmark.
* @param model Noise model for the scalar squared-range residual.
* @throws std::invalid_argument if measuredRange is negative. 
*/
  KnownLandmarkSquaredRangeFactor(Key key, const Point& wL, double measuredRange,
                      const SharedNoiseModel& model)
      : Base(model, key), wL_(wL), measuredRangeSquared_(measuredRange * measuredRange) {
    if (measuredRange < 0.0){
      throw std::invalid_argument(
        "KnownLandmarkSquareRangeFactor: range must be non-negative.");
    }
  }

  NonlinearFactor::shared_ptr clone() const override {
    return std::make_shared<This>(*this);
  }

  bool equals(const NonlinearFactor& expected,
              double tol = 1e-9) const override {
    const auto* e = dynamic_cast<const This*>(&expected);
    return e != nullptr && Base::equals(*e, tol) &&
           traits<Point>::Equals(wL_, e->wL_, tol) &&
           traits<double>::Equals(measuredRangeSquared_, e->measuredRangeSquared_, tol);
  }

  /** Evaluate squared predicted range minus squared measured range */
  Vector evaluateError(const Point& wX, OptionalMatrixType H) const override {
    const Point difference = wX - wL_;
    const double predictedRangeSquared = difference.squaredNorm();

    const double error = predictedRangeSquared - measuredRangeSquared_;
    if (H) {
      *H = 2.0 * difference.transpose();
    }
    return Vector1(error);
  }
};

}  // namespace gtsam
