/* ----------------------------------------------------------------------------
 * GTSAM Copyright 2010-2024, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/*
 * @file TransferFactor.h
 * @brief TransferFactor class
 * @author Frank Dellaert
 * @date October 24, 2024
 */

#pragma once

#include <gtsam/base/numericalDerivative.h>
#include <gtsam/geometry/FundamentalMatrix.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

namespace gtsam {

template <typename F>
struct TripletError {
  Point2 p0, p1, p2;

  /// vector of errors returns 6D vector
  Vector evaluateError(const F& F01, const F& F12, const F& F20,  //
                       Matrix* H01, Matrix* H12, Matrix* H20) const {
    std::function<Vector6(F, F, F)> fn = [&](const F& F01, const F& F12,
                                             const F& F20) {
      Vector6 error;
      error <<  //
          F::transfer(F01.matrix(), p1, F20.matrix().transpose(), p2) - p0,
          F::transfer(F01.matrix().transpose(), p0, F12.matrix(), p2) - p1,
          F::transfer(F20.matrix(), p0, F12.matrix().transpose(), p1) - p2;
      return error;
    };
    if (H01) *H01 = numericalDerivative31<Vector6, F, F, F>(fn, F01, F12, F20);
    if (H12) *H12 = numericalDerivative32<Vector6, F, F, F>(fn, F01, F12, F20);
    if (H20) *H20 = numericalDerivative33<Vector6, F, F, F>(fn, F01, F12, F20);
    return fn(F01, F12, F20);
  }
};

template <typename F>
class TransferFactor : public NoiseModelFactorN<F, F> {
  Point2 p0, p1, p2;

 public:
  // Constructor
  TransferFactor(Key key1, Key key2, const Point2& p0, const Point2& p1,
                 const Point2& p2, const SharedNoiseModel& model = nullptr)
      : NoiseModelFactorN<F, F>(model, key1, key2), p0(p0), p1(p1), p2(p2) {}

  /// vector of errors returns 2D vector
  Vector evaluateError(const F& F12, const F& F20,  //
                       OptionalMatrixType H12 = nullptr,
                       OptionalMatrixType H20 = nullptr) const override {
    std::function<Vector2(F, F)> fn = [&](const F& F12, const F& F20) {
      Vector2 error;
      error <<  //
          F::transfer(F20.matrix(), p0, F12.matrix().transpose(), p1) - p2;
      return error;
    };
    if (H12) *H12 = numericalDerivative21<Vector2, F, F>(fn, F12, F20);
    if (H20) *H20 = numericalDerivative22<Vector2, F, F>(fn, F12, F20);
    return fn(F12, F20);
  }
};

}  // namespace gtsam
