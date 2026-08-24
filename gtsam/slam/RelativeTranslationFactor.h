/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    RelativeTranslationFactor.h
 * @brief   Quadratic translation factor: the translation part of a relative
 *          SE(d) pose measurement. Ternary factor over t_i, t_j and R_i, given
 *          the measured displacement t_ij from pose (R_i, t_i) to pose
 *          (R_j, t_j), expressed in frame i. The term it contributes, under an
 *          isotropic noise model, is
 *
 *            min    tau_ij ||t_j - t_i - R_i t_ij||^2
 *            over   t_i, t_j in R^d,  R_i in SO(d)
 *
 *          Quadratic in all three variables, and the translations are
 *          unconstrained.
 * @author  Zhexin Xu
 */

#pragma once

#include <gtsam/base/Matrix.h>
#include <gtsam/constrained/QcqpProblem.h>
#include <gtsam/constrained/QpCost.h>
#include <gtsam/geometry/Rot2.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/nonlinear/NoiseModelFactorN.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

#include <cmath>
#include <stdexcept>
#include <string>
#include <type_traits>

namespace gtsam {

template <int d>
class RelativeTranslationFactor
    : public NoiseModelFactorN<
          typename std::conditional<d == 2, Rot2, Rot3>::type,
          Eigen::Matrix<double, d, 1>, Eigen::Matrix<double, d, 1>> {
  static_assert(d == 2 || d == 3,
                "RelativeTranslationFactor supports d = 2 or 3.");

  using Rot = typename std::conditional<d == 2, Rot2, Rot3>::type;
  using Point = Eigen::Matrix<double, d, 1>;
  using Base = NoiseModelFactorN<Rot, Point, Point>;

  Point measured_;  ///< measured translation offset in the body frame of pose i
  double weight_;   ///< measurement precision, tau

 public:
  using Base::evaluateError;

  /**
   * @param rotationKey Key of pose i's rotation.
   * @param translationKey1 Key of pose i's translation.
   * @param translationKey2 Key of pose j's translation.
   * @param measured Translation offset in the body frame of pose i.
   * @param weight Measurement precision, tau.
   */
  RelativeTranslationFactor(Key rotationKey, Key translationKey1,
                            Key translationKey2, const Point& measured,
                            double weight)
      : Base(noiseModel::Unit::Create(d), rotationKey, translationKey1,
             translationKey2),
        measured_(measured),
        weight_(weight) {
    if (weight <= 0.0) {
      throw std::invalid_argument(
          "RelativeTranslationFactor: weight must be positive.");
    }
  }

  const Point& measured() const { return measured_; }
  double weight() const { return weight_; }

  void print(const std::string& s = "",
             const KeyFormatter& keyFormatter =
                 DefaultKeyFormatter) const override {
    std::cout << s << "RelativeTranslationFactor(" << keyFormatter(this->key1())
              << "," << keyFormatter(this->key2()) << ","
              << keyFormatter(this->key3()) << ") weight=" << weight_
              << " measured=[" << measured_.transpose() << "]\n";
  }

  bool equals(const NonlinearFactor& other, double tol = 1e-9) const override {
    const auto* e = dynamic_cast<const RelativeTranslationFactor*>(&other);
    return e != nullptr && Base::equals(other, tol) &&
           traits<Point>::Equals(measured_, e->measured_, tol) &&
           std::abs(weight_ - e->weight_) < tol;
  }

  /// Weighted residual `sqrt(weight) * (t_j - t_i - R_i * measured)`.
  Vector evaluateError(const Rot& Ri, const Point& ti, const Point& tj,
                       OptionalMatrixType H1, OptionalMatrixType H2,
                       OptionalMatrixType H3) const override {
    const double sw = std::sqrt(weight_);

    Matrix rotationJacobian;
    const Point rotated =
        Ri.rotate(measured_, H1 ? &rotationJacobian : nullptr);

    if (H1) *H1 = -sw * rotationJacobian;
    if (H2) *H2 = -sw * Matrix::Identity(d, d);
    if (H3) *H3 = sw * Matrix::Identity(d, d);
    return sw * (tj - ti - rotated);
  }

  gtsam::NonlinearFactor::shared_ptr clone() const override {
    return std::static_pointer_cast<gtsam::NonlinearFactor>(
        gtsam::NonlinearFactor::shared_ptr(
            new RelativeTranslationFactor(*this)));
  }

  /// Add this translation factor as a QCQP cost when traits exist.
  void qcqpFactors(NonlinearFactorGraph* costs,
                   NonlinearEqualityConstraints* /*constraints*/,
                   size_t columnDimension = 1) const override {
    if (columnDimension < static_cast<size_t>(d)) {
      throw std::invalid_argument(
          "RelativeTranslationFactor::qcqpFactors requires columnDimension "
          ">= d.");
    }
    if (!costs) {
      throw std::invalid_argument(
          "RelativeTranslationFactor::qcqpFactors: costs is null.");
    }

    const double sw = std::sqrt(weight_);
    Matrix B = Matrix::Zero(1, d + 2);
    B.block(0, 0, 1, d) = -sw * measured_.transpose();  // rotation row
    B(0, d) = -sw;                                      // pose i translation
    B(0, d + 1) = sw;                                   // pose j translation

    const Matrix Q = B.transpose() * B;
    const SymmetricBlockMatrix blockQ(std::vector<DenseIndex>{d, 1, 1}, Q);
    costs->push_back(std::make_shared<QpCost>(
        KeyVector{this->key1(), this->key2(), this->key3()}, blockQ,
        columnDimension));
  }
};

using RelativeTranslationFactor2 = RelativeTranslationFactor<2>;
using RelativeTranslationFactor3 = RelativeTranslationFactor<3>;

}  // namespace gtsam
