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
#include <gtsam/geometry/EssentialMatrix.h>
#include <gtsam/geometry/FundamentalMatrix.h>
#include <gtsam/inference/EdgeKey.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

#include <cstdint>

namespace gtsam {

/**
 * Base class that holds the EdgeKeys and provides the getMatrices method.
 */
template <typename F>
class TransferEdges {
 protected:
  EdgeKey edge1_, edge2_;  ///< The two EdgeKeys.
  uint32_t c_;             ///< The transfer target

 public:
  TransferEdges(EdgeKey edge1, EdgeKey edge2)
      : edge1_(edge1), edge2_(edge2), c_(ViewC(edge1, edge2)) {}

  /// Returns the view A index based on the EdgeKeys
  static size_t ViewA(const EdgeKey& edge1, const EdgeKey& edge2) {
    size_t c = ViewC(edge1, edge2);
    return (edge1.i() == c) ? edge1.j() : edge1.i();
  }

  /// Returns the view B index based on the EdgeKeys
  static size_t ViewB(const EdgeKey& edge1, const EdgeKey& edge2) {
    size_t c = ViewC(edge1, edge2);
    return (edge2.i() == c) ? edge2.j() : edge2.i();
  }

  /// Returns the view C index based on the EdgeKeys
  static size_t ViewC(const EdgeKey& edge1, const EdgeKey& edge2) {
    if (edge1.i() == edge2.i() || edge1.i() == edge2.j())
      return edge1.i();
    else if (edge1.j() == edge2.i() || edge1.j() == edge2.j())
      return edge1.j();
    else
      throw std::runtime_error(
          "EssentialTransferFactorK: No common key in edge keys.");
  }

  /// Create Matrix3 objects based on EdgeKey configurations.
  std::pair<Matrix3, Matrix3> getMatrices(const F& F1, const F& F2) const {
    // Determine whether to transpose F1
    const Matrix3 Fca =
        edge1_.i() == c_ ? F1.matrix() : F1.matrix().transpose();

    // Determine whether to transpose F2
    const Matrix3 Fcb =
        edge2_.i() == c_ ? F2.matrix() : F2.matrix().transpose();

    return {Fca, Fcb};
  }
};

/**
 * Binary factor in the context of Structure from Motion (SfM).
 * It is used to transfer transfer corresponding points from two views to a
 * third based on two fundamental matrices. The factor computes the error
 * between the transferred points `pa` and `pb`, and the actual point `pc` in
 * the target view. Jacobians are done using numerical differentiation.
 */
template <typename F>
class TransferFactor : public NoiseModelFactorN<F, F>, public TransferEdges<F> {
 public:
  using Base = NoiseModelFactorN<F, F>;
  using Triplet = std::tuple<Point2, Point2, Point2>;

 protected:
  std::vector<Triplet> triplets_;  ///< Point triplets.

 public:
  /**
   * @brief Constructor that accepts a vector of point triplets.
   *
   * @param edge1 First EdgeKey specifying F1: (a, c) or (c, a).
   * @param edge2 Second EdgeKey specifying F2: (b, c) or (c, b).
   * @param triplets A vector of triplets containing (pa, pb, pc).
   * @param model An optional SharedNoiseModel that defines the noise model
   *              for this factor. Defaults to nullptr.
   */
  TransferFactor(EdgeKey edge1, EdgeKey edge2,
                 const std::vector<Triplet>& triplets,
                 const SharedNoiseModel& model = nullptr)
      : Base(model, edge1, edge2),
        TransferEdges<F>(edge1, edge2),
        triplets_(triplets) {}

  /// Vector of errors returns 2*N vector.
  Vector evaluateError(const F& F1, const F& F2,
                       OptionalMatrixType H1 = nullptr,
                       OptionalMatrixType H2 = nullptr) const override {
    std::function<Vector(const F&, const F&)> errorFunction = [&](const F& f1,
                                                                  const F& f2) {
      Vector errors(2 * triplets_.size());
      size_t idx = 0;
      auto [Fca, Fcb] = this->getMatrices(f1, f2);
      for (const auto& [pa, pb, pc] : triplets_) {
        errors.segment<2>(idx) = EpipolarTransfer(Fca, pa, Fcb, pb) - pc;
        idx += 2;
      }
      return errors;
    };

    if (H1) *H1 = numericalDerivative21(errorFunction, F1, F2);
    if (H2) *H2 = numericalDerivative22(errorFunction, F1, F2);
    return errorFunction(F1, F2);
  }
};

/**
 * @class EssentialTransferFactor
 * @brief Transfers points between views using essential matrices with a shared
 * calibration.
 *
 * This factor is templated on the calibration class K and extends
 * the TransferFactor for EssentialMatrices. It involves two essential matrices
 * and a shared calibration object (K). The evaluateError function calibrates
 * the image points, calls the base class's transfer method, and computes the
 * error using bulk numerical differentiation.
 */
template <typename K>
class EssentialTransferFactor : public TransferFactor<EssentialMatrix> {
  using EM = EssentialMatrix;
  using Triplet = std::tuple<Point2, Point2, Point2>;
  std::shared_ptr<K> calibration_;  ///< Shared pointer to calibration object

 public:
  using Base = TransferFactor<EM>;
  using This = EssentialTransferFactor<K>;
  using shared_ptr = std::shared_ptr<This>;

  /**
   * @brief Constructor that accepts a vector of point triplets and a shared
   * calibration.
   *
   * @param edge1 First EdgeKey specifying E1: (a, c) or (c, a)
   * @param edge2 Second EdgeKey specifying E2: (b, c) or (c, b)
   * @param triplets A vector of triplets containing (pa, pb, pc)
   * @param calibration Shared pointer to calibration object
   * @param model An optional SharedNoiseModel
   */
  EssentialTransferFactor(EdgeKey edge1, EdgeKey edge2,
                          const std::vector<Triplet>& triplets,
                          const std::shared_ptr<K>& calibration,
                          const SharedNoiseModel& model = nullptr)
      : Base(edge1, edge2, triplets, model), calibration_(calibration) {}

  /// Transfer points pa and pb to view c and evaluate error.
  Vector2 TransferError(const Matrix3& Eca, const Point2& pa,
                        const Matrix3& Ecb, const Point2& pb,
                        const Point2& pc) const {
    const Point2 pA = calibration_->calibrate(pa);
    const Point2 pB = calibration_->calibrate(pb);
    const Point2 pC = EpipolarTransfer(Eca, pA, Ecb, pB);
    return calibration_->uncalibrate(pC) - pc;
  }

  /// Evaluate error function
  Vector evaluateError(const EM& E1, const EM& E2,
                       OptionalMatrixType H1 = nullptr,
                       OptionalMatrixType H2 = nullptr) const override {
    std::function<Vector(const EM&, const EM&)> errorFunction =
        [&](const EM& e1, const EM& e2) {
          Vector errors(2 * this->triplets_.size());
          size_t idx = 0;
          auto [Eca, Ecb] = this->getMatrices(e1, e2);
          for (const auto& [pa, pb, pc] : this->triplets_) {
            errors.segment<2>(idx) = TransferError(Eca, pa, Ecb, pb, pc);
            idx += 2;
          }
          return errors;
        };

    // Compute error
    Vector errors = errorFunction(E1, E2);

    // Compute Jacobians if requested
    if (H1) *H1 = numericalDerivative21(errorFunction, E1, E2);
    if (H2) *H2 = numericalDerivative22(errorFunction, E1, E2);

    return errors;
  }
};

/**
 * @class EssentialTransferFactorK
 * @brief Transfers points between views using essential matrices, optimizes for
 * calibrations of the views, as well. Note that the EssentialMatrixFactor4 does
 * something similar but without transfer.
 *
 * @note Derives calibration keys from edges, and uses symbol 'k'.
 *
 * This factor is templated on the calibration class K and extends
 * the TransferFactor for EssentialMatrices. It involves two essential matrices
 * and three calibration objects (Ka, Kb, Kc). The evaluateError
 * function calibrates the image points, calls the base class's transfer method,
 * and computes the error using bulk numerical differentiation.
 */
template <typename K>
class EssentialTransferFactorK
    : public NoiseModelFactorN<EssentialMatrix, EssentialMatrix, K, K, K>,
      TransferEdges<EssentialMatrix> {
  using EM = EssentialMatrix;
  using Triplet = std::tuple<Point2, Point2, Point2>;
  std::vector<Triplet> triplets_;  ///< Point triplets

 public:
  using Base = NoiseModelFactorN<EM, EM, K, K, K>;
  using This = EssentialTransferFactorK<K>;
  using shared_ptr = std::shared_ptr<This>;

  /**
   * @brief Constructor that accepts a vector of point triplets.
   *
   * @param edge1 First EdgeKey specifying E1: (a, c) or (c, a)
   * @param edge2 Second EdgeKey specifying E2: (b, c) or (c, b)
   * @param triplets A vector of triplets containing (pa, pb, pc)
   * @param model An optional SharedNoiseModel
   */
  EssentialTransferFactorK(EdgeKey edge1, EdgeKey edge2,
                           const std::vector<Triplet>& triplets,
                           const SharedNoiseModel& model = nullptr)
      : Base(model, edge1, edge2,
             Symbol('k', ViewA(edge1, edge2)),   // calibration key for view a
             Symbol('k', ViewB(edge1, edge2)),   // calibration key for view b
             Symbol('k', ViewC(edge1, edge2))),  // calibration key for target c
        TransferEdges<EM>(edge1, edge2),
        triplets_(triplets) {}

  /// Transfer points pa and pb to view c and evaluate error.
  Vector2 TransferError(const Matrix3& Eca, const K& Ka, const Point2& pa,
                        const Matrix3& Ecb, const K& Kb, const Point2& pb,
                        const K& Kc, const Point2& pc) const {
    const Point2 pA = Ka.calibrate(pa);
    const Point2 pB = Kb.calibrate(pb);
    const Point2 pC = EpipolarTransfer(Eca, pA, Ecb, pB);
    return Kc.uncalibrate(pC) - pc;
  }

  /// Evaluate error function
  Vector evaluateError(const EM& E1, const EM& E2, const K& Ka, const K& Kb,
                       const K& Kc, OptionalMatrixType H1 = nullptr,
                       OptionalMatrixType H2 = nullptr,
                       OptionalMatrixType H3 = nullptr,
                       OptionalMatrixType H4 = nullptr,
                       OptionalMatrixType H5 = nullptr) const override {
    std::function<Vector(const EM&, const EM&, const K&, const K&, const K&)>
        errorFunction = [&](const EM& e1, const EM& e2, const K& kA,
                            const K& kB, const K& kC) {
          Vector errors(2 * triplets_.size());
          size_t idx = 0;
          auto [Eca, Ecb] = this->getMatrices(e1, e2);
          for (const auto& [pa, pb, pc] : triplets_) {
            errors.segment<2>(idx) =
                TransferError(Eca, kA, pa, Ecb, kB, pb, kC, pc);
            idx += 2;
          }
          return errors;
        };

    // Compute error
    Vector errors = errorFunction(E1, E2, Ka, Kb, Kc);

    // Compute Jacobians if requested
    if (H1) *H1 = numericalDerivative51(errorFunction, E1, E2, Ka, Kb, Kc);
    if (H2) *H2 = numericalDerivative52(errorFunction, E1, E2, Ka, Kb, Kc);
    if (H3) *H3 = numericalDerivative53(errorFunction, E1, E2, Ka, Kb, Kc);
    if (H4) *H4 = numericalDerivative54(errorFunction, E1, E2, Ka, Kb, Kc);
    if (H5) *H5 = numericalDerivative55(errorFunction, E1, E2, Ka, Kb, Kc);

    return errors;
  }

  /// Return the dimension of the factor
  size_t dim() const override { return 2 * triplets_.size(); }
};

}  // namespace gtsam