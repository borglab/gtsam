/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file FastSync.h
 * @brief Fast initialization for synchronization over matrix Lie groups.
 */

#pragma once

#include <gtsam/base/FastMap.h>
#include <gtsam/base/MatrixLieGroup.h>
#include <gtsam/dllexport.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot2.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/SL4.h>
#include <gtsam/geometry/Similarity2.h>
#include <gtsam/geometry/Similarity3.h>
#include <gtsam/linear/GaussianFactorGraph.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/PriorFactor.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/BetweenFactor.h>

#include <Eigen/SVD>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <vector>

namespace gtsam {

class GaussianConditional;

/**
 * Projection customization point used by fastSync().
 *
 * Specialize this trait for an additional fixed-size matrix Lie group and
 * provide a `project` function accepting its fixed-size square matrix
 * representation to enable fastSync<T>().
 */
template <class T>
struct FastSyncProjection;

/**
 * Project an ambient 2-by-2 matrix onto Rot2.
 *
 * Rot2 represents SO(2), the group of planar orientation-preserving rotations.
 * The projection selects the closest orthogonal matrix with determinant +1 in
 * Frobenius norm.
 */
template <>
struct FastSyncProjection<Rot2> {
  /// Return the closest proper planar rotation to `matrix`.
  static Rot2 project(const Matrix2& matrix) { return Rot2::ClosestTo(matrix); }
};

/**
 * Project an ambient 3-by-3 matrix onto Rot3.
 *
 * Rot3 represents SO(3), the group of three-dimensional
 * orientation-preserving rotations. The projection uses the SVD-based
 * closest-rotation operation, including reflection correction so the result
 * has determinant +1.
 */
template <>
struct FastSyncProjection<Rot3> {
  /// Return the closest proper three-dimensional rotation to `matrix`.
  static Rot3 project(const Matrix3& matrix) { return Rot3::ClosestTo(matrix); }
};

/**
 * Project an ambient 3-by-3 homogeneous matrix onto Pose2.
 *
 * Pose2 represents SE(2), the group of planar rigid transformations. The
 * upper-left block is rounded to the closest Rot2, while the relaxed
 * upper-right translation is retained unchanged.
 */
template <>
struct FastSyncProjection<Pose2> {
  /// Round the rotation block and retain the recovered planar translation.
  static Pose2 project(const Matrix3& matrix) {
    return Pose2(Rot2::ClosestTo(matrix.topLeftCorner<2, 2>()),
                 matrix.topRightCorner<2, 1>());
  }
};

/**
 * Project an ambient 4-by-4 homogeneous matrix onto Pose3.
 *
 * Pose3 represents SE(3), the group of three-dimensional rigid
 * transformations. The upper-left block is rounded to the closest Rot3,
 * while the relaxed upper-right translation is retained unchanged.
 */
template <>
struct FastSyncProjection<Pose3> {
  /// Round the rotation block and retain the recovered 3D translation.
  static Pose3 project(const Matrix4& matrix) {
    return Pose3(Rot3::ClosestTo(matrix.topLeftCorner<3, 3>()),
                 matrix.topRightCorner<3, 1>());
  }
};

/**
 * Project an ambient 3-by-3 matrix onto Similarity2.
 *
 * Similarity2 represents Sim(2), the group of planar rotations,
 * translations, and positive uniform scales. In GTSAM's matrix convention the
 * lower-right entry is the inverse scale. The projection rounds the rotation,
 * retains the translation, and recovers a positive scale from the absolute
 * inverse-scale entry. A zero or non-finite inverse scale falls back to unit
 * scale.
 */
template <>
struct FastSyncProjection<Similarity2> {
  /// Round rotation and recover translation and positive planar scale.
  static Similarity2 project(const Matrix3& matrix) {
    double inverseScale = matrix(2, 2);
    if (!std::isfinite(inverseScale)) inverseScale = 1.0;
    inverseScale = std::abs(inverseScale);
    if (inverseScale <= std::numeric_limits<double>::epsilon()) {
      inverseScale = 1.0;
    }
    return Similarity2(Rot2::ClosestTo(matrix.topLeftCorner<2, 2>()),
                       matrix.topRightCorner<2, 1>(), 1.0 / inverseScale);
  }
};

/**
 * Project an ambient 4-by-4 matrix onto Similarity3.
 *
 * Similarity3 represents Sim(3), the group of three-dimensional rotations,
 * translations, and positive uniform scales. In GTSAM's matrix convention the
 * lower-right entry is the inverse scale. The projection rounds the rotation,
 * retains the translation, and recovers a positive scale from the absolute
 * inverse-scale entry. A zero or non-finite inverse scale falls back to unit
 * scale.
 */
template <>
struct FastSyncProjection<Similarity3> {
  /// Round rotation and recover translation and positive 3D scale.
  static Similarity3 project(const Matrix4& matrix) {
    double inverseScale = matrix(3, 3);
    if (!std::isfinite(inverseScale)) inverseScale = 1.0;
    inverseScale = std::abs(inverseScale);
    if (inverseScale <= std::numeric_limits<double>::epsilon()) {
      inverseScale = 1.0;
    }
    return Similarity3(Rot3::ClosestTo(matrix.topLeftCorner<3, 3>()),
                       matrix.topRightCorner<3, 1>(), 1.0 / inverseScale);
  }
};

/**
 * Project an ambient 4-by-4 matrix onto SL4.
 *
 * SL4 is the special linear group of real 4-by-4 matrices with determinant
 * one. A preliminary SVD detects numerically singular or non-finite input and
 * returns identity in that case. Otherwise the SL4 constructor corrects
 * orientation and normalizes the determinant magnitude to one.
 */
template <>
struct FastSyncProjection<SL4> {
  /// Normalize a nonsingular ambient matrix to SL(4), or return identity.
  static SL4 project(const Matrix4& matrix) {
    const Eigen::JacobiSVD<Matrix4> svd(
        matrix, Eigen::ComputeFullU | Eigen::ComputeFullV);
    const auto singularValues = svd.singularValues();
    const double determinantMagnitude = singularValues.prod();
    if (!std::isfinite(determinantMagnitude) || determinantMagnitude <= 1e-12) {
      return SL4::Identity();
    }
    return SL4(matrix);
  }
};

/**
 * Solver for the fixed-size ambient linear problem underlying FAST-Sync.
 *
 * The constructor extracts matching between factors and builds the reduced
 * Gaussian graph. `solve()` performs the ordered Cholesky solve and returns
 * ambient matrix estimates; `projectAndAlign()` rounds those estimates to T
 * and applies an optional matching prior.
 */
template <class T>
struct FastSync {
  using LieAlgebra = typename T::LieAlgebra;
  static constexpr int N = LieAlgebra::RowsAtCompileTime;
  static_assert(N != Eigen::Dynamic && N > 0,
                "FastSync requires a positive compile-time matrix dimension");
  static_assert(LieAlgebra::ColsAtCompileTime == N,
                "FastSync requires a square matrix representation");

  using MatrixN = Eigen::Matrix<double, N, N>;
  using VectorN = Eigen::Matrix<double, N, 1>;

  /**
   * Extract matching factors, validate their noise models, and build the
   * reduced Gaussian graph. Factors for other types are ignored.
   */
  explicit FastSync(const NonlinearFactorGraph& graph);

  /**
   * Solve the relaxed ambient matrix problem and return one matrix per key.
   *
   * The selected ordering's final key is used as the identity gauge. Cholesky
   * elimination and reverse block back-substitution then recover all ambient
   * N-by-N estimates. Projection to T is intentionally deferred to
   * `projectAndAlign()`.
   *
   * @param orderingType Fill-reducing ordering, defaulting to METIS.
   */
  Values solve(Ordering::OrderingType orderingType = Ordering::METIS) const;

  /**
   * Solve using a caller-supplied complete ordering.
   *
   * The ordering must contain every measurement-graph key exactly once. Its
   * final key is used as the identity gauge.
   *
   * @param ordering Complete variable elimination ordering.
   */
  Values solve(const Ordering& ordering) const;

  /**
   * Project relaxed matrices to T and align them to the optional matching
   * prior stored by the constructor.
   *
   * The alignment is a single common left transformation, so all relative
   * estimates are preserved. The input must be the complete result of
   * `solve()`.
   */
  Values projectAndAlign(const Values& relaxed) const;

 private:
  size_t priorCount_ = 0;
  Key priorKey_ = 0;
  T priorValue_ = traits<T>::Identity();
  GaussianFactorGraph reducedGraph_;

  /// Extract the isotropic sigma from a noise model, or throw if not isotropic.
  static double isotropicSigma(const SharedNoiseModel& model);

  /// Back-substitute one conditional, skipping the identity gauge variable.
  static void backSubstituteConditional(const GaussianConditional& conditional,
                                        const Key& gaugeKey, Values& solution);

  /// Solve with an already validated ordering.
  Values solveOrdered(const Ordering& ordering) const;
};

/**
 * Initialize a synchronization graph using FAST-Sync.
 *
 * The graph may contain BetweenFactor<T> measurements and at most one matching
 * PriorFactor<T>; factors of other types are ignored. Between-factor noise
 * models must have the tangent dimension of T and be finite, positive,
 * non-robust, unconstrained, diagonal, and isotropic. Arbitrary keys are
 * preserved in the returned Values.
 *
 * T must be a fixed-size square matrix Lie group with a
 * FastSyncProjection<T> specialization.
 *
 * @param graph Synchronization factor graph.
 * @param orderingType Fill-reducing ordering used by sequential elimination.
 *
 * @throws std::invalid_argument for invalid noise models, an empty measurement
 * graph, a prior outside the measurement graph, or multiple matching priors.
 * @throws IndeterminateSystemException when the measurement graph is
 * disconnected or otherwise underconstrained.
 * @throws std::runtime_error for an unsupported ordering type, or when METIS
 * is selected but unavailable in the current build.
 */
template <class T>
Values fastSync(const NonlinearFactorGraph& graph,
                Ordering::OrderingType orderingType = Ordering::METIS);

/**
 * Initialize a synchronization graph using a caller-supplied ordering.
 *
 * @param graph Synchronization factor graph.
 * @param ordering Complete variable elimination ordering.
 */
template <class T>
Values fastSync(const NonlinearFactorGraph& graph, const Ordering& ordering);

}  // namespace gtsam

#include <gtsam/slam/FastSync-inl.h>
