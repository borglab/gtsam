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

namespace internal {

/** A group-independent measurement used by the reduced FAST-Sync solver. */
struct FastSyncMeasurement {
  Key key1;
  Key key2;
  Matrix measured;
  double sigma;
};

/** Extract the common finite, positive sigma from an isotropic noise model. */
GTSAM_EXPORT double FastSyncIsotropicSigma(const SharedNoiseModel& model);

/** Solve the reduced linear problem and return unprojected matrix estimates. */
GTSAM_EXPORT FastMap<Key, Matrix> FastSyncSolveMatrices(
    const std::vector<FastSyncMeasurement>& measurements,
    size_t matrixDimension);

}  // namespace internal

/**
 * Projection customization point used by fastSync().
 *
 * Specialize this trait for an additional fixed-size matrix Lie group and
 * provide `static T project(const Matrix&)` to enable fastSync<T>().
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
  static Rot2 project(const Matrix& matrix) {
    return Rot2::ClosestTo(Matrix2(matrix));
  }
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
  static Rot3 project(const Matrix& matrix) {
    return Rot3::ClosestTo(Matrix3(matrix));
  }
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
  static Pose2 project(const Matrix& matrix) {
    return Pose2(Rot2::ClosestTo(Matrix2(matrix.topLeftCorner(2, 2))),
                 Vector2(matrix.topRightCorner(2, 1)));
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
  static Pose3 project(const Matrix& matrix) {
    return Pose3(Rot3::ClosestTo(Matrix3(matrix.topLeftCorner(3, 3))),
                 Vector3(matrix.topRightCorner(3, 1)));
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
  static Similarity2 project(const Matrix& matrix) {
    double inverseScale = matrix(2, 2);
    if (!std::isfinite(inverseScale)) inverseScale = 1.0;
    inverseScale = std::abs(inverseScale);
    if (inverseScale <= std::numeric_limits<double>::epsilon()) {
      inverseScale = 1.0;
    }
    return Similarity2(Rot2::ClosestTo(Matrix2(matrix.topLeftCorner(2, 2))),
                       Vector2(matrix.topRightCorner(2, 1)),
                       1.0 / inverseScale);
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
  static Similarity3 project(const Matrix& matrix) {
    double inverseScale = matrix(3, 3);
    if (!std::isfinite(inverseScale)) inverseScale = 1.0;
    inverseScale = std::abs(inverseScale);
    if (inverseScale <= std::numeric_limits<double>::epsilon()) {
      inverseScale = 1.0;
    }
    return Similarity3(Rot3::ClosestTo(Matrix3(matrix.topLeftCorner(3, 3))),
                       Vector3(matrix.topRightCorner(3, 1)),
                       1.0 / inverseScale);
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
  static SL4 project(const Matrix& matrix) {
    const Matrix4 input(matrix);
    const Eigen::JacobiSVD<Matrix4> svd(
        input, Eigen::ComputeFullU | Eigen::ComputeFullV);
    const auto singularValues = svd.singularValues();
    const double determinantMagnitude = singularValues.prod();
    if (!std::isfinite(determinantMagnitude) || determinantMagnitude <= 1e-12) {
      return SL4::Identity();
    }
    return SL4(input);
  }
};

/**
 * Initialize a connected synchronization graph using FAST-Sync.
 *
 * The graph may contain BetweenFactor<T> measurements and at most one matching
 * PriorFactor<T>; factors of other types are ignored. Between-factor noise
 * models must have the tangent dimension of T and be finite, positive,
 * non-robust, unconstrained, diagonal, and isotropic. Arbitrary keys are
 * preserved in the returned Values.
 *
 * The implementation proceeds in four stages:
 *
 * 1. Extract matching between measurements as group matrix representations
 *    and validate their noise models. Record one optional prior for later frame
 *    alignment rather than adding it to the relaxed objective.
 * 2. Solve the reduced ambient-space least-squares problem. Each measurement
 *    contributes the block factor `[-Z.transpose(), I]`; METIS nested
 *    dissection chooses an ordering, its final key is the identity gauge, and
 *    explicit QR elimination is followed by reverse block back-substitution.
 * 3. After every ambient matrix has been recovered, independently round it
 *    through FastSyncProjection<T>. Delaying projection preserves the linear
 *    coupling used by the global chordal solve.
 * 4. If a prior was supplied, compute the one global left transformation that
 *    maps the rounded value at the prior key onto the prior, and apply that
 *    transformation to every result. Relative estimates therefore do not
 *    depend on the prior.
 *
 * T must be a fixed-size square matrix Lie group with a
 * FastSyncProjection<T> specialization.
 *
 * @throws std::invalid_argument for invalid noise models, an empty or
 * disconnected measurement graph, a prior outside the measurement graph, or
 * multiple matching priors.
 * @throws std::runtime_error if GTSAM was built without METIS support.
 */
template <class T>
Values fastSync(const NonlinearFactorGraph& graph) {
  // Enforce the fixed-size square matrix representation required by the
  // reduced block system and the type-specific projection trait.
  GTSAM_CONCEPT_ASSERT(IsMatrixLieGroup<T>);
  constexpr int N = T::LieAlgebra::RowsAtCompileTime;
  static_assert(N != Eigen::Dynamic && N > 0,
                "fastSync requires a fixed-size matrix Lie group");
  static_assert(T::LieAlgebra::ColsAtCompileTime == N,
                "fastSync requires a square matrix representation");

  std::vector<internal::FastSyncMeasurement> measurements;
  measurements.reserve(graph.size());
  size_t priorCount = 0;
  Key priorKey = 0;
  T priorValue = traits<T>::Identity();

  // Extract only factors matching T. The optional prior is saved for final
  // alignment and deliberately does not influence the ambient-space solve.
  for (const auto& factor : graph) {
    if (const auto between =
            std::dynamic_pointer_cast<BetweenFactor<T>>(factor)) {
      if (between->noiseModel()->dim() != T::dimension) {
        throw std::invalid_argument(
            "fastSync noise dimension must match the group dimension");
      }
      measurements.push_back(
          {between->key1(), between->key2(), between->measured().matrix(),
           internal::FastSyncIsotropicSigma(between->noiseModel())});
    } else if (const auto prior =
                   std::dynamic_pointer_cast<PriorFactor<T>>(factor)) {
      ++priorCount;
      if (priorCount > 1) {
        throw std::invalid_argument(
            "fastSync supports at most one matching prior");
      }
      priorKey = prior->key();
      priorValue = prior->prior();
    }
  }

  FastMap<Key, Matrix> relaxed =
      internal::FastSyncSolveMatrices(measurements, N);

  // Round only after all ambient matrices have been jointly recovered.
  Values projected;
  for (const auto& keyMatrix : relaxed) {
    projected.insert(keyMatrix.first,
                     FastSyncProjection<T>::project(keyMatrix.second));
  }

  if (priorCount == 0) return projected;
  if (!projected.exists(priorKey)) {
    throw std::invalid_argument(
        "fastSync prior key is not in the measurement graph");
  }

  // Select the prior's global frame with one common left transformation,
  // preserving every relative estimate from the rounded solution.
  const T estimatedPrior = projected.at<T>(priorKey);
  const T alignment =
      traits<T>::Compose(priorValue, traits<T>::Inverse(estimatedPrior));
  Values aligned;
  for (const auto& keyValue : projected.extract<T>()) {
    aligned.insert(keyValue.first,
                   traits<T>::Compose(alignment, keyValue.second));
  }
  return aligned;
}

}  // namespace gtsam
