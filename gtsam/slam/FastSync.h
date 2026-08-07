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

template <>
struct FastSyncProjection<Rot2> {
  static Rot2 project(const Matrix& matrix) {
    return Rot2::ClosestTo(Matrix2(matrix));
  }
};

template <>
struct FastSyncProjection<Rot3> {
  static Rot3 project(const Matrix& matrix) {
    return Rot3::ClosestTo(Matrix3(matrix));
  }
};

template <>
struct FastSyncProjection<Pose2> {
  static Pose2 project(const Matrix& matrix) {
    return Pose2(Rot2::ClosestTo(Matrix2(matrix.topLeftCorner(2, 2))),
                 Vector2(matrix.topRightCorner(2, 1)));
  }
};

template <>
struct FastSyncProjection<Pose3> {
  static Pose3 project(const Matrix& matrix) {
    return Pose3(Rot3::ClosestTo(Matrix3(matrix.topLeftCorner(3, 3))),
                 Vector3(matrix.topRightCorner(3, 1)));
  }
};

template <>
struct FastSyncProjection<Similarity2> {
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

template <>
struct FastSyncProjection<Similarity3> {
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

template <>
struct FastSyncProjection<SL4> {
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
 * The graph may contain BetweenFactor<T> measurements and at most one
 * PriorFactor<T>. Other factor types are ignored. Between-factor noise models
 * must be non-robust, unconstrained, diagonal, and isotropic. The METIS nested
 * dissection ordering chooses the linear gauge; after projection, a matching
 * prior (if present) is imposed by a single global left transformation.
 *
 * @throws std::invalid_argument for invalid noise models, an empty or
 * disconnected measurement graph, or multiple matching priors.
 * @throws std::runtime_error if GTSAM was built without METIS support.
 */
template <class T>
Values fastSync(const NonlinearFactorGraph& graph) {
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
