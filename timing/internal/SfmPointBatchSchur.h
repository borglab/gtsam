/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010-2026, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file SfmPointBatchSchur.h
 * @brief Backend-neutral compact Schur assembly for explicit-point BAL.
 */

#pragma once

#include <gtsam/linear/BatchJacobianFactor.h>
#include <gtsam/linear/GaussianFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>

#include <cstdint>
#include <memory>
#include <vector>

namespace gtsam::timing::bal {

using PointBatchJacobian = BatchJacobianFactor<2, 9, 3>;
using PointDampingBatch = BatchJacobianFactor<3, 3>;
using CameraDampingBatch = BatchJacobianFactor<9, 9>;
using CameraHessianBlocks =
    std::vector<Matrix99, Eigen::aligned_allocator<Matrix99>>;

/** Compact upper-triangular 9x9 camera Schur system. */
struct CompactCameraSystem {
  struct Landmark {
    std::shared_ptr<const PointBatchJacobian> batch;
    Key key;
    Matrix3 pointCovariance;
    Vector3 pointInformationRhs;
  };

  size_t cameraCount = 0;
  CameraHessianBlocks blocks;
  std::vector<uint8_t> usedBlocks;
  Vector rhs;
  std::vector<Landmark> landmarks;
  KeyVector zeroLandmarks;
};

/** Return the packed upper-triangular block index for a camera pair. */
size_t upperCameraBlockIndex(size_t row, size_t column, size_t cameraCount);

/**
 * Assemble a damped point-batch graph directly into fixed 9x9 camera blocks.
 *
 * Each active landmark retains only its compact measurement batch, inverse
 * 3x3 point block, and point information RHS for later back-substitution.
 * `numThreads == 0` selects the hardware concurrency.
 */
CompactCameraSystem buildPointBatchCameraSystemParallel(
    const GaussianFactorGraph& graph, size_t numThreads = 0);

/** Recover landmark deltas after a backend has solved the camera system. */
VectorValues backSubstitutePointBatchLandmarksParallel(
    const CompactCameraSystem& system,
    const VectorValues& cameraSolution);

/**
 * Common compact point-batch LM integration independent of a sparse backend.
 *
 * Derived optimizers provide only the camera-system solve. Damping is emitted
 * as two compact unary batches and model-fidelity evaluation is parallelized.
 */
class PointBatchSchurLevenbergMarquardtOptimizer
    : public LevenbergMarquardtOptimizer {
 protected:
  GaussianFactorGraph buildDampedSystem(
      const GaussianFactorGraph& linear,
      const VectorValues& sqrtHessianDiagonal) const override;

  double linearDeltaError(const GaussianFactorGraph& linear,
                          const VectorValues& delta, double* oldError,
                          double* newError) const override;

 public:
  PointBatchSchurLevenbergMarquardtOptimizer(
      const NonlinearFactorGraph& graph, const Values& initial,
      const LevenbergMarquardtParams& parameters)
      : LevenbergMarquardtOptimizer(graph, initial, parameters) {}
};

}  // namespace gtsam::timing::bal
