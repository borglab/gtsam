/* ----------------------------------------------------------------------------
 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

#pragma once

#include <gtsam/base/Matrix.h>
#include <gtsam/inference/Key.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>

#include <map>
#include <vector>

namespace gtsam {
namespace internal {

/// Homogeneous objective coefficient in the source factor's key order.
struct DirectSDPCost {
  KeyVector keys;
  Matrix matrix;
};

/// Unary affine Gram constraint: inner product of matrix and X_kk equals rhs.
struct DirectSDPConstraint {
  Key key;
  Matrix matrix;
  double rhs;
};

/// Solver-independent coefficients for auditing the direct transcription.
struct DirectSDPData {
  std::map<Key, DenseIndex> dimensions;
  std::vector<DirectSDPCost> costs;
  std::vector<DirectSDPConstraint> constraints;
  std::map<Key, Vector> anchors;
};

/**
 * Transcribe homogeneous Pose2 or Pose3 Frobenius graphs without QCQP lowering.
 * Costs use 0.5 * <Q, X>. Manifold and exact-prior constraints are independent
 * of the QCQP constraint generator; the coordinate convention is shared.
 * Unsupported factors, robust noise, and partially hard priors are rejected.
 */
GTSAM_EXPORT DirectSDPData buildDirectSDP(const NonlinearFactorGraph& graph);

}  // namespace internal
}  // namespace gtsam
