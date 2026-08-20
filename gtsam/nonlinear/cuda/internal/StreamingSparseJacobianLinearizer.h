/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    StreamingSparseJacobianLinearizer.h
 * @brief   Linearizes a factor graph into a preplanned CSR host buffer
 * @author  Ruogu Li
 * @date    Jul 14, 2026
 */

#pragma once

#include <gtsam/dllexport.h>
#include <gtsam/nonlinear/cuda/SparseLevenbergMarquardt.h>
#include <gtsam/nonlinear/cuda/internal/HostSparseJacobian.h>
#include <gtsam/nonlinear/cuda/internal/SparseJacobianPlan.h>

#include <cstddef>
#include <string>

namespace gtsam {

class GaussianFactorGraph;

namespace cuda {

struct StreamingLinearizationStats {
  size_t sendableFactors = 0;
  size_t nonSendableFactors = 0;
};

/// Aggregate worker time, in seconds, summed across all factors.
struct StreamingLinearizationProfile {
  double factorLinearizationCpuSum = 0.0;
  double csrPackingCpuSum = 0.0;
};

/**
 * Linearizes directly into a reusable, preplanned CSR host buffer.
 *
 * This is a CUDA transfer-boundary primitive, not a replacement for
 * NonlinearFactorGraph::linearize or GaussianFactorGraph::sparseJacobian.
 * Those general APIs materialize a GaussianFactorGraph and then allocate
 * sparse triplets. The resident CUDA optimizer instead reuses one exact CSR
 * structure across nonlinear iterations, so its plan maps every factor block
 * to disjoint stable output slots and only numeric values are rewritten and
 * uploaded. Factor linearization and whitening retain the standard GTSAM
 * semantics.
 */
class GTSAM_EXPORT StreamingSparseJacobianLinearizer {
 public:
  /**
   * Linearize and scatter factors into a preplanned host Jacobian. The caller
   * must clear output before every call; null or inactive factors intentionally
   * leave their reserved rows unchanged. A failing factor never partially
   * writes its own range, although other successful factors may already be
   * present. If stats is non-null, it is reset and receives the number of
   * non-null sendable and non-sendable factors. Setting
   * validateStructure=false skips only the deep Values and graph-topology
   * match: output sizes, slot count, sendability/slot safety, and every
   * returned factor's shape and finite coefficients are still validated. If
   * profile is non-null, it is reset and receives separate sums of per-factor
   * linearization and CSR-packing elapsed time. These sums can exceed wall
   * time when sendable factors execute in parallel.
   */
  DirectJacobianStatus linearize(
      const NonlinearFactorGraph& graph, const Values& values,
      const SparseJacobianColumnLayout& columns, const SparseJacobianPlan& plan,
      HostSparseJacobian* output, StreamingLinearizationStats* stats = nullptr,
      bool validateStructure = true,
      StreamingLinearizationProfile* profile = nullptr) const;

  /**
   * Scatter a slot-aligned GaussianFactorGraph into a preplanned host
   * Jacobian. The caller must clear output before every call; null slots leave
   * their reserved rows unchanged. A failing factor never partially writes its
   * own range, although other successful factors may already be present.
   * Output sizes, slot count, and every returned factor's shape and finite
   * coefficients are always validated.
   */
  DirectJacobianStatus packGaussianFactorGraph(
      const GaussianFactorGraph& linear, const SparseJacobianPlan& plan,
      HostSparseJacobian* output) const;
};

}  // namespace cuda
}  // namespace gtsam
