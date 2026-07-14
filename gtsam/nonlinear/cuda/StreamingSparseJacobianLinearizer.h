#pragma once

#include <gtsam/dllexport.h>
#include <gtsam/nonlinear/cuda/HostSparseJacobian.h>
#include <gtsam/nonlinear/cuda/SparseJacobianPlan.h>

#include <cstddef>
#include <limits>
#include <string>

namespace gtsam {

class GaussianFactorGraph;

namespace cuda {

enum class DirectJacobianFailure {
  None,
  StructuralMismatch,
  UnsupportedGaussianFactor,
  ConstrainedFactor,
  NonFiniteValues,
};

struct DirectJacobianStatus {
  DirectJacobianFailure failure = DirectJacobianFailure::None;
  size_t factorIndex = std::numeric_limits<size_t>::max();
  std::string detail;

  bool ok() const { return failure == DirectJacobianFailure::None; }
};

struct StreamingLinearizationStats {
  size_t sendableFactors = 0;
  size_t nonSendableFactors = 0;
};

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
   * returned factor's shape and finite coefficients are still validated.
   */
  DirectJacobianStatus linearize(
      const NonlinearFactorGraph& graph, const Values& values,
      const SparseJacobianColumnLayout& columns,
      const SparseJacobianPlan& plan, HostSparseJacobian* output,
      StreamingLinearizationStats* stats = nullptr,
      bool validateStructure = true) const;

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
