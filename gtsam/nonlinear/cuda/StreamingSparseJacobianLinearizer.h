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

class GTSAM_EXPORT StreamingSparseJacobianLinearizer {
 public:
  DirectJacobianStatus linearize(
      const NonlinearFactorGraph& graph, const Values& values,
      const SparseJacobianColumnLayout& columns,
      const SparseJacobianPlan& plan, HostSparseJacobian* output,
      bool validateStructure = true) const;

  DirectJacobianStatus packGaussianFactorGraph(
      const GaussianFactorGraph& linear, const SparseJacobianPlan& plan,
      HostSparseJacobian* output) const;
};

}  // namespace cuda
}  // namespace gtsam
