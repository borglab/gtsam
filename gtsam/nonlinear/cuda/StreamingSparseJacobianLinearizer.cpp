#include <gtsam/nonlinear/cuda/StreamingSparseJacobianLinearizer.h>

#include <gtsam/config.h>
#include <gtsam/base/types.h>
#include <gtsam/linear/GaussianFactorGraph.h>
#include <gtsam/linear/JacobianFactor.h>
#include <gtsam/linear/NoiseModel.h>

#ifdef GTSAM_USE_TBB
#include <tbb/blocked_range.h>
#include <tbb/parallel_for.h>
#endif

#include <cmath>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace gtsam::cuda {
namespace {

DirectJacobianStatus Failure(DirectJacobianFailure failure,
                             std::string detail) {
  DirectJacobianStatus status;
  status.failure = failure;
  status.detail = std::move(detail);
  return status;
}

DirectJacobianStatus FactorFailure(DirectJacobianFailure failure,
                                   size_t factorIndex,
                                   std::string detail) {
  DirectJacobianStatus status;
  status.failure = failure;
  status.factorIndex = factorIndex;
  status.detail = std::move(detail);
  return status;
}

DirectJacobianStatus ValidateOutput(const SparseJacobianPlan& plan,
                                    const HostSparseJacobian* output) {
  if (!output) {
    return Failure(DirectJacobianFailure::StructuralMismatch,
                   "output is null");
  }
  if (output->valuesSize() != static_cast<size_t>(plan.nonzeros())) {
    return Failure(DirectJacobianFailure::StructuralMismatch,
                   "output values size does not match the sparse plan");
  }
  if (output->rhsSize() != static_cast<size_t>(plan.rows())) {
    return Failure(DirectJacobianFailure::StructuralMismatch,
                   "output RHS size does not match the sparse plan");
  }
  return {};
}

DirectJacobianStatus ScatterOneFactor(
    const std::shared_ptr<GaussianFactor>& gaussian, size_t factorIndex,
    const SparseJacobianPlan& plan, HostSparseJacobian* output) {
  if (!gaussian) {
    return {};
  }

  const auto* jacobian = dynamic_cast<const JacobianFactor*>(gaussian.get());
  if (!jacobian) {
    return FactorFailure(
        DirectJacobianFailure::UnsupportedGaussianFactor, factorIndex,
        "linearization result is not a JacobianFactor");
  }

  const SparseJacobianFactorWritePlan& factorPlan = plan.factor(factorIndex);
  if (jacobian->rows() != static_cast<size_t>(factorPlan.rowCount)) {
    return FactorFailure(DirectJacobianFailure::StructuralMismatch,
                         factorIndex,
                         "Jacobian row count does not match the sparse plan");
  }
  if (jacobian->size() != factorPlan.blocks.size()) {
    return FactorFailure(
        DirectJacobianFailure::StructuralMismatch, factorIndex,
        "Jacobian block count does not match the sparse plan");
  }
  if (jacobian->isConstrained()) {
    return FactorFailure(DirectJacobianFailure::ConstrainedFactor,
                         factorIndex,
                         "constrained JacobianFactor is unsupported");
  }

  std::optional<JacobianFactor> whitened;
  const JacobianFactor* source = jacobian;
  if (source->get_model() && !source->get_model()->isUnit()) {
    whitened.emplace(source->whiten());
    source = &*whitened;
  }

  for (const SparseJacobianBlockWritePlan& block : factorPlan.blocks) {
    if (block.localBlockIndex >= source->size()) {
      return FactorFailure(
          DirectJacobianFailure::StructuralMismatch, factorIndex,
          "planned local Jacobian block index is out of range");
    }
    if (source->keys()[block.localBlockIndex] != block.key) {
      return FactorFailure(DirectJacobianFailure::StructuralMismatch,
                           factorIndex,
                           "Jacobian key order does not match the sparse plan");
    }

    const auto A = source->getA(source->begin() + block.localBlockIndex);
    if (A.rows() != factorPlan.rowCount) {
      return FactorFailure(
          DirectJacobianFailure::StructuralMismatch, factorIndex,
          "Jacobian block row count does not match the sparse plan");
    }
    if (A.cols() != block.width) {
      return FactorFailure(
          DirectJacobianFailure::StructuralMismatch, factorIndex,
          "Jacobian block width does not match the sparse plan");
    }
  }

  const auto b = source->getb();
  if (b.size() != factorPlan.rowCount) {
    return FactorFailure(DirectJacobianFailure::StructuralMismatch,
                         factorIndex,
                         "Jacobian RHS size does not match the sparse plan");
  }

  for (const SparseJacobianBlockWritePlan& block : factorPlan.blocks) {
    const auto A = source->getA(source->begin() + block.localBlockIndex);
    for (Eigen::Index row = 0; row < A.rows(); ++row) {
      for (Eigen::Index column = 0; column < A.cols(); ++column) {
        if (!std::isfinite(A(row, column))) {
          return FactorFailure(
              DirectJacobianFailure::NonFiniteValues, factorIndex,
              "Jacobian block contains a non-finite coefficient");
        }
      }
    }
  }
  for (Eigen::Index row = 0; row < b.size(); ++row) {
    if (!std::isfinite(b(row))) {
      return FactorFailure(DirectJacobianFailure::NonFiniteValues,
                           factorIndex,
                           "Jacobian RHS contains a non-finite coefficient");
    }
  }

  for (const SparseJacobianBlockWritePlan& block : factorPlan.blocks) {
    const auto A = source->getA(source->begin() + block.localBlockIndex);
    for (int localRow = 0; localRow < factorPlan.rowCount; ++localRow) {
      const int globalRow = factorPlan.rowBegin + localRow;
      const int valueBegin =
          plan.rowPointers()[static_cast<size_t>(globalRow)] +
          block.valueOffsetWithinRow;
      for (int localColumn = 0; localColumn < block.width; ++localColumn) {
        output->valuesData()[static_cast<size_t>(valueBegin + localColumn)] =
            A(localRow, localColumn);
      }
    }
  }
  for (int localRow = 0; localRow < factorPlan.rowCount; ++localRow) {
    output->rhsData()[static_cast<size_t>(factorPlan.rowBegin + localRow)] =
        b(localRow);
  }
  return {};
}

}  // namespace

DirectJacobianStatus StreamingSparseJacobianLinearizer::linearize(
    const NonlinearFactorGraph& graph, const Values& values,
    const SparseJacobianColumnLayout& columns,
    const SparseJacobianPlan& plan, HostSparseJacobian* output,
    StreamingLinearizationStats* stats, bool validateStructure) const {
  if (stats) {
    *stats = {};
  }

  DirectJacobianStatus status = ValidateOutput(plan, output);
  if (!status.ok()) {
    return status;
  }
  if (graph.size() != plan.factors().size()) {
    return Failure(DirectJacobianFailure::StructuralMismatch,
                   "nonlinear graph slot count does not match the sparse plan");
  }
  if (validateStructure && !columns.matches(values)) {
    return Failure(DirectJacobianFailure::StructuralMismatch,
                   "Values dimensions do not match the column layout");
  }

  DirectJacobianStatus sendabilityStatus;
  for (size_t factorIndex = 0; factorIndex < graph.size(); ++factorIndex) {
    const NonlinearFactor::shared_ptr& factor = graph[factorIndex];
    if (!factor) {
      continue;
    }

    const bool isSendable = factor->sendable();
    if (stats) {
      if (isSendable) {
        ++stats->sendableFactors;
      } else {
        ++stats->nonSendableFactors;
      }
    }
    if (isSendable != plan.factor(factorIndex).sendable &&
        sendabilityStatus.ok()) {
      sendabilityStatus = FactorFailure(
          DirectJacobianFailure::StructuralMismatch, factorIndex,
          "factor sendability does not match the sparse plan");
    }
  }
  if (!sendabilityStatus.ok()) {
    return sendabilityStatus;
  }

  if (validateStructure && !plan.matches(graph, columns)) {
    return Failure(DirectJacobianFailure::StructuralMismatch,
                   "nonlinear graph structure does not match the sparse plan");
  }

  std::vector<DirectJacobianStatus> statuses(graph.size());

#ifdef GTSAM_USE_TBB
  TbbOpenMPMixedScope threadLimiter;
  tbb::parallel_for(
      tbb::blocked_range<size_t>(0, graph.size()),
      [&](const tbb::blocked_range<size_t>& range) {
        for (size_t factorIndex = range.begin();
             factorIndex != range.end(); ++factorIndex) {
          const NonlinearFactor::shared_ptr& factor = graph[factorIndex];
          if (!factor || !plan.factor(factorIndex).sendable) {
            continue;
          }
          std::shared_ptr<GaussianFactor> gaussian =
              factor->linearize(values);
          statuses[factorIndex] =
              ScatterOneFactor(gaussian, factorIndex, plan, output);
        }
      });

  for (size_t factorIndex = 0; factorIndex < graph.size(); ++factorIndex) {
    const NonlinearFactor::shared_ptr& factor = graph[factorIndex];
    if (!factor || plan.factor(factorIndex).sendable) {
      continue;
    }
    std::shared_ptr<GaussianFactor> gaussian = factor->linearize(values);
    statuses[factorIndex] =
        ScatterOneFactor(gaussian, factorIndex, plan, output);
  }
#else
  for (size_t factorIndex = 0; factorIndex < graph.size(); ++factorIndex) {
    const NonlinearFactor::shared_ptr& factor = graph[factorIndex];
    if (!factor) {
      continue;
    }
    std::shared_ptr<GaussianFactor> gaussian = factor->linearize(values);
    statuses[factorIndex] =
        ScatterOneFactor(gaussian, factorIndex, plan, output);
  }
#endif

  for (size_t factorIndex = 0; factorIndex < statuses.size();
       ++factorIndex) {
    if (!statuses[factorIndex].ok()) {
      return statuses[factorIndex];
    }
  }
  return {};
}

DirectJacobianStatus
StreamingSparseJacobianLinearizer::packGaussianFactorGraph(
    const GaussianFactorGraph& linear, const SparseJacobianPlan& plan,
    HostSparseJacobian* output) const {
  DirectJacobianStatus status = ValidateOutput(plan, output);
  if (!status.ok()) {
    return status;
  }
  if (linear.size() != plan.factors().size()) {
    return Failure(DirectJacobianFailure::StructuralMismatch,
                   "Gaussian graph slot count does not match the sparse plan");
  }

  for (size_t factorIndex = 0; factorIndex < linear.size(); ++factorIndex) {
    status = ScatterOneFactor(linear[factorIndex], factorIndex, plan, output);
    if (!status.ok()) {
      return status;
    }
  }
  return {};
}

}  // namespace gtsam::cuda
