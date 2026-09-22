/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    StreamingSparseJacobianLinearizer.cpp
 * @brief   Linearizes a factor graph into a preplanned CSR host buffer
 * @author  Ruogu Li
 * @date    Jul 14, 2026
 */

#include <gtsam/base/types.h>
#include <gtsam/config.h>
#include <gtsam/linear/GaussianFactorGraph.h>
#include <gtsam/linear/JacobianFactor.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/nonlinear/cuda/internal/StreamingSparseJacobianLinearizer.h>

#ifdef GTSAM_USE_TBB
#include <tbb/blocked_range.h>
#include <tbb/parallel_for.h>
#endif

#include <chrono>
#include <cmath>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace gtsam::cuda {
namespace {

using ProfileClock = std::chrono::steady_clock;

struct PerFactorTiming {
  double factorLinearization = 0.0;
  double csrPacking = 0.0;
};

bool columnsMatchValues(const KeyInfo& columns, const Values& values) {
  const auto dimensions = values.dims();
  if (columns.size() != dimensions.size()) return false;
  for (const auto& [key, entry] : columns) {
    const auto found = dimensions.find(key);
    if (found == dimensions.end() || found->second != entry.dim) return false;
  }
  return true;
}

DirectJacobianStatus failureStatus(DirectJacobianFailure failure,
                             std::string detail) {
  DirectJacobianStatus status;
  status.failure = failure;
  status.detail = std::move(detail);
  return status;
}

DirectJacobianStatus factorFailureStatus(DirectJacobianFailure failure,
                                   size_t factorIndex, std::string detail) {
  DirectJacobianStatus status;
  status.failure = failure;
  status.factorIndex = factorIndex;
  status.detail = std::move(detail);
  return status;
}

DirectJacobianStatus validateOutput(const SparseJacobianPlan& plan,
                                    const HostSparseJacobian* output) {
  if (!output) {
    return failureStatus(DirectJacobianFailure::StructuralMismatch, "output is null");
  }
  if (output->structuralFingerprint() != plan.structuralFingerprint()) {
    return failureStatus(
        DirectJacobianFailure::StructuralMismatch,
        "output structural fingerprint does not match the sparse plan");
  }
  if (output->valuesSize() != static_cast<size_t>(plan.nonzeros())) {
    return failureStatus(DirectJacobianFailure::StructuralMismatch,
                   "output values size does not match the sparse plan");
  }
  if (output->rhsSize() != static_cast<size_t>(plan.rows())) {
    return failureStatus(DirectJacobianFailure::StructuralMismatch,
                   "output RHS size does not match the sparse plan");
  }
  return {};
}

DirectJacobianStatus scatterOneFactor(
    const std::shared_ptr<GaussianFactor>& gaussian, size_t factorIndex,
    const SparseJacobianPlan& plan, HostSparseJacobian* output) {
  if (!gaussian) {
    return {};
  }

  const auto* jacobian = dynamic_cast<const JacobianFactor*>(gaussian.get());
  if (!jacobian) {
    return factorFailureStatus(DirectJacobianFailure::UnsupportedGaussianFactor,
                         factorIndex,
                         "linearization result is not a JacobianFactor");
  }

  const SparseJacobianFactorWritePlan& factorPlan = plan.factor(factorIndex);
  if (jacobian->rows() != static_cast<size_t>(factorPlan.rowCount)) {
    return factorFailureStatus(DirectJacobianFailure::StructuralMismatch, factorIndex,
                         "Jacobian row count does not match the sparse plan");
  }
  if (jacobian->size() != factorPlan.blocks.size()) {
    return factorFailureStatus(DirectJacobianFailure::StructuralMismatch, factorIndex,
                         "Jacobian block count does not match the sparse plan");
  }
  if (jacobian->isConstrained()) {
    return factorFailureStatus(DirectJacobianFailure::ConstrainedFactor, factorIndex,
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
      return factorFailureStatus(
          DirectJacobianFailure::StructuralMismatch, factorIndex,
          "planned local Jacobian block index is out of range");
    }
    if (source->keys()[block.localBlockIndex] != block.key) {
      return factorFailureStatus(DirectJacobianFailure::StructuralMismatch,
                           factorIndex,
                           "Jacobian key order does not match the sparse plan");
    }

    const auto A = source->getA(source->begin() + block.localBlockIndex);
    if (A.rows() != factorPlan.rowCount) {
      return factorFailureStatus(
          DirectJacobianFailure::StructuralMismatch, factorIndex,
          "Jacobian block row count does not match the sparse plan");
    }
    if (A.cols() != block.width) {
      return factorFailureStatus(
          DirectJacobianFailure::StructuralMismatch, factorIndex,
          "Jacobian block width does not match the sparse plan");
    }
  }

  const auto b = source->getb();
  if (b.size() != factorPlan.rowCount) {
    return factorFailureStatus(DirectJacobianFailure::StructuralMismatch, factorIndex,
                         "Jacobian RHS size does not match the sparse plan");
  }

  for (const SparseJacobianBlockWritePlan& block : factorPlan.blocks) {
    const auto A = source->getA(source->begin() + block.localBlockIndex);
    for (Eigen::Index row = 0; row < A.rows(); ++row) {
      for (Eigen::Index column = 0; column < A.cols(); ++column) {
        if (!std::isfinite(A(row, column))) {
          return factorFailureStatus(
              DirectJacobianFailure::NonFiniteValues, factorIndex,
              "Jacobian block contains a non-finite coefficient");
        }
      }
    }
  }
  for (Eigen::Index row = 0; row < b.size(); ++row) {
    if (!std::isfinite(b(row))) {
      return factorFailureStatus(DirectJacobianFailure::NonFiniteValues, factorIndex,
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
    const KeyInfo& columns, const SparseJacobianPlan& plan,
    HostSparseJacobian* output, StreamingLinearizationStats* stats,
    bool validateStructure, StreamingLinearizationProfile* profile) const {
  if (stats) {
    *stats = {};
  }
  if (profile) {
    *profile = {};
  }

  DirectJacobianStatus status = validateOutput(plan, output);
  if (!status.ok()) {
    return status;
  }
  if (graph.size() != plan.factors().size()) {
    return failureStatus(DirectJacobianFailure::StructuralMismatch,
                   "nonlinear graph slot count does not match the sparse plan");
  }
  if (validateStructure && !columnsMatchValues(columns, values)) {
    return failureStatus(DirectJacobianFailure::StructuralMismatch,
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
      sendabilityStatus =
          factorFailureStatus(DirectJacobianFailure::StructuralMismatch, factorIndex,
                        "factor sendability does not match the sparse plan");
    }
  }
  if (!sendabilityStatus.ok()) {
    return sendabilityStatus;
  }

  if (validateStructure && !plan.matches(graph, columns)) {
    return failureStatus(DirectJacobianFailure::StructuralMismatch,
                   "nonlinear graph structure does not match the sparse plan");
  }

  std::vector<DirectJacobianStatus> statuses(graph.size());
  std::vector<PerFactorTiming> timingSlots;
  if (profile) {
    timingSlots.resize(graph.size());
  }

  const auto linearizeAndScatter = [&](size_t factorIndex) {
    const NonlinearFactor::shared_ptr& factor = graph[factorIndex];
    if (!profile) {
      std::shared_ptr<GaussianFactor> gaussian = factor->linearize(values);
      statuses[factorIndex] =
          scatterOneFactor(gaussian, factorIndex, plan, output);
      return;
    }

    PerFactorTiming& timing = timingSlots[factorIndex];
    const ProfileClock::time_point linearizationBegin = ProfileClock::now();
    std::shared_ptr<GaussianFactor> gaussian = factor->linearize(values);
    const ProfileClock::time_point linearizationEnd = ProfileClock::now();
    timing.factorLinearization =
        std::chrono::duration<double>(linearizationEnd - linearizationBegin)
            .count();

    const ProfileClock::time_point packingBegin = ProfileClock::now();
    DirectJacobianStatus factorStatus =
        scatterOneFactor(gaussian, factorIndex, plan, output);
    const ProfileClock::time_point packingEnd = ProfileClock::now();
    timing.csrPacking =
        std::chrono::duration<double>(packingEnd - packingBegin).count();
    statuses[factorIndex] = std::move(factorStatus);
  };

#ifdef GTSAM_USE_TBB
  // Preserve NonlinearFactorGraph's sendability rule while writing factors
  // directly into disjoint preplanned CSR ranges. Calling graph.linearize()
  // here would materialize the intermediate GaussianFactorGraph that this
  // transfer-boundary path is specifically designed to avoid.
  TbbOpenMPMixedScope threadLimiter;
  tbb::parallel_for(tbb::blocked_range<size_t>(0, graph.size()),
                    [&](const tbb::blocked_range<size_t>& range) {
                      for (size_t factorIndex = range.begin();
                           factorIndex != range.end(); ++factorIndex) {
                        const NonlinearFactor::shared_ptr& factor =
                            graph[factorIndex];
                        if (!factor || !plan.factor(factorIndex).sendable) {
                          continue;
                        }
                        linearizeAndScatter(factorIndex);
                      }
                    });

  for (size_t factorIndex = 0; factorIndex < graph.size(); ++factorIndex) {
    const NonlinearFactor::shared_ptr& factor = graph[factorIndex];
    if (!factor || plan.factor(factorIndex).sendable) {
      continue;
    }
    linearizeAndScatter(factorIndex);
  }
#else
  for (size_t factorIndex = 0; factorIndex < graph.size(); ++factorIndex) {
    const NonlinearFactor::shared_ptr& factor = graph[factorIndex];
    if (!factor) {
      continue;
    }
    linearizeAndScatter(factorIndex);
  }
#endif

  if (profile) {
    for (const PerFactorTiming& timing : timingSlots) {
      profile->factorLinearizationCpuSum += timing.factorLinearization;
      profile->csrPackingCpuSum += timing.csrPacking;
    }
  }

  for (size_t factorIndex = 0; factorIndex < statuses.size(); ++factorIndex) {
    if (!statuses[factorIndex].ok()) {
      return statuses[factorIndex];
    }
  }
  return {};
}

DirectJacobianStatus StreamingSparseJacobianLinearizer::packGaussianFactorGraph(
    const GaussianFactorGraph& linear, const SparseJacobianPlan& plan,
    HostSparseJacobian* output) const {
  DirectJacobianStatus status = validateOutput(plan, output);
  if (!status.ok()) {
    return status;
  }
  if (linear.size() != plan.factors().size()) {
    return failureStatus(DirectJacobianFailure::StructuralMismatch,
                   "Gaussian graph slot count does not match the sparse plan");
  }

  for (size_t factorIndex = 0; factorIndex < linear.size(); ++factorIndex) {
    status = scatterOneFactor(linear[factorIndex], factorIndex, plan, output);
    if (!status.ok()) {
      return status;
    }
  }
  return {};
}

}  // namespace gtsam::cuda
