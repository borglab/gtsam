/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010-2026, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file SfmLevenbergMarquardt.cpp
 * @brief CPU SFM Levenberg-Marquardt implementation.
 */

#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Unit3.h>
#include <gtsam/sfm/SfmLevenbergMarquardt.h>
#include <gtsam/symbolic/SymbolicFactorGraph.h>

#include <algorithm>
#include <cmath>
#include <iostream>
#include <stdexcept>
#include <unordered_set>
#include <utility>

namespace gtsam {
namespace {

const char* eliminationModeName(SfmEliminationMode mode) {
  switch (mode) {
    case SfmEliminationMode::Full:
      return "Full";
    case SfmEliminationMode::Schur:
      return "Schur";
  }
  return "Unknown";
}

struct SchurPartition {
  Ordering eliminated;
  Ordering retained;
};

SchurPartition createSchurPartition(const NonlinearFactorGraph& graph,
                                    const Values& initialValues) {
  const auto points = initialValues.extract<Point3>();
  const auto directions = initialValues.extract<Unit3>();
  SchurPartition result;
  for (Key key : graph.keys()) {
    if (!initialValues.exists(key)) {
      throw std::invalid_argument(
          "SFM graph key is missing from the initial values");
    }
    if (points.count(key) || directions.count(key)) {
      result.eliminated.push_back(key);
    } else {
      result.retained.push_back(key);
    }
  }
  if (result.eliminated.empty() && !graph.empty()) {
    throw std::invalid_argument(
        "SFM Schur mode found no active Point3 or Unit3 values to eliminate");
  }
  if (result.retained.empty() && !graph.empty()) {
    throw std::invalid_argument(
        "SFM Schur mode found no variables to retain after eliminating "
        "Point3 and Unit3 values");
  }
  return result;
}

Ordering resolveSchurOrdering(const NonlinearFactorGraph& graph,
                              const Values& initialValues,
                              const SfmLevenbergMarquardtParams& params) {
  const SchurPartition partition = createSchurPartition(graph, initialValues);
  const std::unordered_set<Key> retainedSet(partition.retained.begin(),
                                            partition.retained.end());

  if (params.ordering) {
    if (params.ordering->size() != partition.retained.size()) {
      throw std::invalid_argument(
          "SFM Schur ordering must contain every active non-Point3/Unit3 "
          "variable exactly once");
    }
    std::unordered_set<Key> seen;
    for (Key key : *params.ordering) {
      if (!retainedSet.count(key)) {
        throw std::invalid_argument(
            "SFM Schur ordering may contain retained non-Point3/Unit3 keys "
            "only");
      }
      if (!seen.insert(key).second) {
        throw std::invalid_argument(
            "SFM Schur ordering contains a duplicate retained key");
      }
    }
    return *params.ordering;
  }
  return SfmLevenbergMarquardtOptimizer::CreateSchurOrdering(graph,
                                                             initialValues);
}

template <class FACTOR_GRAPH>
Ordering completeSchurOrdering(const FACTOR_GRAPH& graph,
                               const Ordering& schurOrdering) {
  const KeySet graphKeys = graph.keys();
  const std::unordered_set<Key> retainedSet(schurOrdering.begin(),
                                            schurOrdering.end());
  Ordering result;
  result.reserve(graphKeys.size());
  for (Key key : graphKeys) {
    if (!retainedSet.count(key)) result.push_back(key);
  }
  for (Key key : schurOrdering) {
    if (graphKeys.count(key)) result.push_back(key);
  }
  return result;
}

SfmLevenbergMarquardtParams resolveParams(const NonlinearFactorGraph& graph,
                                          const Values& initialValues,
                                          SfmLevenbergMarquardtParams params) {
  if (params.eliminationMode == SfmEliminationMode::Schur) {
    const Ordering schurOrdering =
        resolveSchurOrdering(graph, initialValues, params);
    params.ordering =
        params.linearSolverType == NonlinearOptimizerParams::MULTIFRONTAL_SOLVER
            ? SfmLevenbergMarquardtOptimizer::CreatePointFirstOrdering(
                  graph, schurOrdering)
            : schurOrdering;
    params.orderingType = Ordering::CUSTOM;
  } else if (!params.ordering) {
    const Ordering schurOrdering =
        SfmLevenbergMarquardtOptimizer::CreateSchurOrdering(graph,
                                                            initialValues);
    params.ordering = SfmLevenbergMarquardtOptimizer::CreatePointFirstOrdering(
        graph, schurOrdering);
    params.orderingType = Ordering::CUSTOM;
  }
  return params;
}

Ordering retainedOrdering(const GaussianFactorGraph& graph,
                          const Ordering& schurOrdering) {
  const KeySet keys = graph.keys();
  Ordering result;
  result.reserve(schurOrdering.size());
  for (Key key : schurOrdering) {
    if (keys.count(key)) result.push_back(key);
  }
  return result;
}

}  // namespace

/* ************************************************************************* */
SfmLevenbergMarquardtParams::SfmLevenbergMarquardtParams() {
  setLinearSolver(NonlinearOptimizerParams::MULTIFRONTAL_SOLVER);
}

/* ************************************************************************* */
Ordering SfmLevenbergMarquardtOptimizer::CreateSchurOrdering(
    const NonlinearFactorGraph& graph, const Values& initialValues) {
  const SchurPartition partition = createSchurPartition(graph, initialValues);
  SymbolicFactorGraph symbolic;
  symbolic.reserve(graph.size());
  for (const NonlinearFactor::shared_ptr& factor : graph) {
    if (factor) symbolic.emplace_shared<SymbolicFactor>(*factor);
  }
  const SymbolicFactorGraph::shared_ptr reduced =
      symbolic.eliminatePartialSequential(partition.eliminated).second;
  Ordering schurOrdering = Ordering::Metis(*reduced);
  if (schurOrdering.size() != partition.retained.size()) {
    throw std::runtime_error("SFM automatic Schur ordering is incomplete");
  }
  return schurOrdering;
}

/* ************************************************************************* */
Ordering SfmLevenbergMarquardtOptimizer::CreatePointFirstOrdering(
    const NonlinearFactorGraph& graph, const Ordering& schurOrdering) {
  return completeSchurOrdering(graph, schurOrdering);
}

/* ************************************************************************* */
SfmLevenbergMarquardtParams SfmLevenbergMarquardtParams::legacyDefaults() {
  SfmLevenbergMarquardtParams result;
  static_cast<LevenbergMarquardtParams&>(result) =
      LevenbergMarquardtParams::LegacyDefaults();
  result.eliminationMode = SfmEliminationMode::Full;
  result.setLinearSolver(NonlinearOptimizerParams::MULTIFRONTAL_SOLVER);
  return result;
}

/* ************************************************************************* */
SfmLevenbergMarquardtParams SfmLevenbergMarquardtParams::ceresDefaults() {
  SfmLevenbergMarquardtParams result;
  static_cast<LevenbergMarquardtParams&>(result) =
      LevenbergMarquardtParams::CeresDefaults();
  result.eliminationMode = SfmEliminationMode::Full;
  result.setLinearSolver(NonlinearOptimizerParams::MULTIFRONTAL_SOLVER);
  return result;
}

/* ************************************************************************* */
void SfmLevenbergMarquardtParams::print(const std::string& str) const {
  LevenbergMarquardtParams::print(str);
  std::cout << "       SFM elimination mode: "
            << eliminationModeName(eliminationMode) << '\n';
}

/* ************************************************************************* */
bool SfmLevenbergMarquardtParams::equals(
    const SfmLevenbergMarquardtParams& other, double tol) const {
  return NonlinearOptimizerParams::equals(other, tol) &&
         std::abs(lambdaInitial - other.lambdaInitial) <= tol &&
         std::abs(lambdaFactor - other.lambdaFactor) <= tol &&
         std::abs(lambdaUpperBound - other.lambdaUpperBound) <= tol &&
         std::abs(lambdaLowerBound - other.lambdaLowerBound) <= tol &&
         std::abs(minModelFidelity - other.minModelFidelity) <= tol &&
         verbosityLM == other.verbosityLM && logFile == other.logFile &&
         useFixedLambdaFactor == other.useFixedLambdaFactor &&
         dampingParams.diagonalDamping == other.dampingParams.diagonalDamping &&
         dampingParams.exactHessianDiagonal ==
             other.dampingParams.exactHessianDiagonal &&
         std::abs(dampingParams.minDiagonal -
                  other.dampingParams.minDiagonal) <= tol &&
         std::abs(dampingParams.maxDiagonal -
                  other.dampingParams.maxDiagonal) <= tol &&
         eliminationMode == other.eliminationMode;
}

struct SfmLevenbergMarquardtOptimizer::SchurState {
  std::unique_ptr<MultifrontalSolver> partialSolver;
  Ordering fullOrdering;
  Ordering retainedOrdering;
  size_t eliminatedCount = 0;
};

/* ************************************************************************* */
SfmLevenbergMarquardtOptimizer::SfmLevenbergMarquardtOptimizer(
    const NonlinearFactorGraph& graph, const Values& initialValues,
    const SfmLevenbergMarquardtParams& params)
    : LevenbergMarquardtOptimizer(graph, initialValues,
                                  resolveParams(graph, initialValues, params)),
      sfmParams_(params),
      schurState_(std::make_unique<SchurState>()) {
  const SfmEliminationMode eliminationMode = sfmParams_.eliminationMode;
  static_cast<LevenbergMarquardtParams&>(sfmParams_) =
      LevenbergMarquardtOptimizer::params_;
  sfmParams_.eliminationMode = eliminationMode;
  if (eliminationMode == SfmEliminationMode::Schur &&
      sfmParams_.linearSolverType ==
          NonlinearOptimizerParams::MULTIFRONTAL_SOLVER) {
    const size_t retainedCount =
        createSchurPartition(graph, initialValues).retained.size();
    const Ordering& fullOrdering = *sfmParams_.ordering;
    sfmParams_.ordering =
        Ordering(fullOrdering.end() - retainedCount, fullOrdering.end());
  }
}

SfmLevenbergMarquardtOptimizer::~SfmLevenbergMarquardtOptimizer() = default;

/* ************************************************************************* */
bool SfmLevenbergMarquardtOptimizer::ensureMultifrontalSolver(
    const NonlinearOptimizerParams& params, const Values& values) const {
  if (sfmParams_.eliminationMode == SfmEliminationMode::Schur &&
      params.linearSolverType !=
          NonlinearOptimizerParams::MULTIFRONTAL_SOLVER) {
    return false;
  }
  return LevenbergMarquardtOptimizer::ensureMultifrontalSolver(params, values);
}

/* ************************************************************************* */
VectorValues SfmLevenbergMarquardtOptimizer::solve(
    const GaussianFactorGraph& graph,
    const NonlinearOptimizerParams& params) const {
  if (sfmParams_.eliminationMode == SfmEliminationMode::Full) {
    return NonlinearOptimizer::solve(graph, params);
  }
  if (!params.ordering) {
    throw std::logic_error(
        "SFM Schur mode requires a resolved retained-variable ordering");
  }

  if (params.linearSolverType ==
      NonlinearOptimizerParams::MULTIFRONTAL_SOLVER) {
    const Ordering ordering =
        params.ordering->size() == graph.keys().size()
            ? *params.ordering
            : completeSchurOrdering(graph, *params.ordering);
    MultifrontalSolver solver(graph, ordering, params.multifrontalParams);
    solver.eliminateInPlace(graph);
    return solver.updateSolution();
  }

  SchurState& state = *schurState_;
  if (!state.partialSolver) {
    state.retainedOrdering = *params.ordering;
    state.fullOrdering = completeSchurOrdering(graph, state.retainedOrdering);
    state.eliminatedCount =
        state.fullOrdering.size() -
        retainedOrdering(graph, state.retainedOrdering).size();
    if (state.eliminatedCount == 0 ||
        state.eliminatedCount >= state.fullOrdering.size()) {
      throw std::invalid_argument(
          "SFM Schur mode requires at least one eliminated Point3/Unit3 "
          "variable and one retained variable");
    }
    MultifrontalParameters pointEliminationParams = params.multifrontalParams;
    pointEliminationParams.qrMode = MultifrontalParameters::QRMode::Off;
    state.partialSolver = std::make_unique<MultifrontalSolver>(
        graph, state.fullOrdering, state.eliminatedCount,
        pointEliminationParams);
  }

  state.partialSolver->eliminatePartialInPlace(graph);
  const GaussianFactorGraph reduced =
      state.partialSolver->remainingFactorGraph();
  const Ordering reducedOrdering =
      retainedOrdering(reduced, state.retainedOrdering);

  LevenbergMarquardtParams reducedParams = sfmParams_;
  reducedParams.ordering = reducedOrdering;
  reducedParams.orderingType = Ordering::CUSTOM;
  const VectorValues retainedDelta =
      NonlinearOptimizer::solve(reduced, reducedParams);

  return state.partialSolver->updateSolution(retainedDelta);
}

}  // namespace gtsam
