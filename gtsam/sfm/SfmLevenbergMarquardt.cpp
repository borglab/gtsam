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
  Ordering reduced;
};

SchurPartition createSchurPartition(const NonlinearFactorGraph& graph,
                                    const Values& initialValues) {
  SchurPartition result;
  for (Key key : graph.keys()) {
    const Value& value = initialValues.at(key);
    if (dynamic_cast<const GenericValue<Point3>*>(&value) ||
        dynamic_cast<const GenericValue<Unit3>*>(&value)) {
      result.eliminated.push_back(key);
    } else {
      result.reduced.push_back(key);
    }
  }
  return result;
}

Ordering createReducedOrdering(const NonlinearFactorGraph& graph,
                               const Ordering& eliminated) {
  SymbolicFactorGraph symbolic;
  symbolic.reserve(graph.size());
  for (const NonlinearFactor::shared_ptr& factor : graph) {
    if (factor) symbolic.emplace_shared<SymbolicFactor>(*factor);
  }
  const SymbolicFactorGraph::shared_ptr reduced =
      symbolic.eliminatePartialSequential(eliminated).second;
  return Ordering::Metis(*reduced);
}

Ordering resolveReducedOrdering(const NonlinearFactorGraph& graph,
                                const Values& initialValues,
                                const SfmLevenbergMarquardtParams& params) {
  const SchurPartition partition = createSchurPartition(graph, initialValues);

  if (params.ordering) {
    std::unordered_set<Key> missing(partition.reduced.begin(),
                                    partition.reduced.end());
    for (Key key : *params.ordering) {
      if (missing.erase(key) == 0) {
        throw std::invalid_argument(
            "SFM reduced ordering must contain every reduced-system key "
            "exactly once");
      }
    }
    if (!missing.empty()) {
      throw std::invalid_argument(
          "SFM reduced ordering must contain every reduced-system key exactly "
          "once");
    }
    return *params.ordering;
  }
  return createReducedOrdering(graph, partition.eliminated);
}

Ordering createSchurOrdering(const KeySet& graphKeys,
                             const Ordering& reducedOrdering) {
  const std::unordered_set<Key> reducedSet(reducedOrdering.begin(),
                                           reducedOrdering.end());
  Ordering result;
  result.reserve(graphKeys.size());
  for (Key key : graphKeys) {
    if (!reducedSet.count(key)) result.push_back(key);
  }
  result.insert(result.end(), reducedOrdering.begin(), reducedOrdering.end());
  return result;
}

SfmLevenbergMarquardtParams resolveParams(const NonlinearFactorGraph& graph,
                                          const Values& initialValues,
                                          SfmLevenbergMarquardtParams params) {
  if (params.eliminationMode == SfmEliminationMode::Schur) {
    const Ordering reducedOrdering =
        resolveReducedOrdering(graph, initialValues, params);
    params.ordering =
        params.linearSolverType == NonlinearOptimizerParams::MULTIFRONTAL_SOLVER
            ? SfmLevenbergMarquardtOptimizer::CreateSchurOrdering(
                  graph, reducedOrdering)
            : reducedOrdering;
    params.orderingType = Ordering::CUSTOM;
  } else if (!params.ordering) {
    const Ordering reducedOrdering =
        SfmLevenbergMarquardtOptimizer::CreateReducedOrdering(graph,
                                                              initialValues);
    params.ordering = SfmLevenbergMarquardtOptimizer::CreateSchurOrdering(
        graph, reducedOrdering);
    params.orderingType = Ordering::CUSTOM;
  }
  return params;
}

Ordering orderingForReducedGraph(const GaussianFactorGraph& graph,
                                 const Ordering& reducedOrdering) {
  const KeySet keys = graph.keys();
  Ordering result;
  result.reserve(reducedOrdering.size());
  for (Key key : reducedOrdering) {
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
Ordering SfmLevenbergMarquardtOptimizer::CreateReducedOrdering(
    const NonlinearFactorGraph& graph, const Values& initialValues) {
  const SchurPartition partition = createSchurPartition(graph, initialValues);
  return createReducedOrdering(graph, partition.eliminated);
}

/* ************************************************************************* */
Ordering SfmLevenbergMarquardtOptimizer::CreateSchurOrdering(
    const NonlinearFactorGraph& graph, const Ordering& reducedOrdering) {
  return createSchurOrdering(graph.keys(), reducedOrdering);
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
  Ordering schurOrdering;
  Ordering reducedOrdering;
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
    const size_t reducedCount =
        createSchurPartition(graph, initialValues).reduced.size();
    const Ordering& schurOrdering = *sfmParams_.ordering;
    sfmParams_.ordering =
        Ordering(schurOrdering.end() - reducedCount, schurOrdering.end());
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
        "SFM Schur mode requires a resolved reduced ordering");
  }

  if (params.linearSolverType ==
      NonlinearOptimizerParams::MULTIFRONTAL_SOLVER) {
    const Ordering schurOrdering =
        params.ordering->size() == graph.keys().size()
            ? *params.ordering
            : createSchurOrdering(graph.keys(), *params.ordering);
    MultifrontalSolver solver(graph, schurOrdering, params.multifrontalParams);
    solver.eliminateInPlace(graph);
    return solver.updateSolution();
  }

  SchurState& state = *schurState_;
  if (!state.partialSolver) {
    state.reducedOrdering = *params.ordering;
    state.schurOrdering =
        createSchurOrdering(graph.keys(), state.reducedOrdering);
    state.eliminatedCount =
        state.schurOrdering.size() -
        orderingForReducedGraph(graph, state.reducedOrdering).size();
    if (state.eliminatedCount == 0 ||
        state.eliminatedCount >= state.schurOrdering.size()) {
      throw std::invalid_argument(
          "SFM Schur mode requires at least one eliminated Point3/Unit3 "
          "variable and one reduced-system variable");
    }
    MultifrontalParameters pointEliminationParams = params.multifrontalParams;
    pointEliminationParams.qrMode = MultifrontalParameters::QRMode::Off;
    state.partialSolver = std::make_unique<MultifrontalSolver>(
        graph, state.schurOrdering, state.eliminatedCount,
        pointEliminationParams);
  }

  state.partialSolver->eliminatePartialInPlace(graph);
  const GaussianFactorGraph reduced =
      state.partialSolver->remainingFactorGraph();
  const Ordering reducedOrdering =
      orderingForReducedGraph(reduced, state.reducedOrdering);

  LevenbergMarquardtParams reducedParams = sfmParams_;
  reducedParams.ordering = reducedOrdering;
  reducedParams.orderingType = Ordering::CUSTOM;
  const VectorValues reducedDelta =
      NonlinearOptimizer::solve(reduced, reducedParams);

  return state.partialSolver->updateSolution(reducedDelta);
}

}  // namespace gtsam
