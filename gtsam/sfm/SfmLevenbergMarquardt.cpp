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

#include <gtsam/sfm/SfmData.h>
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

KeyVector activeCameraKeys(const NonlinearFactorGraph& graph,
                           const Values& initialValues) {
  const auto cameras = initialValues.extract<SfmCamera>();
  KeyVector result;
  for (Key key : graph.keys()) {
    if (cameras.find(key) != cameras.end()) result.push_back(key);
  }
  if (result.empty() && !graph.empty()) {
    throw std::invalid_argument(
        "SFM Schur mode found no active SfmCamera values");
  }
  return result;
}

Ordering resolveCameraOrdering(const NonlinearFactorGraph& graph,
                               const Values& initialValues,
                               const SfmLevenbergMarquardtParams& params) {
  const KeyVector cameras = activeCameraKeys(graph, initialValues);
  const std::unordered_set<Key> cameraSet(cameras.begin(), cameras.end());

  if (params.ordering) {
    if (params.ordering->size() != cameras.size()) {
      throw std::invalid_argument(
          "SFM Schur ordering must contain every active camera exactly once");
    }
    std::unordered_set<Key> seen;
    for (Key key : *params.ordering) {
      if (!cameraSet.count(key)) {
        throw std::invalid_argument(
            "SFM Schur ordering may contain camera keys only");
      }
      if (!seen.insert(key).second) {
        throw std::invalid_argument(
            "SFM Schur ordering contains a duplicate camera key");
      }
    }
    return *params.ordering;
  }
  return SfmLevenbergMarquardtOptimizer::CreateCameraOrdering(graph,
                                                              initialValues);
}

template <class FACTOR_GRAPH>
Ordering completeSchurOrdering(const FACTOR_GRAPH& graph,
                               const Ordering& cameraOrdering) {
  const KeySet graphKeys = graph.keys();
  const std::unordered_set<Key> cameraSet(cameraOrdering.begin(),
                                          cameraOrdering.end());
  Ordering result;
  result.reserve(graphKeys.size());
  for (Key key : graphKeys) {
    if (!cameraSet.count(key)) result.push_back(key);
  }
  for (Key key : cameraOrdering) {
    if (graphKeys.count(key)) result.push_back(key);
  }
  return result;
}

SfmLevenbergMarquardtParams resolveParams(const NonlinearFactorGraph& graph,
                                          const Values& initialValues,
                                          SfmLevenbergMarquardtParams params) {
  if (params.eliminationMode == SfmEliminationMode::Schur) {
    const Ordering cameraOrdering =
        resolveCameraOrdering(graph, initialValues, params);
    params.ordering =
        params.linearSolverType == NonlinearOptimizerParams::MULTIFRONTAL_SOLVER
            ? SfmLevenbergMarquardtOptimizer::CreatePointFirstOrdering(
                  graph, cameraOrdering)
            : cameraOrdering;
    params.orderingType = Ordering::CUSTOM;
  } else if (!params.ordering) {
    const Ordering cameraOrdering =
        SfmLevenbergMarquardtOptimizer::CreateCameraOrdering(graph,
                                                             initialValues);
    params.ordering = SfmLevenbergMarquardtOptimizer::CreatePointFirstOrdering(
        graph, cameraOrdering);
    params.orderingType = Ordering::CUSTOM;
  }
  return params;
}

Ordering retainedOrdering(const GaussianFactorGraph& graph,
                          const Ordering& cameraOrdering) {
  const KeySet keys = graph.keys();
  Ordering result;
  result.reserve(cameraOrdering.size());
  for (Key key : cameraOrdering) {
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
Ordering SfmLevenbergMarquardtOptimizer::CreateCameraOrdering(
    const NonlinearFactorGraph& graph, const Values& initialValues) {
  const KeyVector cameras = activeCameraKeys(graph, initialValues);
  const std::unordered_set<Key> cameraSet(cameras.begin(), cameras.end());

  Ordering eliminated;
  for (Key key : graph.keys()) {
    if (!cameraSet.count(key)) eliminated.push_back(key);
  }
  SymbolicFactorGraph symbolic;
  symbolic.reserve(graph.size());
  for (const NonlinearFactor::shared_ptr& factor : graph) {
    if (factor) symbolic.emplace_shared<SymbolicFactor>(*factor);
  }
  const SymbolicFactorGraph::shared_ptr reduced =
      symbolic.eliminatePartialSequential(eliminated).second;
  Ordering cameraOrdering = Ordering::Metis(*reduced);
  if (cameraOrdering.size() != cameras.size()) {
    throw std::runtime_error(
        "SFM Schur automatic camera ordering is incomplete");
  }
  return cameraOrdering;
}

/* ************************************************************************* */
Ordering SfmLevenbergMarquardtOptimizer::CreatePointFirstOrdering(
    const NonlinearFactorGraph& graph, const Ordering& cameraOrdering) {
  return completeSchurOrdering(graph, cameraOrdering);
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
  Ordering cameraOrdering;
  size_t pointCount = 0;
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
    const size_t cameraCount = activeCameraKeys(graph, initialValues).size();
    const Ordering& fullOrdering = *sfmParams_.ordering;
    sfmParams_.ordering =
        Ordering(fullOrdering.end() - cameraCount, fullOrdering.end());
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
        "SFM Schur mode requires a resolved camera ordering");
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
    state.cameraOrdering = *params.ordering;
    state.fullOrdering = completeSchurOrdering(graph, state.cameraOrdering);
    state.pointCount = state.fullOrdering.size() -
                       retainedOrdering(graph, state.cameraOrdering).size();
    if (state.pointCount == 0 ||
        state.pointCount >= state.fullOrdering.size()) {
      throw std::invalid_argument(
          "SFM Schur mode requires at least one eliminated point and one "
          "retained camera");
    }
    MultifrontalParameters pointEliminationParams = params.multifrontalParams;
    pointEliminationParams.qrMode = MultifrontalParameters::QRMode::Off;
    state.partialSolver = std::make_unique<MultifrontalSolver>(
        graph, state.fullOrdering, state.pointCount, pointEliminationParams);
  }

  state.partialSolver->eliminatePartialInPlace(graph);
  const GaussianFactorGraph reduced =
      state.partialSolver->remainingFactorGraph();
  const Ordering reducedOrdering =
      retainedOrdering(reduced, state.cameraOrdering);

  LevenbergMarquardtParams reducedParams = sfmParams_;
  reducedParams.ordering = reducedOrdering;
  reducedParams.orderingType = Ordering::CUSTOM;
  const VectorValues cameraDelta =
      NonlinearOptimizer::solve(reduced, reducedParams);

  return state.partialSolver->updateSolution(cameraDelta);
}

}  // namespace gtsam
