/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file SparseDualSolver.cpp
 * Numerical multiplier recovery for the shared active-set solver.
 *
 * The ordinary sparse graph solve and rank-deficient constrained-QR fallback
 * are isolated here so the active-set iteration can inspect their residuals
 * without embedding elimination details. Only frontal multiplier columns are
 * pivoted; original separator equations and stationarity checks are preserved.
 */

#include <gtsam/constrained/internal/SparseDualSolver.h>
#include <gtsam/linear/JacobianFactor.h>
#include <gtsam/linear/linearExceptions.h>

#include <Eigen/QR>
#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <tuple>

namespace gtsam {

namespace internal {
namespace {

/* ************************************************************************* */
std::pair<double, double> DualGraphResidual(const GaussianFactorGraph& graph,
                                            const VectorValues& multipliers) {
  for (const auto& [_, value] : multipliers)
    if (!value.allFinite())
      throw std::runtime_error("ActiveSetSolver: non-finite multipliers.");
  double residual = 0.0, rhsNorm = 0.0;
  for (const auto& factor : graph) {
    const JacobianFactor whitened = JacobianFactor(*factor).whiten();
    residual = std::max(
        residual,
        whitened.unweighted_error(multipliers).lpNorm<Eigen::Infinity>());
    rhsNorm = std::max(rhsNorm, whitened.getb().lpNorm<Eigen::Infinity>());
  }
  return {residual, rhsNorm};
}

/* ************************************************************************* */
// Retain original column scales: a dependent column can reach a later clique
// as roundoff alone, which must not become its new reference scale.
std::map<Key, double> dualColumnScales(const GaussianFactorGraph& graph) {
  std::map<Key, double> scales;
  for (const auto& factor : graph) {
    const JacobianFactor whitened = JacobianFactor(*factor).whiten();
    for (auto key = whitened.begin(); key != whitened.end(); ++key) {
      const auto block = whitened.getA(key);
      if (block.cols() != 1)
        throw std::logic_error("Expected scalar multiplier keys");
      scales[*key] = std::hypot(scales[*key], block.stableNorm());
    }
  }
  return scales;
}

/* ************************************************************************* */
// Pivot only among frontal columns. Separator equations must survive unchanged
// so that the rest of the Bayes tree still solves the original least squares.
KeyVector freeDualFrontalKeys(const JacobianFactor& joint,
                              const Ordering& frontals,
                              const std::map<Key, double>& scales) {
  if (joint.rows() == 0) return KeyVector(frontals.begin(), frontals.end());
  Matrix frontalMatrix(joint.rows(), frontals.size());
  for (size_t column = 0; column < frontals.size(); ++column) {
    const Key key = frontals[column];
    const double scale = scales.at(key);
    frontalMatrix.col(column) =
        joint.getA(joint.find(key)) / (scale > 0.0 ? scale : 1.0);
  }
  Eigen::ColPivHouseholderQR<Matrix> qr(frontalMatrix);
  const double threshold = 64.0 * std::numeric_limits<double>::epsilon() *
                           std::max(frontalMatrix.rows(), frontalMatrix.cols());
  // Eigen's threshold is relative to the largest pivot; ours is relative to
  // the original columns after normalization, including at a nearly zero root.
  qr.setThreshold(qr.maxPivot() > 0.0 ? threshold / qr.maxPivot() : 1.0);
  KeyVector freeKeys;
  for (Eigen::Index column = qr.rank(); column < frontalMatrix.cols(); ++column)
    freeKeys.push_back(frontals[qr.colsPermutation().indices()(column)]);
  return freeKeys;
}

/* ************************************************************************* */
// Dual-only elimination: choose zero for free scalar multipliers, then return
// an ordinary nonsingular conditional through GTSAM's constrained QR. For
// exact dependencies this selects one solution without changing the reduced
// objective on the separator. The caller checks the original graph residual.
GaussianFactorGraph::EliminationResult eliminateDualWithGauge(
    const GaussianFactorGraph& factors, const Ordering& frontals,
    const std::map<Key, double>& scales) {
  auto joint = std::make_shared<JacobianFactor>(factors, frontals);
  *joint = joint->whiten();
  GaussianFactorGraph gauged;
  gauged.push_back(joint);
  for (Key key : freeDualFrontalKeys(*joint, frontals, scales))
    gauged.emplace_shared<JacobianFactor>(key, Matrix::Ones(1, 1),
                                          Vector::Zero(1),
                                          noiseModel::Constrained::All(1));
  return EliminateQR(gauged, frontals);
}

/* ************************************************************************* */
DualSolveResult solveRankDeficientDualGraph(const GaussianFactorGraph& graph,
                                            const Ordering& ordering,
                                            double tolerance) {
  const auto scales = dualColumnScales(graph);
  DualSolveResult result;
  result.multipliers = graph.optimize(
      ordering,
      [&](const GaussianFactorGraph& factors, const Ordering& frontals) {
        return eliminateDualWithGauge(factors, frontals, scales);
      });
  double rhsNorm;
  std::tie(result.residual, rhsNorm) =
      DualGraphResidual(graph, result.multipliers);
  result.threshold = std::max(1e-12, tolerance) * (1.0 + rhsNorm);
  return result;
}

}  // namespace

/* ************************************************************************* */
DualSolveResult solveSparseDualGraph(const GaussianFactorGraph& graph,
                                     const Ordering& ordering,
                                     double tolerance,
                                     bool checkStationarity) {
  if (graph.empty()) return {};
  try {
    // The usual graph solve is faster, but a successful factorization alone
    // does not prove that an overdetermined stationarity equation is exact.
    DualSolveResult result;
    result.multipliers = graph.optimize(ordering);
    if (!checkStationarity) return result;
    double rhsNorm;
    std::tie(result.residual, rhsNorm) =
        DualGraphResidual(graph, result.multipliers);
    result.threshold = std::max(1e-12, tolerance) * (1.0 + rhsNorm);
    if (result.consistent()) return result;
  } catch (const IndeterminateSystemException&) {
    // The graph QR fallback fixes free multiplier coordinates clique by clique.
  }
  DualSolveResult result =
      solveRankDeficientDualGraph(graph, ordering, tolerance);
  if (!checkStationarity && !result.consistent())
    throw std::runtime_error("Multiplier graph QR: inconsistent stationarity");
  return result;
}

}  // namespace internal
}  // namespace gtsam
