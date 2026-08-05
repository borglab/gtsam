/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    LiftedSDPProblem.h
 * @brief   Lifted SDP formulations and MOSEK-backed solvers for QCQP problems.
 * @author  Avinash Subramanian
 */

#pragma once

#include <gtsam/config.h>
#include <gtsam/constrained/QcqpProblem.h>
#include <gtsam/inference/Key.h>
#include <gtsam/inference/Ordering.h>
#include <gtsam/symbolic/SymbolicBayesTree.h>

#include <cmath>
#include <map>
#include <memory>
#include <stdexcept>
#include <string>
#include <type_traits>
#include <vector>

namespace gtsam {

/// Formulation tag for a single monolithic positive semidefinite variable.
struct MonolithicSDP {};

/// Formulation tag for a chordally decomposed positive semidefinite variable.
struct ChordalSDP {};

/// Variable ordering strategy used to construct the chordal decomposition.
enum class ChordalOrderingType {
  Metis,  ///< Nested-dissection ordering computed with METIS.
  Colamd  ///< Column approximate minimum-degree ordering.
};

/// Solver tag for the optional MOSEK Fusion backend.
struct MosekSDPSolver {};

/**
 * Lifted SDP problem selected by a formulation tag and solver tag.
 *
 * @tparam SDPFormulation Lifted SDP formulation.
 * @tparam SDPSolver SDP solver backend.
 */
template <typename SDPFormulation, typename SDPSolver>
class LiftedSDPProblem;

#ifdef GTSAM_USE_MOSEK
/**
 * Monolithic lifted SDP relaxation backed by MOSEK Fusion.
 *
 * The problem owns its MOSEK model. Call solve() before querying solver
 * results or recovered variables.
 */
template <>
class GTSAM_EXPORT LiftedSDPProblem<MonolithicSDP, MosekSDPSolver> {
 public:
  /**
   * Construct the monolithic SDP relaxation of a QCQP problem.
   *
   * @param problem QCQP to relax.
   */
  explicit LiftedSDPProblem(const QcqpProblem& problem);

  /// Dispose of the owned MOSEK model.
  ~LiftedSDPProblem();

  /**
   * Solve the SDP with MOSEK.
   *
   * @param mosekParams MOSEK solver parameter overrides.
   * @return True after MOSEK completes the solve.
   */
  bool solve(const std::map<std::string, double>& mosekParams = {});

  /// Return the primal objective value after solve().
  double objectiveValue() const;

  /// Return the MOSEK problem status after solve().
  std::string problemStatus() const;

  /// Return MOSEK's optimizer time in seconds after solve().
  double solveTimeSeconds() const;

  /// Return one keyed D=1 QCQP vector per diagonal SDP block after solve().
  Values qcqpValues() const;

  /// Return largest-to-second-largest eigenvalue ratios for recovered blocks.
  std::vector<double> variableEVRs() const;

  /// Return QCQP variable keys in SDP block order.
  const KeyVector& orderedKeys() const;

  /// Return the QCQP dimension associated with each ordered key.
  const std::map<Key, DenseIndex>& orderedKeyDims() const;

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

/**
 * Chordally decomposed lifted SDP relaxation backed by MOSEK Fusion.
 *
 * The problem owns its MOSEK model. Call solve() before querying solver
 * results or recovered variables.
 */
template <>
class GTSAM_EXPORT LiftedSDPProblem<ChordalSDP, MosekSDPSolver> {
 public:
  /**
   * Construct the chordal SDP relaxation of a QCQP problem.
   *
   * @param problem QCQP to relax.
   * @param orderingType Ordering used for symbolic elimination.
   */
  LiftedSDPProblem(const QcqpProblem& problem,
                   ChordalOrderingType orderingType);

  /// Dispose of the owned MOSEK model.
  ~LiftedSDPProblem();

  /**
   * Solve the SDP with MOSEK.
   *
   * @param mosekParams MOSEK solver parameter overrides.
   * @return True after MOSEK completes the solve.
   */
  bool solve(const std::map<std::string, double>& mosekParams = {});

  /// Return the primal objective value after solve().
  double objectiveValue() const;

  /// Return the MOSEK problem status after solve().
  std::string problemStatus() const;

  /// Return MOSEK's optimizer time in seconds after solve().
  double solveTimeSeconds() const;

  /// Return one keyed D=1 QCQP vector per diagonal SDP block after solve().
  Values qcqpValues() const;

  /// Return largest-to-second-largest eigenvalue ratios for recovered blocks.
  std::vector<double> variableEVRs() const;

  /// Return QCQP variable keys in SDP block order.
  const KeyVector& orderedKeys() const;

  /// Return the QCQP dimension associated with each ordered key.
  const std::map<Key, DenseIndex>& orderedKeyDims() const;

  /// Return the symbolic Bayes tree defining the chordal decomposition.
  const SymbolicBayesTree& bayesTree() const;

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

typedef LiftedSDPProblem<MonolithicSDP, MosekSDPSolver> MosekMonolithicSDP;
typedef LiftedSDPProblem<ChordalSDP, MosekSDPSolver> MosekChordalSDP;
#endif

}  // namespace gtsam
