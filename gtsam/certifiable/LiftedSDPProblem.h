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
#include <gtsam/geometry/Rot2.h>
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

/** Formulation tag for a single monolithic positive semidefinite variable. */
struct MonolithicSDP {};

/** Formulation tag for a chordally decomposed positive semidefinite variable. */
struct ChordalSDP {};

/** Variable ordering strategy used to construct the chordal decomposition. */
enum class ChordalOrderingType {
  Metis,  ///< Nested-dissection ordering computed with METIS.
  Colamd  ///< Column approximate minimum-degree ordering.
};

/** Solver tag for the optional MOSEK Fusion backend. */
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
/** Monolithic lifted SDP backed by MOSEK Fusion. */
template <>
class GTSAM_EXPORT LiftedSDPProblem<MonolithicSDP, MosekSDPSolver> {
 public:
  /** Construct the monolithic SDP relaxation of a QCQP problem. */
  explicit LiftedSDPProblem(const QcqpProblem& problem);

  /** Dispose of the MOSEK model owned by this problem. */
  ~LiftedSDPProblem();

  // # copied from gtsam-private
  /**
   * Solve the SDP with MOSEK.
   *
   * @param mosek_params MOSEK solver parameter overrides.
   * @return True after MOSEK completes the solve.
   */
  bool solve(const std::map<std::string, double>& mosek_params = {});

  /** Return the primal objective value after solve(). */
  double objectiveValue() const;

  /** Return the MOSEK problem status after solve(). */
  std::string problemStatus() const;

  /** Return MOSEK's optimizer time in seconds after solve(). */
  double solveTimeSeconds() const;

  /** Recover one lifted vector from each diagonal SDP block. */
  void recoverLiftedVectors();

  /** Return recovered lifted vectors in orderedKeys() order. */
  const std::vector<Vector>& getRecoveredLiftedVectors() const;

  /** Return the rank-one eigenvalue ratio for each recovered variable. */
  const std::vector<double>& getRecoveredVariableEVRs() const;

  // # copied from gtsam-private
  /**
   * Decode the recovered lifted vectors as manifold values.
   *
   * @tparam T Manifold type with a supported lifted-vector decoder.
   * @return Recovered values in orderedKeys() order.
   */
  template <typename T>
  std::vector<T> getRecoveredPoses() const {
    const auto& liftedVectors = getRecoveredLiftedVectors();
    std::vector<T> recoveredPoses;
    recoveredPoses.reserve(liftedVectors.size());
    for (const Vector& x : liftedVectors) {
      if constexpr (std::is_same_v<T, Rot2>) {
        if (x.size() != 5 || std::abs(x(0)) < 1e-9) {
          throw std::runtime_error(
              "getRecoveredPoses<Rot2>: invalid lifted vector.");
        }

        const Vector xCanonical = x / x(0);
        Matrix2 Rraw;
        // QcqpValue<Rot2, 1> stores vec(R) in column-major order.
        Rraw << xCanonical(1), xCanonical(3), xCanonical(2), xCanonical(4);
        recoveredPoses.push_back(Rot2::ClosestTo(Rraw));
      } else {
        static_assert(std::is_same_v<T, Rot2>,
                      "No lifted-vector decoder exists for this type.");
      }
    }
    return recoveredPoses;
  }

  // # copied from gtsam-private
  /**
   * Compute local-coordinate errors between recovered and reference values.
   *
   * @tparam T Manifold type with a supported lifted-vector decoder.
   * @param groundTruth Reference values in orderedKeys() order.
   * @return One local-coordinate error norm per recovered value.
   */
  template <typename T>
  std::vector<double> getRecoveredPoseErrorNorms(
      const std::vector<T>& groundTruth) const {
    const std::vector<T> recoveredPoses = getRecoveredPoses<T>();
    if (groundTruth.size() != recoveredPoses.size()) {
      throw std::runtime_error(
          "getRecoveredPoseErrorNorms: ground-truth size does not match key "
          "count.");
    }

    std::vector<double> errors(recoveredPoses.size());
    for (size_t index = 0; index < recoveredPoses.size(); ++index) {
      errors[index] =
          groundTruth[index].localCoordinates(recoveredPoses[index]).norm();
    }
    return errors;
  }

  /** Return QCQP variable keys in the SDP block ordering. */
  const KeyVector& orderedKeys() const;

  /** Return the QCQP dimension associated with each ordered key. */
  const std::map<Key, DenseIndex>& orderedKeyDims() const;

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

/** Chordally decomposed lifted SDP backed by MOSEK Fusion. */
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

  /** Dispose of the MOSEK model owned by this problem. */
  ~LiftedSDPProblem();

  // # modified from gtsam-private
  /**
   * Solve the SDP with MOSEK.
   *
   * @param mosek_params MOSEK solver parameter overrides.
   * @return True after MOSEK completes the solve.
   */
  bool solve(const std::map<std::string, double>& mosek_params = {});

  /** Return the primal objective value after solve(). */
  double objectiveValue() const;

  /** Return the MOSEK problem status after solve(). */
  std::string problemStatus() const;

  /** Return MOSEK's optimizer time in seconds after solve(). */
  double solveTimeSeconds() const;

  /** Recover one lifted vector from each diagonal SDP block. */
  void recoverLiftedVectors();

  /** Return recovered lifted vectors in orderedKeys() order. */
  const std::vector<Vector>& getRecoveredLiftedVectors() const;

  /** Return the rank-one eigenvalue ratio for each recovered variable. */
  const std::vector<double>& getRecoveredVariableEVRs() const;

  // # modified from gtsam-private
  /**
   * Decode the recovered lifted vectors as manifold values.
   *
   * @tparam T Manifold type with a supported lifted-vector decoder.
   * @return Recovered values in orderedKeys() order.
   */
  template <typename T>
  std::vector<T> getRecoveredPoses() const {
    const auto& liftedVectors = getRecoveredLiftedVectors();
    std::vector<T> recoveredPoses;
    recoveredPoses.reserve(liftedVectors.size());
    for (const Vector& x : liftedVectors) {
      if constexpr (std::is_same_v<T, Rot2>) {
        if (x.size() != 5 || std::abs(x(0)) < 1e-9) {
          throw std::runtime_error(
              "getRecoveredPoses<Rot2>: invalid lifted vector.");
        }

        const Vector xCanonical = x / x(0);
        Matrix2 Rraw;
        // QcqpValue<Rot2, 1> stores vec(R) in column-major order.
        Rraw << xCanonical(1), xCanonical(3), xCanonical(2), xCanonical(4);
        recoveredPoses.push_back(Rot2::ClosestTo(Rraw));
      } else {
        static_assert(std::is_same_v<T, Rot2>,
                      "No lifted-vector decoder exists for this type.");
      }
    }
    return recoveredPoses;
  }

  // # modified from gtsam-private
  /**
   * Compute local-coordinate errors between recovered and reference values.
   *
   * @tparam T Manifold type with a supported lifted-vector decoder.
   * @param groundTruth Reference values in orderedKeys() order.
   * @return One local-coordinate error norm per recovered value.
   */
  template <typename T>
  std::vector<double> getRecoveredPoseErrorNorms(
      const std::vector<T>& groundTruth) const {
    const std::vector<T> recoveredPoses = getRecoveredPoses<T>();
    if (groundTruth.size() != recoveredPoses.size()) {
      throw std::runtime_error(
          "getRecoveredPoseErrorNorms: ground-truth size does not match key "
          "count.");
    }

    std::vector<double> errors(recoveredPoses.size());
    for (size_t index = 0; index < recoveredPoses.size(); ++index) {
      errors[index] =
          groundTruth[index].localCoordinates(recoveredPoses[index]).norm();
    }
    return errors;
  }

  /** Return QCQP variable keys in the SDP block ordering. */
  const KeyVector& orderedKeys() const;

  /** Return the QCQP dimension associated with each ordered key. */
  const std::map<Key, DenseIndex>& orderedKeyDims() const;

  /** Return the symbolic Bayes tree defining the chordal decomposition. */
  const SymbolicBayesTree& bayesTree() const;

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
#endif

}  // namespace gtsam
