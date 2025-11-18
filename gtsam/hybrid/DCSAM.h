/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    DCSAM.h
 * @brief   Discrete-Continuous Smoothing and Mapping for Factored Models
 * @author  Kevin Doherty
 * @author  Varun Agrawal
 * @date    March 2025
 */

#pragma once

#include <gtsam/discrete/DiscreteFactor.h>
#include <gtsam/discrete/DiscreteFactorGraph.h>
#include <gtsam/discrete/DiscreteKey.h>
#include <gtsam/discrete/DiscreteMarginals.h>
#include <gtsam/hybrid/HybridNonlinearFactorGraph.h>
#include <gtsam/inference/FactorGraph.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>

#include <map>
#include <set>
#include <tuple>
#include <utility>
#include <vector>

namespace gtsam {

/**
 * @brief Class which implements Discrete-Continuous Smoothing And Mapping,
 * as detailed in Doherty22ral (https://arxiv.org/abs/2204.11936).
 *
 * Performs hybrid optimization by alternatively optimizing
 * for continous and discrete variables.
 *
 */
class GTSAM_EXPORT DCSAM {
 private:
  /// The factor graph for all continuous factors
  NonlinearFactorGraph nfg_;
  /// The factor graph for all discrete factors
  DiscreteFactorGraph dfg_;
  /// The factor graph for hybrid factors
  HybridNonlinearFactorGraph hfg_;

  // TODO(Varun): Allow for using other continuous and discrete solvers

  /// ISAM2 optimizer for continuous optimization
  ISAM2 isam_;

  /// The current best estimate for continuous values
  Values currContinuous_;
  /// The current best estimate for discrete values
  DiscreteValues currDiscrete_;

 public:
  DCSAM();

  explicit DCSAM(const ISAM2Params &isam_params);

  /**
   * For this solver, runs an iteration of alternating minimization between
   * discrete and continuous variables, adding any user-supplied factors (with
   * initial guess) first.
   *
   * 1. Adds new discrete factors (if any) as supplied by a user to the
   * discrete factor graph, then adds any discrete-continuous factors to the
   * discrete factor graph, appropriately initializing their continuous
   * variables to those from the last solve and any supplied by the initial
   * guess.
   *
   * 2. Update the solution for the discrete variables.
   *
   * 3. For all new discrete-continuous factors to be passed to the continuous
   * solver, update/set the latest discrete variables (prior to adding).
   *
   * 4. In one step: add new factors, new values, and earmarked old factor keys
   * to iSAM. Specifically, loop over DC factors already in iSAM, updating their
   * discrete information, then call isam_.update() with the (initialized) new
   * DC factors, any new continuous factors, and the initial guess to be
   * supplied.
   *
   * 5. Calculate the latest continuous variables from iSAM.
   *
   * 6. Update the discrete factors in the discrete factor graph dfg_ with the
   * latest information from the continuous solve.
   *
   * @param graph - a HybridNonlinearFactorGraph containing all factors to add.
   * @param initialGuessContinuous - an initial guess for any new
   * continuous keys that appear in the updated factors
   * (or if one wants to force override previously obtained continuous values).
   * @param initialGuessDiscrete - Initial guess for discrete variables.
   */
  void update(const HybridNonlinearFactorGraph &graph,
              const HybridValues &initialGuess = HybridValues());

  /**
   * Inline convenience function to allow "skipping" the initial guess for
   * continuous variables while adding an initial guess for discrete variables.
   */
  inline void update(const HybridNonlinearFactorGraph &graph,
                     const DiscreteValues &initialGuessDiscrete) {
    update(graph, HybridValues(VectorValues(), initialGuessDiscrete));
  }

  /**
   * Simply used to call `update` without any new factors. Runs an iteration of
   * optimization.
   */
  void update();

  /**
   * This is the primary function used to extract an estimate from the solver.
   * Internally, calls `isam_.calculateEstimate()` and `dfg_.optimize()`
   * to obtain an estimate for the continuous (resp. discrete) variables
   * and packages them into a `HybridValues`.
   *
   * @return a HybridValues object containing an estimate
   * of the most probable assignment to the continuous (HybridValues.nonlinear)
   * and discrete (HybridValues.discrete) variables.
   */
  HybridValues calculateEstimate() const;

  /**
   * Used to obtain the marginals from the solver.
   *
   * NOTE: not obviously correct (see DCSAM.cpp implementation) at the moment.
   * Should perhaps retrieve the marginals for the factor graph obtained as
   * isam_.getFactorsUnsafe() and `dfg_` rather than taking as a parameter?
   * I think this was originally intended to mimic the gtsam `Marginals` class.
   *
   * @param graph
   * @param continuousEst
   * @param dfg
   */
  // DCMarginals getMarginals(const NonlinearFactorGraph &graph,
  //                          const Values &continuousEst,
  //                          const DiscreteFactorGraph &dfg);

  const DiscreteFactorGraph &getDiscreteFactorGraph() const { return dfg_; }

  const NonlinearFactorGraph &getNonlinearFactorGraph() const {
    return isam_.getFactorsUnsafe();
  }

 protected:
  /**
   * Add factors in `dfg` to member discrete factor graph `dfg_`,
   * then optimize for most probable explanation and
   * update the current discrete estimate.
   *
   * @param dfg - A discrete factor graph containing the factors to add
   * @param discreteValues - An assignment to the continuous variables
   * (or subset thereof).
   */
  void updateDiscrete(const DiscreteFactorGraph &dfg = DiscreteFactorGraph(),
                      const DiscreteValues &discreteVals = DiscreteValues());

  /**
   * Given the latest discrete values (currDiscrete_),
   * a set of new factors (newFactors),
   * and an initial guess for any new keys (initialGuess),
   * this method
   * 1. Selects the appropriate continuous factors from the hybridfactors
   * 2. Marks any affected keys as such
   * 3. Calls `isam_.update` with the new factors and initial guess.
   *
   * See implementation for more detail.
   */
  void updateContinuous(const NonlinearFactorGraph &newFactors,
                        const Values &initialGuess);

  /**
   * Solve for discrete variables given continuous variables.
   *
   *  Internally, this method computes DiscreteBoundaryFactors from the hybrid
   * factors using the current continuous estimate `currContinuous_`, and then
   * calls `dfg_.optimize()` to get the most probable estimate.
   *
   * @return An assignment (DiscreteValues) to the discrete variables
   * in the graph.
   */
  DiscreteValues solveDiscrete() const;
};

}  // namespace gtsam
