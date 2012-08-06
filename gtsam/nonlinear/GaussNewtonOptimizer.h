/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    GaussNewtonOptimizer.h
 * @brief   
 * @author  Richard Roberts
 * @date 	Feb 26, 2012
 */

#pragma once

#include <gtsam/nonlinear/SuccessiveLinearizationOptimizer.h>

namespace gtsam {

class GaussNewtonOptimizer;

/** Parameters for Gauss-Newton optimization, inherits from
 * NonlinearOptimizationParams.
 */
class GaussNewtonParams : public SuccessiveLinearizationParams {
};

class GaussNewtonState : public NonlinearOptimizerState {
protected:
  GaussNewtonState(const NonlinearFactorGraph& graph, const Values& values, unsigned int iterations = 0) :
    NonlinearOptimizerState(graph, values, iterations) {}

  friend class GaussNewtonOptimizer;
};

/**
 * This class performs Gauss-Newton nonlinear optimization
 */
class GaussNewtonOptimizer : public NonlinearOptimizer {

protected:
	GaussNewtonParams params_;
	GaussNewtonState state_;

public:
  /// @name Standard interface
  /// @{

  /** Standard constructor, requires a nonlinear factor graph, initial
   * variable assignments, and optimization parameters.  For convenience this
   * version takes plain objects instead of shared pointers, but internally
   * copies the objects.
   * @param graph The nonlinear factor graph to optimize
   * @param initialValues The initial variable assignments
   * @param params The optimization parameters
   */
  GaussNewtonOptimizer(const NonlinearFactorGraph& graph, const Values& initialValues,
      const GaussNewtonParams& params = GaussNewtonParams()) :
        NonlinearOptimizer(graph), params_(ensureHasOrdering(params, graph, initialValues)), state_(graph, initialValues) {}

  /** Standard constructor, requires a nonlinear factor graph, initial
   * variable assignments, and optimization parameters.  For convenience this
   * version takes plain objects instead of shared pointers, but internally
   * copies the objects.
   * @param graph The nonlinear factor graph to optimize
   * @param initialValues The initial variable assignments
   */
  GaussNewtonOptimizer(const NonlinearFactorGraph& graph, const Values& initialValues, const Ordering& ordering) :
        NonlinearOptimizer(graph), state_(graph, initialValues) {
    params_.ordering = ordering; }

  /// @}

  /// @name Advanced interface
  /// @{

  /** Virtual destructor */
  virtual ~GaussNewtonOptimizer() {}

  /** Perform a single iteration, returning a new NonlinearOptimizer class
   * containing the updated variable assignments, which may be retrieved with
   * values().
   */
  virtual void iterate();

  /** Access the parameters */
  const GaussNewtonParams& params() const { return params_; }

  /** Access the last state */
  const GaussNewtonState& state() const { return state_; }

  /// @}

protected:
  /** Access the parameters (base class version) */
  virtual const NonlinearOptimizerParams& _params() const { return params_; }

  /** Access the state (base class version) */
  virtual const NonlinearOptimizerState& _state() const { return state_; }

  /** Internal function for computing a COLAMD ordering if no ordering is specified */
  GaussNewtonParams ensureHasOrdering(GaussNewtonParams params, const NonlinearFactorGraph& graph, const Values& values) const {
    if(!params.ordering)
      params.ordering = *graph.orderingCOLAMD(values);
    return params;
  }

};

}
