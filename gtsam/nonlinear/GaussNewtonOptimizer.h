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
 * @created Feb 26, 2012
 */

#pragma once

#include <gtsam/nonlinear/SuccessiveLinearizationOptimizer.h>

namespace gtsam {

/** Parameters for Gauss-Newton optimization, inherits from
 * NonlinearOptimizationParams.
 */
class GaussNewtonParams : public SuccessiveLinearizationParams {
};

class GaussNewtonState : public SuccessiveLinearizationState {
};

/**
 * This class performs Gauss-Newton nonlinear optimization
 * TODO: use make_shared
 */
class GaussNewtonOptimizer : public SuccessiveLinearizationOptimizer {

public:

  typedef boost::shared_ptr<GaussNewtonParams> SharedParams;

  /// @name Standard interface
  /// @{

  /** Standard constructor, requires a nonlinear factor graph, initial
   * variable assignments, and optimization parameters.  For convenience this
   * version takes plain objects instead of shared pointers, but internally
   * copies the objects.
   * @param graph The nonlinear factor graph to optimize
   * @param values The initial variable assignments
   * @param params The optimization parameters
   */
  GaussNewtonOptimizer(const NonlinearFactorGraph& graph,
      const GaussNewtonParams& params = GaussNewtonParams()) :
        SuccessiveLinearizationOptimizer(SharedGraph(new NonlinearFactorGraph(graph))),
        params_(new GaussNewtonParams(params)) {}

  /** Standard constructor, requires a nonlinear factor graph, initial
   * variable assignments, and optimization parameters.  For convenience this
   * version takes plain objects instead of shared pointers, but internally
   * copies the objects.
   * @param graph The nonlinear factor graph to optimize
   * @param values The initial variable assignments
   * @param params The optimization parameters
   */
  GaussNewtonOptimizer(const NonlinearFactorGraph& graph, const Ordering& ordering) :
        SuccessiveLinearizationOptimizer(SharedGraph(new NonlinearFactorGraph(graph))),
        params_(new GaussNewtonParams()) {
    params_->ordering = ordering; }

  /** Standard constructor, requires a nonlinear factor graph, initial
   * variable assignments, and optimization parameters.
   * @param graph The nonlinear factor graph to optimize
   * @param values The initial variable assignments
   * @param params The optimization parameters
   */
  GaussNewtonOptimizer(const SharedGraph& graph,
      const GaussNewtonParams& params = GaussNewtonParams()) :
        SuccessiveLinearizationOptimizer(graph),
        params_(new GaussNewtonParams(params)) {}

  /** Access the parameters */
  virtual NonlinearOptimizer::SharedParams params() const { return params_; }

  /// @}

  /// @name Advanced interface
  /// @{

  /** Virtual destructor */
  virtual ~GaussNewtonOptimizer() {}

  /** Perform a single iteration, returning a new NonlinearOptimizer class
   * containing the updated variable assignments, which may be retrieved with
   * values().
   */
  virtual NonlinearOptimizer::SharedState iterate(const NonlinearOptimizer::SharedState& current) const;

  /** Create an initial state with the specified variable assignment values and
   * all other default state.
   */
  virtual NonlinearOptimizer::SharedState initialState(const Values& initialValues) const;

  /// @}

protected:

  const SharedParams params_;
};

}
