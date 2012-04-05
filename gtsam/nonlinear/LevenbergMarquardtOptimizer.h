/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    LevenbergMarquardtOptimizer.h
 * @brief   
 * @author  Richard Roberts
 * @created Feb 26, 2012
 */

#pragma once

#include <gtsam/nonlinear/SuccessiveLinearizationOptimizer.h>

namespace gtsam {

/** Parameters for Levenberg-Marquardt optimization.  Note that this parameters
 * class inherits from NonlinearOptimizerParams, which specifies the parameters
 * common to all nonlinear optimization algorithms.  This class also contains
 * all of those parameters.
 */
class LevenbergMarquardtParams : public SuccessiveLinearizationParams {
public:
  /** See LevenbergMarquardtParams::lmVerbosity */
  enum LMVerbosity {
    SILENT,
    LAMBDA,
    TRYLAMBDA,
    TRYCONFIG,
    TRYDELTA,
    DAMPED
  };

  double lambdaInitial; ///< The initial Levenberg-Marquardt damping term (default: 1e-5)
  double lambdaFactor; ///< The amount by which to multiply or divide lambda when adjusting lambda (default: 10.0)
  double lambdaUpperBound; ///< The maximum lambda to try before assuming the optimization has failed (default: 1e5)
  LMVerbosity lmVerbosity; ///< The verbosity level for Levenberg-Marquardt (default: SILENT), see also NonlinearOptimizerParams::verbosity

  LevenbergMarquardtParams() :
    lambdaInitial(1e-5), lambdaFactor(10.0), lambdaUpperBound(1e5), lmVerbosity(SILENT) {}

  virtual ~LevenbergMarquardtParams() {}

  virtual void print(const std::string& str = "") const {
    SuccessiveLinearizationParams::print(str);
    std::cout << "              lambdaInitial: " << lambdaInitial << "\n";
    std::cout << "               lambdaFactor: " << lambdaFactor << "\n";
    std::cout << "           lambdaUpperBound: " << lambdaUpperBound << "\n";
    std::cout.flush();
  }
};

/**
 * State for LevenbergMarquardtOptimizer
 */
class LevenbergMarquardtState : public SuccessiveLinearizationState {
public:

  double lambda;

};

/**
 * This class performs Levenberg-Marquardt nonlinear optimization
 * TODO: use make_shared
 */
class LevenbergMarquardtOptimizer : public SuccessiveLinearizationOptimizer {

public:

  typedef boost::shared_ptr<LevenbergMarquardtParams> SharedParams;
  typedef boost::shared_ptr<LevenbergMarquardtState> SharedState;
  typedef boost::shared_ptr<LevenbergMarquardtOptimizer> shared_ptr;

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
  LevenbergMarquardtOptimizer(const NonlinearFactorGraph& graph,
      const LevenbergMarquardtParams& params = LevenbergMarquardtParams()) :
        SuccessiveLinearizationOptimizer(SharedGraph(new NonlinearFactorGraph(graph))),
        params_(new LevenbergMarquardtParams(params)) {}

  /** Standard constructor, requires a nonlinear factor graph, initial
   * variable assignments, and optimization parameters.  For convenience this
   * version takes plain objects instead of shared pointers, but internally
   * copies the objects.
   * @param graph The nonlinear factor graph to optimize
   * @param values The initial variable assignments
   * @param params The optimization parameters
   */
  LevenbergMarquardtOptimizer(const NonlinearFactorGraph& graph, const Ordering& ordering) :
        SuccessiveLinearizationOptimizer(SharedGraph(new NonlinearFactorGraph(graph))),
        params_(new LevenbergMarquardtParams()) {
    params_->ordering = ordering; }

  /** Standard constructor, requires a nonlinear factor graph, initial
   * variable assignments, and optimization parameters.
   * @param graph The nonlinear factor graph to optimize
   * @param values The initial variable assignments
   * @param params The optimization parameters
   */
  LevenbergMarquardtOptimizer(const SharedGraph& graph,
      const LevenbergMarquardtParams& params = LevenbergMarquardtParams()) :
        SuccessiveLinearizationOptimizer(graph),
        params_(new LevenbergMarquardtParams(params)) {}

  /** Access the parameters */
  virtual NonlinearOptimizer::SharedParams params() const { return params_; }

  /// @}

  /// @name Advanced interface
  /// @{

  /** Virtual destructor */
  virtual ~LevenbergMarquardtOptimizer() {}

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

  typedef boost::shared_ptr<const std::vector<size_t> > SharedDimensions;

  SharedParams params_;
  mutable SharedDimensions dimensions_; // Mutable because we compute it when needed and cache it
};

}
