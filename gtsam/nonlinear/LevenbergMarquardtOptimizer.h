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

#include <gtsam/nonlinear/DirectOptimizer.h>

namespace gtsam {

/** Parameters for Levenberg-Marquardt optimization.  Note that this parameters
 * class inherits from NonlinearOptimizerParams, which specifies the parameters
 * common to all nonlinear optimization algorithms.  This class also contains
 * all of those parameters.
 */
class LevenbergMarquardtParams : public DirectOptimizerParams {
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
    DirectOptimizerParams::print(str);
    std::cout << "              lambdaInitial: " << lambdaInitial << "\n";
    std::cout << "               lambdaFactor: " << lambdaFactor << "\n";
    std::cout << "           lambdaUpperBound: " << lambdaUpperBound << "\n";
    std::cout.flush();
  }
};

/**
 * State for LevenbergMarquardtOptimizer
 */
class LevenbergMarquardtState : public NonlinearOptimizerState {
public:

  double lambda;

};

/**
 * This class performs Levenberg-Marquardt nonlinear optimization
 * TODO: use make_shared
 */
class LevenbergMarquardtOptimizer : public DirectOptimizer {

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
      const LevenbergMarquardtParams& params = LevenbergMarquardtParams(),
      const Ordering& ordering = Ordering()) :
        DirectOptimizer(SharedGraph(new NonlinearFactorGraph(graph))),
        params_(new LevenbergMarquardtParams(params)) {}

  /** Standard constructor, requires a nonlinear factor graph, initial
   * variable assignments, and optimization parameters.  For convenience this
   * version takes plain objects instead of shared pointers, but internally
   * copies the objects.
   * @param graph The nonlinear factor graph to optimize
   * @param values The initial variable assignments
   * @param params The optimization parameters
   */
  LevenbergMarquardtOptimizer(const NonlinearFactorGraph& graph,
      const Ordering& ordering) :
        DirectOptimizer(SharedGraph(new NonlinearFactorGraph(graph))),
        params_(new LevenbergMarquardtParams()) {}

  /** Standard constructor, requires a nonlinear factor graph, initial
   * variable assignments, and optimization parameters.
   * @param graph The nonlinear factor graph to optimize
   * @param values The initial variable assignments
   * @param params The optimization parameters
   */
  LevenbergMarquardtOptimizer(const SharedGraph& graph,
      const LevenbergMarquardtParams& params = LevenbergMarquardtParams(),
      const SharedOrdering& ordering = SharedOrdering()) :
        DirectOptimizer(graph),
        params_(new LevenbergMarquardtParams(params)) {}

  /** Access the parameters */
  virtual const NonlinearOptimizer::SharedParams& params() const { return params_; }

  /** Access the parameters */
  const LevenbergMarquardtOptimizer::SharedParams& params() const { return params_; }

  /// @}

  /// @name Advanced interface
  /// @{

  /** Virtual destructor */
  virtual ~LevenbergMarquardtOptimizer() {}

  /** Perform a single iteration, returning a new NonlinearOptimizer class
   * containing the updated variable assignments, which may be retrieved with
   * values().
   */
  virtual NonlinearOptimizer::SharedState iterate(const SharedState& current) const;

  /** Create a copy of the NonlinearOptimizer */
  virtual NonlinearOptimizer::shared_ptr clone() const {
    return boost::make_shared<LevenbergMarquardtOptimizer>(*this); }

  /// @}

protected:

  typedef boost::shared_ptr<const std::vector<size_t> > SharedDimensions;

  const SharedParams params_;
  const SharedDimensions dimensions_;
};

}
