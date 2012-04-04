/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file NonlinearOptimizer.h
 * @brief Base class and parameters for nonlinear optimization algorithms
 * @author Richard Roberts
 * @date Sep 7, 2009
 */

#pragma once

#include <gtsam/nonlinear/NonlinearFactorGraph.h>

namespace gtsam {

/** The common parameters for Nonlinear optimizers.  Most optimizers
 * deriving from NonlinearOptimizer also subclass the parameters.
 */
class NonlinearOptimizerParams {
public:
  /** See NonlinearOptimizerParams::verbosity */
  enum Verbosity {
    SILENT,
    ERROR,
    VALUES,
    DELTA,
    LINEAR
  };

  int maxIterations; ///< The maximum iterations to stop iterating (default 100)
  double relativeErrorTol; ///< The maximum relative error decrease to stop iterating (default 1e-5)
  double absoluteErrorTol; ///< The maximum absolute error decrease to stop iterating (default 1e-5)
  double errorTol; ///< The maximum total error to stop iterating (default 0.0)
  Verbosity verbosity; ///< The printing verbosity during optimization (default SILENT)

  NonlinearOptimizerParams() :
    maxIterations(100.0), relativeErrorTol(1e-5), absoluteErrorTol(1e-5),
    errorTol(0.0), verbosity(SILENT) {}

  virtual void print(const std::string& str = "") const {
    std::cout << str << "\n";
    std::cout << "relative decrease threshold: " << relativeErrorTol << "\n";
    std::cout << "absolute decrease threshold: " << absoluteErrorTol << "\n";
    std::cout << "      total error threshold: " << errorTol << "\n";
    std::cout << "         maximum iterations: " << maxIterations << "\n";
    std::cout << "            verbosity level: " << verbosity << std::endl;
  }

  virtual ~NonlinearOptimizerParams() {}
};


/**
 * Base class for a nonlinear optimization state, including the current estimate
 * of the variable values, error, and number of iterations.  Optimizers derived
 * from NonlinearOptimizer usually also define a derived state class containing
 * additional state specific to the algorithm (for example, Dogleg state
 * contains the current trust region radius).
 */
class NonlinearOptimizerState {
public:

  /** The current estimate of the variable values. */
  Values values;

  /** The factor graph error on the current values. */
  double error;

  /** The number of optimization iterations performed. */
  unsigned int iterations;

  /** Virtual destructor */
  virtual ~NonlinearOptimizerState() {}

  /** Clone the state (i.e. make a copy of the derived class) */
  virtual boost::shared_ptr<NonlinearOptimizerState> clone() = 0;
};


/**
 * This is the abstract interface for classes that can optimize for the
 * maximum-likelihood estimate of a NonlinearFactorGraph.
 *
 * To use a class derived from this interface, construct the class with a
 * NonlinearFactorGraph and an initial Values variable assignment.  Next, call the
 * optimize() method, which returns a new NonlinearOptimizer object containing
 * the optimized variable assignment.  Call the values() method to retrieve the
 * optimized estimate.  Alternatively, to take a shortcut, instead of calling
 * optimize(), call optimized(), which performs full optimization and returns
 * the resulting Values instead of the new optimizer.
 *
 * Note:  This class is immutable, optimize() and iterate() return new
 * NonlinearOptimizer objects, so be sure to use the returned object and not
 * simply keep the unchanged original.
 *
 * Simple and compact example:
 * \code
// One-liner to do full optimization and use the result.
// Note use of "optimized()" to directly return Values, instead of "optimize()" that returns a new optimizer.
Values::const_shared_ptr result = DoglegOptimizer(graph, initialValues).optimized();
\endcode
 *
 * Example exposing more functionality and details:
 * \code
// Create initial optimizer
DoglegOptimizer initial(graph, initialValues);

// Run full optimization until convergence.
// Note use of "optimize()" to return a new optimizer, instead of "optimized()" that returns only the Values.
// NonlinearOptimizer pointers are always returned, though they are actually a derived optimizer type.
NonlinearOptimizer::auto_ptr final = initial->optimize();

// The new optimizer has results and statistics
cout << "Converged in " << final->iterations() << " iterations "
        "with final error " << final->error() << endl;

// The values are a const_shared_ptr (boost::shared_ptr<const Values>)
Values::const_shared_ptr result = final->values();

// Use the results
useTheResult(result);
\endcode
 *
 * Example of setting parameters before optimization:
 * \code
// Each derived optimizer type has its own parameters class, which inherits from NonlinearOptimizerParams
DoglegParams params;
params.factorization = DoglegParams::QR;
params.relativeErrorTol = 1e-3;
params.absoluteErrorTol = 1e-3;

// Optimize
Values::const_shared_ptr result = DoglegOptimizer(graph, initialValues, params).optimized();
\endcode
 *
 * This interface also exposes an iterate() method, which performs one
 * iteration, returning a NonlinearOptimizer containing the adjusted variable
 * assignment.  The optimize() method simply calls iterate() multiple times,
 * until the error changes less than a threshold.  We expose iterate() so that
 * you can easily control what happens between iterations, such as drawing or
 * printing, moving points from behind the camera to in front, etc.
 *
 * To modify the graph, values, or parameters between iterations, call the
 * update() functions, which preserve all other state (for example, the trust
 * region size in DoglegOptimizer).  Derived optimizer classes also have
 * additional update methods, not in this abstract interface, for updating
 * algorithm-specific state.
 *
 * For more flexibility, since all functions are virtual, you may override them
 * in your own derived class.
 */
class NonlinearOptimizer {

public:

  /** A shared pointer to this class */
  typedef boost::shared_ptr<const NonlinearOptimizer> shared_ptr;

  /** A const shared_ptr to a NonlinearFactorGraph */
  typedef boost::shared_ptr<const NonlinearFactorGraph> SharedGraph;

  /** A const shared_ptr to the parameters */
  typedef boost::shared_ptr<const NonlinearOptimizerParams> SharedParams;

  /** A shared_ptr to an optimizer state */
  typedef boost::shared_ptr<NonlinearOptimizerState> SharedState;

  /// @name Standard interface
  /// @{

  /** Optimize for the maximum-likelihood estimate, returning a new
   * NonlinearOptimizer class containing the optimized variable assignments,
   * which may be retrieved with values().
   *
   * This function simply calls iterate() in a loop, checking for convergence
   * with check_convergence().  For fine-grain control over the optimization
   * process, you may call iterate() and check_convergence() yourself, and if
   * needed modify the optimization state between iterations.
   */
  virtual SharedState optimize(const SharedState& initial) const { return defaultOptimize(initial); }

  SharedState optimize(const Values& initialization) const { return optimize(initialState(initialization)); }

  /** Shortcut to optimize and return the resulting Values of the maximum-
   * likelihood estimate.  To access statistics and information such as the
   * final error and number of iterations, use optimize() instead.
   * @return The maximum-likelihood estimate.
   */
  virtual Values optimized(const SharedState& initial) const { return this->optimize(initial)->values; }

  Values optimized(const Values& initialization) const { return optimized(initialState(initialization)); }

  /** Retrieve the parameters. */
  virtual SharedParams params() const = 0;

  /// @}

  /// @name Advanced interface
  /// @{

  /** Virtual destructor */
  virtual ~NonlinearOptimizer() {}

  /** Perform a single iteration, returning a new NonlinearOptimizer class
   * containing the updated variable assignments, which may be retrieved with
   * values().
   */
  virtual SharedState iterate(const SharedState& current) const = 0;

  /** Create an initial state from a variable assignment Values, with all
   * other state values at their default initial values.
   */
  virtual SharedState initialState(const Values& initialValues) const = 0;

  /// @}

protected:

  const SharedGraph graph_;

  /** A default implementation of the optimization loop, which calls iterate()
   * until checkConvergence returns true.
   */
  SharedState defaultOptimize(const SharedState& initial) const;

  /** Initialize a state, using the current error and 0 iterations */
  void defaultInitialState(SharedState& initial) const;

  /** Constructor for initial construction of base classes.
   */
  NonlinearOptimizer(const SharedGraph& graph) : graph_(graph) {}

};

/** Check whether the relative error decrease is less than relativeErrorTreshold,
 * the absolute error decrease is less than absoluteErrorTreshold, <emph>or</emph>
 * the error itself is less than errorThreshold.
 */
bool checkConvergence(double relativeErrorTreshold,
    double absoluteErrorTreshold, double errorThreshold,
    double currentError, double newError, NonlinearOptimizerParams::Verbosity verbosity);

} // gtsam
