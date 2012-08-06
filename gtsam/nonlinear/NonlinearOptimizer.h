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

class NonlinearOptimizer;

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

  size_t maxIterations; ///< The maximum iterations to stop iterating (default 100)
  double relativeErrorTol; ///< The maximum relative error decrease to stop iterating (default 1e-5)
  double absoluteErrorTol; ///< The maximum absolute error decrease to stop iterating (default 1e-5)
  double errorTol; ///< The maximum total error to stop iterating (default 0.0)
  Verbosity verbosity; ///< The printing verbosity during optimization (default SILENT)

  NonlinearOptimizerParams() :
    maxIterations(100), relativeErrorTol(1e-5), absoluteErrorTol(1e-5),
    errorTol(0.0), verbosity(SILENT) {}

  virtual ~NonlinearOptimizerParams() {}
  virtual void print(const std::string& str = "") const ;

  size_t getMaxIterations() const { return maxIterations; }
  double getRelativeErrorTol() const { return relativeErrorTol; }
  double getAbsoluteErrorTol() const { return absoluteErrorTol; }
  double getErrorTol() const { return errorTol; }
  std::string getVerbosity() const { return verbosityTranslator(verbosity); }

  void setMaxIterations(size_t value) { maxIterations = value; }
  void setRelativeErrorTol(double value) { relativeErrorTol = value; }
  void setAbsoluteErrorTol(double value) { absoluteErrorTol = value; }
  void setErrorTol(double value) { errorTol  = value ; }
  void setVerbosity(const std::string &src) { verbosity = verbosityTranslator(src); }

private:
  Verbosity verbosityTranslator(const std::string &s) const;
  std::string verbosityTranslator(Verbosity value) const;
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

  NonlinearOptimizerState() {}

  /** Virtual destructor */
  virtual ~NonlinearOptimizerState() {}

protected:
  NonlinearOptimizerState(const NonlinearFactorGraph& graph, const Values& values, unsigned int iterations = 0) :
    values(values), error(graph.error(values)), iterations(iterations) {}

  NonlinearOptimizerState(const Values& values, double error, unsigned int iterations) :
    values(values), error(error), iterations(iterations) {}

  friend class NonlinearOptimizer;
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

protected:
  NonlinearFactorGraph graph_;

public:
  /** A shared pointer to this class */
  typedef boost::shared_ptr<const NonlinearOptimizer> shared_ptr;

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
  virtual const Values& optimize() { defaultOptimize(); return values(); }

	/**
	 * Optimize, but return empty result if any uncaught exception is thrown
	 * Intended for MATLAB. In C++, use above and catch exceptions.
	 * No message is printed: it is up to the caller to check the result
	 * @param optimizer a non-linear optimizer
	 */
	const Values& optimizeSafely();

	/// return error
  double error() const { return _state().error; }

  /// return number of iterations
  unsigned int iterations() const { return _state().iterations; }

  /// return values
  const Values& values() const { return _state().values; }

  /// @}

  /// @name Advanced interface
  /// @{

  /** Virtual destructor */
  virtual ~NonlinearOptimizer() {}

  /** Perform a single iteration, returning a new NonlinearOptimizer class
   * containing the updated variable assignments, which may be retrieved with
   * values().
   */
  virtual void iterate() = 0;

  /// @}

protected:	
  /** A default implementation of the optimization loop, which calls iterate()
   * until checkConvergence returns true.
   */
  void defaultOptimize();

  virtual const NonlinearOptimizerState& _state() const = 0;

  virtual const NonlinearOptimizerParams& _params() const = 0;

  /** Constructor for initial construction of base classes. */
  NonlinearOptimizer(const NonlinearFactorGraph& graph) : graph_(graph) {}

};

/** Check whether the relative error decrease is less than relativeErrorTreshold,
 * the absolute error decrease is less than absoluteErrorTreshold, <em>or</em>
 * the error itself is less than errorThreshold.
 */
bool checkConvergence(double relativeErrorTreshold,
    double absoluteErrorTreshold, double errorThreshold,
    double currentError, double newError, NonlinearOptimizerParams::Verbosity verbosity);

} // gtsam
