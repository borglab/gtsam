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
#include <gtsam/nonlinear/NonlinearOptimizerParams.h>

namespace gtsam {

namespace internal { struct NonlinearOptimizerState; }

/**
 * This is the abstract interface for classes that can optimize for the
 * maximum-likelihood estimate of a NonlinearFactorGraph.
 *
 * To use a class derived from this interface, construct the class with a
 * NonlinearFactorGraph and an initial Values variable assignment.  Next, call the
 * optimize() method which returns the optimized variable assignment.
 *
 * Simple and compact example:
 * \code
// One-liner to do full optimization and use the result.
Values result = DoglegOptimizer(graph, initialValues).optimize();
\endcode
 *
 * Example exposing more functionality and details:
 * \code
// Create initial optimizer
DoglegOptimizer optimizer(graph, initialValues);

// Run full optimization until convergence.
Values result = optimizer->optimize();

// The new optimizer has results and statistics
cout << "Converged in " << optimizer.iterations() << " iterations "
        "with final error " << optimizer.error() << endl;
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
Values result = DoglegOptimizer(graph, initialValues, params).optimize();
\endcode
 *
 * This interface also exposes an iterate() method, which performs one
 * iteration.  The optimize() method simply calls iterate() multiple times,
 * until the error changes less than a threshold.  We expose iterate() so that
 * you can easily control what happens between iterations, such as drawing or
 * printing, moving points from behind the camera to in front, etc.
 *
 * For more flexibility you may override virtual methods in your own derived class.
 */
class GTSAM_EXPORT NonlinearOptimizer {

protected:
  NonlinearFactorGraph graph_; ///< The graph with nonlinear factors

  std::unique_ptr<internal::NonlinearOptimizerState> state_; ///< PIMPL'd state

public:
  /** A shared pointer to this class */
  using shared_ptr = std::shared_ptr<const NonlinearOptimizer>;

  /// @name Standard interface
  /// @{

  /** 
   * Optimize for the maximum-likelihood estimate, returning a the optimized 
   * variable assignments.
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

  /// return error in current optimizer state
  double error() const;

  /// return number of iterations in current optimizer state
  size_t iterations() const;

  /// return values in current optimizer state
  const Values &values() const;

  /// return the graph with nonlinear factors
  const NonlinearFactorGraph &graph() const { return graph_; }

  /// @}

  /// @name Advanced interface
  /// @{

  /** Virtual destructor */
  virtual ~NonlinearOptimizer();

  /** Default function to do linear solve, i.e. optimize a GaussianFactorGraph */
  virtual VectorValues solve(const GaussianFactorGraph &gfg,
      const NonlinearOptimizerParams& params) const;

  /** 
   * Perform a single iteration, returning GaussianFactorGraph corresponding to 
   * the linearized factor graph.
   */
  virtual GaussianFactorGraph::shared_ptr iterate() = 0;

  /// @}

protected:
  /** A default implementation of the optimization loop, which calls iterate()
   * until checkConvergence returns true.
   */
  void defaultOptimize();

  virtual const NonlinearOptimizerParams& _params() const = 0;

  /** Constructor for initial construction of base classes. Takes ownership of state. */
  NonlinearOptimizer(const NonlinearFactorGraph& graph,
                     std::unique_ptr<internal::NonlinearOptimizerState> state);
};

/** Check whether the relative error decrease is less than relativeErrorTreshold,
 * the absolute error decrease is less than absoluteErrorTreshold, <em>or</em>
 * the error itself is less than errorThreshold.
 */
GTSAM_EXPORT bool checkConvergence(double relativeErrorTreshold,
    double absoluteErrorTreshold, double errorThreshold,
    double currentError, double newError, NonlinearOptimizerParams::Verbosity verbosity = NonlinearOptimizerParams::SILENT);

GTSAM_EXPORT bool checkConvergence(const NonlinearOptimizerParams& params, double currentError,
                                   double newError);

} // gtsam
