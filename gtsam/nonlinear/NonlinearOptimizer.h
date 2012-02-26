/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file NonlinearOptimizer.h
 * @brief Encapsulates nonlinear optimization state
 * @author Frank Dellaert
 * @date Sep 7, 2009
 */

#pragma once

#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/NonlinearOptimizationParameters.h>

namespace gtsam {

/** The common parameters for Nonlinear optimizers.  Most optimizers
 * deriving from NonlinearOptimizer also subclass the parameters.
 */
class NonlinearOptimizerParams {
public:

  /** Control the printing verbosity */
  enum Verbosity {
    SILENT,
    ERROR,
    LAMBDA,
    TRYLAMBDA,
    VALUES,
    DELTA,
    TRYCONFIG,
    TRYDELTA,
    LINEAR,
    DAMPED
  };

  int maxIterations; ///< The maximum iterations to stop iterating (default 100)
  double relativeErrorTol; ///< The maximum relative error decrease to stop iterating (default 1e-5)
  double absoluteErrorTol; ///< The maximum absolute error decrease to stop iterating (default 0.0)
  double errorTol; ///< The maximum total error to stop iterating (default 0.0)
  Verbosity verbosity; ///< The printing verbosity during optimization (default SILENT)

  NonlinearOptimizerParams() :
    maxIterations(100.0), relativeErrorTol(1e-5), absoluteErrorTol(0.0),
    errorTol(0.0), verbosity(SILENT) {}

  void print(const std::string& str = "") const {
    cout << s << "\n";
    cout << "relative decrease threshold: " << relativeErrorTol << "\n";
    cout << "absolute decrease threshold: " << absoluteErrorTol << "\n";
    cout << "      total error threshold: " << errorTol << "\n";
    cout << "         maximum iterations: " << maxIterations << "\n";
    cout << "            verbosity level: " << verbosity << "\n";
  }
};


/**
 * This is the abstract interface for classes that can optimize for the
 * maximum-likelihood estimate of a NonlinearFactorGraph.
 *
 * To use a class derived from this interface, construct the class with a
 * NonlinearFactorGraph and an initial variable assignment.  Next, call the
 * optimize() method, which returns a new NonlinearOptimizer object containing
 * the optimized variable assignment.  Call the values() method to retrieve the
 * optimized estimate.
 *
 * Note:  This class is immutable, optimize() and iterate() return new
 * NonlinearOptimizer objects, so be sure to use the returned object and not
 * simply keep the unchanged original.
 *
 * Example:
 * \code
NonlinearOptimizer::shared_ptr optimizer = DoglegOptimizer::Create(graph, initialValues);
optimizer = optimizer->optimizer();
Values result = optimizer->values();
useTheResult(result);
\endcode
 *
 * Equivalent, but more compact:
 * \code
useTheResult(DoglegOptimizer(graph, initialValues).optimize()->values());
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

  /** An auto pointer to this class */
  typedef std::auto_ptr<NonlinearOptimizer> auto_ptr;

  /** A const shared_ptr to a NonlinearFactorGraph */
  typedef boost::shared_ptr<const NonlinearFactorGraph> SharedGraph;

  /** A const shared_ptr to a NonlinearFactorGraph */
  typedef boost::shared_ptr<const Values> SharedValues;

  /** A const shared_ptr to the parameters */
  typedef boost::shared_ptr<const NonlinearOptimizerParams> SharedParams;

protected:

  const SharedGraph graph_;
  const SharedValues values_;
  const SharedParams params_;
  const double error_;
  const int iterations_;

  NonlinearOptimizer(const SharedGraph& graph, const SharedValues& values,
      const SharedParams& params) :
    graph_(graph), values_(values), params_(params),
    error_(graph_->error(*values_)), iterations_(0) {}

  NonlinearOptimizer(const SharedGraph& graph, const SharedValues& values,
      const SharedParams& params, double error, int iterations) :
    graph_(graph), values_(values), params_(params),
    error_(error), iterations_(iterations) {}

public:

  /// @name Standard interface
  /// @{

  /** Optimize for the maximum-likelihood estimate, returning a new
   * NonlinearOptimizer class containing the optimized variable assignments,
   * which may be retrieved with values().
   */
  virtual auto_ptr optimize() const { return defaultOptimize(); }

  /** Retrieve the current variable assignment estimate. */
  virtual const SharedValues& values() const { return values_; }

  /** Retrieve the parameters. */
  virtual const SharedParams& params() const { return params_; }

  /** Return the current factor graph error */
  virtual double error() const { return error_; }

  /** Return the number of iterations that have been performed */
  virtual int iterations() const { return iterations_; }

  /// @}

  /// @name Advanced interface
  /// @{

  /** Perform a single iteration, returning a new NonlinearOptimizer class
   * containing the updated variable assignments, which may be retrieved with
   * values().
   */
  virtual auto_ptr iterate() const = 0;

  /** Update the nonlinear factor graph, leaving all other state the same.
   * Returns a new updated NonlinearOptimzier object, the original is not
   * modified.
   */
  virtual auto_ptr update(const SharedGraph& newGraph) const = 0;

  /** Update the variable assignments, leaving all other state the same.
   * Returns a new updated NonlinearOptimzier object, the original is not
   * modified.
   */
  virtual auto_ptr update(const SharedValues& newValues) const = 0;

  /** Update the parameters, leaving all other state the same.
   * Returns a new updated NonlinearOptimzier object, the original is not
   * modified.
   */
  virtual auto_ptr update(const SharedParams& newParams) const = 0;

  /** Create a copy of the NonlinearOptimizer */
  virtual auto_ptr clone() const = 0;

  /// @}

protected:

  /** A default implementation of the optimization loop, which calls iterate()
   * until checkConvergence returns true.
   */
  auto_ptr defaultOptimize() const;

};

/** Check whether the relative error decrease is less than relativeErrorTreshold,
 * the absolute error decrease is less than absoluteErrorTreshold, <emph>or</emph>
 * the error itself is less than errorThreshold.
 */
bool checkConvergence(double relativeErrorTreshold,
    double absoluteErrorTreshold, double errorThreshold,
    double currentError, double newError, NonlinearOptimizerParams::Verbosity verbosity);

} // gtsam
