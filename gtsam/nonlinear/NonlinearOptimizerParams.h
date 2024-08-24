/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file   NonlinearOptimizerParams.h
 * @brief  Parameters for nonlinear optimization
 * @author Yong-Dian Jian
 * @author Richard Roberts
 * @author Frank Dellaert
 * @author Andrew Melim
 * @date   Apr 1, 2012
 */

#pragma once

#include <gtsam/linear/GaussianFactorGraph.h>
#include <gtsam/linear/SubgraphSolver.h>

#include <string>
#include <optional>

namespace gtsam {

/** The common parameters for Nonlinear optimizers.  Most optimizers
 * deriving from NonlinearOptimizer also subclass the parameters.
 */
class GTSAM_EXPORT NonlinearOptimizerParams {
public:
  /** See NonlinearOptimizerParams::verbosity */
  enum Verbosity {
    SILENT, TERMINATION, ERROR, VALUES, DELTA, LINEAR
  };

  size_t maxIterations = 100; ///< The maximum iterations to stop iterating (default 100)
  double relativeErrorTol = 1e-5; ///< The maximum relative error decrease to stop iterating (default 1e-5)
  double absoluteErrorTol = 1e-5; ///< The maximum absolute error decrease to stop iterating (default 1e-5)
  double errorTol = 0.0; ///< The maximum total error to stop iterating (default 0.0)
  Verbosity verbosity = SILENT; ///< The printing verbosity during optimization (default SILENT)
  Ordering::OrderingType orderingType = Ordering::COLAMD; ///< The method of ordering use during variable elimination (default COLAMD)

  size_t getMaxIterations() const { return maxIterations; }
  double getRelativeErrorTol() const { return relativeErrorTol; }
  double getAbsoluteErrorTol() const { return absoluteErrorTol; }
  double getErrorTol() const { return errorTol; }
  std::string getVerbosity() const { return verbosityTranslator(verbosity); }

  void setMaxIterations(int value) { maxIterations = value; }
  void setRelativeErrorTol(double value) { relativeErrorTol = value; }
  void setAbsoluteErrorTol(double value) { absoluteErrorTol = value; }
  void setErrorTol(double value) { errorTol = value; }
  void setVerbosity(const std::string& src) {
    verbosity = verbosityTranslator(src);
  }

  static Verbosity verbosityTranslator(const std::string &s) ;
  static std::string verbosityTranslator(Verbosity value) ;

  /** Type for an optional user-provided hook to be called after each
   * internal optimizer iteration. See iterationHook below. */
  using IterationHook = std::function<
    void(size_t /*iteration*/, double/*errorBefore*/, double/*errorAfter*/)>;

  /** Optional user-provided iteration hook to be called after each
   * optimization iteration (Default: none).
   * Note that `IterationHook` is defined as a std::function<> with this
   * signature:
   * \code
   *  void(size_t iteration, double errorBefore, double errorAfter)
   * \endcode
   * which allows binding by means of a reference to a regular function:
   * \code
   *  void foo(size_t iteration, double errorBefore, double errorAfter);
   *  // ...
   *  lmOpts.iterationHook = &foo;
   * \endcode
   * or to a C++11 lambda (preferred if you need to capture additional
   * context variables, such that the optimizer object itself, the factor graph,
   * etc.):
   * \code
   *  lmOpts.iterationHook = [&](size_t iter, double oldError, double newError)
   *  {
   *    // ...
   *  };
   * \endcode
   * or to the result of a properly-formed `std::bind` call.
   */
  IterationHook iterationHook;

  /** See NonlinearOptimizerParams::linearSolverType */
  enum LinearSolverType {
    MULTIFRONTAL_CHOLESKY,
    MULTIFRONTAL_QR,
    SEQUENTIAL_CHOLESKY,
    SEQUENTIAL_QR,
    Iterative, /* Experimental Flag */
    CHOLMOD, /* Experimental Flag */
  };

  LinearSolverType linearSolverType = MULTIFRONTAL_CHOLESKY; ///< The type of linear solver to use in the nonlinear optimizer
  std::optional<Ordering> ordering; ///< The optional variable elimination ordering, or empty to use COLAMD (default: empty)
  IterativeOptimizationParameters::shared_ptr iterativeParams; ///< The container for iterativeOptimization parameters. used in CG Solvers.

  NonlinearOptimizerParams() = default;
  virtual ~NonlinearOptimizerParams() {
  }

  virtual void print(const std::string& str = "") const;

  bool equals(const NonlinearOptimizerParams& other, double tol = 1e-9);

  inline bool isMultifrontal() const {
    return (linearSolverType == MULTIFRONTAL_CHOLESKY)
        || (linearSolverType == MULTIFRONTAL_QR);
  }

  inline bool isSequential() const {
    return (linearSolverType == SEQUENTIAL_CHOLESKY)
        || (linearSolverType == SEQUENTIAL_QR);
  }

  inline bool isCholmod() const {
    return (linearSolverType == CHOLMOD);
  }

  inline bool isIterative() const {
    return (linearSolverType == Iterative);
  }

  GaussianFactorGraph::Eliminate getEliminationFunction() const {
    switch (linearSolverType) {
    case MULTIFRONTAL_CHOLESKY:
    case SEQUENTIAL_CHOLESKY:
      return EliminatePreferCholesky;

    case MULTIFRONTAL_QR:
    case SEQUENTIAL_QR:
      return EliminateQR;

    default:
      throw std::runtime_error(
          "Nonlinear optimization parameter \"factorization\" is invalid");
    }
  }

  std::string getLinearSolverType() const {
    return linearSolverTranslator(linearSolverType);
  }

  void setLinearSolverType(const std::string& solver) {
    linearSolverType = linearSolverTranslator(solver);
  }

  void setIterativeParams(const std::shared_ptr<IterativeOptimizationParameters> params);

  void setOrdering(const Ordering& ordering) {
    this->ordering = ordering;
    this->orderingType = Ordering::CUSTOM;
  }

  std::string getOrderingType() const {
    return orderingTypeTranslator(orderingType);
  }

  // Note that if you want to use a custom ordering, you must set the ordering directly, this will switch to custom type
  void setOrderingType(const std::string& ordering){
    orderingType = orderingTypeTranslator(ordering);
  }

private:
  std::string linearSolverTranslator(LinearSolverType linearSolverType) const;
  LinearSolverType linearSolverTranslator(const std::string& linearSolverType) const;
  std::string orderingTypeTranslator(Ordering::OrderingType type) const;
  Ordering::OrderingType orderingTypeTranslator(const std::string& type) const;
};

// For backward compatibility:
typedef NonlinearOptimizerParams SuccessiveLinearizationParams;

} /* namespace gtsam */
