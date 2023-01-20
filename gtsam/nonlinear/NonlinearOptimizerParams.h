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
#include <gtsam/linear/LinearSolverParams.h>
#include <boost/optional.hpp>
#include <string>
#include <gtsam/linear/LinearSolver.h>

namespace gtsam {

// forward declaration
class IterativeOptimizationParameters;

/** The common parameters for Nonlinear optimizers.  Most optimizers
 * deriving from NonlinearOptimizer also subclass the parameters.
 */
class GTSAM_EXPORT NonlinearOptimizerParams {
 public:
  /** See NonlinearOptimizerParams::verbosity */
  enum Verbosity {
    SILENT, TERMINATION, ERROR, VALUES, DELTA, LINEAR
  };

  size_t maxIterations; ///< The maximum iterations to stop iterating (default 100)
  double relativeErrorTol; ///< The maximum relative error decrease to stop iterating (default 1e-5)
  double absoluteErrorTol; ///< The maximum absolute error decrease to stop iterating (default 1e-5)
  double errorTol; ///< The maximum total error to stop iterating (default 0.0)
  Verbosity verbosity; ///< The printing verbosity during optimization (default SILENT)

  NonlinearOptimizerParams()
      : maxIterations(100),
        relativeErrorTol(1e-5),
        absoluteErrorTol(1e-5),
        errorTol(0.0),
        verbosity(SILENT) {}

  // copy constructor
  NonlinearOptimizerParams(const NonlinearOptimizerParams& other)
      : maxIterations(other.maxIterations),
        relativeErrorTol(other.relativeErrorTol),
        absoluteErrorTol(other.absoluteErrorTol),
        errorTol(other.errorTol),
        verbosity(other.verbosity),
        linearSolverParams(other.linearSolverParams) {}

  // move constructor
  NonlinearOptimizerParams(NonlinearOptimizerParams&& other) noexcept
      : maxIterations(other.maxIterations),
        relativeErrorTol(other.relativeErrorTol),
        absoluteErrorTol(other.absoluteErrorTol),
        errorTol(other.errorTol),
        verbosity(other.verbosity),
        linearSolverParams(std::move(other.linearSolverParams)) {}

  // copy assignment
  NonlinearOptimizerParams& operator=(const NonlinearOptimizerParams& other) {
    return *this = NonlinearOptimizerParams(other);
  }

  // move assignment
  NonlinearOptimizerParams& operator=(
      NonlinearOptimizerParams&& other) noexcept {
    maxIterations = other.maxIterations;
    relativeErrorTol = other.relativeErrorTol;
    absoluteErrorTol = other.absoluteErrorTol;
    errorTol = other.errorTol;
    verbosity = other.verbosity;
    std::swap(linearSolverParams, other.linearSolverParams);
    return *this;
  }

  virtual ~NonlinearOptimizerParams() {
  }
  virtual void print(const std::string& str = "") const;

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

  /// The parameters for the linear backend solver
  LinearSolverParams linearSolverParams;

  /**
   * @name Linear Properties (for Backwards Compatibility)
   * These member variables and functions reference LinearSolverParams but must
   * be accessible as members of NonlinearOptimizerParams for backwards
   * compatibility reasons
   */
  ///@{

  /** See LinearSolverParams::LinearSolverType */
  typedef LinearSolverParams::LinearSolverType LinearSolverType;
  static constexpr LinearSolverType MULTIFRONTAL_CHOLESKY = LinearSolverParams::MULTIFRONTAL_CHOLESKY;
  static constexpr LinearSolverType MULTIFRONTAL_QR = LinearSolverParams::MULTIFRONTAL_QR;
  static constexpr LinearSolverType SEQUENTIAL_CHOLESKY = LinearSolverParams::SEQUENTIAL_CHOLESKY;
  static constexpr LinearSolverType SEQUENTIAL_QR = LinearSolverParams::SEQUENTIAL_QR;
  static constexpr LinearSolverType Iterative = LinearSolverParams::Iterative; /* Experimental Flag */
  static constexpr LinearSolverType CHOLMOD = LinearSolverParams::CHOLMOD; /* Experimental Flag */
  static constexpr LinearSolverType PCG = LinearSolverParams::PCG;
  static constexpr LinearSolverType SUBGRAPH = LinearSolverParams::SUBGRAPH;
  static constexpr LinearSolverType EIGEN_QR = LinearSolverParams::EIGEN_QR;
  static constexpr LinearSolverType EIGEN_CHOLESKY = LinearSolverParams::EIGEN_CHOLESKY;
  static constexpr LinearSolverType SUITESPARSE = LinearSolverParams::SUITESPARSE;
  static constexpr LinearSolverType CUSPARSE = LinearSolverParams::CUSPARSE;
  static constexpr LinearSolverType LAST = LinearSolverParams::LAST;

  /// The type of linear solver to use in the nonlinear optimizer
  /// (default: MULTIFRONTAL_CHOLESKY)
  LinearSolverType &linearSolverType {linearSolverParams.linearSolverType};
  /// The method of ordering use during variable elimination (default: COLAMD)
  Ordering::OrderingType &orderingType {linearSolverParams.orderingType};
  /// The variable elimination ordering, or empty to use COLAMD (default: empty)
  boost::optional<Ordering> &ordering {linearSolverParams.ordering};
  /// The container for iterativeOptimization parameters. used in CG Solvers.
  boost::shared_ptr<IterativeOptimizationParameters>& iterativeParams{
      linearSolverParams.iterativeParams};

  NonlinearOptimizerParams() = default;
  virtual ~NonlinearOptimizerParams() {
  }

  virtual void print(const std::string& str = "") const;

  bool equals(const NonlinearOptimizerParams& other, double tol = 1e-9) const {
    return maxIterations == other.getMaxIterations()
        && std::abs(relativeErrorTol - other.getRelativeErrorTol()) <= tol
        && std::abs(absoluteErrorTol - other.getAbsoluteErrorTol()) <= tol
        && std::abs(errorTol - other.getErrorTol()) <= tol
        && verbosityTranslator(verbosity) == other.getVerbosity();
    //  && orderingType.equals(other.getOrderingType()_;
    // && linearSolverType == other.getLinearSolverType();
    // TODO: check ordering, iterativeParams, and iterationsHook
  }

  inline bool isMultifrontal() const {
    return linearSolverParams.isMultifrontal();
  }

  inline bool isSequential() const {
    return linearSolverParams.isSequential();
  }

  inline bool isCholmod() const {
    return linearSolverParams.isCholmod();
  }

  inline bool isIterative() const {
    return linearSolverParams.isIterative();
  }

  GaussianFactorGraph::Eliminate getEliminationFunction() const {
    return linearSolverParams.getEliminationFunction();
  }

  std::string getLinearSolverType() const {
    return linearSolverParams.getLinearSolverType();
  }

  void setLinearSolverType(const std::string& solver) {
    linearSolverParams.setLinearSolverType(solver);
  }

  void setIterativeParams(const boost::shared_ptr<IterativeOptimizationParameters> params) {
    linearSolverParams.setIterativeParams(params);
  }

  void setOrdering(const Ordering& ordering) {
    linearSolverParams.setOrdering(ordering);
  }

  std::string getOrderingType() const {
    return linearSolverParams.getOrderingType();
  }

  // Note that if you want to use a custom ordering, you must set the ordering directly, this will switch to custom type
  void setOrderingType(const std::string& ordering){
    linearSolverParams.setOrderingType(ordering);
  }

  ///@}
};

// For backward compatibility:
typedef NonlinearOptimizerParams SuccessiveLinearizationParams;

} /* namespace gtsam */
