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
 * @date   Feb 26, 2012
 */

#pragma once

#include <gtsam/nonlinear/NonlinearOptimizer.h>
#include <gtsam/linear/VectorValues.h>
#include <boost/date_time/posix_time/posix_time.hpp>

class NonlinearOptimizerMoreOptimizationTest;

namespace gtsam {

class LevenbergMarquardtOptimizer;

/** Parameters for Levenberg-Marquardt optimization.  Note that this parameters
 * class inherits from NonlinearOptimizerParams, which specifies the parameters
 * common to all nonlinear optimization algorithms.  This class also contains
 * all of those parameters.
 */
class GTSAM_EXPORT LevenbergMarquardtParams: public NonlinearOptimizerParams {

public:
  /** See LevenbergMarquardtParams::lmVerbosity */
  enum VerbosityLM {
    SILENT = 0, TERMINATION, LAMBDA, TRYLAMBDA, TRYCONFIG, DAMPED, TRYDELTA
  };

  static VerbosityLM verbosityLMTranslator(const std::string &s);
  static std::string verbosityLMTranslator(VerbosityLM value);

public:

  double lambdaInitial; ///< The initial Levenberg-Marquardt damping term (default: 1e-5)
  double lambdaFactor; ///< The amount by which to multiply or divide lambda when adjusting lambda (default: 10.0)
  double lambdaUpperBound; ///< The maximum lambda to try before assuming the optimization has failed (default: 1e5)
  double lambdaLowerBound; ///< The minimum lambda used in LM (default: 0)
  VerbosityLM verbosityLM; ///< The verbosity level for Levenberg-Marquardt (default: SILENT), see also NonlinearOptimizerParams::verbosity
  double minModelFidelity; ///< Lower bound for the modelFidelity to accept the result of an LM iteration
  std::string logFile; ///< an optional CSV log file, with [iteration, time, error, labda]
  bool diagonalDamping; ///< if true, use diagonal of Hessian
  bool reuse_diagonal_; ///< an additional option in Ceres for diagonalDamping (related to efficiency)
  bool useFixedLambdaFactor_; ///< if true applies constant increase (or decrease) to lambda according to lambdaFactor
  double min_diagonal_; ///< when using diagonal damping saturates the minimum diagonal entries (default: 1e-6)
  double max_diagonal_; ///< when using diagonal damping saturates the maximum diagonal entries (default: 1e32)

  LevenbergMarquardtParams() :
      lambdaInitial(1e-5), lambdaFactor(10.0), lambdaUpperBound(1e5), lambdaLowerBound(
          0.0), verbosityLM(SILENT), minModelFidelity(1e-3),
          diagonalDamping(false), reuse_diagonal_(false), useFixedLambdaFactor_(true),
          min_diagonal_(1e-6), max_diagonal_(1e32) {
  }
  virtual ~LevenbergMarquardtParams() {
  }

  virtual void print(const std::string& str = "") const;

  inline double getlambdaInitial() const {
    return lambdaInitial;
  }
  inline double getlambdaFactor() const {
    return lambdaFactor;
  }
  inline double getlambdaUpperBound() const {
    return lambdaUpperBound;
  }
  inline double getlambdaLowerBound() const {
    return lambdaLowerBound;
  }
  inline std::string getVerbosityLM() const {
    return verbosityLMTranslator(verbosityLM);
  }
  inline std::string getLogFile() const {
    return logFile;
  }
  inline bool getDiagonalDamping() const {
    return diagonalDamping;
  }

  inline void setlambdaInitial(double value) {
    lambdaInitial = value;
  }
  inline void setlambdaFactor(double value) {
    lambdaFactor = value;
  }
  inline void setlambdaUpperBound(double value) {
    lambdaUpperBound = value;
  }
  inline void setlambdaLowerBound(double value) {
    lambdaLowerBound = value;
  }
  inline void setVerbosityLM(const std::string &s) {
    verbosityLM = verbosityLMTranslator(s);
  }
  inline void setLogFile(const std::string &s) {
    logFile = s;
  }
  inline void setDiagonalDamping(bool flag) {
    diagonalDamping = flag;
  }
  inline void setUseFixedLambdaFactor(bool flag) {
    useFixedLambdaFactor_ = flag;
  }
};

/**
 * State for LevenbergMarquardtOptimizer
 */
class GTSAM_EXPORT LevenbergMarquardtState: public NonlinearOptimizerState {

public:
  double lambda;
  int totalNumberInnerIterations; // The total number of inner iterations in the optimization (for each iteration, LM may try multiple iterations with different lambdas)
  boost::posix_time::ptime startTime;
  VectorValues hessianDiagonal; //only update hessianDiagonal when reuse_diagonal_ = false

  LevenbergMarquardtState() {
    initTime();
  }

  void initTime() {
    startTime = boost::posix_time::microsec_clock::universal_time();
  }

  virtual ~LevenbergMarquardtState() {
  }

protected:
  LevenbergMarquardtState(const NonlinearFactorGraph& graph,
      const Values& initialValues, const LevenbergMarquardtParams& params,
      unsigned int iterations = 0) :
      NonlinearOptimizerState(graph, initialValues, iterations), lambda(
          params.lambdaInitial), totalNumberInnerIterations(0) {
    initTime();
  }

  friend class LevenbergMarquardtOptimizer;
};

/**
 * This class performs Levenberg-Marquardt nonlinear optimization
 */
class GTSAM_EXPORT LevenbergMarquardtOptimizer: public NonlinearOptimizer {

protected:
  LevenbergMarquardtParams params_; ///< LM parameters
  LevenbergMarquardtState state_; ///< optimization state

public:
  typedef boost::shared_ptr<LevenbergMarquardtOptimizer> shared_ptr;

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
  LevenbergMarquardtOptimizer(const NonlinearFactorGraph& graph,
      const Values& initialValues, const LevenbergMarquardtParams& params =
          LevenbergMarquardtParams()) :
      NonlinearOptimizer(graph), params_(ensureHasOrdering(params, graph)), state_(
          graph, initialValues, params_) {
  }

  /** Standard constructor, requires a nonlinear factor graph, initial
   * variable assignments, and optimization parameters.  For convenience this
   * version takes plain objects instead of shared pointers, but internally
   * copies the objects.
   * @param graph The nonlinear factor graph to optimize
   * @param initialValues The initial variable assignments
   */
  LevenbergMarquardtOptimizer(const NonlinearFactorGraph& graph,
      const Values& initialValues, const Ordering& ordering) :
      NonlinearOptimizer(graph) {
    params_.ordering = ordering;
    state_ = LevenbergMarquardtState(graph, initialValues, params_);
  }

  /// Access the current damping value
  double lambda() const {
    return state_.lambda;
  }

  // Apply policy to increase lambda if the current update was successful (stepQuality not used in the naive policy)
  void increaseLambda();

  // Apply policy to decrease lambda if the current update was NOT successful (stepQuality not used in the naive policy)
  void decreaseLambda(double stepQuality);

  /// Access the current number of inner iterations
  int getInnerIterations() const {
    return state_.totalNumberInnerIterations;
  }

  /// print
  virtual void print(const std::string& str = "") const {
    std::cout << str << "LevenbergMarquardtOptimizer" << std::endl;
    this->params_.print("  parameters:\n");
  }

  /// @}

  /// @name Advanced interface
  /// @{

  /** Virtual destructor */
  virtual ~LevenbergMarquardtOptimizer() {
  }

  /** Perform a single iteration, returning a new NonlinearOptimizer class
   * containing the updated variable assignments, which may be retrieved with
   * values().
   */
  virtual void iterate();

  /** Read-only access the parameters */
  const LevenbergMarquardtParams& params() const {
    return params_;
  }

  /** Read/write access the parameters */
  LevenbergMarquardtParams& params() {
    return params_;
  }

  /** Read-only access the last state */
  const LevenbergMarquardtState& state() const {
    return state_;
  }

  /** Read/write access the last state. When modifying the state, the error, etc. must be consistent before calling iterate() */
  LevenbergMarquardtState& state() {
    return state_;
  }

  /** Build a damped system for a specific lambda */
  GaussianFactorGraph::shared_ptr buildDampedSystem(const GaussianFactorGraph& linear);
  friend class ::NonlinearOptimizerMoreOptimizationTest;

  void writeLogFile(double currentError);

  /// @}

protected:

  /** Access the parameters (base class version) */
  virtual const NonlinearOptimizerParams& _params() const {
    return params_;
  }

  /** Access the state (base class version) */
  virtual const NonlinearOptimizerState& _state() const {
    return state_;
  }

  /** Internal function for computing a COLAMD ordering if no ordering is specified */
  LevenbergMarquardtParams ensureHasOrdering(LevenbergMarquardtParams params,
      const NonlinearFactorGraph& graph) const;

  /** linearize, can  be overwritten */
  virtual GaussianFactorGraph::shared_ptr linearize() const;
};

}
