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
    SILENT = 0, SUMMARY, TERMINATION, LAMBDA, TRYLAMBDA, TRYCONFIG, DAMPED, TRYDELTA
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
  std::string logFile; ///< an optional CSV log file, with [iteration, time, error, lambda]
  bool diagonalDamping; ///< if true, use diagonal of Hessian
  bool useFixedLambdaFactor; ///< if true applies constant increase (or decrease) to lambda according to lambdaFactor
  double minDiagonal; ///< when using diagonal damping saturates the minimum diagonal entries (default: 1e-6)
  double maxDiagonal; ///< when using diagonal damping saturates the maximum diagonal entries (default: 1e32)

  LevenbergMarquardtParams()
      : verbosityLM(SILENT),
        diagonalDamping(false),
        minDiagonal(1e-6),
        maxDiagonal(1e32) {
    SetLegacyDefaults(this);
  }

  static void SetLegacyDefaults(LevenbergMarquardtParams* p) {
    // Relevant NonlinearOptimizerParams:
    p->maxIterations = 100;
    p->relativeErrorTol = 1e-5;
    p->absoluteErrorTol = 1e-5;
    // LM-specific:
    p->lambdaInitial = 1e-5;
    p->lambdaFactor = 10.0;
    p->lambdaUpperBound = 1e5;
    p->lambdaLowerBound = 0.0;
    p->minModelFidelity = 1e-3;
    p->diagonalDamping = false;
    p->useFixedLambdaFactor = true;
  }

  // these do seem to work better for SFM
  static void SetCeresDefaults(LevenbergMarquardtParams* p) {
    // Relevant NonlinearOptimizerParams:
    p->maxIterations = 50;
    p->absoluteErrorTol = 0;     // No corresponding option in CERES
    p->relativeErrorTol = 1e-6;  // This is function_tolerance
    // LM-specific:
    p->lambdaUpperBound = 1e32;
    p->lambdaLowerBound = 1e-16;
    p->lambdaInitial = 1e-04;
    p->lambdaFactor = 2.0;
    p->minModelFidelity = 1e-3;  // options.min_relative_decrease in CERES
    p->diagonalDamping = true;
    p->useFixedLambdaFactor = false;  // This is important
  }

  static LevenbergMarquardtParams LegacyDefaults() {
    LevenbergMarquardtParams p;
    SetLegacyDefaults(&p);
    return p;
  }

  static LevenbergMarquardtParams CeresDefaults() {
    LevenbergMarquardtParams p;
    SetCeresDefaults(&p);
    return p;
  }

  virtual ~LevenbergMarquardtParams() {}
  virtual void print(const std::string& str = "") const;

  /// @name Getters/Setters, mainly for MATLAB. Use fields above in C++.
  /// @{
  bool getDiagonalDamping() const { return diagonalDamping; }
  double getlambdaFactor() const { return lambdaFactor; }
  double getlambdaInitial() const { return lambdaInitial; }
  double getlambdaLowerBound() const { return lambdaLowerBound; }
  double getlambdaUpperBound() const { return lambdaUpperBound; }
  std::string getLogFile() const { return logFile; }
  std::string getVerbosityLM() const { return verbosityLMTranslator(verbosityLM);}
  void setDiagonalDamping(bool flag) { diagonalDamping = flag; }
  void setlambdaFactor(double value) { lambdaFactor = value; }
  void setlambdaInitial(double value) { lambdaInitial = value; }
  void setlambdaLowerBound(double value) { lambdaLowerBound = value; }
  void setlambdaUpperBound(double value) { lambdaUpperBound = value; }
  void setLogFile(const std::string& s) { logFile = s; }
  void setUseFixedLambdaFactor(bool flag) { useFixedLambdaFactor = flag;}
  void setVerbosityLM(const std::string& s) { verbosityLM = verbosityLMTranslator(s);}
  // @}
};

/**
 * State for LevenbergMarquardtOptimizer
 */
class GTSAM_EXPORT LevenbergMarquardtState: public NonlinearOptimizerState {

public:
  double lambda;
  boost::posix_time::ptime startTime;
  int totalNumberInnerIterations; //< The total number of inner iterations in the optimization (for each iteration, LM may try multiple iterations with different lambdas)
  VectorValues hessianDiagonal; //< we only update hessianDiagonal when reuseDiagonal = false
  bool reuseDiagonal; ///< an additional option in Ceres for diagonalDamping

  LevenbergMarquardtState() {} // used in LM constructor but immediately overwritten

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
          params.lambdaInitial), totalNumberInnerIterations(0),reuseDiagonal(false) {
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
  LevenbergMarquardtOptimizer(
      const NonlinearFactorGraph& graph, const Values& initialValues,
      const LevenbergMarquardtParams& params = LevenbergMarquardtParams())
      : NonlinearOptimizer(graph),
        params_(ensureHasOrdering(params, graph)),
        state_(graph, initialValues, params_) {}

  /** Standard constructor, requires a nonlinear factor graph, initial
   * variable assignments, and optimization parameters.  For convenience this
   * version takes plain objects instead of shared pointers, but internally
   * copies the objects.
   * @param graph The nonlinear factor graph to optimize
   * @param initialValues The initial variable assignments
   */
  LevenbergMarquardtOptimizer(
      const NonlinearFactorGraph& graph, const Values& initialValues,
      const Ordering& ordering,
      const LevenbergMarquardtParams& params = LevenbergMarquardtParams())
      : NonlinearOptimizer(graph), params_(params) {
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

  /** Small struct to cache objects needed for damping.
   * This is used in buildDampedSystem  */
  struct NoiseCacheItem {
    Matrix A;
    Vector b;
    SharedDiagonal model;
  };

  /// Noise model Cache
  typedef std::vector<NoiseCacheItem> NoiseCacheVector;

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
