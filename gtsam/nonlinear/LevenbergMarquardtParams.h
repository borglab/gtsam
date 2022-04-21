/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    LevenbergMarquardtParams.h
 * @brief   Parameters for Levenberg-Marquardt trust-region scheme
 * @author  Richard Roberts
 * @author  Frank Dellaert
 * @author  Luca Carlone
 * @date    Feb 26, 2012
 */

#pragma once

#include <gtsam/nonlinear/NonlinearOptimizerParams.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>

namespace gtsam {

class LevenbergMarquardtOptimizer;

/** Parameters for Levenberg-Marquardt optimization.  Note that this parameters
 * class inherits from NonlinearOptimizerParams, which specifies the parameters
 * common to all nonlinear optimization algorithms.  This class also contains
 * all of those parameters.
 */
class GTSAM_EXPORT LevenbergMarquardtParams: public NonlinearOptimizerParams {

public:
  /** See LevenbergMarquardtParams::verbosityLM */
  enum VerbosityLM {
    SILENT = 0, SUMMARY, TERMINATION, LAMBDA, TRYLAMBDA, TRYCONFIG, DAMPED, TRYDELTA
  };

  static VerbosityLM verbosityLMTranslator(const std::string &s);
  static std::string verbosityLMTranslator(VerbosityLM value);
  using OptimizerType = LevenbergMarquardtOptimizer;

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

  static LevenbergMarquardtParams EnsureHasOrdering(LevenbergMarquardtParams params,
                                                    const NonlinearFactorGraph& graph) {
    if (!params.ordering)
      params.ordering = Ordering::Create(params.orderingType, graph);
    return params;
  }

  static LevenbergMarquardtParams ReplaceOrdering(LevenbergMarquardtParams params,
                                                  const Ordering& ordering) {
    params.ordering = ordering;
    return params;
  }

  ~LevenbergMarquardtParams() override {}
  void print(const std::string& str = "") const override;

  /// @name Getters/Setters, mainly for wrappers. Use fields above in C++.
  /// @{
  bool getDiagonalDamping() const { return diagonalDamping; }
  double getlambdaFactor() const { return lambdaFactor; }
  double getlambdaInitial() const { return lambdaInitial; }
  double getlambdaLowerBound() const { return lambdaLowerBound; }
  double getlambdaUpperBound() const { return lambdaUpperBound; }
  bool getUseFixedLambdaFactor() { return useFixedLambdaFactor; }
  std::string getLogFile() const { return logFile; }
  std::string getVerbosityLM() const { return verbosityLMTranslator(verbosityLM);}
  
  void setDiagonalDamping(bool flag) { diagonalDamping = flag; }
  void setlambdaFactor(double value) { lambdaFactor = value; }
  void setlambdaInitial(double value) { lambdaInitial = value; }
  void setlambdaLowerBound(double value) { lambdaLowerBound = value; }
  void setlambdaUpperBound(double value) { lambdaUpperBound = value; }
  void setUseFixedLambdaFactor(bool flag) { useFixedLambdaFactor = flag;}
  void setLogFile(const std::string& s) { logFile = s; }
  void setVerbosityLM(const std::string& s) { verbosityLM = verbosityLMTranslator(s);}
  // @}
  /// @name Clone
  /// @{

  /// @return a deep copy of this object
  boost::shared_ptr<NonlinearOptimizerParams> clone() const {
    return boost::shared_ptr<NonlinearOptimizerParams>(new LevenbergMarquardtParams(*this));
  }

  /// @}
};

}
