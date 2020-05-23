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
#include <gtsam/linear/LinearSolverParams.h>
#include <boost/optional.hpp>
#include <string>
#include <gtsam/linear/LinearSolver.h>

namespace gtsam {

/** The common parameters for Nonlinear optimizers.  Most optimizers
 * deriving from NonlinearOptimizer also subclass the parameters.
 */
class GTSAM_EXPORT NonlinearOptimizerParams: public LinearSolverParams {
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

  NonlinearOptimizerParams() :
      maxIterations(100), relativeErrorTol(1e-5), absoluteErrorTol(1e-5), errorTol(
          0.0), verbosity(SILENT) {}

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
};

// For backward compatibility:
typedef NonlinearOptimizerParams SuccessiveLinearizationParams;

} /* namespace gtsam */
