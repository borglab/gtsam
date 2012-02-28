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
 * @created Feb 26, 2012
 */

#pragma once

#include <gtsam/nonlinear/NonlinearOptimizer.h>

namespace gtsam {

/** Parameters for Levenberg-Marquardt optimization, inherits from
 * NonlinearOptimizationParams.
 */
class LevenbergMarquardtParams : public NonlinearOptimizerParams {
public:
  /** See LevenbergMarquardtParams::elimination */
  enum Elimination {
    MULTIFRONTAL,
    SEQUENTIAL
  };

  /** See LevenbergMarquardtParams::factorization */
  enum Factorization {
    LDL,
    QR,
  };

  /** See LevenbergMarquardtParams::lmVerbosity */
  enum LMVerbosity {

  };

  Elimination elimination; ///< The elimination algorithm to use (default: MULTIFRONTAL)
  Factorization factorization; ///< The numerical factorization (default: LDL)
  Ordering::shared_ptr ordering; ///< The variable elimination ordering (default: empty -> COLAMD)
  double lambda; ///< The initial (and current after each iteration) Levenberg-Marquardt damping term (default: 1e-5)
  double lambdaFactor; ///< The amount by which to multiply or divide lambda when adjusting lambda (default: 10.0)
  double lambdaUpperBound; ///< The maximum lambda to try before assuming the optimization has failed (default: 1e5)

  LevenbergMarquardtParams() :
    elimination(MULTIFRONTAL), factorization(LDL), lambda(1e-5), lambdaFactor(10.0) {}

  virtual void print(const std::string& str = "") const {
    NonlinearOptimizerParams::print(str);
    if(elimination == MULTIFRONTAL)
      std::cout << "         elimination method: MULTIFRONTAL\n";
    else if(elimination == SEQUENTIAL)
      std::cout << "         elimination method: SEQUENTIAL\n";
    else
      std::cout << "         elimination method: (invalid)\n";

    if(factorization == LDL)
      std::cout << "       factorization method: LDL\n";
    else if(factorization == QR)
      std::cout << "       factorization method: QR\n";
    else if(factorization == CHOLESKY)
      std::cout << "       factorization method: CHOLESKY\n";
    else
      std::cout << "       factorization method: (invalid)\n";

    std::cout << "                     lambda: " << lambda << "\n";
    std::cout << "               lambdaFactor: " << lambdaFactor << "\n";

    std::cout.flush();
  }
};

/**
 * This class performs Levenberg-Marquardt nonlinear optimization
 */
class LevenbergMarquardtOptimizer : public NonlinearOptimizer {

public:

  typedef boost::shared_ptr<LevenbergMarquardtParams> SharedLMParams;

  /// @name Standard interface
  /// @{

  /** Standard constructor, requires a nonlinear factor graph, initial
   * variable assignments, and optimization parameters.  For convenience this
   * version takes plain objects instead of shared pointers, but internally
   * copies the objects.
   * @param graph The nonlinear factor graph to optimize
   * @param values The initial variable assignments
   * @param params The optimization parameters
   */
  LevenbergMarquardtOptimizer(const NonlinearFactorGraph& graph, const Values& values,
      const LevenbergMarquardtParams& params) :
        NonlinearOptimizer(
            SharedGraph(new NonlinearFactorGraph(graph)),
            SharedValues(new Values(values)),
            SharedLMParams(new LevenbergMarquardtParams(params))),
        lmParams_(boost::static_pointer_cast<LevenbergMarquardtParams>(params_)) {}

  /** Standard constructor, requires a nonlinear factor graph, initial
   * variable assignments, and optimization parameters.
   * @param graph The nonlinear factor graph to optimize
   * @param values The initial variable assignments
   * @param params The optimization parameters
   */
  LevenbergMarquardtOptimizer(const SharedGraph& graph, const SharedValues& values,
      const SharedLMParams& params) :
        NonlinearOptimizer(graph, values, params), lmParams_(params) {}

  /// @}

  /// @name Advanced interface
  /// @{

  /** Virtual destructor */
  virtual ~LevenbergMarquardtOptimizer() {}

  /** Perform a single iteration, returning a new NonlinearOptimizer class
   * containing the updated variable assignments, which may be retrieved with
   * values().
   */
  virtual auto_ptr iterate() const;

  /** Update the graph, values, and/or parameters, leaving all other state
   * the same.  Any of these that are empty shared pointers are left unchanged
   * in the returned optimizer object.  Returns a new updated
   * NonlinearOptimzier object, the original is not modified.
   */
  virtual auto_ptr update(
      const SharedGraph& newGraph = SharedGraph(),
      const SharedValues& newValues = SharedValues(),
      const SharedParams& newParams = SharedParams()) const {
    return new LevenbergMarquardtOptimizer(*this, newGraph, newValues,
        boost::dynamic_pointer_cast<LevenbergMarquardtParams>(newParams));
  }

  /** Create a copy of the NonlinearOptimizer */
  virtual auto_ptr clone() const {
    return new LevenbergMarquardtOptimizer(*this);
  }

  /// @}

protected:

  const SharedLMParams lmParams_;

  LevenbergMarquardtOptimizer(const SharedGraph& graph, const SharedValues& values,
      const SharedLMParams& params, double error, int iterations) :
    NonlinearOptimizer(graph, values, params, error, iterations), lmParams_(params) {}

  LevenbergMarquardtOptimizer(const LevenbergMarquardtOptimizer& original, const SharedGraph& newGraph,
      const SharedValues& newValues, const SharedLMParams& newParams) :
    NonlinearOptimizer(original, newGraph, newValues, newParams),
    lmParams_(newParams ? newParams : original.lmParams_) {}

};

}
