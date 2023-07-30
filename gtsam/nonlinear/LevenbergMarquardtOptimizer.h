/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    LevenbergMarquardtOptimizer.h
 * @brief   A nonlinear optimizer that uses the Levenberg-Marquardt trust-region scheme
 * @author  Richard Roberts
 * @author  Frank Dellaert
 * @author  Luca Carlone
 * @date    Feb 26, 2012
 */

#pragma once

#include <gtsam/nonlinear/NonlinearOptimizer.h>
#include <gtsam/nonlinear/LevenbergMarquardtParams.h>
#include <gtsam/linear/VectorValues.h>
#include <chrono>

class NonlinearOptimizerMoreOptimizationTest;

namespace gtsam {

/**
 * This class performs Levenberg-Marquardt nonlinear optimization
 */
class GTSAM_EXPORT LevenbergMarquardtOptimizer: public NonlinearOptimizer {

protected:
  const LevenbergMarquardtParams params_; ///< LM parameters
  
  // startTime_ is a chrono time point
  std::chrono::time_point<std::chrono::high_resolution_clock> startTime_; ///< time when optimization started

  void initTime();

public:
  typedef std::shared_ptr<LevenbergMarquardtOptimizer> shared_ptr;

  /// @name Constructors/Destructor
  /// @{

  /** Standard constructor, requires a nonlinear factor graph, initial
   * variable assignments, and optimization parameters.  For convenience this
   * version takes plain objects instead of shared pointers, but internally
   * copies the objects.
   * @param graph The nonlinear factor graph to optimize
   * @param initialValues The initial variable assignments
   * @param params The optimization parameters
   */
  LevenbergMarquardtOptimizer(const NonlinearFactorGraph& graph, const Values& initialValues,
                              const LevenbergMarquardtParams& params = LevenbergMarquardtParams());

  /** Standard constructor, requires a nonlinear factor graph, initial
   * variable assignments, and optimization parameters.  For convenience this
   * version takes plain objects instead of shared pointers, but internally
   * copies the objects.
   * @param graph The nonlinear factor graph to optimize
   * @param initialValues The initial variable assignments
   */
  LevenbergMarquardtOptimizer(const NonlinearFactorGraph& graph, const Values& initialValues,
                              const Ordering& ordering,
                              const LevenbergMarquardtParams& params = LevenbergMarquardtParams());

  /** Virtual destructor */
  ~LevenbergMarquardtOptimizer() override {
  }

  /// @}

  /// @name Standard interface
  /// @{

  /// Access the current damping value
  double lambda() const;

  /// Access the current number of inner iterations
  int getInnerIterations() const;

  /// print
  void print(const std::string& str = "") const {
    std::cout << str << "LevenbergMarquardtOptimizer" << std::endl;
    this->params_.print("  parameters:\n");
  }

  /// @}

  /// @name Advanced interface
  /// @{

  /** 
   * Perform a single iteration, returning GaussianFactorGraph corresponding to 
   * the linearized factor graph.
   */
  GaussianFactorGraph::shared_ptr iterate() override;

  /** Read-only access the parameters */
  const LevenbergMarquardtParams& params() const {
    return params_;
  }

  void writeLogFile(double currentError);

  /** linearize, can be overwritten */
  virtual GaussianFactorGraph::shared_ptr linearize() const;

  /** Build a damped system for a specific lambda -- for testing only */
  GaussianFactorGraph buildDampedSystem(const GaussianFactorGraph& linear,
                                        const VectorValues& sqrtHessianDiagonal) const;

  /** Inner loop, changes state, returns true if successful or giving up */
  bool tryLambda(const GaussianFactorGraph& linear, const VectorValues& sqrtHessianDiagonal);

  /// @}

protected:

  /** Access the parameters (base class version) */
  const NonlinearOptimizerParams& _params() const override {
    return params_;
  }
};

}
