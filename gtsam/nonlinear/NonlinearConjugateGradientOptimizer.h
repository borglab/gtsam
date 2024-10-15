/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file   NonlinearConjugateGradientOptimizer.h
 * @brief  Simple non-linear optimizer that solves using *non-preconditioned* CG
 * @author Yong-Dian Jian
 * @date   June 11, 2012
 */

#pragma once

#include <gtsam/base/Manifold.h>
#include <gtsam/nonlinear/NonlinearOptimizer.h>

namespace gtsam {

/**  An implementation of the nonlinear CG method using the template below */
class GTSAM_EXPORT NonlinearConjugateGradientOptimizer
    : public NonlinearOptimizer {
 public:
  typedef NonlinearOptimizer Base;
  typedef NonlinearOptimizerParams Parameters;
  typedef std::shared_ptr<NonlinearConjugateGradientOptimizer> shared_ptr;

 protected:
  Parameters params_;

  const NonlinearOptimizerParams &_params() const override { return params_; }

 public:
  /// Constructor
  NonlinearConjugateGradientOptimizer(const NonlinearFactorGraph &graph,
                                      const Values &initialValues,
                                      const Parameters &params = Parameters());

  /// Destructor
  ~NonlinearConjugateGradientOptimizer() override {}

  double error(const Values &state) const;

  VectorValues gradient(const Values &state) const;

  Values advance(const Values &current, const double alpha,
                 const VectorValues &g) const;

  /**
   * Perform a single iteration, returning GaussianFactorGraph corresponding to
   * the linearized factor graph.
   */
  GaussianFactorGraph::shared_ptr iterate() override;

  /**
   * Optimize for the maximum-likelihood estimate, returning a the optimized
   * variable assignments.
   */
  const Values &optimize() override;

  /** Implement the golden-section line search algorithm */
  double lineSearch(const Values &currentValues,
                    const VectorValues &gradient) const;

  /**
   * Implement the nonlinear conjugate gradient method using the Polak-Ribiere
   * formula suggested in
   * http://en.wikipedia.org/wiki/Nonlinear_conjugate_gradient_method.
   *
   * The last parameter is a switch between gradient-descent and conjugate
   * gradient
   */
  std::tuple<Values, int> nonlinearConjugateGradient(
      const Values &initial, const NonlinearOptimizerParams &params,
      const bool singleIteration, const bool gradientDescent = false) const;
};

}  // namespace gtsam
