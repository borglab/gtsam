/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 *  @file  ExpressionFactorGraph.h
 *  @brief Factor graph that supports adding ExpressionFactors directly
 *  @author Frank Dellaert
 *  @date December 2014
 */

#pragma once

#include <gtsam/nonlinear/ExpressionFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>

namespace gtsam {

/**
 * Factor graph that supports adding ExpressionFactors directly
 */
class ExpressionFactorGraph: public NonlinearFactorGraph {

public:

  /// @name Adding Factors
  /// @{

  /**
   * Directly add ExpressionFactor that implements |h(x)-z|^2_R
   * @param h expression that implements measurement function
   * @param z measurement
   * @param R model
   */
  template<typename T>
  void addExpressionFactor(const Expression<T>& h, const T& z,
      const SharedNoiseModel& R) {
    using F = ExpressionFactor<T>;
    push_back(boost::allocate_shared<F>(Eigen::aligned_allocator<F>(), R, z, h));
  }

  /// @}
};

}
