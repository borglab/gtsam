/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    ExtendedKalmanFilter.h
 * @brief   Class to perform generic Kalman Filtering using nonlinear factor graphs
 * @author  Stephen Williams
 * @author  Chris Beall
 */

// \callgraph
#pragma once

#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

namespace gtsam {

/**
 * This is a generic Extended Kalman Filter class implemented using nonlinear factors. GTSAM
 * basically does SRIF with Cholesky to solve the filter problem, making this an efficient,
 *numerically
 * stable Kalman Filter implementation.
 *
 * The Kalman Filter relies on two models: a motion model that predicts the next state using
 * the current state, and a measurement model that predicts the measurement value at a given
 * state. Because these two models are situation-dependent, base classes for each have been
 * provided above, from which the user must derive a class and incorporate the actual modeling
 * equations.
 *
 * The class provides a "predict" and "update" function to perform these steps independently.
 * TODO: a "predictAndUpdate" that combines both steps for some computational savings.
 * \nosubgrouping
 */

template <class VALUE>
class ExtendedKalmanFilter {
  // Check that VALUE type is a testable Manifold
  GTSAM_CONCEPT_ASSERT(IsTestable<VALUE>);
  GTSAM_CONCEPT_ASSERT(IsManifold<VALUE>);

 public:
  typedef std::shared_ptr<ExtendedKalmanFilter<VALUE> > shared_ptr;
  typedef VALUE T;

 protected:
  T x_;                                     // linearization point
  JacobianFactor::shared_ptr priorFactor_;  // Gaussian density on x_

  static T solve_(const GaussianFactorGraph& linearFactorGraph, const Values& linearizationPoints,
                  Key x, JacobianFactor::shared_ptr* newPrior);

 public:
  /// @name Standard Constructors
  /// @{

  ExtendedKalmanFilter(Key key_initial, T x_initial, noiseModel::Gaussian::shared_ptr P_initial);

  /// @}
  /// @name Testable
  /// @{

  /// print
  void print(const std::string& s = "") const {
    std::cout << s << "\n";
    x_.print(s + "x");
    priorFactor_->print(s + "density");
  }

  /// @}
  /// @name Interface
  /// @{

  /**
   * Calculate predictive density
   *     \f$ P(x_) ~ \int  P(x_min) P(x_min, x_)\f$ 
   * The motion model should be given as a factor with key1 for \f$x_min\f$ and key2 for \f$x_\f$ 
   */
  T predict(const NoiseModelFactor& motionFactor);

  /**
   * Calculate posterior density P(x_) ~ L(z|x) P(x)
   * The likelihood L(z|x) should be given as a unary factor on x
   */
  T update(const NoiseModelFactor& measurementFactor);

  /// Return current predictive (if called after predict)/posterior (if called after update)
  const JacobianFactor::shared_ptr Density() const {
    return priorFactor_;
  }

  /// @}
};

}  // namespace

#include <gtsam/nonlinear/ExtendedKalmanFilter-inl.h>
