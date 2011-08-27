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

#include <gtsam/nonlinear/NonlinearFactorGraph-inl.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

namespace gtsam {

/**
 * This is a generic Extended Kalman Filter class implemented using nonlinear factors. GTSAM
 * basically does SRIF with LDL to solve the filter problem, making this an efficient, numerically
 * stable Kalman Filter implementation.
 *
 * The Kalman Filter relies on two models: a motion model that predicts the next state using
 * the current state, and a measurement model that predicts the measurement value at a given
 * state. Because these two models are situation-dependent, base classes for each have been
 * provided above, from which the user must derive a class and incorporate the actual modeling
 * equations. A pointer to an instance of each of these classes must be given to the Kalman
 * Filter at creation, allowing the Kalman Filter to create factors as needed.
 *
 * The Kalman Filter class provides a "predict" function and an "update" function to perform
 * these steps independently, as well as a "predictAndUpdate" that combines both steps for some
 * computational savings.
 */

template<class VALUES, class KEY>
class ExtendedKalmanFilter {
public:

  typedef boost::shared_ptr<ExtendedKalmanFilter<VALUES, KEY> > shared_ptr;
  typedef typename KEY::Value T;
  typedef NonlinearFactor2<VALUES, KEY, KEY> MotionFactor;
  typedef typename MotionFactor::shared_ptr SharedMotionFactor;
  typedef NonlinearFactor1<VALUES, KEY> MeasurementFactor;
  typedef typename MeasurementFactor::shared_ptr SharedMeasurementFactor;

protected:
  JacobianFactor::shared_ptr priorFactor_;
  T x_;

  T solve_(const GaussianFactorGraph& linearFactorGraph, const Ordering& ordering, const VALUES& linearizationPoints, const KEY& x,
      JacobianFactor::shared_ptr& newPrior) const;

public:
  ExtendedKalmanFilter(T x_initial, noiseModel::Gaussian::shared_ptr P_initial);

  T predict(const MotionFactor& motionFactor);
  T update(const MeasurementFactor& measurementFactor);
};

} // namespace
