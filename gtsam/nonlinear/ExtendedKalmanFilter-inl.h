/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    ExtendedKalmanFilter-inl.h
 * @brief   Class to perform generic Kalman Filtering using nonlinear factor graphs
 * @author  Stephen Williams
 * @author  Chris Beall
 */

#pragma once

#include <gtsam/nonlinear/ExtendedKalmanFilter.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/linear/GaussianBayesNet.h>
#include <gtsam/linear/GaussianFactorGraph.h>

namespace gtsam {

  /* ************************************************************************* */
  template<class VALUE>
  typename ExtendedKalmanFilter<VALUE>::T ExtendedKalmanFilter<VALUE>::solve_(
      const GaussianFactorGraph& linearFactorGraph,
      const Values& linearizationPoint, Key lastKey,
      JacobianFactor::shared_ptr* newPrior)
  {
    // Compute the marginal on the last key
    // Solve the linear factor graph, converting it into a linear Bayes Network
    // P(x0,x1) = P(x0|x1)*P(x1)
    const Ordering lastKeyAsOrdering{lastKey};
    const GaussianConditional::shared_ptr marginal =
      linearFactorGraph.marginalMultifrontalBayesNet(lastKeyAsOrdering)->front();

    // Extract the current estimate of x1,P1
    VectorValues result = marginal->solve(VectorValues());
    const T& current = linearizationPoint.at<T>(lastKey);
    T x = traits<T>::Retract(current, result[lastKey]);

    // Create a Jacobian Factor from the root node of the produced Bayes Net.
    // This will act as a prior for the next iteration.
    // The linearization point of this prior must be moved to the new estimate of x,
    // and the key/index needs to be reset to 0, the first key in the next iteration.
    assert(marginal->nrFrontals() == 1);
    assert(marginal->nrParents() == 0);
    *newPrior = std::make_shared<JacobianFactor>(
      marginal->keys().front(),
      marginal->getA(marginal->begin()),
      marginal->getb() - marginal->getA(marginal->begin()) * result[lastKey],
      marginal->get_model());

    return x;
  }

  /* ************************************************************************* */
  template <class VALUE>
  ExtendedKalmanFilter<VALUE>::ExtendedKalmanFilter(
      Key key_initial, T x_initial, noiseModel::Gaussian::shared_ptr P_initial)
      : x_(x_initial)  // Set the initial linearization point
  {
    // Create a Jacobian Prior Factor directly P_initial.
    // Since x0 is set to the provided mean, the b vector in the prior will be zero
    // TODO(Frank): is there a reason why noiseModel is not simply P_initial?
    int n = traits<T>::GetDimension(x_initial);
    priorFactor_ = JacobianFactor::shared_ptr(
        new JacobianFactor(key_initial, P_initial->R(), Vector::Zero(n),
            noiseModel::Unit::Create(n)));
  }

  /* ************************************************************************* */
  template<class VALUE>
  typename ExtendedKalmanFilter<VALUE>::T ExtendedKalmanFilter<VALUE>::predict(
      const NoiseModelFactor& motionFactor) {
    const auto keys = motionFactor.keys();

    // Create a Gaussian Factor Graph
    GaussianFactorGraph linearFactorGraph;

    // Add in previous posterior as prior on the first state
    linearFactorGraph.push_back(priorFactor_);

    // Linearize motion model and add it to the Kalman Filter graph
    Values linearizationPoint;
    linearizationPoint.insert(keys[0], x_);
    linearizationPoint.insert(keys[1], x_); // TODO should this really be x_ ?
    linearFactorGraph.push_back(motionFactor.linearize(linearizationPoint));

    // Solve the factor graph and update the current state estimate
    // and the posterior for the next iteration.
    x_ = solve_(linearFactorGraph, linearizationPoint, keys[1], &priorFactor_);

    return x_;
  }

  /* ************************************************************************* */
  template<class VALUE>
  typename ExtendedKalmanFilter<VALUE>::T ExtendedKalmanFilter<VALUE>::update(
      const NoiseModelFactor& measurementFactor) {
    const auto keys = measurementFactor.keys();

    // Create a Gaussian Factor Graph
    GaussianFactorGraph linearFactorGraph;

    // Add in the prior on the first state
    linearFactorGraph.push_back(priorFactor_);

    // Linearize measurement factor and add it to the Kalman Filter graph
    Values linearizationPoint;
    linearizationPoint.insert(keys[0], x_);
    linearFactorGraph.push_back(measurementFactor.linearize(linearizationPoint));

    // Solve the factor graph and update the current state estimate
    // and the prior factor for the next iteration
    x_ = solve_(linearFactorGraph, linearizationPoint, keys[0], &priorFactor_);

    return x_;
  }

} // namespace gtsam
