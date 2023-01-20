/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/*
 * @file KarcherMeanFactor.cpp
 * @author Frank Dellaert
 * @date March 2019
 */

#pragma once

#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/slam/KarcherMeanFactor.h>

using namespace std;

namespace gtsam {

template <class T, class ALLOC>
T FindKarcherMeanImpl(const vector<T, ALLOC>& rotations) {
  // Cost function C(R) = \sum PriorFactor(R_i)::error(R)
  // No closed form solution.
  NonlinearFactorGraph graph;
  static const Key kKey(0);
  for (const auto& R : rotations) {
    graph.addPrior<T>(kKey, R);
  }
  Values initial;
  initial.insert<T>(kKey, T());
  auto result = GaussNewtonOptimizer(graph, initial).optimize();
  return result.at<T>(kKey);
}

template <class T>
T FindKarcherMean(const std::vector<T>& rotations) {
  return FindKarcherMeanImpl(rotations);
}

template <class T>
T FindKarcherMean(const std::vector<T, Eigen::aligned_allocator<T>>& rotations) {
  return FindKarcherMeanImpl(rotations);
}

template <class T>
T FindKarcherMean(std::initializer_list<T>&& rotations) {
  return FindKarcherMeanImpl(std::vector<T, Eigen::aligned_allocator<T> >(rotations));
}

template <class T>
template <typename CONTAINER>
KarcherMeanFactor<T>::KarcherMeanFactor(const CONTAINER &keys, int d,
                                        boost::optional<double> beta)
    : NonlinearFactor(keys), d_(static_cast<size_t>(d)) {
  if (d <= 0) {
    throw std::invalid_argument(
        "KarcherMeanFactor needs dimension for dynamic types.");
  }
  // Create the constant Jacobian made of d*d identity matrices,
  // where d is the dimensionality of the manifold.
  Matrix A = Matrix::Identity(d, d);
  if (beta) A *= std::sqrt(*beta);
  std::map<Key, Matrix> terms;
  for (Key j : keys) {
    terms[j] = A;
  }
  whitenedJacobian_ =
      boost::make_shared<JacobianFactor>(terms, Vector::Zero(d));
}
}  // namespace gtsam