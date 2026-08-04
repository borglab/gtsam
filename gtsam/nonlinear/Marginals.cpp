/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file Marginals.cpp
 * @brief
 * @author Richard Roberts
 * @author Frank Dellaert
 * @date May 14, 2012
 */

#include <gtsam/base/timing.h>
#include <gtsam/linear/JacobianFactor.h>
#include <gtsam/linear/HessianFactor.h>
#include <gtsam/linear/GaussianBayesNet.h>
#include <gtsam/linear/GaussianBayesTreeQueries.h>
#include <gtsam/nonlinear/Marginals.h>

using namespace std;

namespace gtsam {

/* ************************************************************************* */
GaussianFactorGraph::Eliminate Marginals::eliminationFunction() const {
  if (factorization_ == CHOLESKY) {
    return EliminatePreferCholesky;
  } else if (factorization_ == QR) {
    return EliminateQR;
  }
  throw std::runtime_error(
      "Marginals::eliminationFunction: Unknown factorization");
}

/* ************************************************************************* */
Marginals::Marginals(const NonlinearFactorGraph& graph, const Values& solution, Factorization factorization)
                     : values_(solution), factorization_(factorization) {
  gttic(MarginalsConstructor);
  graph_ = *graph.linearize(solution);
  computeBayesTree();
}

/* ************************************************************************* */
Marginals::Marginals(const NonlinearFactorGraph& graph, const Values& solution, const Ordering& ordering,
                     Factorization factorization)
                     : values_(solution), factorization_(factorization) {
  gttic(MarginalsConstructor);
  graph_ = *graph.linearize(solution);
  computeBayesTree(ordering);
}

/* ************************************************************************* */
Marginals::Marginals(const GaussianFactorGraph& graph, const Values& solution, Factorization factorization)
                     : graph_(graph), values_(solution), factorization_(factorization) {
  gttic(MarginalsConstructor);
  computeBayesTree();
}

/* ************************************************************************* */
Marginals::Marginals(const GaussianFactorGraph& graph, const Values& solution, const Ordering& ordering,
                     Factorization factorization)
                     : graph_(graph), values_(solution), factorization_(factorization) {
  gttic(MarginalsConstructor);
  computeBayesTree(ordering);
}

/* ************************************************************************* */
Marginals::Marginals(const GaussianFactorGraph& graph, const VectorValues& solution, Factorization factorization)
                     : graph_(graph), factorization_(factorization) {
  gttic(MarginalsConstructor);
  for (const auto& keyValue: solution) {
    values_.insert(keyValue.first, keyValue.second);
  }
  computeBayesTree();
}

/* ************************************************************************* */
Marginals::Marginals(const GaussianFactorGraph& graph, const VectorValues& solution, const Ordering& ordering,
                     Factorization factorization)
                     : graph_(graph), factorization_(factorization) {
  gttic(MarginalsConstructor);
  for (const auto& keyValue: solution) {
    values_.insert(keyValue.first, keyValue.second);
  }
  computeBayesTree(ordering);
}

/* ************************************************************************* */
Marginals::Marginals(GaussianBayesTree&& bayesTree,
                     const VectorValues& solution,
                     Factorization factorization)
    : factorization_(factorization), bayesTree_(std::move(bayesTree)) {
  gttic(MarginalsConstructor);
  for (const auto& keyValue: solution) {
    values_.insert(keyValue.first, keyValue.second);
  }
  bayesTree_.addFactorsToGraph(&graph_);
}

/* ************************************************************************* */
void Marginals::computeBayesTree() {
  // The default ordering to use.
  const Ordering::OrderingType defaultOrderingType = Ordering::COLAMD;
  // Compute BayesTree
  if (factorization_ == CHOLESKY)
    bayesTree_ = *graph_.eliminateMultifrontal(defaultOrderingType,
                                               EliminatePreferCholesky);
  else if (factorization_ == QR)
    bayesTree_ =
        *graph_.eliminateMultifrontal(defaultOrderingType, EliminateQR);
}

/* ************************************************************************* */
void Marginals::computeBayesTree(const Ordering& ordering) {
  // Compute BayesTree
  if(factorization_ == CHOLESKY)
    bayesTree_ = *graph_.eliminateMultifrontal(ordering, EliminatePreferCholesky);
  else if(factorization_ == QR)
    bayesTree_ = *graph_.eliminateMultifrontal(ordering, EliminateQR);
}

/* ************************************************************************* */
void Marginals::print(const std::string& str, const KeyFormatter& keyFormatter) const
{
  graph_.print(str+"Graph: ");
  values_.print(str+"Solution: ", keyFormatter);
  bayesTree_.print(str+"Bayes Tree: ");
}

/* ************************************************************************* */
GaussianFactor::shared_ptr Marginals::marginalFactor(Key variable) const {
  gttic(marginalFactor);

  // Compute marginal factor
  if(factorization_ == CHOLESKY)
    return bayesTree_.marginalFactor(variable, EliminatePreferCholesky);
  else if(factorization_ == QR)
    return bayesTree_.marginalFactor(variable, EliminateQR);
  else
    throw std::runtime_error("Marginals::marginalFactor: Unknown factorization");
}

/* ************************************************************************* */
Matrix Marginals::marginalInformation(Key variable) const {
  gttic(marginalInformation);
  return bayesTree_.marginalInformation(variable, eliminationFunction());
}

/* ************************************************************************* */
Matrix Marginals::marginalCovariance(Key variable) const {
  return bayesTree_.marginalCovariance(variable, eliminationFunction());
}

/* ************************************************************************* */
JointMarginal Marginals::jointMarginalCovariance(
    const KeyVector& variables) const {
  return bayesTree_.jointMarginalCovariance(variables, eliminationFunction());
}

/* ************************************************************************* */
JointMarginal Marginals::jointMarginalInformation(
    const KeyVector& variables) const {
  return bayesTree_.jointMarginalInformation(variables, eliminationFunction());
}

/* ************************************************************************* */
VectorValues Marginals::optimize() const {
  return bayesTree_.optimize();
}

/* ************************************************************************* */
void Marginals::deleteCachedShortcuts() {
  bayesTree_.deleteCachedShortcuts();
}

} /* namespace gtsam */
