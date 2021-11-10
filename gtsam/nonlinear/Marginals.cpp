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
 * @date May 14, 2012
 */

#include <gtsam/base/timing.h>
#include <gtsam/linear/JacobianFactor.h>
#include <gtsam/linear/HessianFactor.h>
#include <gtsam/nonlinear/Marginals.h>

using namespace std;

namespace gtsam {

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

  // Get information matrix (only store upper-right triangle)
  gttic(marginalInformation);
  return marginalFactor(variable)->information();
}

/* ************************************************************************* */
Matrix Marginals::marginalCovariance(Key variable) const {
  return marginalInformation(variable).inverse();
}

/* ************************************************************************* */
JointMarginal Marginals::jointMarginalCovariance(const KeyVector& variables) const {
  JointMarginal info = jointMarginalInformation(variables);
  info.blockMatrix_.invertInPlace();
  return info;
}

/* ************************************************************************* */
JointMarginal Marginals::jointMarginalInformation(const KeyVector& variables) const {

  // If 2 variables, we can use the BayesTree::joint function, otherwise we
  // have to use sequential elimination.
  if(variables.size() == 1)
  {
    Matrix info = marginalInformation(variables.front());
    std::vector<size_t> dims;
    dims.push_back(info.rows());
    return JointMarginal(info, dims, variables);
  }
  else
  {
    // Compute joint marginal factor graph.
    GaussianFactorGraph jointFG;
    if(variables.size() == 2) {
      if(factorization_ == CHOLESKY)
        jointFG = *bayesTree_.joint(variables[0], variables[1], EliminatePreferCholesky);
      else if(factorization_ == QR)
        jointFG = *bayesTree_.joint(variables[0], variables[1], EliminateQR);
    } else {
      if(factorization_ == CHOLESKY)
        jointFG = GaussianFactorGraph(*graph_.marginalMultifrontalBayesTree(variables, EliminatePreferCholesky));
      else if(factorization_ == QR)
        jointFG = GaussianFactorGraph(*graph_.marginalMultifrontalBayesTree(variables, EliminateQR));
    }

    // Get information matrix
    Matrix augmentedInfo = jointFG.augmentedHessian();
    Matrix info = augmentedInfo.topLeftCorner(augmentedInfo.rows()-1, augmentedInfo.cols()-1);

    // Information matrix will be returned with sorted keys
    KeyVector variablesSorted = variables;
    std::sort(variablesSorted.begin(), variablesSorted.end());

    // Get dimensions from factor graph
    std::vector<size_t> dims;
    dims.reserve(variablesSorted.size());
    for(const auto& key: variablesSorted) {
      dims.push_back(values_.at(key).dim());
    }

    return JointMarginal(info, dims, variablesSorted);
  }
}

/* ************************************************************************* */
VectorValues Marginals::optimize() const {
  return bayesTree_.optimize();
}

/* ************************************************************************* */
void JointMarginal::print(const std::string& s, const KeyFormatter& formatter) const {
  cout << s << "Joint marginal on keys ";
  bool first = true;
  for(const auto& key: keys_) {
    if(!first)
      cout << ", ";
    else
      first = false;
    cout << formatter(key);
  }
  cout << ".  Use 'at' or 'operator()' to query matrix blocks." << endl;
}

} /* namespace gtsam */
