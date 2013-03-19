/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    GenericSequentialSolver-inl.h
 * @brief   Implementation for generic sequential solver
 * @author  Richard Roberts
 * @date    Oct 21, 2010
 */

#pragma once

#include <gtsam/inference/Factor.h>
#include <gtsam/inference/FactorGraph.h>
#include <gtsam/inference/EliminationTree.h>
#include <gtsam/inference/BayesNet.h>
#include <gtsam/inference/inference.h>

#include <boost/foreach.hpp>

namespace gtsam {

  /* ************************************************************************* */
  template<class FACTOR>
  GenericSequentialSolver<FACTOR>::GenericSequentialSolver(const FactorGraph<FACTOR>& factorGraph) {
    gttic(GenericSequentialSolver_constructor1);
    assert(factorGraph.size());
    factors_.reset(new FactorGraph<FACTOR>(factorGraph));
    structure_.reset(new VariableIndex(factorGraph));
    eliminationTree_ = EliminationTree<FACTOR>::Create(*factors_, *structure_);
  }

  /* ************************************************************************* */
  template<class FACTOR>
  GenericSequentialSolver<FACTOR>::GenericSequentialSolver(
      const sharedFactorGraph& factorGraph,
      const boost::shared_ptr<VariableIndex>& variableIndex)
  {
    gttic(GenericSequentialSolver_constructor2);
    factors_ = factorGraph;
    structure_ = variableIndex;
    eliminationTree_ = EliminationTree<FACTOR>::Create(*factors_, *structure_);
  }

  /* ************************************************************************* */
  template<class FACTOR>
  void GenericSequentialSolver<FACTOR>::print(const std::string& s) const {
    this->factors_->print(s + " factors:");
    this->structure_->print(s + " structure:\n");
    this->eliminationTree_->print(s + " etree:");
  }

  /* ************************************************************************* */
  template<class FACTOR>
  bool GenericSequentialSolver<FACTOR>::equals(
      const GenericSequentialSolver& expected, double tol) const {
    if (!this->factors_->equals(*expected.factors_, tol))
      return false;
    if (!this->structure_->equals(*expected.structure_, tol))
      return false;
    if (!this->eliminationTree_->equals(*expected.eliminationTree_, tol))
      return false;
    return true;
  }

  /* ************************************************************************* */
  template<class FACTOR>
  void GenericSequentialSolver<FACTOR>::replaceFactors(
      const sharedFactorGraph& factorGraph) {
    // Reset this shared pointer first to deallocate if possible - for big
    // problems there may not be enough memory to store two copies.
    eliminationTree_.reset();
    factors_ = factorGraph;
    eliminationTree_ = EliminationTree<FACTOR>::Create(*factors_, *structure_);
  }

  /* ************************************************************************* */
  template<class FACTOR>
  typename GenericSequentialSolver<FACTOR>::sharedBayesNet //
  GenericSequentialSolver<FACTOR>::eliminate(Eliminate function) const {
    return eliminationTree_->eliminate(function);
  }

  /* ************************************************************************* */
  template<class FACTOR>
  typename GenericSequentialSolver<FACTOR>::sharedBayesNet //
  GenericSequentialSolver<FACTOR>::eliminate(const Permutation& permutation,
      Eliminate function, boost::optional<size_t> nrToEliminate) const
  {
    gttic(GenericSequentialSolver_eliminate);
    // Create inverse permutation
    Permutation::shared_ptr permutationInverse(permutation.inverse());

    // Permute the factors - NOTE that this permutes the original factors, not
    // copies.  Other parts of the code may hold shared_ptr's to these factors so
    // we must undo the permutation before returning.
    BOOST_FOREACH(const typename boost::shared_ptr<FACTOR>& factor, *factors_)
      if (factor)
        factor->permuteWithInverse(*permutationInverse);

    // Eliminate using elimination tree provided
    typename EliminationTree<FACTOR>::shared_ptr etree = EliminationTree<FACTOR>::Create(*factors_);
    sharedBayesNet bayesNet;
    if(nrToEliminate)
      bayesNet = etree->eliminatePartial(function, *nrToEliminate);
    else
      bayesNet = etree->eliminate(function);

    // Undo the permutation on the original factors and on the structure.
    BOOST_FOREACH(const typename boost::shared_ptr<FACTOR>& factor, *factors_)
      if (factor)
        factor->permuteWithInverse(permutation);

    // Undo the permutation on the conditionals
    BOOST_FOREACH(const boost::shared_ptr<Conditional>& c, *bayesNet)
      c->permuteWithInverse(permutation);

    return bayesNet;
  }

  /* ************************************************************************* */
  template<class FACTOR>
  typename GenericSequentialSolver<FACTOR>::sharedBayesNet //
  GenericSequentialSolver<FACTOR>::conditionalBayesNet(
      const std::vector<Index>& js, size_t nrFrontals,
      Eliminate function) const
  {
    gttic(GenericSequentialSolver_conditionalBayesNet);
    // Compute a COLAMD permutation with the marginal variables constrained to the end.
    // TODO in case of nrFrontals, the order of js has to be respected here !
    Permutation::shared_ptr permutation(
        inference::PermutationCOLAMD(*structure_, js, true));

    // Eliminate only variables J \cup F from P(J,F,S) to get P(F|S)
    size_t nrVariables = structure_->size();
    size_t nrMarginalized = nrVariables - js.size();
    size_t nrToEliminate = nrMarginalized + nrFrontals;
    sharedBayesNet bayesNet = eliminate(*permutation, function, nrToEliminate);
    // Get rid of conditionals on variables that we want to marginalize out
    for (size_t i = 0; i < nrMarginalized; i++)
      bayesNet->pop_front();

    return bayesNet;
  }

  /* ************************************************************************* */
  template<class FACTOR>
  typename GenericSequentialSolver<FACTOR>::sharedBayesNet //
  GenericSequentialSolver<FACTOR>::jointBayesNet(const std::vector<Index>& js,
      Eliminate function) const
  {
    gttic(GenericSequentialSolver_jointBayesNet);
    // Compute a COLAMD permutation with the marginal variables constrained to the end.
    Permutation::shared_ptr permutation(
        inference::PermutationCOLAMD(*structure_, js));

    // Eliminate all variables
    sharedBayesNet bayesNet = eliminate(*permutation, function);

    // Get rid of conditionals on variables that we want to marginalize out
    size_t nrMarginalized = bayesNet->size() - js.size();
    for (size_t i = 0; i < nrMarginalized; i++)
      bayesNet->pop_front();

    return bayesNet;
  }

  /* ************************************************************************* */
  template<class FACTOR>
  typename FactorGraph<FACTOR>::shared_ptr //
  GenericSequentialSolver<FACTOR>::jointFactorGraph(
      const std::vector<Index>& js, Eliminate function) const
  {
    gttic(GenericSequentialSolver_jointFactorGraph);
    // Eliminate all variables
    typename BayesNet<Conditional>::shared_ptr bayesNet = jointBayesNet(js, function);

    return boost::make_shared<FactorGraph<FACTOR> >(*bayesNet);
  }

  /* ************************************************************************* */
  template<class FACTOR>
  typename boost::shared_ptr<FACTOR> //
  GenericSequentialSolver<FACTOR>::marginalFactor(Index j, Eliminate function) const {
    gttic(GenericSequentialSolver_marginalFactor);
    // Create a container for the one variable index
    std::vector<Index> js(1);
    js[0] = j;

    // Call joint and return the only factor in the factor graph it returns
    // TODO: just call jointBayesNet and grab last conditional, then toFactor....
    return (*this->jointFactorGraph(js, function))[0];
  }

} // namespace gtsam
