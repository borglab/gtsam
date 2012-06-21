/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    GenericMultifrontalSolver-inl.h
 * @author  Richard Roberts
 * @date    Oct 21, 2010
 */

#pragma once

#include <gtsam/inference/Factor-inl.h>
#include <gtsam/inference/JunctionTree.h>
#include <gtsam/inference/BayesNet-inl.h>

namespace gtsam {

	/* ************************************************************************* */
	template<class F, class JT>
	GenericMultifrontalSolver<F, JT>::GenericMultifrontalSolver(
			const FactorGraph<F>& graph) :
			structure_(new VariableIndex(graph)), junctionTree_(
					new JT(graph, *structure_)) {
	}

	/* ************************************************************************* */
	template<class F, class JT>
	GenericMultifrontalSolver<F, JT>::GenericMultifrontalSolver(
			const sharedGraph& graph,
			const VariableIndex::shared_ptr& variableIndex) :
			structure_(variableIndex), junctionTree_(new JT(*graph, *structure_)) {
	}

  /* ************************************************************************* */
	template<class F, class JT>
  void GenericMultifrontalSolver<F, JT>::print(const std::string& s) const {
    this->structure_->print(s + " structure:\n");
    this->junctionTree_->print(s + " jtree:");
  }

  /* ************************************************************************* */
	template<class F, class JT>
  bool GenericMultifrontalSolver<F, JT>::equals(
      const GenericMultifrontalSolver& expected, double tol) const {
    if (!this->structure_->equals(*expected.structure_, tol)) return false;
    if (!this->junctionTree_->equals(*expected.junctionTree_, tol)) return false;
    return true;
  }

	/* ************************************************************************* */
	template<class F, class JT>
	void GenericMultifrontalSolver<F, JT>::replaceFactors(const sharedGraph& graph) {
		junctionTree_.reset(new JT(*graph, *structure_));
	}

	/* ************************************************************************* */
	template<class FACTOR, class JUNCTIONTREE>
	typename BayesTree<typename FACTOR::ConditionalType>::shared_ptr
	GenericMultifrontalSolver<FACTOR, JUNCTIONTREE>::eliminate(Eliminate function) const {

		// eliminate junction tree, returns pointer to root
		typename BayesTree<typename FACTOR::ConditionalType>::sharedClique
			root = junctionTree_->eliminate(function);

		// create an empty Bayes tree and insert root clique
		typename BayesTree<typename FACTOR::ConditionalType>::shared_ptr
			bayesTree(new BayesTree<typename FACTOR::ConditionalType>);
		bayesTree->insert(root);

		// return the Bayes tree
		return bayesTree;
	}

	/* ************************************************************************* */
	template<class F, class JT>
	typename FactorGraph<F>::shared_ptr GenericMultifrontalSolver<F, JT>::jointFactorGraph(
			const std::vector<Index>& js, Eliminate function) const {

		// FIXME: joint for arbitrary sets of variables not present
		// TODO: develop and implement theory for shortcuts of more than two variables

		if (js.size() != 2) throw std::domain_error(
				"*MultifrontalSolver::joint(js) currently can only compute joint marginals\n"
						"for exactly two variables.  You can call marginal to compute the\n"
						"marginal for one variable.  *SequentialSolver::joint(js) can compute the\n"
						"joint marginal over any number of variables, so use that if necessary.\n");

		return eliminate(function)->joint(js[0], js[1], function);
	}

	/* ************************************************************************* */
	template<class F, class JT>
	typename boost::shared_ptr<F> GenericMultifrontalSolver<F, JT>::marginalFactor(
			Index j, Eliminate function) const {
		return eliminate(function)->marginalFactor(j, function);
	}

}

