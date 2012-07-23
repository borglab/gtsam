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
	GenericSequentialSolver<FACTOR>::GenericSequentialSolver(
			const FactorGraph<FACTOR>& factorGraph) :
			factors_(new FactorGraph<FACTOR>(factorGraph)),
			structure_(new VariableIndex(factorGraph)),
			eliminationTree_(EliminationTree<FACTOR>::Create(*factors_, *structure_)) {
	}

	/* ************************************************************************* */
	template<class FACTOR>
	GenericSequentialSolver<FACTOR>::GenericSequentialSolver(
			const sharedFactorGraph& factorGraph,
			const boost::shared_ptr<VariableIndex>& variableIndex) :
			factors_(factorGraph), structure_(variableIndex),
			eliminationTree_(EliminationTree<FACTOR>::Create(*factors_, *structure_)) {
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
		if (!this->factors_->equals(*expected.factors_, tol)) return false;
		if (!this->structure_->equals(*expected.structure_, tol)) return false;
		if (!this->eliminationTree_->equals(*expected.eliminationTree_, tol)) return false;
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
	typename boost::shared_ptr<BayesNet<typename FACTOR::ConditionalType> > //
	GenericSequentialSolver<FACTOR>::eliminate(Eliminate function) const {
		return eliminationTree_->eliminate(function);
	}

	/* ************************************************************************* */
	template<class FACTOR>
	typename FactorGraph<FACTOR>::shared_ptr //
	GenericSequentialSolver<FACTOR>::jointFactorGraph(
			const std::vector<Index>& js, Eliminate function) const {

		// Compute a COLAMD permutation with the marginal variables constrained to the end.
		Permutation::shared_ptr permutation(inference::PermutationCOLAMD(*structure_, js));
		Permutation::shared_ptr permutationInverse(permutation->inverse());

		// Permute the factors - NOTE that this permutes the original factors, not
		// copies.  Other parts of the code may hold shared_ptr's to these factors so
		// we must undo the permutation before returning.
		BOOST_FOREACH(const typename boost::shared_ptr<FACTOR>& factor, *factors_)
			if (factor) factor->permuteWithInverse(*permutationInverse);

		// Eliminate all variables
		typename BayesNet<typename FACTOR::ConditionalType>::shared_ptr
			bayesNet(EliminationTree<FACTOR>::Create(*factors_)->eliminate(function));

		// Undo the permuation on the original factors and on the structure.
		BOOST_FOREACH(const typename boost::shared_ptr<FACTOR>& factor, *factors_)
			if (factor) factor->permuteWithInverse(*permutation);

		// Take the joint marginal from the Bayes net.
		sharedFactorGraph joint(new FactorGraph<FACTOR> );
		joint->reserve(js.size());
		typename BayesNet<typename FACTOR::ConditionalType>::const_reverse_iterator
			conditional = bayesNet->rbegin();

		for (size_t i = 0; i < js.size(); ++i)
			joint->push_back((*(conditional++))->toFactor());

		// Undo the permutation on the eliminated joint marginal factors
		BOOST_FOREACH(const typename boost::shared_ptr<FACTOR>& factor, *joint)
			factor->permuteWithInverse(*permutation);

		return joint;
	}

	/* ************************************************************************* */
	template<class FACTOR>
	typename boost::shared_ptr<FACTOR> //
	GenericSequentialSolver<FACTOR>::marginalFactor(Index j, Eliminate function) const {
		// Create a container for the one variable index
		std::vector<Index> js(1);
		js[0] = j;

		// Call joint and return the only factor in the factor graph it returns
		return (*this->jointFactorGraph(js, function))[0];
	}

} // namespace gtsam
