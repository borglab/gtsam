/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 *  @file DiscreteFactorGraph.cpp
 *  @date Feb 14, 2011
 *  @author Duy-Nguyen Ta
 *  @author Frank Dellaert
 */

//#define ENABLE_TIMING
#include <gtsam/discrete/DiscreteFactorGraph.h>
#include <gtsam/discrete/DiscreteConditional.h>
#include <gtsam/inference/EliminationTree-inl.h>
#include <boost/make_shared.hpp>

namespace gtsam {

	// Explicitly instantiate so we don't have to include everywhere
	template class FactorGraph<DiscreteFactor> ;
	template class EliminationTree<DiscreteFactor> ;

	/* ************************************************************************* */
	DiscreteFactorGraph::DiscreteFactorGraph() {
	}

	/* ************************************************************************* */
	DiscreteFactorGraph::DiscreteFactorGraph(
			const BayesNet<DiscreteConditional>& bayesNet) :
			FactorGraph<DiscreteFactor>(bayesNet) {
	}

	/* ************************************************************************* */
	FastSet<Index> DiscreteFactorGraph::keys() const {
		FastSet<Index> keys;
		BOOST_FOREACH(const sharedFactor& factor, *this)
		if (factor) keys.insert(factor->begin(), factor->end());
		return keys;
	}

	/* ************************************************************************* */
	DecisionTreeFactor DiscreteFactorGraph::product() const {
		DecisionTreeFactor result;
		BOOST_FOREACH(const sharedFactor& factor, *this)
			if (factor) result = (*factor) * result;
		return result;
	}

	/* ************************************************************************* */
	double DiscreteFactorGraph::operator()(
			const DiscreteFactor::Values &values) const {
		double product = 1.0;
		BOOST_FOREACH( const sharedFactor& factor, factors_ )
			product *= (*factor)(values);
		return product;
	}

	/* ************************************************************************* */
	void DiscreteFactorGraph::print(const std::string& s,
			const IndexFormatter& formatter) const {
		std::cout << s << std::endl;
		std::cout << "size: " << size() << std::endl;
		for (size_t i = 0; i < factors_.size(); i++) {
			std::stringstream ss;
			ss << "factor " << i << ": ";
			if (factors_[i] != NULL) factors_[i]->print(ss.str(), formatter);
		}
	}

	/* ************************************************************************* */
	std::pair<DiscreteConditional::shared_ptr, DecisionTreeFactor::shared_ptr>  //
	EliminateDiscrete(const FactorGraph<DiscreteFactor>& factors, size_t num) {

		// PRODUCT: multiply all factors
		tic(1, "product");
		DecisionTreeFactor product;
		BOOST_FOREACH(const DiscreteFactor::shared_ptr& factor, factors){
			product = (*factor) * product;
		}

		toc(1, "product");

		// sum out frontals, this is the factor on the separator
		tic(2, "sum");
		DecisionTreeFactor::shared_ptr sum = product.sum(num);
		toc(2, "sum");

		// now divide product/sum to get conditional
		tic(3, "divide");
		DiscreteConditional::shared_ptr cond(new DiscreteConditional(product, *sum));
		toc(3, "divide");
		tictoc_finishedIteration();

		return std::make_pair(cond, sum);
	}


/* ************************************************************************* */
} // namespace

