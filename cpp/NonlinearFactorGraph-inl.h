/**
 * @file    NonlinearFactorGraph-inl.h
 * @brief   Factor Graph Consisting of non-linear factors
 * @author  Frank Dellaert
 * @author  Carlos Nieto
 * @author  Christian Potthast
 */

#pragma once

#include "GaussianFactorGraph.h"
#include "NonlinearFactorGraph.h"

namespace gtsam {

	/* ************************************************************************* */
	template<class Config>
	double NonlinearFactorGraph<Config>::error(const Config& c) const {
		double total_error = 0.;
		// iterate over all the factors_ to accumulate the log probabilities
		typedef typename FactorGraph<NonlinearFactor<Config> >::const_iterator
				const_iterator;
		for (const_iterator factor = this->factors_.begin(); factor
				!= this->factors_.end(); factor++)
			total_error += (*factor)->error(c);

		return total_error;
	}
	/* ************************************************************************* */
	template<class Config>
	boost::shared_ptr<GaussianFactorGraph> NonlinearFactorGraph<Config>::linearize_(
			const Config& config) const{

		// create an empty linear FG
		boost::shared_ptr<GaussianFactorGraph>  linearFG(new GaussianFactorGraph);

		typedef typename FactorGraph<NonlinearFactor<Config> >::const_iterator
		const_iterator;
		// linearize all factors
		for (const_iterator factor = this->factors_.begin(); factor
		< this->factors_.end(); factor++) {
			boost::shared_ptr<GaussianFactor> lf = (*factor)->linearize(config);
			linearFG->push_back(lf);
		}
		return linearFG;
	}
	/* ************************************************************************* */
	template<class Config>
	GaussianFactorGraph NonlinearFactorGraph<Config>::linearize(
			const Config& config) const {
		return *linearize_(config);
	}
}
