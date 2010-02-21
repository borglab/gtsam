/**
 * @file    NonlinearFactorGraph-inl.h
 * @brief   Factor Graph Consisting of non-linear factors
 * @author  Frank Dellaert
 * @author  Carlos Nieto
 * @author  Christian Potthast
 */

#pragma once

#include <boost/foreach.hpp>
#include "GaussianFactorGraph.h"
#include "NonlinearFactorGraph.h"
#include "FactorGraph-inl.h"

#define INSTANTIATE_NONLINEAR_FACTOR_GRAPH(C) \
  INSTANTIATE_FACTOR_GRAPH(NonlinearFactor<C>); \
  template class NonlinearFactorGraph<C>;

using namespace std;

namespace gtsam {

	/* ************************************************************************* */
	template<class Config>
	Vector NonlinearFactorGraph<Config>::unwhitenedError(const Config& c) const {
		list<Vector> errors;
		BOOST_FOREACH(typename NonlinearFactorGraph<Config>::sharedFactor factor, this->factors_)
			errors.push_back(factor->unwhitenedError(c));
		return concatVectors(errors);
	}

	/* ************************************************************************* */
	template<class Config>
	double NonlinearFactorGraph<Config>::error(const Config& c) const {
		double total_error = 0.;
		// iterate over all the factors_ to accumulate the log probabilities
		BOOST_FOREACH(typename NonlinearFactorGraph<Config>::sharedFactor factor, this->factors_)
			total_error += factor->error(c);
		return total_error;
	}

	/* ************************************************************************* */
	template<class Config>
	boost::shared_ptr<GaussianFactorGraph> NonlinearFactorGraph<Config>::linearize(
			const Config& config) const{

		// create an empty linear FG
		boost::shared_ptr<GaussianFactorGraph> linearFG(new GaussianFactorGraph);

		// linearize all factors
		typedef typename NonlinearFactorGraph<Config>::sharedFactor Factor;
		BOOST_FOREACH(const Factor& factor, this->factors_) {
			boost::shared_ptr<GaussianFactor> lf = factor->linearize(config);
			linearFG->push_back(lf);
		}

		return linearFG;
	}

	/* ************************************************************************* */

} // namespace gtsam
