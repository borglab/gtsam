/**
 * @file    NonlinearFactorGraph-inl.h
 * @brief   Factor Graph Consisting of non-linear factors
 * @author  Frank Dellaert
 * @author  Carlos Nieto
 * @author  Christian Potthast
 */

#pragma once

#include "LinearFactorGraph.h"
#include "NonlinearFactorGraph.h"

namespace gtsam {

/* ************************************************************************* */
template<class Config>
LinearFactorGraph NonlinearFactorGraph<Config>::linearize(const Config& config) const {
	// TODO speed up the function either by returning a pointer or by
	// returning the linearisation as a second argument and returning
	// the reference

	// create an empty linear FG
	LinearFactorGraph linearFG;

	typedef typename FactorGraph<NonlinearFactor<Config> ,Config>:: const_iterator const_iterator;
	// linearize all factors
	for (const_iterator factor = this->factors_.begin(); factor
			< this->factors_.end(); factor++) {
		boost::shared_ptr<LinearFactor> lf = (*factor)->linearize(config);
		linearFG.push_back(lf);
	}

	return linearFG;
}

/* ************************************************************************* */

}
