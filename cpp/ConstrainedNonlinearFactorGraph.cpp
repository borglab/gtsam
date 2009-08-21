/*
 * ConstrainedNonlinearFactorGraph.cpp
 *
 *  Created on: Aug 10, 2009
 *      Author: alexgc
 */

#include "ConstrainedNonlinearFactorGraph.h"

namespace gtsam {

ConstrainedNonlinearFactorGraph::ConstrainedNonlinearFactorGraph()
{
}

ConstrainedNonlinearFactorGraph::ConstrainedNonlinearFactorGraph(
		const NonlinearFactorGraph& nfg)
: NonlinearFactorGraph(nfg)
{
}

ConstrainedNonlinearFactorGraph::~ConstrainedNonlinearFactorGraph()
{
}

ConstrainedLinearFactorGraph ConstrainedNonlinearFactorGraph::linearize(const FGConfig& config) const
{
	ConstrainedLinearFactorGraph ret;

	// linearize all nonlinear factors
	for(const_iterator factor=factors.begin(); factor<factors.end(); factor++){
		LinearFactor::shared_ptr lf = (*factor)->linearize(config);
		ret.push_back(lf);
	}

	// linearize the equality factors (set to zero because they are now in delta space)
	for(eq_const_iterator e_factor=eq_factors.begin(); e_factor<eq_factors.end(); e_factor++){
		EqualityFactor::shared_ptr eq = (*e_factor)->linearize();
		ret.push_back_eq(eq);
	}

	return ret;
}

NonlinearFactorGraph ConstrainedNonlinearFactorGraph::convert() const
{
	NonlinearFactorGraph ret;
	BOOST_FOREACH(boost::shared_ptr<NonlinearFactor> f, factors)
	{
		ret.push_back(f);
	}
	return ret;
}

}
