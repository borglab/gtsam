/*
 * ConstrainedNonlinearFactorGraph.h
 *
 *  Created on: Aug 10, 2009
 *      Author: alexgc
 */

#ifndef CONSTRAINEDNONLINEARFACTORGRAPH_H_
#define CONSTRAINEDNONLINEARFACTORGRAPH_H_

#include "NonlinearFactorGraph.h"
#include "EqualityFactor.h"
#include "ConstrainedLinearFactorGraph.h"

namespace gtsam {

class ConstrainedNonlinearFactorGraph: public NonlinearFactorGraph {
protected:
	/** collection of equality factors */
	std::vector<EqualityFactor::shared_ptr> eq_factors;

public:
	// iterators over equality factors
	typedef std::vector<EqualityFactor::shared_ptr>::const_iterator eq_const_iterator;
	typedef std::vector<EqualityFactor::shared_ptr>::iterator eq_iterator;

	/**
	 * Default constructor
	 */
	ConstrainedNonlinearFactorGraph();

	/**
	 * Copy constructor from regular NLFGs
	 */
	ConstrainedNonlinearFactorGraph(const NonlinearFactorGraph& nfg);

	virtual ~ConstrainedNonlinearFactorGraph();

	/**
	 * Linearize a nonlinear graph to produce a linear graph
	 * Note that equality constraints will just pass through
	 */
	ConstrainedLinearFactorGraph linearize(const FGConfig& initial) const;

	/**
	 * Insert a equality factor into the graph
	 */
	void push_back_eq(const EqualityFactor::shared_ptr& eq) {
		eq_factors.push_back(eq);
	}

	/**
	 * converts the graph to a regular nonlinear graph - removes equality constraints
	 */
	NonlinearFactorGraph convert() const;
};

}

#endif /* CONSTRAINEDNONLINEARFACTORGRAPH_H_ */
