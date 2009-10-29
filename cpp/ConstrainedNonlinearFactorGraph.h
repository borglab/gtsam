/*
 * ConstrainedNonlinearFactorGraph.h
 *
 *  Created on: Aug 10, 2009
 *      Author: alexgc
 */

#ifndef CONSTRAINEDNONLINEARFACTORGRAPH_H_
#define CONSTRAINEDNONLINEARFACTORGRAPH_H_

#include <boost/shared_ptr.hpp>
#include "NonlinearFactorGraph.h"
#include "LinearConstraint.h"
#include "ConstrainedLinearFactorGraph.h"

namespace gtsam {

/**
 * A nonlinear factor graph with the addition of equality constraints.
 * 
 * Templated on factor and configuration type.
 * TODO FD: this is totally wrong: it can only work if Config==VectorConfig
 * as LinearConstraint is only defined for VectorConfig
 * To fix it, we need to think more deeply about this.
 */
template<class Factor, class Config>
class ConstrainedNonlinearFactorGraph: public FactorGraph<Factor> {
protected:
	/** collection of equality factors */
	std::vector<LinearConstraint::shared_ptr> eq_factors;

public:
	// iterators over equality factors
	typedef std::vector<LinearConstraint::shared_ptr>::const_iterator	eq_const_iterator;
	typedef std::vector<LinearConstraint::shared_ptr>::iterator eq_iterator;

	/**
	 * Default constructor
	 */
	ConstrainedNonlinearFactorGraph() {
	}

	/**
	 * Copy constructor from regular NLFGs
	 */
	ConstrainedNonlinearFactorGraph(const NonlinearFactorGraph<Config>& nfg) :
		FactorGraph<Factor> (nfg) {
	}

	typedef typename boost::shared_ptr<Factor> shared_factor;
	typedef typename std::vector<shared_factor>::const_iterator const_iterator;

	/**
	 * Linearize a nonlinear graph to produce a linear graph
	 * Note that equality constraints will just pass through
	 */
	ConstrainedLinearFactorGraph linearize(const Config& config) const {
		ConstrainedLinearFactorGraph ret;

		// linearize all nonlinear factors
		// TODO uncomment
		for (const_iterator factor = this->factors_.begin(); factor < this->factors_.end(); factor++) {
			LinearFactor::shared_ptr lf = (*factor)->linearize(config);
			ret.push_back(lf);
		}

		// linearize the equality factors (set to zero because they are now in delta space)
		for (eq_const_iterator e_factor = eq_factors.begin(); e_factor
				< eq_factors.end(); e_factor++) {
//			LinearConstraint::shared_ptr eq = (*e_factor)->linearize();
//			ret.push_back_eq(eq);
		}

		return ret;
	}

	/**
	 * Insert a factor into the graph
	 */
	void push_back(const shared_factor& f) {
		FactorGraph<Factor>::push_back(f);
	}

	/**
	 * Insert a equality factor into the graph
	 */
	void push_back_eq(const LinearConstraint::shared_ptr& eq) {
		eq_factors.push_back(eq);
	}

};

}

#endif /* CONSTRAINEDNONLINEARFACTORGRAPH_H_ */
