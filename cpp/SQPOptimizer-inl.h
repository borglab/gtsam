/*
 * @file SQPOptimizer-inl.h
 * @brief Implementation of the SQP Optimizer
 * @author Alex Cunningham
 */

#pragma once

#include <boost/foreach.hpp>
#include "GaussianFactorGraph.h"
#include "SQPOptimizer.h"

using namespace std;
namespace gtsam {

/* **************************************************************** */
template <class G, class C>
SQPOptimizer<G,C>::SQPOptimizer(const G& graph, const Ordering& ordering,
		shared_config config)
: graph_(&graph), ordering_(&ordering), config_(config)
{
	// TODO: assign a value to the lagrange config

}

/* **************************************************************** */
template <class G, class C>
SQPOptimizer<G,C>::SQPOptimizer(const G& graph, const Ordering& ordering,
		shared_config config, shared_vconfig lagrange)
: graph_(&graph), ordering_(&ordering), config_(config), lagrange_config_(lagrange)
{

}

/* **************************************************************** */
template<class G, class C>
SQPOptimizer<G, C> SQPOptimizer<G, C>::iterate(Verbosity v) const {
	bool verbose = v == SQPOptimizer<G, C>::FULL;

	// local typedefs
	typedef typename G::const_iterator const_iterator;
	typedef NonlinearConstraint<C> NLConstraint;
	typedef boost::shared_ptr<NLConstraint > shared_c;

	// linearize the graph
	GaussianFactorGraph fg;

	// iterate over all factors and linearize
	for (const_iterator factor = graph_->begin(); factor < graph_->end(); factor++) {
		const shared_c constraint = boost::shared_dynamic_cast<NLConstraint >(*factor);
		if (constraint == NULL) {
			// if a regular factor, linearize using the default linearization
			GaussianFactor::shared_ptr f = (*factor)->linearize(*config_);
			if (verbose) f->print("Regular Factor");
			fg.push_back(f);
		} else {
			// if a constraint, linearize using the constraint method (2 configs)
			GaussianFactor::shared_ptr f, c;
			boost::tie(f,c) = constraint->linearize(*config_, *lagrange_config_);
			if (verbose) f->print("Constrained Factor");
			if (verbose) c->print("Constraint");
			fg.push_back(f);
			fg.push_back(c);
		}
	}
	if (verbose) fg.print("Before Optimization");

	// optimize linear graph to get full delta config
	VectorConfig delta = fg.optimize(*ordering_).scale(-1.0);

	if (verbose) delta.print("Delta Config");

	// update both state variables
	shared_config newConfig(new C(config_->exmap(delta)));
	shared_vconfig newLamConfig(new VectorConfig(lagrange_config_->exmap(delta)));

	// construct a new optimizer
	return SQPOptimizer<G, C>(*graph_, *ordering_, newConfig, newLamConfig);
}

}
