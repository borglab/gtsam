/*
 * @file SQPOptimizer-inl.h
 * @brief Implementation of the SQP Optimizer
 * @author Alex Cunningham
 */

#pragma once

#include <boost/foreach.hpp>
#include <boost/assign/std/list.hpp> // for operator +=
#include <boost/assign/std/map.hpp> // for insert
#include "GaussianFactorGraph.h"
#include "NonlinearFactorGraph.h"
#include "SQPOptimizer.h"

// implementations
#include "NonlinearConstraint-inl.h"
#include "NonlinearFactorGraph-inl.h"

using namespace std;
using namespace boost::assign;

namespace gtsam {

/* **************************************************************** */
template <class G, class C>
double constraintError(const G& graph, const C& config) {
	// local typedefs
	typedef typename G::const_iterator const_iterator;
	typedef NonlinearConstraint<C> NLConstraint;
	typedef boost::shared_ptr<NLConstraint > shared_c;

	// accumulate error
	double error = 0;

	// find the constraints
	for (const_iterator factor = graph.begin(); factor < graph.end(); factor++) {
		const shared_c constraint = boost::shared_dynamic_cast<NLConstraint >(*factor);
		if (constraint != NULL) {
			Vector e = constraint->error_vector(config);
			error += inner_prod(trans(e),e);
		}
	}
	return error;
}

/* **************************************************************** */
template <class G, class C>
SQPOptimizer<G,C>::SQPOptimizer(const G& graph, const Ordering& ordering,
		shared_config config)
: graph_(&graph), ordering_(&ordering), full_ordering_(ordering),
  config_(config), lagrange_config_(new VectorConfig), error_(graph.error(*config)),
  constraint_error_(constraintError(graph, *config))
{
	// local typedefs
	typedef typename G::const_iterator const_iterator;
	typedef NonlinearConstraint<C> NLConstraint;
	typedef boost::shared_ptr<NLConstraint > shared_c;

	// find the constraints
	for (const_iterator factor = graph_->begin(); factor < graph_->end(); factor++) {
		const shared_c constraint = boost::shared_dynamic_cast<NLConstraint >(*factor);
		if (constraint != NULL) {
			size_t p = constraint->nrConstraints();
			// update ordering
			string key = constraint->lagrangeKey();
			full_ordering_ += key;
			// initialize lagrange multipliers
			lagrange_config_->insert(key, ones(p));
		}
	}
}

/* **************************************************************** */
template <class G, class C>
SQPOptimizer<G,C>::SQPOptimizer(const G& graph, const Ordering& ordering,
		shared_config config, shared_vconfig lagrange)
: graph_(&graph), ordering_(&ordering), full_ordering_(ordering),
  config_(config), lagrange_config_(lagrange), error_(graph.error(*config)),
  constraint_error_(constraintError(graph, *config))
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

	// prepare an ordering of lagrange multipliers to remove
	Ordering keysToRemove;

	// iterate over all factors and linearize
	for (const_iterator factor = graph_->begin(); factor < graph_->end(); factor++) {
		const shared_c constraint = boost::shared_dynamic_cast<NLConstraint >(*factor);
		if (constraint == NULL) {
			// if a regular factor, linearize using the default linearization
			GaussianFactor::shared_ptr f = (*factor)->linearize(*config_);
			if (verbose) f->print("Regular Factor");
			fg.push_back(f);
		} else if (constraint->active(*config_)) {
			// if a constraint, linearize using the constraint method (2 configs)
			GaussianFactor::shared_ptr f, c;
			boost::tie(f,c) = constraint->linearize(*config_, *lagrange_config_);
			if (verbose) f->print("Constrained Factor");
			if (verbose) c->print("Constraint");
			fg.push_back(f);
			fg.push_back(c);
		} else {
			if (verbose) constraint->print("Skipping...");
			keysToRemove += constraint->lagrangeKey();
		}
	}
	if (verbose) fg.print("Before Optimization");

	// optimize linear graph to get full delta config
	VectorConfig delta = fg.optimize(full_ordering_.subtract(keysToRemove));

	if (verbose) delta.print("Delta Config");

	// update both state variables
	shared_config newConfig(new C(config_->exmap(delta)));
	shared_vconfig newLambdas(new VectorConfig(lagrange_config_->exmap(delta)));

	// construct a new optimizer
	return SQPOptimizer<G, C>(*graph_, full_ordering_, newConfig, newLambdas);
}

/* **************************************************************** */
template<class G, class C>
SQPOptimizer<G, C> SQPOptimizer<G, C>::iterateSolve(double relThresh, double absThresh,
		double constraintThresh, size_t maxIterations, Verbosity v) const {
	bool verbose = v == SQPOptimizer<G, C>::FULL;

	// do an iteration
	SQPOptimizer<G, C> next = iterate(v);

	// if converged or out of iterations, return result
	if (maxIterations == 1 ||
			next.checkConvergence(relThresh, absThresh, constraintThresh,
								  error_, constraint_error_))
		return next;
	else // otherwise, recurse with a lower maxIterations
		return next.iterateSolve(relThresh, absThresh, constraintThresh,
				maxIterations-1, v);
}

/* **************************************************************** */
template<class G, class C>
bool SQPOptimizer<G, C>::checkConvergence(double relThresh, double absThresh,
		double constraintThresh, double full_error, double constraint_error) const {
	// if error sufficiently low, then the system has converged
	if (error_ < absThresh && constraint_error_ < constraintThresh)
		return true;

	// TODO: determine other cases
	return false;
}

/* **************************************************************** */
template<class G, class C>
void SQPOptimizer<G, C>::print(const std::string& s) {
	graph_->print("Nonlinear Graph");
	ordering_->print("Initial Ordering");
	full_ordering_.print("Ordering including all Lagrange Multipliers");
	config_->print("Real Config");
	lagrange_config_->print("Lagrange Multiplier Config");
}

}
