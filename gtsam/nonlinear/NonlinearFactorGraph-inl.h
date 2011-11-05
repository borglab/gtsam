/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    NonlinearFactorGraph-inl.h
 * @brief   Factor Graph Consisting of non-linear factors
 * @author  Frank Dellaert
 * @author  Carlos Nieto
 * @author  Christian Potthast
 */

#pragma once

#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/inference/FactorGraph-inl.h>
#include <gtsam/inference/inference.h>
#include <boost/foreach.hpp>
#include <cmath>

#define INSTANTIATE_NONLINEAR_FACTOR_GRAPH(C) \
  INSTANTIATE_FACTOR_GRAPH(NonlinearFactor<C>); \
  template class NonlinearFactorGraph<C>;

using namespace std;

namespace gtsam {

	/* ************************************************************************* */
	template<class VALUES>
	double NonlinearFactorGraph<VALUES>::probPrime(const VALUES& c) const {
		return exp(-0.5 * error(c));
	}

	/* ************************************************************************* */
	template<class VALUES>
	void NonlinearFactorGraph<VALUES>::print(const std::string& str) const {
		Base::print(str);
	}

	/* ************************************************************************* */
	template<class VALUES>
	double NonlinearFactorGraph<VALUES>::error(const VALUES& c) const {
		double total_error = 0.;
		// iterate over all the factors_ to accumulate the log probabilities
		BOOST_FOREACH(const sharedFactor& factor, this->factors_) {
		  if(factor)
		    total_error += factor->error(c);
		}
		return total_error;
	}

	/* ************************************************************************* */
	template<class VALUES>
	std::set<Symbol> NonlinearFactorGraph<VALUES>::keys() const {
		std::set<Symbol> keys;
		BOOST_FOREACH(const sharedFactor& factor, this->factors_) {
		  if(factor)
		    keys.insert(factor->begin(), factor->end());
		}
		return keys;
	}

	/* ************************************************************************* */
	template<class VALUES>
	Ordering::shared_ptr NonlinearFactorGraph<VALUES>::orderingCOLAMD(
			const VALUES& config) const {

		// Create symbolic graph and initial (iterator) ordering
		SymbolicFactorGraph::shared_ptr symbolic;
		Ordering::shared_ptr ordering;
		boost::tie(symbolic, ordering) = this->symbolic(config);

		// Compute the VariableIndex (column-wise index)
		VariableIndex variableIndex(*symbolic, ordering->size());
		if (config.size() != variableIndex.size()) throw std::runtime_error(
				"orderingCOLAMD: some variables in the graph are not constrained!");

		// Compute a fill-reducing ordering with COLAMD
		Permutation::shared_ptr colamdPerm(Inference::PermutationCOLAMD(
				variableIndex));

		// Permute the Ordering with the COLAMD ordering
		ordering->permuteWithInverse(*colamdPerm->inverse());

		// Return the Ordering and VariableIndex to be re-used during linearization
		// and elimination
		return ordering;
	}

	/* ************************************************************************* */
	template<class VALUES>
	SymbolicFactorGraph::shared_ptr NonlinearFactorGraph<VALUES>::symbolic(
			const VALUES& config, const Ordering& ordering) const {
		// Generate the symbolic factor graph
		SymbolicFactorGraph::shared_ptr symbolicfg(new SymbolicFactorGraph);
		symbolicfg->reserve(this->size());

		BOOST_FOREACH(const sharedFactor& factor, this->factors_) {
		  if(factor)
		    symbolicfg->push_back(factor->symbolic(ordering));
		  else
		    symbolicfg->push_back(SymbolicFactorGraph::sharedFactor());
		}

		return symbolicfg;
	}

	/* ************************************************************************* */
	template<class VALUES>
	pair<SymbolicFactorGraph::shared_ptr, Ordering::shared_ptr> NonlinearFactorGraph<
			VALUES>::symbolic(const VALUES& config) const {
		// Generate an initial key ordering in iterator order
		Ordering::shared_ptr ordering(config.orderingArbitrary());
		return make_pair(symbolic(config, *ordering), ordering);
	}

	/* ************************************************************************* */
	template<class VALUES>
	typename GaussianFactorGraph::shared_ptr NonlinearFactorGraph<VALUES>::linearize(
			const VALUES& config, const Ordering& ordering) const {

		// create an empty linear FG
		typename GaussianFactorGraph::shared_ptr linearFG(new GaussianFactorGraph);
		linearFG->reserve(this->size());

		// linearize all factors
		BOOST_FOREACH(const sharedFactor& factor, this->factors_) {
		  if(factor) {
		    GaussianFactor::shared_ptr lf = factor->linearize(config, ordering);
		    if (lf) linearFG->push_back(lf);
		  } else
		    linearFG->push_back(GaussianFactor::shared_ptr());
		}

		return linearFG;
	}

/* ************************************************************************* */

} // namespace gtsam
