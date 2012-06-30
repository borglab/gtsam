/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    NonlinearFactorGraph.cpp
 * @brief   Factor Graph Consisting of non-linear factors
 * @author  Frank Dellaert
 * @author  Carlos Nieto
 * @author  Christian Potthast
 */

#include <cmath>
#include <boost/foreach.hpp>
#include <gtsam/inference/FactorGraph.h>
#include <gtsam/inference/inference.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>

using namespace std;

namespace gtsam {

/* ************************************************************************* */
double NonlinearFactorGraph::probPrime(const Values& c) const {
	return exp(-0.5 * error(c));
}

/* ************************************************************************* */
void NonlinearFactorGraph::print(const std::string& str, const KeyFormatter& keyFormatter) const {
	cout << str << "size: " << size() << endl;
	for (size_t i = 0; i < factors_.size(); i++) {
		stringstream ss;
		ss << "factor " << i << ": ";
		if (factors_[i] != NULL) factors_[i]->print(ss.str(), keyFormatter);
	}
}

/* ************************************************************************* */
double NonlinearFactorGraph::error(const Values& c) const {
	double total_error = 0.;
	// iterate over all the factors_ to accumulate the log probabilities
	BOOST_FOREACH(const sharedFactor& factor, this->factors_) {
		if(factor)
			total_error += factor->error(c);
	}
	return total_error;
}

/* ************************************************************************* */
FastSet<Key> NonlinearFactorGraph::keys() const {
	FastSet<Key> keys;
	BOOST_FOREACH(const sharedFactor& factor, this->factors_) {
		if(factor)
			keys.insert(factor->begin(), factor->end());
	}
	return keys;
}

/* ************************************************************************* */
Ordering::shared_ptr NonlinearFactorGraph::orderingCOLAMD(
		const Values& config) const {

	// Create symbolic graph and initial (iterator) ordering
	SymbolicFactorGraph::shared_ptr symbolic;
	Ordering::shared_ptr ordering;
	boost::tie(symbolic, ordering) = this->symbolic(config);

	// Compute the VariableIndex (column-wise index)
	VariableIndex variableIndex(*symbolic, ordering->size());
	if (config.size() != variableIndex.size()) throw std::runtime_error(
			"orderingCOLAMD: some variables in the graph are not constrained!");

	// Compute a fill-reducing ordering with COLAMD
	Permutation::shared_ptr colamdPerm(inference::PermutationCOLAMD(
			variableIndex));

	// Permute the Ordering with the COLAMD ordering
	ordering->permuteWithInverse(*colamdPerm->inverse());
	return ordering;
}

/* ************************************************************************* */
Ordering::shared_ptr NonlinearFactorGraph::orderingCOLAMDConstrained(const Values& config,
		const std::map<Key, int>& constraints) const {
	// Create symbolic graph and initial (iterator) ordering
	SymbolicFactorGraph::shared_ptr symbolic;
	Ordering::shared_ptr ordering;
	boost::tie(symbolic, ordering) = this->symbolic(config);

	// Compute the VariableIndex (column-wise index)
	VariableIndex variableIndex(*symbolic, ordering->size());
	if (config.size() != variableIndex.size()) throw std::runtime_error(
			"orderingCOLAMD: some variables in the graph are not constrained!");

	// Create a constraint list with correct indices
	typedef std::map<Key, int>::value_type constraint_type;
	std::map<Index, int> index_constraints;
	BOOST_FOREACH(const constraint_type& p, constraints)
		index_constraints[ordering->at(p.first)] = p.second;

	// Compute a constrained fill-reducing ordering with COLAMD
	Permutation::shared_ptr colamdPerm(inference::PermutationCOLAMDGrouped(
			variableIndex, index_constraints));

	// Permute the Ordering with the COLAMD ordering
	ordering->permuteWithInverse(*colamdPerm->inverse());
	return ordering;
}

/* ************************************************************************* */
SymbolicFactorGraph::shared_ptr NonlinearFactorGraph::symbolic(const Ordering& ordering) const {
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
pair<SymbolicFactorGraph::shared_ptr, Ordering::shared_ptr> NonlinearFactorGraph::symbolic(
		const Values& config) const {
	// Generate an initial key ordering in iterator order
	Ordering::shared_ptr ordering(config.orderingArbitrary());
	return make_pair(symbolic(*ordering), ordering);
}

/* ************************************************************************* */
GaussianFactorGraph::shared_ptr NonlinearFactorGraph::linearize(
		const Values& config, const Ordering& ordering) const {

	// create an empty linear FG
	GaussianFactorGraph::shared_ptr linearFG(new GaussianFactorGraph);
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
NonlinearFactorGraph NonlinearFactorGraph::clone() const {
	NonlinearFactorGraph result;
	BOOST_FOREACH(const sharedFactor& f, *this) {
		if (f)
			result.push_back(f->clone());
		else
			result.push_back(sharedFactor()); // Passes on null factors so indices remain valid
	}
	return result;
}

/* ************************************************************************* */
NonlinearFactorGraph NonlinearFactorGraph::rekey(const std::map<Key,Key>& rekey_mapping) const {
	NonlinearFactorGraph result;
	BOOST_FOREACH(const sharedFactor& f, *this) {
		if (f)
			result.push_back(f->rekey(rekey_mapping));
		else
			result.push_back(sharedFactor());
	}
	return result;
}

/* ************************************************************************* */

} // namespace gtsam
