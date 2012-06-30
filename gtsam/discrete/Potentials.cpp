/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file Potentials.cpp
 * @date March 24, 2011
 * @author Frank Dellaert
 */

#include <gtsam/discrete/Potentials.h>
#include <gtsam/discrete/DecisionTree-inl.h>
#include <boost/format.hpp>

using namespace std;

namespace gtsam {

	// explicit instantiation
	template class DecisionTree<Index, double> ;
	template class AlgebraicDecisionTree<Index> ;

	/* ************************************************************************* */
	double Potentials::safe_div(const double& a, const double& b) {
		// cout << boost::format("%g / %g = %g\n") % a % b % ((a == 0) ? 0 : (a / b));
		// The use for safe_div is when we divide the product factor by the sum factor.
		// If the product or sum is zero, we accord zero probability to the event.
		return (a == 0 || b == 0) ? 0 : (a / b);
	}

	/* ******************************************************************************** */
	Potentials::Potentials() :
			ADT(1.0) {
	}

	/* ******************************************************************************** */
	Potentials::Potentials(const DiscreteKeys& keys, const ADT& decisionTree) :
			ADT(decisionTree), cardinalities_(keys.cardinalities()) {
	}

	/* ************************************************************************* */
	bool Potentials::equals(const Potentials& other, double tol) const {
		return ADT::equals(other, tol);
	}

	/* ************************************************************************* */
	void Potentials::print(const string& s,
			const IndexFormatter& formatter) const {
		cout << s << "\n  Cardinalities: ";
		BOOST_FOREACH(const DiscreteKey& key, cardinalities_)
			cout << formatter(key.first) << "=" << formatter(key.second) << " ";
		cout << endl;
		ADT::print(" ");
	}

	/* ************************************************************************* */
	void Potentials::permuteWithInverse(const Permutation& permutation) {
		// Permute the _cardinalities (TODO: Inefficient Consider Improving)
		DiscreteKeys keys;
		map<Index, Index> ordering;

		// Get the orginal keys from cardinalities_
		BOOST_FOREACH(const DiscreteKey& key, cardinalities_)
			keys & key;

		// Perform Permutation
		BOOST_FOREACH(DiscreteKey& key, keys) {
			ordering[key.first] = permutation[key.first];
			key.first = ordering[key.first];
		}

		// Change *this
		AlgebraicDecisionTree<Index> permuted((*this), ordering);
		*this = permuted;
		cardinalities_ = keys.cardinalities();
	}

	/* ************************************************************************* */

} // namespace gtsam
