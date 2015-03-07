/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file DecisionTreeFactor.cpp
 * @brief discrete factor
 * @date Feb 14, 2011
 * @author Duy-Nguyen Ta
 * @author Frank Dellaert
 */

#include <gtsam/discrete/DecisionTreeFactor.h>
#include <gtsam/discrete/DiscreteConditional.h>
#include <gtsam/base/FastSet.h>

#include <boost/foreach.hpp>
#include <boost/make_shared.hpp>

using namespace std;

namespace gtsam {

	/* ******************************************************************************** */
	DecisionTreeFactor::DecisionTreeFactor() {
	}

	/* ******************************************************************************** */
	DecisionTreeFactor::DecisionTreeFactor(const DiscreteKeys& keys,
			const ADT& potentials) :
			DiscreteFactor(keys.indices()), Potentials(keys, potentials) {
	}

	/* *************************************************************************/
	DecisionTreeFactor::DecisionTreeFactor(const DiscreteConditional& c) :
			DiscreteFactor(c.keys()), Potentials(c) {
	}

	/* ************************************************************************* */
	bool DecisionTreeFactor::equals(const This& other, double tol) const {
		return IndexFactor::equals(other, tol) && Potentials::equals(other, tol);
	}

	/* ************************************************************************* */
	void DecisionTreeFactor::print(const string& s,
			const IndexFormatter& formatter) const {
		cout << s;
		IndexFactor::print("IndexFactor:",formatter);
		Potentials::print("Potentials:",formatter);
	}

	/* ************************************************************************* */
	DecisionTreeFactor DecisionTreeFactor::apply //
	(const DecisionTreeFactor& f, ADT::Binary op) const {
		map<Index,size_t> cs; // new cardinalities
		// make unique key-cardinality map
		BOOST_FOREACH(Index j, keys()) cs[j] = cardinality(j);
		BOOST_FOREACH(Index j, f.keys()) cs[j] = f.cardinality(j);
		// Convert map into keys
		DiscreteKeys keys;
		BOOST_FOREACH(const DiscreteKey& key, cs)
			keys.push_back(key);
		// apply operand
		ADT result = ADT::apply(f, op);
		// Make a new factor
		return DecisionTreeFactor(keys, result);
	}

	/* ************************************************************************* */
	DecisionTreeFactor::shared_ptr DecisionTreeFactor::combine //
	(size_t nrFrontals, ADT::Binary op) const {

		if (nrFrontals == 0 || nrFrontals > size()) throw invalid_argument(
				(boost::format(
						"DecisionTreeFactor::combine: invalid number of frontal keys %d, nr.keys=%d")
						% nrFrontals % size()).str());

		// sum over nrFrontals keys
		size_t i;
		ADT result(*this);
		for (i = 0; i < nrFrontals; i++) {
			Index j = keys()[i];
			result = result.combine(j, cardinality(j), op);
		}

		// create new factor, note we start keys after nrFrontals
		DiscreteKeys dkeys;
		for (; i < keys().size(); i++) {
			Index j = keys()[i];
			dkeys.push_back(DiscreteKey(j,cardinality(j)));
		}
		return boost::make_shared<DecisionTreeFactor>(dkeys, result);
	}

/* ************************************************************************* */
} // namespace gtsam
