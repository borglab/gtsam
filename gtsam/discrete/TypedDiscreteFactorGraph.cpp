/*
 * @file		TypedDiscreteFactorGraph.cpp
 * @brief		
 * @author	Duy-Nguyen Ta
 * @date	Mar 1, 2011
 */

#include <iostream>
#include <fstream>
#include <stdexcept>
#include <boost/lexical_cast.hpp>
#include <boost/foreach.hpp>
#include <gtsam/discrete/DiscreteFactor.h>
#include <gtsam/discrete/TypedDiscreteFactorGraph.h>
#include <gtsam/discrete/parseUAI.h>

#include <gtsam/inference/FactorGraph.h>

using namespace std;

namespace gtsam {

	/* ************************************************************************* */
	TypedDiscreteFactorGraph::TypedDiscreteFactorGraph() {
	}

	/* ************************************************************************* */
	TypedDiscreteFactorGraph::TypedDiscreteFactorGraph(const string& filename) {
		bool success = parseUAI(filename, *this);
		if (!success) throw runtime_error(
				"TypedDiscreteFactorGraph constructor from filename failed");
	}

	/* ************************************************************************* */
	void TypedDiscreteFactorGraph::add//
	(const Indices& keys, const string& table) {
		push_back(boost::shared_ptr<TypedDiscreteFactor>//
				(new TypedDiscreteFactor(keys, table)));
	}

	/* ************************************************************************* */
	void TypedDiscreteFactorGraph::add//
	(const Indices& keys, const vector<double>& table) {
		push_back(boost::shared_ptr<TypedDiscreteFactor>//
				(new TypedDiscreteFactor(keys, table)));
	}

	/* ************************************************************************* */
	void TypedDiscreteFactorGraph::print(const string s) {
		cout << s << endl;
		cout << "Factors: " << endl;
		BOOST_FOREACH(const sharedFactor factor, factors_)
						factor->print();
	}

	/* ************************************************************************* */
	double TypedDiscreteFactorGraph::operator()(
			const TypedDiscreteFactor::Values& values) const {
		// Loop over all factors and multiply their probabilities
		double p = 1.0;
		BOOST_FOREACH(const sharedFactor& factor, *this)
						p *= (*factor)(values);
		return p;
	}

/* ************************************************************************* */

}
