/*
 * @file		TypedDiscreteFactor.cpp
 * @brief		
 * @author	Duy-Nguyen Ta
 * @date	Mar 5, 2011
 */

#include <gtsam_unstable/discrete/TypedDiscreteFactor.h>
#include <gtsam/discrete/DecisionTree.h>
#include <gtsam/inference/Factor-inl.h>
#include <boost/make_shared.hpp>

using namespace std;

namespace gtsam {

	/* ******************************************************************************** */
	TypedDiscreteFactor::TypedDiscreteFactor(const Indices& keys,
			const string& table) :
		Factor<Index> (keys.begin(), keys.end()), potentials_(keys, table) {
	}

	/* ******************************************************************************** */
	TypedDiscreteFactor::TypedDiscreteFactor(const Indices& keys,
			const vector<double>& table) :
		Factor<Index> (keys.begin(), keys.end()), potentials_(keys, table) {
		//#define DEBUG_FACTORS
#ifdef DEBUG_FACTORS
		static size_t count = 0;
		string dotfile = (boost::format("Factor-%03d") % ++count).str();
		potentials_.dot(dotfile);
		if (count == 57) potentials_.print("57");
#endif
	}

	/* ************************************************************************* */
	double TypedDiscreteFactor::operator()(const Values& values) const {
		return potentials_(values);
	}

	/* ************************************************************************* */
	void TypedDiscreteFactor::print(const string&s) const {
		Factor<Index>::print(s);
		potentials_.print();
	}

	/* ************************************************************************* */
	bool TypedDiscreteFactor::equals(const TypedDiscreteFactor& other, double tol) const {
		return potentials_.equals(other.potentials_, tol);
	}

	/* ******************************************************************************** */
	DiscreteFactor::shared_ptr TypedDiscreteFactor::toDiscreteFactor(
			const KeyOrdering& ordering) const {
		throw std::runtime_error("broken");
		//return boost::make_shared<DiscreteFactor>(keys(), ordering, potentials_);
	}

#ifdef OLD
DiscreteFactor TypedDiscreteFactor::toDiscreteFactor(
		const KeyOrdering& ordering, const ProblemType problemType) const {
	{
		static bool debug = false;

		// instantiate vector keys and column index in order
		DiscreteFactor::ColumnIndex orderColumnIndex;
		vector<Index> keys;
		BOOST_FOREACH(const KeyOrdering::value_type& ord, ordering)
		{
			if (debug) cout << "Key: " << ord.first;

			// find the key with ord.first in this factor
			vector<Index>::const_iterator it = std::find(keys_.begin(),
					keys_.end(), ord.first);

			// if found
			if (it != keys_.end()) {
				if (debug) cout << "it found: " << (*it) << ", index: "
				<< ord.second << endl;

				keys.push_back(ord.second); // push back the ordering index
				orderColumnIndex[ord.second] = columnIndex_.at(ord.first.name());

				if (debug) cout << "map " << ord.second << " with name: "
				<< ord.first.name() << " to " << columnIndex_.at(
						ord.first.name()) << endl;
			}
		}

		DiscreteFactor f(keys, potentials_, orderColumnIndex, problemType);
		return f;
	}

	/* ******************************************************************************** */
	std::vector<size_t> TypedDiscreteFactor::init(const Indices& keys) {
		vector<size_t> cardinalities;
		for (size_t j = 0; j < keys.size(); j++) {
			Index key = keys[j];
			keys_.push_back(key);
			columnIndex_[key.name()] = j;
			cardinalities.push_back(key.cardinality());
		}
		return cardinalities;
	}

	/* ******************************************************************************** */
	double TypedDiscreteFactor::potential(const TypedValues& values) const {
		vector<size_t> assignment(values.size());
		BOOST_FOREACH(const TypedValues::value_type& val, values)
		if (columnIndex_.find(val.first) != columnIndex_.end()) assignment[columnIndex_.at(
				val.first)] = val.second;
		return potentials_(assignment);
	}

#endif

} // namespace
