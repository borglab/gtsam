/**
 * @file LinearApproxFactor.h
 * @brief A dummy factor that allows a linear factor to act as a nonlinear factor
 * @author Alex Cunningham
 */

#pragma once

#include <iostream>
#include <boost/foreach.hpp>

#include <gtsam/slam/LinearApproxFactor.h>

namespace gtsam {

/* ************************************************************************* */
template <class Values, class Key>
LinearApproxFactor<Values,Key>::LinearApproxFactor(
		GaussianFactor::shared_ptr lin_factor, const Ordering& ordering, const Values& lin_points)
: Base(noiseModel::Unit::Create(lin_factor->get_model()->dim())),
  b_(lin_factor->getb()), model_(lin_factor->get_model()), lin_points_(lin_points)
{
	BOOST_FOREACH(const Ordering::Map::value_type& p, ordering) {
		Symbol key = p.first;
		varid_t var = p.second;

		// check if actually in factor
		Factor::const_iterator it = lin_factor->find(var);
		if (it != lin_factor->end()) {
			// store matrix
			Matrix A = lin_factor->getA(it);
			matrices_.insert(make_pair(key, A));

			// store keys
			nonlinearKeys_.push_back(Key(key.index()));
			this->keys_.push_back(key);
		}
	}
}

/* ************************************************************************* */
template <class Values, class Key>
Vector LinearApproxFactor<Values,Key>::unwhitenedError(const Values& c) const {
	// extract the points in the new config
	VectorValues delta;
	//	BOOST_FOREACH(const Key& key, nonlinearKeys_) {
	//		X newPt = c[key], linPt = lin_points_[key];
	//		Vector d = linPt.logmap(newPt);
	//		delta.insert(key, d);
	//	}

	return zero(b_.size()); //FIXME: PLACEHOLDER!
}

/* ************************************************************************* */
template <class Values, class Key>
boost::shared_ptr<GaussianFactor>
LinearApproxFactor<Values,Key>::linearize(const Values& c, const Ordering& ordering) const {

	// sort by varid - only known at linearization time
	typedef std::map<varid_t, Matrix> VarMatrixMap;
	VarMatrixMap sorting_terms;
	BOOST_FOREACH(const SymbolMatrixMap::value_type& p, matrices_)
		sorting_terms.insert(std::make_pair(ordering[p.first], p.second));

	// move into terms
	std::vector<std::pair<varid_t, Matrix> > terms;
	BOOST_FOREACH(const VarMatrixMap::value_type& p, sorting_terms)
		terms.push_back(p);

	return boost::shared_ptr<GaussianFactor>(new GaussianFactor(terms, b_, model_));
}

/* ************************************************************************* */
template <class Values, class Key>
Factor::shared_ptr
LinearApproxFactor<Values,Key>::symbolic(const Ordering& ordering) const {
	std::vector<varid_t> key_ids(this->keys_.size());
	size_t i=0;
	BOOST_FOREACH(const Symbol& key, this->keys_)
	key_ids[i++] = ordering[key];
	std::sort(key_ids.begin(), key_ids.end());
	return boost::shared_ptr<Factor>(new Factor(key_ids.begin(), key_ids.end()));
}

/* ************************************************************************* */
template <class Values, class Key>
void LinearApproxFactor<Values,Key>::print(const std::string& s) const {
	LinearApproxFactor<Values,Key>::Base::print(s);
	BOOST_FOREACH(const SymbolMatrixMap::value_type& p, matrices_) {
		gtsam::print(p.second, (std::string) p.first);
	}
	gtsam::print(b_, std::string("b"));
	model_->print("model");
}

} // \namespace gtsam
