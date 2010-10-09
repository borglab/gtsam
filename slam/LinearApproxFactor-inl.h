/*
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
LinearApproxFactor<Values,Key>::LinearApproxFactor(GaussianFactor::shared_ptr lin_factor, const Values& lin_points)
: Base(noiseModel::Unit::Create(lin_factor->get_model()->dim())),
  lin_factor_(lin_factor), lin_points_(lin_points)
  {
	// create the keys and store them
	BOOST_FOREACH(Symbol key, lin_factor->keys()) {
		nonlinearKeys_.push_back(Key(key.index()));
		this->keys_.push_back(key);
	}
  }

/* ************************************************************************* */
template <class Values, class Key>
Vector LinearApproxFactor<Values,Key>::unwhitenedError(const Values& c) const {
	// extract the points in the new config
	VectorValues delta;
	BOOST_FOREACH(const Key& key, nonlinearKeys_) {
		X newPt = c[key], linPt = lin_points_[key];
		Vector d = linPt.logmap(newPt);
		delta.insert(key, d);
	}

	return lin_factor_->unweighted_error(delta);
}

/* ************************************************************************* */
template <class Values, class Key>
boost::shared_ptr<GaussianFactor>
LinearApproxFactor<Values,Key>::linearize(const Values& c) const {
  Vector b = lin_factor_->getb();
  SharedDiagonal model = lin_factor_->get_model();
  std::vector<std::pair<Symbol, Matrix> > terms;
  BOOST_FOREACH(Symbol key, lin_factor_->keys()) {
    terms.push_back(std::make_pair(key, lin_factor_->get_A(key)));
  }

  return boost::shared_ptr<GaussianFactor>(
      new GaussianFactor(terms, b, model));
}

/* ************************************************************************* */
template <class Values, class Key>
void LinearApproxFactor<Values,Key>::print(const std::string& s) const {
	LinearApproxFactor<Values,Key>::Base::print(s);
	lin_factor_->print();
}

} // \namespace gtsam
