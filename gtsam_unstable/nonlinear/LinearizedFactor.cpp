/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file LinearizedFactor.cpp
 * @brief A dummy factor that allows a linear factor to act as a nonlinear factor
 * @author Alex Cunningham
 */

#include <iostream>
#include <boost/foreach.hpp>

#include <gtsam_unstable/nonlinear/LinearizedFactor.h>

namespace gtsam {

using namespace std;

/* ************************************************************************* */
LinearizedFactor::LinearizedFactor(JacobianFactor::shared_ptr lin_factor,
		const KeyLookup& decoder, const Values& lin_points)
: Base(lin_factor->get_model()), b_(lin_factor->getb()), model_(lin_factor->get_model()) {
	BOOST_FOREACH(const Index& idx, *lin_factor) {
		// find full symbol
		KeyLookup::const_iterator decode_it = decoder.find(idx);
		if (decode_it == decoder.end())
			throw runtime_error("LinearizedFactor: could not find index in decoder!");
		Key key(decode_it->second);

		// extract linearization point
		assert(lin_points.exists(key));
		this->lin_points_.insert(key, lin_points.at(key));  // NOTE: will not overwrite

		// extract Jacobian
		Matrix A = lin_factor->getA(lin_factor->find(idx));
		this->matrices_.insert(std::make_pair(key, A));

		// store keys
		this->keys_.push_back(key);
	}
}

/* ************************************************************************* */
LinearizedFactor::LinearizedFactor(JacobianFactor::shared_ptr lin_factor,
			const Ordering& ordering, const Values& lin_points)
: Base(lin_factor->get_model()), b_(lin_factor->getb()), model_(lin_factor->get_model()) {
	const KeyLookup decoder = ordering.invert();
	BOOST_FOREACH(const Index& idx, *lin_factor) {
		// find full symbol
		KeyLookup::const_iterator decode_it = decoder.find(idx);
		if (decode_it == decoder.end())
			throw runtime_error("LinearizedFactor: could not find index in decoder!");
		Key key(decode_it->second);

		// extract linearization point
		assert(lin_points.exists(key));
		this->lin_points_.insert(key, lin_points.at(key));  // NOTE: will not overwrite

		// extract Jacobian
		Matrix A = lin_factor->getA(lin_factor->find(idx));
		this->matrices_.insert(std::make_pair(key, A));

		// store keys
		this->keys_.push_back(key);
	}
}

/* ************************************************************************* */
Vector LinearizedFactor::unwhitenedError(const Values& c,
		boost::optional<std::vector<Matrix>&> H) const {
	// extract the points in the new values
	Vector ret = - b_;

	if (H) H->resize(this->size());
	size_t i=0;
	BOOST_FOREACH(Key key, this->keys()) {
		const Value& newPt = c.at(key);
		const Value& linPt = lin_points_.at(key);
		Vector d = linPt.localCoordinates_(newPt);
		const Matrix& A = matrices_.at(key);
		ret += A * d;
		if (H) (*H)[i++] = A;
	}

	return ret;
}

/* ************************************************************************* */
boost::shared_ptr<GaussianFactor>
LinearizedFactor::linearize(const Values& c, const Ordering& ordering) const {

	// sort by varid - only known at linearization time
	typedef std::map<Index, Matrix> VarMatrixMap;
	VarMatrixMap sorting_terms;
	BOOST_FOREACH(const KeyMatrixMap::value_type& p, matrices_)
		sorting_terms.insert(std::make_pair(ordering[p.first], p.second));

	// move into terms
	std::vector<std::pair<Index, Matrix> > terms;
	BOOST_FOREACH(const VarMatrixMap::value_type& p, sorting_terms)
		terms.push_back(p);

	// compute rhs: adjust current by whitened update
	Vector b = unwhitenedError(c) + b_; // remove original b
	this->noiseModel_->whitenInPlace(b);

	return boost::shared_ptr<GaussianFactor>(new JacobianFactor(terms, b - b_, model_));
}

/* ************************************************************************* */
void LinearizedFactor::print(const std::string& s, const KeyFormatter& keyFormatter) const {
	this->noiseModel_->print(s + std::string(" model"));
	BOOST_FOREACH(const KeyMatrixMap::value_type& p, matrices_) {
		gtsam::print(p.second, keyFormatter(p.first));
	}
	gtsam::print(b_, std::string("b"));
	std::cout << " nonlinear keys: ";
	BOOST_FOREACH(const Key& key, this->keys())
		cout << keyFormatter(key) << "  ";
	lin_points_.print("Linearization Point");
}

/* ************************************************************************* */
bool LinearizedFactor::equals(const NonlinearFactor& other, double tol) const {
  const LinearizedFactor* e = dynamic_cast<const LinearizedFactor*>(&other);
	if (!e || !Base::equals(other, tol)
			|| !lin_points_.equals(e->lin_points_, tol)
			|| !equal_with_abs_tol(b_, e->b_, tol)
			|| !model_->equals(*e->model_, tol)
			|| matrices_.size() != e->matrices_.size())
		return false;

	KeyMatrixMap::const_iterator map1 = matrices_.begin(), map2 = e->matrices_.begin();
	for (; map1 != matrices_.end() && map2 != e->matrices_.end(); ++map1, ++map2)
		if ((map1->first != map2->first) || !equal_with_abs_tol(map1->second, map2->second, tol))
			return false;
	return true;
}

} // \namespace gtsam
