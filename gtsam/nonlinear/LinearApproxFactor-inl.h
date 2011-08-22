/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file LinearApproxFactor.h
 * @brief A dummy factor that allows a linear factor to act as a nonlinear factor
 * @author Alex Cunningham
 */

#pragma once

#include <gtsam/nonlinear/LinearApproxFactor.h>
#include <gtsam/linear/JacobianFactor.h>

#include <iostream>
#include <boost/foreach.hpp>

namespace gtsam {

/* ************************************************************************* */
template <class VALUES, class KEY>
Vector LinearApproxFactor<VALUES,KEY>::unwhitenedError(const VALUES& c) const {
	// extract the points in the new values
	Vector ret = b_;

	BOOST_FOREACH(const KEY& key, nonlinearKeys_) {
		X newPt = c[key], linPt = lin_points_[key];
		Vector d = linPt.logmap(newPt);
		const Matrix& A = matrices_.at(key);
		ret -= A * d;
	}

	return ret;
}

/* ************************************************************************* */
template <class VALUES, class KEY>
boost::shared_ptr<GaussianFactor>
LinearApproxFactor<VALUES,KEY>::linearize(const VALUES& c, const Ordering& ordering) const {

	// sort by varid - only known at linearization time
	typedef std::map<Index, Matrix> VarMatrixMap;
	VarMatrixMap sorting_terms;
	BOOST_FOREACH(const typename KeyMatrixMap::value_type& p, matrices_)
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
template <class VALUES, class KEY>
IndexFactor::shared_ptr
LinearApproxFactor<VALUES,KEY>::symbolic(const Ordering& ordering) const {
	std::vector<Index> key_ids(this->keys_.size());
	size_t i=0;
	BOOST_FOREACH(const Symbol& key, this->keys_)
		key_ids[i++] = ordering[key];
	std::sort(key_ids.begin(), key_ids.end());
	return boost::shared_ptr<IndexFactor>(new IndexFactor(key_ids.begin(), key_ids.end()));
}

/* ************************************************************************* */
template <class VALUES, class KEY>
void LinearApproxFactor<VALUES,KEY>::print(const std::string& s) const {
	this->noiseModel_->print(s + std::string(" model"));
	BOOST_FOREACH(const typename KeyMatrixMap::value_type& p, matrices_) {
		gtsam::print(p.second, (std::string) p.first);
	}
	gtsam::print(b_, std::string("b"));
	std::cout << " nonlinear keys: ";
	BOOST_FOREACH(const KEY& key, nonlinearKeys_)
		key.print("  ");
	lin_points_.print("Linearization Point");
}

} // \namespace gtsam
