/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file DSFVector.cpp
 * @date Jun 25, 2010
 * @author Kai Ni
 * @brief a faster implementation for DSF, which uses vector rather than btree.
 */

#include <boost/make_shared.hpp>
#include <boost/foreach.hpp>
#include <gtsam/base/DSFVector.h>

using namespace std;

namespace gtsam {

/* ************************************************************************* */
DSFVector::DSFVector (const size_t numNodes) {
	v_ = boost::make_shared<V>(numNodes);
	int index = 0;
	keys_.reserve(numNodes);
	for(V::iterator it = v_->begin(); it!=v_->end(); it++, index++) {
		*it = index;
		keys_.push_back(index);
	}
}

/* ************************************************************************* */
DSFVector::DSFVector(const boost::shared_ptr<V>& v_in, const std::vector<size_t>& keys) : keys_(keys) {
	v_ = v_in;
	BOOST_FOREACH(const size_t key, keys)
		(*v_)[key] = key;
}

/* ************************************************************************* */
bool DSFVector::isSingleton(const Label& label) const {
	bool result = false;
	V::const_iterator it = keys_.begin();
	for (; it != keys_.end(); ++it) {
		if(findSet(*it) == label) {
			if (!result) // find the first occurrence
				result = true;
			else
				return false;
		}
	}
	return result;
}

/* ************************************************************************* */
std::set<size_t> DSFVector::set(const Label& label) const {
	std::set<size_t> set;
	V::const_iterator it = keys_.begin();
	for (; it != keys_.end(); it++) {
		if (findSet(*it) == label)
			set.insert(*it);
	}
	return set;
}

/* ************************************************************************* */
std::map<DSFVector::Label, std::set<size_t> > DSFVector::sets() const {
	std::map<Label, std::set<size_t> > sets;
	V::const_iterator it = keys_.begin();
	for (; it != keys_.end(); it++) {
		sets[findSet(*it)].insert(*it);
	}
	return sets;
}

/* ************************************************************************* */
std::map<DSFVector::Label, std::vector<size_t> > DSFVector::arrays() const {
	std::map<Label, std::vector<size_t> > arrays;
	V::const_iterator it = keys_.begin();
	for (; it != keys_.end(); it++) {
		arrays[findSet(*it)].push_back(*it);
	}
	return arrays;
}

/* ************************************************************************* */
void DSFVector::makeUnionInPlace(const size_t& i1, const size_t& i2)  {
	(*v_)[findSet(i2)] = findSet(i1);
}

} // namespace  gtsam

