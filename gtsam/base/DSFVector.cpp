/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/*
 * DSFVector.cpp
 *
 *   Created on: Jun 25, 2010
 *       Author: nikai
 *  Description: a faster implementation for DSF, which uses vector rather than btree.
 *               As a result, the size of the forest is prefixed.
 */

#include <gtsam/base/DSFVector.h>

using namespace std;

namespace gtsam {

	/* ************************************************************************* */
	DSFVector::DSFVector (const size_t numNodes) {
		resize(numNodes);
		int index = 0;
		for(iterator it = begin(); it!=end(); it++, index++)
			*it = index;
	}

	/* ************************************************************************* */
	DSFVector::Label DSFVector::findSet(const size_t& key) const {
		size_t parent = at(key);
		return parent == key ? key : findSet(parent);
	}

	/* ************************************************************************* */
	bool DSFVector::isSingleton(const Label& label) const {
		bool result = false;
		std::vector<size_t>::const_iterator it = begin();
		for (; it != end(); ++it) {
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
		size_t key = 0;
		std::vector<size_t>::const_iterator it = begin();
		for (; it != end(); it++, key++) {
			if (findSet(*it) == label)
				set.insert(key);
		}
		return set;
	}

	/* ************************************************************************* */
	std::map<DSFVector::Label, std::set<size_t> > DSFVector::sets() const {
		std::map<Label, std::set<size_t> > sets;
		size_t key = 0;
		std::vector<size_t>::const_iterator it = begin();
		for (; it != end(); it++, key++) {
			sets[findSet(*it)].insert(key);
		}
		return sets;
	}

	/* ************************************************************************* */
	void DSFVector::makeUnionInPlace(const size_t& i1, const size_t& i2)  {
		at(findSet(i2)) = findSet(i1);
	}

} // namespace

