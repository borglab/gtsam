/*
 * DSFVector.cpp
 *
 *   Created on: Jun 25, 2010
 *       Author: nikai
 *  Description: a faster implementation for DSF, which uses vector rather than btree.
 *               As a result, the size of the forest is prefixed.
 */

#include "DSFVector.h"

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
	size_t DSFVector::findSet(const size_t& key) const {
		size_t parent = at(key);
		return parent == key ? key : findSet(parent);
	}

	/* ************************************************************************* */
	std::set<size_t> DSFVector::set(const std::size_t& label) const {
		std::set<size_t> set;
		size_t key = 0;
		std::vector<std::size_t>::const_iterator it = begin();
		for (; it != end(); it++, key++) {
			if (*it == label || findSet(*it) == label)
				set.insert(key);
		}
		return set;
	}

	/* ************************************************************************* */
	void DSFVector::makeUnionInPlace(const std::size_t& i1, const std::size_t& i2)  {
		at(findSet(i2)) = findSet(i1);
	}

	/* ************************************************************************* */
	std::map<size_t, std::set<size_t> > DSFVector::sets() const {
		std::map<size_t, std::set<size_t> > sets;
		size_t key = 0;
		std::vector<std::size_t>::const_iterator it = begin();
		for (; it != end(); it++, key++) {
			sets[findSet(*it)].insert(key);
		}
		return sets;
	}

}
