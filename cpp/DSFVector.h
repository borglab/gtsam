/*
 * DSFVector.h
 *
 *   Created on: Jun 25, 2010
 *       Author: nikai
 *  Description: a faster implementation for DSF, which uses vector rather than btree.
 *               As a result, the size of the forest is prefixed.
 */

#pragma once

#include <vector>
#include <map>
#include <set>

namespace gtsam {

	/**
	 * A fast impelementation of disjoint set forests
	 */
	class DSFVector : protected std::vector<std::size_t> {
	private:

	public:
		// constructor
		DSFVector(const std::size_t numNodes);

		// find the label of the set in which {key} lives
		size_t findSet(const size_t& key) const;

		// the in-place version of makeUnion
		void makeUnionInPlace(const std::size_t& i1, const std::size_t& i2);

		// get the nodes in the tree with the given label
		std::set<size_t> set(const std::size_t& label) const;

		// return all sets, i.e. a partition of all elements
		std::map<size_t, std::set<size_t> > sets() const;
	};

}
