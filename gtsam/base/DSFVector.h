/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

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
	 * A fast impelementation of disjoint set forests that uses vector as underly data structure.
	 */
	class DSFVector : protected std::vector<std::size_t> {
	private:

	public:
		typedef size_t Label;

		// constructor
		DSFVector(const std::size_t numNodes);

		// find the label of the set in which {key} lives
		Label findSet(const size_t& key) const;

		// find whether there is one and only one occurrence for the given {label}
		bool isSingleton(const Label& label) const;

		// get the nodes in the tree with the given label
		std::set<size_t> set(const Label& label) const;

		// return all sets, i.e. a partition of all elements
		std::map<Label, std::set<size_t> > sets() const;

		// the in-place version of makeUnion
		void makeUnionInPlace(const std::size_t& i1, const std::size_t& i2);

	};

}
