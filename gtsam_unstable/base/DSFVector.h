/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file DSFVector.h
 * @date Jun 25, 2010
 * @author Kai Ni
 * @brief A faster implementation for DSF, which uses vector rather than btree. As a result, the size of the forest is prefixed.
 */

#pragma once

#include <vector>
#include <map>
#include <set>
#include <boost/shared_ptr.hpp>

namespace gtsam {

	/**
	 * A fast implementation of disjoint set forests that uses vector as underly data structure.
	 * @addtogroup base
	 */
	class DSFVector {

	public:
		typedef std::vector<size_t> V; ///< Vector of ints
		typedef size_t Label;          ///< Label type
		typedef V::const_iterator const_iterator; ///< const iterator over V
		typedef V::iterator iterator;///< iterator over V

	private:
		 // TODO could use existing memory to improve the efficiency
		boost::shared_ptr<V> v_;
		std::vector<size_t> keys_;

	public:
		/// constructor that allocate a new memory
		DSFVector(const size_t numNodes);

		/// constructor that uses the existing memory
		DSFVector(const boost::shared_ptr<V>& v_in, const std::vector<size_t>& keys);

		/// find the label of the set in which {key} lives
		inline Label findSet(size_t key) const {
			size_t parent = (*v_)[key];
			while (parent != key) {
				key = parent;
				parent = (*v_)[key];
			}
			return parent;
		}

		/// find whether there is one and only one occurrence for the given {label}
		bool isSingleton(const Label& label) const;

		/// get the nodes in the tree with the given label
		std::set<size_t> set(const Label& label) const;

		/// return all sets, i.e. a partition of all elements
		std::map<Label, std::set<size_t> > sets() const;

		/// return all sets, i.e. a partition of all elements
		std::map<Label, std::vector<size_t> > arrays() const;

		/// the in-place version of makeUnion
		void makeUnionInPlace(const size_t& i1, const size_t& i2);

	};

}
