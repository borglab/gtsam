/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file DSFMap.h
 * @date Oct 26, 2013
 * @author Frank Dellaert
 * @brief Allow for arbitrary type in DSF
 */

#pragma once

#include <map>
#include <set>

namespace gtsam {

/**
 * Disjoint set forest using an STL map data structure underneath
 * Uses rank compression and union by rank, iterator version
 * @addtogroup base
 */
template<class KEY>
class DSFMap {

protected:

	/// We store the forest in an STL map, but parents are done with pointers
	struct Entry {
		KEY key_;
		size_t rank_;
		Entry* parent_;
		Entry(KEY key) :
				key_(key), rank_(0), parent_(0) {
		}
		void makeRoot() {
			parent_ = this;
		}
	};

	typedef std::map<KEY, Entry> Map;
	mutable Map entries_;

	/// Given key, find iterator to initial entry
	typename Map::iterator find__(const KEY& key) const {
		typename Map::iterator it = entries_.find(key);
		// if key does not exist, create and return itself
		if (it == entries_.end()) {
			it = entries_.insert(it, std::make_pair(key, Entry(key)));
			it->second.makeRoot();
		}
		return it;
	}

	/// Given iterator to initial entry, find the root Entry
	Entry* find_(const typename Map::iterator& it) const {
		// follow parent pointers until we reach set representative
		Entry* parent = it->second.parent_;
		while (parent->parent_ != parent)
			parent = parent->parent_; // not yet, recurse!
		it->second.parent_ = parent; // path compression
		return parent;
	}

	/// Given key, find the root Entry
	Entry* find_(const KEY& key) const {
		typename Map::iterator it = find__(key);
		return find_(it);
	}

public:

	typedef std::set<KEY> Set;

	/// constructor
	DSFMap() {
	}

	/// Given key, find the representative key for the set in which it lives
	KEY find(const KEY& key) const {
		Entry* root = find_(key);
		return root->key_;
	}

	/// Merge two sets
	void merge(const KEY& x, const KEY& y) {

		// straight from http://en.wikipedia.org/wiki/Disjoint-set_data_structure
		Entry* xRoot = find_(x);
		Entry* yRoot = find_(y);
		if (xRoot == yRoot)
			return;

		// Merge sets
		if (xRoot->rank_ < yRoot->rank_)
			xRoot->parent_ = yRoot;
		else if (xRoot->rank_ > yRoot->rank_)
			yRoot->parent_ = xRoot;
		else {
			yRoot->parent_ = xRoot;
			xRoot->rank_ = xRoot->rank_ + 1;
		}
	}

	/// return all sets, i.e. a partition of all elements
	std::map<KEY, Set> sets() const {
		std::map<KEY, Set> sets;
		typename Map::iterator it = entries_.begin();
		for(;it!=entries_.end();it++) {
			Entry* root = find_(it);
			sets[root->key_].insert(it->first);
		}
		return sets;
	}

};

}
