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

#include <boost/foreach.hpp>
#include <map>
#include <set>
#include <iostream>

namespace gtsam {

/**
 * Disjoint set forest using an STL map data structure underneath
 * Uses rank compression but not union by rank :-(
 * @addtogroup base
 */
template<class KEY>
class DSFMap {

protected:

	/// We store the forest in an STL map
	typedef std::map<KEY, KEY> Map;
	typedef std::set<KEY> Set;
	typedef std::pair<KEY, KEY> key_pair;
	mutable Map parent_;

public:
	/// constructor
	DSFMap() {
	}

	/// find the label of the set in which {key} lives
	KEY find(const KEY& key) const {
		typename Map::const_iterator it = parent_.find(key);
		// if key does not exist, create and return itself
		if (it == parent_.end()) {
			parent_[key] = key;
			return key;
		} else {
			// follow parent pointers until we reach set representative
			KEY parent = it->second;
			if (parent != key)
				parent = find(parent); // not yet, recurse!
			parent_[key] = parent; // path compression
			return parent;
		}
	}

	/// Merge two sets
	void merge(const KEY& i1, const KEY& i2) {
		parent_[find(i2)] = find(i1);
	}

	/// return all sets, i.e. a partition of all elements
	std::map<KEY, Set> sets() const {
		std::map<KEY, Set> sets;
		BOOST_FOREACH(const key_pair& pair, parent_)
		sets[find(pair.second)].insert(pair.first);
		return sets;
	}

};

/**
 * Disjoint set forest using an STL map data structure underneath
 * Uses rank compression but not union by rank :-(
 * @addtogroup base
 */
template<class KEY>
class DSFMapIt {

protected:

	/// We store the forest in an STL map, but parents are done with pointers
	struct Entry {
		typedef std::map<KEY, Entry> Map;
		typename Map::iterator parent_;
		size_t rank_;
		Entry() :
				rank_(0) {
		}
		void makeRoot(const typename Map::iterator& it) {
			parent_ = it;
		}
	};
	mutable typename Entry::Map entries_;

	/// find the initial Entry
	typename Entry::Map::iterator find__(const KEY& key) const {
		typename Entry::Map::iterator it = entries_.find(key);
		// if key does not exist, create and return itself
		if (it == entries_.end()) {
			it = entries_.insert(it, std::make_pair(key, Entry()));
			it->second.makeRoot(it);
		}
		return it;
	}

	/// find the root Entry
	typename Entry::Map::iterator find_(const KEY& key) const {
		typename Entry::Map::iterator initial = find__(key);
		// follow parent pointers until we reach set representative
		typename Entry::Map::iterator parent = initial->second.parent_;
		while (parent->second.parent_ != parent)
			parent = parent->second.parent_; // not yet, recurse!
		//initial.parent_ = parent; // path compression
		return parent;
	}

public:
	/// constructor
	DSFMapIt() {
	}

	/// find the representative KEY for the set in which key lives
	KEY find(const KEY& key) const {
		typename Entry::Map::iterator root = find_(key);
		return root->first;
	}

	/// Merge two sets
	void merge(const KEY& x, const KEY& y) {

		// straight from http://en.wikipedia.org/wiki/Disjoint-set_data_structure
		typename Entry::Map::iterator xRoot = find_(x);
		typename Entry::Map::iterator yRoot = find_(y);
		if (xRoot == yRoot)
			return;

		// Merge sets
		size_t xRootRank = xRoot->second.rank_, yRootRank = yRoot->second.rank_;
		if (xRootRank < yRootRank)
			xRoot->second.parent_ = yRoot;
		else if (xRootRank > yRootRank)
			yRoot->second.parent_ = xRoot;
		else {
			yRoot->second.parent_ = xRoot;
			xRoot->second.rank_ = xRootRank + 1;
		}
	}

};

/**
 * Disjoint set forest using an STL map data structure underneath
 * Uses rank compression but not union by rank :-(
 * @addtogroup base
 */
template<class KEY>
class DSFMap2 {

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

	/// find the initial Entry
	Entry& find__(const KEY& key) const {
		typename Map::iterator it = entries_.find(key);
		// if key does not exist, create and return itself
		if (it == entries_.end()) {
			it = entries_.insert(it, std::make_pair(key, Entry(key)));
			it->second.makeRoot();
		}
		return it->second;
	}

	/// find the root Entry
	Entry* find_(const KEY& key) const {
		Entry& initial = find__(key);
		// follow parent pointers until we reach set representative
		Entry* parent = initial.parent_;
		while (parent->parent_ != parent)
			parent = parent->parent_; // not yet, recurse!
		initial.parent_ = parent; // path compression
		return parent;
	}

public:
	/// constructor
	DSFMap2() {
	}

	/// find the representative KEY for the set in which key lives
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

};

/**
 * DSFMap version that uses union by rank :-)
 * @addtogroup base
 */
template<class KEY>
class DSFMap3: public DSFMap<KEY> {

	/// We store rank in an STL map as well
	typedef std::map<KEY, size_t> Ranks;
	mutable Ranks rank_;

	size_t rank(const KEY& i) const {
		typename Ranks::const_iterator it = rank_.find(i);
		return it == rank_.end() ? 0 : it->second;
	}

public:
	/// constructor
	DSFMap3() {
	}

	/// Merge two sets
	void merge(const KEY& x, const KEY& y) {

		// straight from http://en.wikipedia.org/wiki/Disjoint-set_data_structure
		KEY xRoot = this->find(x);
		KEY yRoot = this->find(y);
		if (xRoot == yRoot)
			return;

		// Merge sets
		size_t xRootRank = rank(xRoot), yRootRank = rank(yRoot);
		if (xRootRank < yRootRank)
			this->parent_[xRoot] = yRoot;
		else if (xRootRank > yRootRank)
			this->parent_[yRoot] = xRoot;
		else {
			this->parent_[yRoot] = xRoot;
			this->rank_[xRoot] = xRootRank + 1;
		}
	}

};

}
