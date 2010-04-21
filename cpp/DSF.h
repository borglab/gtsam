/*
 * DSF.h
 *
 *  Created on: Mar 26, 2010
 *  Author: nikai
 *  Description: An implementation of Disjoint set forests (see CLR page 446 and up)
 *  						 Quoting from CLR: A disjoint-set data structure maintains a collection
 * 							 S = {S_1,S_2,...} of disjoint dynamic sets. Each set is identified by
 * 							 a representative, which is some member of the set.
 */

#pragma once

#include <iostream>
#include <list>
#include <set>
#include <map>
#include <boost/foreach.hpp>
#include "BTree.h"

namespace gtsam {

	class Symbol;

	template <class Key>
	class DSF : protected BTree<Key, Key> {

	public:
		typedef Key Label; // label can be different from key, but for now they are same
		typedef DSF<Key> Self;
		typedef std::set<Key> Set;
		typedef BTree<Key, Label> Tree;
		typedef std::pair<Key, Label> KeyLabel;

		// constructor
		DSF() : Tree() { }

		// constructor
		DSF(const Tree& tree) : Tree(tree) {}

		// constructor with a list of unconnected keys
		DSF(const std::list<Key>& keys) : Tree() { BOOST_FOREACH(const Key& key, keys) *this = this->add(key, key); }

		// create a new singleton, does nothing if already exists
		Self makeSet(const Key& key) const { if (mem(key)) return *this; else return this->add(key, key); }

		// find the label of the set in which {key} lives
		Label findSet(const Key& key) const {
			Key parent = this->find(key);
			return parent == key ? key : findSet(parent); }

		// return a new DSF where x and y are in the same set. Kai: the caml implementation is not const, and I followed
		Self makeUnion(const Key& key1, const Key& key2) { return this->add(findSet_(key2), findSet_(key1));	}

		// create a new singleton with two connected keys
		Self makePair(const Key& key1, const Key& key2) const { return makeSet(key1).makeSet(key2).makeUnion(key1, key2); }

		// create a new singleton with a list of fully connected keys
		Self makeList(const std::list<Key>& keys) const {
			Self t = *this;
			BOOST_FOREACH(const Key& key, keys)
				t = t.makePair(key, keys.front());
			return t;
		}

		// return a dsf in which all find_set operations will be O(1) due to path compression.
		DSF flatten() const {
			DSF t = *this;
			BOOST_FOREACH(const KeyLabel& pair, (Tree)t)
				t.findSet_(pair.first);
			return t;
		}

		// maps f over all keys, must be invertible
		DSF map(boost::function<Key(const Key&)> func) const {
			DSF t;
			BOOST_FOREACH(const KeyLabel& pair, (Tree)*this)
				t = t.add(func(pair.first), func(pair.second));
			return t;
		}

		// return the number of sets
		size_t numSets() const {
			size_t num = 0;
			BOOST_FOREACH(const KeyLabel& pair, (Tree)*this)
				if (pair.first == pair.second) num++;
			return num;
		}

		// return the numer of keys
		size_t size() const { return Tree::size(); }

		// return all sets, i.e. a partition of all elements
		std::map<Label, Set> sets() const {
			std::map<Label, Set> sets;
			BOOST_FOREACH(const KeyLabel& pair, (Tree)*this)
				sets[findSet(pair.second)].insert(pair.first);
			return sets;
		}

		// return a partition of the given elements {keys}
		std::map<Label, Set> partition(const std::list<Key>& keys) const {
			std::map<Label, Set> partitions;
			BOOST_FOREACH(const Key& key, keys)
				partitions[findSet(key)].insert(key);
			return partitions;
		}

		// get the nodes in the given tree
		Set set(const Label& label) {
			Set set;
			BOOST_FOREACH(const KeyLabel& pair, (Tree)*this)
				if (pair.second==label) set.insert(pair.first);
			return set;
		}

		/** equality */
		bool operator==(const Self& t) const { return (Tree)*this == (Tree)t;	}

		/** inequality */
		bool operator!=(const Self& t) const { return (Tree)*this != (Tree)t;	}

		// print the object
		void print(const std::string& name = "DSF") const {
			std::cout << name << std::endl;
			BOOST_FOREACH(const KeyLabel& pair, (Tree)*this)
				std::cout << (std::string)pair.first << " " << (std::string)pair.second << std::endl;
		}

	protected:

		/**
		 * same as findSet except with path compression: After we have traversed the path to
		 * the root, each parent pointer is made to directly point to it
		 */
		Key findSet_(const Key& key) {
			Key parent = this->find(key);
			if (parent == key)
				return parent;
			else {
				Key label = findSet_(parent);
				*this = this->add(key, label);
				return label;
			}
		}

	};

	// shortcuts
	typedef DSF<int> DSFInt;
	typedef DSF<Symbol> DSFSymbol;

} // namespace gtsam
