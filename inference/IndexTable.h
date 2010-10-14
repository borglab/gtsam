/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/*
 * IndexTable.h
 *
 *  Created on: Jan 21, 2010
 *  @Author: Frank Dellaert
 */

#pragma once

#include <map>
#include <boost/foreach.hpp> // TODO should not be in header
#include <gtsam/base/Testable.h>

namespace gtsam {

	/**
	 * An IndexTable maps from key to size_t index and back
	 * most commonly used templated on Symbol with orderings
	 */
	template<class Key>
	class IndexTable: public std::vector<Key>, public Testable<IndexTable<Key> > {
	private:

		/* map back from key to size_t */
		typedef typename std::map<Key, size_t> Map;
		Map key2index_;

	public:

		/* bake ordering into IndexTable */
		IndexTable(const std::list<Key>& ordering) {
			size_t i = 0;
			BOOST_FOREACH(const Key& key,ordering){
				this->push_back(key);
				key2index_.insert(make_pair(key,i++));
			}
		}

		// Testable
		virtual void print(const std::string& s="") const {
			std::cout << "IndexTable " << s << ":";
			BOOST_FOREACH(Key key,*this) std::cout << (std::string)key << " ";
		}
		virtual bool equals(const IndexTable<Key>& expected, double tol) const {
			return key2index_==expected.key2index_; // TODO, sanity check
		}

		/** Key to index by parentheses ! */
		size_t operator()(const Key& key) const {
			typename Map::const_iterator it = key2index_.find(key);
		  if (it==key2index_.end())
		    throw(std::invalid_argument("IndexTable::[] invalid key"));
		  return it->second;
		}

		/* Index to Key is provided by base class operator[] */
	};

}
