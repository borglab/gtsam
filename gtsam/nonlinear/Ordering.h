/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    Ordering.h
 * @brief   
 * @author  Richard Roberts
 * @created Sep 2, 2010
 */

#pragma once

#include <map>
#include <gtsam/base/Testable.h>
#include <gtsam/nonlinear/Key.h>
#include <gtsam/inference/inference.h>

#include <boost/foreach.hpp>
#include <boost/assign/list_inserter.hpp>
#include <boost/pool/pool_alloc.hpp>

namespace gtsam {

/**
 * An ordering is a map from symbols (non-typed keys) to integer indices
 */
class Ordering : Testable<Ordering> {
protected:
  typedef boost::fast_pool_allocator<std::pair<const Symbol, Index> > Allocator;
  typedef std::map<Symbol, Index, std::less<Symbol>, Allocator> Map;
  Map order_;
  Index nVars_;

public:

  typedef boost::shared_ptr<Ordering> shared_ptr;

  typedef std::pair<const Symbol, Index> value_type;
  typedef Map::iterator iterator;
  typedef Map::const_iterator const_iterator;

  Ordering() : nVars_(0) {}
  Ordering(const std::list<Symbol> & L) ;

  /** One greater than the maximum ordering index. */
  Index nVars() const { return nVars_; }

  /** The number of variables in this ordering. */
  Index size() const { return order_.size(); }

  iterator begin() { return order_.begin(); }
  const_iterator begin() const { return order_.begin(); }
  iterator end() { return order_.end(); }
  const_iterator end() const { return order_.end(); }

  // access to integer indices

  Index& at(const Symbol& key) { return operator[](key); }
  Index at(const Symbol& key) const { return operator[](key); }
  bool tryAt(const Symbol& key, Index& index) const {
    const_iterator i = order_.find(key);
    if(i != order_.end()) {
      index = i->second;
      return true;
    } else
      return false;
  }
  Index& operator[](const Symbol& key) {
    iterator i=order_.find(key); assert(i != order_.end()); return i->second; }
  Index operator[](const Symbol& key) const {
    const_iterator i=order_.find(key); assert(i != order_.end()); return i->second; }

  // adding symbols

  /**
   * Attempts to insert a symbol/order pair with same semantics as stl::Map::insert(),
   * i.e., returns a pair of iterator and success (false if already present)
   */
  std::pair<iterator,bool> tryInsert(const value_type& key_order) {
  	std::pair<iterator,bool> it_ok(order_.insert(key_order));
  	if(it_ok.second == true && key_order.second+1 > nVars_)
  		nVars_ = key_order.second+1;
  	return it_ok;
  }
  std::pair<iterator,bool> tryInsert(const Symbol& key, Index order) { return tryInsert(std::make_pair(key,order)); }

  /** Try insert, but will fail if the key is already present */
  iterator insert(const value_type& key_order) {
  	std::pair<iterator,bool> it_ok(tryInsert(key_order));
  	assert(it_ok.second);
  	return it_ok.first;
  }
  iterator insert(const Symbol& key, Index order) { return insert(std::make_pair(key,order)); }


  bool exists(const Symbol& key) const { return order_.count(key); }

  Index push_back(const Symbol& key) { return insert(std::make_pair(key, nVars_))->second; }

  /** remove the last symbol/index pair from the ordering */
  value_type pop_back();

  /**
   * += operator allows statements like 'ordering += x0,x1,x2,x3;', which are
   * very useful for unit tests.  This functionality is courtesy of
   * boost::assign.
   */
  inline boost::assign::list_inserter<boost::assign_detail::call_push_back<Ordering>, Symbol>
  operator+=(const Symbol& key) {
    return boost::assign::make_list_inserter(boost::assign_detail::call_push_back<Ordering>(*this))(key); }

  /**
   * Reorder the variables with a permutation.  This is typically used
   * internally, permuting an initial key-sorted ordering into a fill-reducing
   * ordering.
   */
  void permuteWithInverse(const Permutation& inversePermutation);

  /** print (from Testable) for testing and debugging */
  void print(const std::string& str = "Ordering:") const;

  /** equals (from Testable) for testing and debugging */
  bool equals(const Ordering& rhs, double tol = 0.0) const;

private:

	/** Serialization function */
	friend class boost::serialization::access;
	template<class ARCHIVE>
	void serialize(ARCHIVE & ar, const unsigned int version) {
		ar & BOOST_SERIALIZATION_NVP(order_);
		ar & BOOST_SERIALIZATION_NVP(nVars_);
	}
};

/**
 * @class Unordered
 * @brief a set of unordered indices
 */
class Unordered: public std::set<Index>, public Testable<Unordered> {
public:
  /** Default constructor creates empty ordering */
  Unordered() { }

  /** Create from a single symbol */
  Unordered(Index key) { insert(key); }

  /** Copy constructor */
  Unordered(const std::set<Index>& keys_in) : std::set<Index>(keys_in) {}

  /** whether a key exists */
  bool exists(const Index& key) { return find(key) != end(); }

  // Testable
  void print(const std::string& s = "Unordered") const;
  bool equals(const Unordered &t, double tol=0) const;
};

}

