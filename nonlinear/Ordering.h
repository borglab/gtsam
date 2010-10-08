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

#include <boost/shared_ptr.hpp>
#include <boost/assign/list_inserter.hpp>
#include <boost/pool/pool_alloc.hpp>

namespace gtsam {

class Ordering : Testable<Ordering> {
protected:
  typedef boost::fast_pool_allocator<std::pair<const Symbol, varid_t> > Allocator;
  typedef std::map<Symbol, varid_t, std::less<Symbol>, Allocator> Map;
  Map order_;
  varid_t nVars_;

public:

  typedef boost::shared_ptr<Ordering> shared_ptr;

  typedef std::pair<const Symbol, varid_t> value_type;
  typedef Map::iterator iterator;
  typedef Map::const_iterator const_iterator;

  Ordering() : nVars_(0) {}

  /** One greater than the maximum ordering index. */
  varid_t nVars() const { return nVars_; }

  /** The number of variables in this ordering. */
  varid_t size() const { return order_.size(); }

  iterator begin() { return order_.begin(); }
  const_iterator begin() const { return order_.begin(); }
  iterator end() { return order_.end(); }
  const_iterator end() const { return order_.end(); }

  varid_t& at(const Symbol& key) { return operator[](key); }
  varid_t at(const Symbol& key) const { return operator[](key); }
  varid_t& operator[](const Symbol& key) {
    iterator i=order_.find(key); assert(i != order_.end()); return i->second; }
  varid_t operator[](const Symbol& key) const {
    const_iterator i=order_.find(key); assert(i != order_.end()); return i->second; }

  iterator insert(const value_type& key_order) {
    std::pair<iterator,bool> it_ok(tryInsert(key_order));
    assert(it_ok.second);
    return it_ok.first; }
  iterator insert(const Symbol& key, varid_t order) { return insert(std::make_pair(key,order)); }
  std::pair<iterator,bool> tryInsert(const value_type& key_order) {
    std::pair<iterator,bool> it_ok(order_.insert(key_order));
    if(key_order.second+1 > nVars_)  nVars_ = key_order.second+1;
    return it_ok; }
  std::pair<iterator,bool> tryInsert(const Symbol& key, varid_t order) { return tryInsert(std::make_pair(key,order)); }

  varid_t push_back(const Symbol& key) { return insert(std::make_pair(key, nVars_))->second; }

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

};

/**
 * @class Unordered
 * @brief a set of unordered indice
 */
class Unordered: public std::set<varid_t>, public Testable<Unordered> {
public:
  /** Default constructor creates empty ordering */
  Unordered() { }

  /** Create from a single symbol */
  Unordered(varid_t key) { insert(key); }

  /** Copy constructor */
  Unordered(const std::set<varid_t>& keys_in) : std::set<varid_t>(keys_in) {}

  /** whether a key exists */
  bool exists(const varid_t& key) { return find(key) != end(); }

  // Testable
  void print(const std::string& s = "Unordered") const;
  bool equals(const Unordered &t, double tol=0) const;
};

}

