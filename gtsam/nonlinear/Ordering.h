/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    Ordering.h
 * @author  Richard Roberts
 * @date    Sep 2, 2010
 */

#pragma once

#include <gtsam/nonlinear/Key.h>
#include <gtsam/inference/inference.h>
#include <gtsam/base/FastMap.h>

#include <boost/foreach.hpp>
#include <boost/assign/list_inserter.hpp>
#include <boost/pool/pool_alloc.hpp>
#include <vector>

namespace gtsam {

/**
 * An ordering is a map from symbols (non-typed keys) to integer indices
 * \nosubgrouping
 */
class GTSAM_EXPORT Ordering {
protected:
  typedef FastMap<Key, Index> Map;
  typedef std::vector<Map::iterator> OrderingIndex;
  Map order_;
  OrderingIndex orderingIndex_;

public:

  typedef boost::shared_ptr<Ordering> shared_ptr;

  typedef std::pair<const Key, Index> value_type;
  typedef Map::iterator iterator;
  typedef Map::const_iterator const_iterator;

  /// @name Standard Constructors
  /// @{

  /// Default constructor for empty ordering
  Ordering() {}

  /// Copy constructor
  Ordering(const Ordering& other);

  /// Construct from list, assigns order indices sequentially to list items.
  Ordering(const std::list<Key> & L);

  /// Assignment operator
  Ordering& operator=(const Ordering& rhs);

  /// @}
  /// @name Standard Interface
  /// @{

  /** The actual number of variables in this ordering, i.e. not including missing indices in the count.  See also nVars(). */
  Index size() const { return orderingIndex_.size(); }

  const_iterator begin() const { return order_.begin(); } /**< Iterator in order of sorted symbols, not in elimination/index order! */
  const_iterator end() const { return order_.end(); } /**< Iterator in order of sorted symbols, not in elimination/index order! */

  Index at(Key key) const { return operator[](key); } ///< Synonym for operator[](Key) const
  Key key(Index index) const {
    if(index >= orderingIndex_.size())
      throw std::out_of_range("Attempting to access out-of-range index in Ordering");
    else
      return orderingIndex_[index]->first; }

  /** Assigns the ordering index of the requested \c key into \c index if the symbol
   * is present in the ordering, otherwise does not modify \c index.  The
   * return value indicates whether the symbol is in fact present in the
   * ordering.
   * @param key The key whose index you request
   * @param [out] index Reference into which to write the index of the requested key, if the key is present.
   * @return true if the key is present and \c index was modified, false otherwise.
   */
  bool tryAt(Key key, Index& index) const {
    const_iterator i = order_.find(key);
    if(i != order_.end()) {
      index = i->second;
      return true;
    } else
      return false; }

  /// Access the index for the requested key, throws std::out_of_range if the
  /// key is not present in the ordering (note that this differs from the
  /// behavior of std::map)
  Index& operator[](Key key) {
    iterator i=order_.find(key);
    if(i == order_.end())  throw std::out_of_range(
        std::string("Attempting to access a key from an ordering that does not contain that key:") + DefaultKeyFormatter(key));
    else                   return i->second; }

  /// Access the index for the requested key, throws std::out_of_range if the
  /// key is not present in the ordering (note that this differs from the
  /// behavior of std::map)
  Index operator[](Key key) const {
    const_iterator i=order_.find(key);
    if(i == order_.end())  throw std::out_of_range(
        std::string("Attempting to access a key from an ordering that does not contain that key:") + DefaultKeyFormatter(key));
    else                   return i->second; }

  /** Returns an iterator pointing to the symbol/index pair with the requested,
   * or the end iterator if it does not exist.
   *
   * @return An iterator pointing to the symbol/index pair with the requested,
   * or the end iterator if it does not exist.
   */
  iterator find(Key key) { return order_.find(key); }

  /** Returns an iterator pointing to the symbol/index pair with the requested,
   * or the end iterator if it does not exist.
   *
   * @return An iterator pointing to the symbol/index pair with the requested,
   * or the end iterator if it does not exist.
   */
  const_iterator find(Key key) const { return order_.find(key); }

  /** Insert a key-index pair, but will fail if the key is already present */
  iterator insert(const value_type& key_order) {
    std::pair<iterator,bool> it_ok(order_.insert(key_order));
    if(it_ok.second) {
      if(key_order.second >= orderingIndex_.size())
        orderingIndex_.resize(key_order.second + 1);
      orderingIndex_[key_order.second] = it_ok.first;
      return it_ok.first;
    } else
      throw std::invalid_argument(std::string("Attempting to insert a key into an ordering that already contains that key")); }

  /// Test if the key exists in the ordering.
  bool exists(Key key) const { return order_.count(key) > 0; }

  /** Insert a key-index pair, but will fail if the key is already present */
  iterator insert(Key key, Index order) { return insert(std::make_pair(key,order)); }

  /// Adds a new key to the ordering with an index of one greater than the current highest index.
  Index push_back(Key key) { return insert(std::make_pair(key, orderingIndex_.size()))->second; }

  /// @}
  /// @name Advanced Interface
  /// @{

  /**
   * Iterator in order of sorted symbols, not in elimination/index order!
   */
  iterator begin() { return order_.begin(); }

  /**
   * Iterator in order of sorted symbols, not in elimination/index order!
   */
  iterator end() { return order_.end(); }

  /** Remove the last (last-ordered, not highest-sorting key) symbol/index pair
   * from the ordering (this version is \f$ O(n) \f$, use it when you do not
   * know the last-ordered key).
   *
   * If you already know the last-ordered symbol, call popback(Key)
   * that accepts this symbol as an argument.
   *
   * @return The symbol and index that were removed.
   */
  value_type pop_back();

  /**
   * += operator allows statements like 'ordering += x0,x1,x2,x3;', which are
   * very useful for unit tests.  This functionality is courtesy of
   * boost::assign.
   */
  inline boost::assign::list_inserter<boost::assign_detail::call_push_back<Ordering>, Key>
  operator+=(Key key) {
    return boost::assign::make_list_inserter(boost::assign_detail::call_push_back<Ordering>(*this))(key); }

  /**
   * Reorder the variables with a permutation.  This is typically used
   * internally, permuting an initial key-sorted ordering into a fill-reducing
   * ordering.
   */
  void permuteInPlace(const Permutation& permutation);

  void permuteInPlace(const Permutation& selector, const Permutation& permutation);

  /// Synonym for operator[](Key)
  Index& at(Key key) { return operator[](key); }

  /// @}
  /// @name Testable
  /// @{

  /** print (from Testable) for testing and debugging */
  void print(const std::string& str = "Ordering:\n", const KeyFormatter& keyFormatter = DefaultKeyFormatter) const;

  /** equals (from Testable) for testing and debugging */
  bool equals(const Ordering& rhs, double tol = 0.0) const;

  /// @}

private:

  /** Serialization function */
  friend class boost::serialization::access;
  template<class ARCHIVE>
  void save(ARCHIVE & ar, const unsigned int version) const
  {
    ar & BOOST_SERIALIZATION_NVP(order_);
    size_t size_ = orderingIndex_.size(); // Save only the size but not the iterators
    ar & BOOST_SERIALIZATION_NVP(size_);
  }
  template<class ARCHIVE>
  void load(ARCHIVE & ar, const unsigned int version)
  {
    ar & BOOST_SERIALIZATION_NVP(order_);
    size_t size_;
    ar & BOOST_SERIALIZATION_NVP(size_);
    orderingIndex_.resize(size_);
    for(iterator item = order_.begin(); item != order_.end(); ++item)
      orderingIndex_[item->second] = item; // Assign the iterators
  }
  BOOST_SERIALIZATION_SPLIT_MEMBER()
}; // \class Ordering

/**
 * @class Unordered
 * @brief a set of unordered indices
 */
class Unordered: public std::set<Index> {
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
  GTSAM_EXPORT void print(const std::string& s = "Unordered") const;
  GTSAM_EXPORT bool equals(const Unordered &t, double tol=0) const;
};

// Create an index formatter that looks up the Key in an inverse ordering, then
// formats the key using the provided key formatter, used in saveGraph.
class GTSAM_EXPORT OrderingIndexFormatter {
private:
  Ordering ordering_;
  KeyFormatter keyFormatter_;
public:
  OrderingIndexFormatter(const Ordering& ordering, const KeyFormatter& keyFormatter) :
      ordering_(ordering), keyFormatter_(keyFormatter) {}
  std::string operator()(Index index) {
    return keyFormatter_(ordering_.key(index)); }
};

/// Version of orderingIndexFormatter using multi-robot formatter
struct GTSAM_EXPORT MultiRobotLinearFormatter : gtsam::OrderingIndexFormatter {
  MultiRobotLinearFormatter(const gtsam::Ordering& ordering)
  : gtsam::OrderingIndexFormatter(ordering, MultiRobotKeyFormatter) {}
};

} // \namespace gtsam

