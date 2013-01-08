/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    Ordering.cpp
 * @author  Richard Roberts
 * @date    Sep 2, 2010
 */

#include "Ordering.h"

#include <string>
#include <iostream>
#include <boost/foreach.hpp>

using namespace std;

namespace gtsam {

/* ************************************************************************* */
Ordering::Ordering(const std::list<Key> & L) {
  int i = 0;
  BOOST_FOREACH( Key s, L )
    insert(s, i++) ;
}

/* ************************************************************************* */
Ordering::Ordering(const Ordering& other) : order_(other.order_), orderingIndex_(other.size()) {
  for(iterator item = order_.begin(); item != order_.end(); ++item)
    orderingIndex_[item->second] = item;
}

/* ************************************************************************* */
Ordering& Ordering::operator=(const Ordering& rhs) {
  order_ = rhs.order_;
  orderingIndex_.resize(rhs.size());
  for(iterator item = order_.begin(); item != order_.end(); ++item)
    orderingIndex_[item->second] = item;
  return *this;
}

/* ************************************************************************* */
void Ordering::permuteInPlace(const Permutation& permutation) {
  gttic(Ordering_permuteInPlace);
  // Allocate new index and permute in map iterators
  OrderingIndex newIndex(permutation.size());
  for(size_t j = 0; j < newIndex.size(); ++j) {
    newIndex[j] = orderingIndex_[permutation[j]]; // Assign the iterator
    newIndex[j]->second = j; // Change the stored index to its permuted value
  }
  // Swap the new index into this Ordering class
  orderingIndex_.swap(newIndex);
}

/* ************************************************************************* */
void Ordering::permuteInPlace(const Permutation& selector, const Permutation& permutation) {
  if(selector.size() != permutation.size())
    throw invalid_argument("Ordering::permuteInPlace (partial permutation version) called with selector and permutation of different sizes.");
  // Create new index the size of the permuted entries
  OrderingIndex newIndex(selector.size());
  // Permute the affected entries into the new index
  for(size_t dstSlot = 0; dstSlot < selector.size(); ++dstSlot)
    newIndex[dstSlot] = orderingIndex_[selector[permutation[dstSlot]]];
  // Put the affected entries back in the new order and fix the indices
  for(size_t slot = 0; slot < selector.size(); ++slot) {
    orderingIndex_[selector[slot]] = newIndex[slot];
    orderingIndex_[selector[slot]]->second = selector[slot];
  }
}

/* ************************************************************************* */
void Ordering::print(const string& str, const KeyFormatter& keyFormatter) const {
  cout << str;
  // Print ordering in index order
  // Print the ordering with varsPerLine ordering entries printed on each line,
  // for compactness.
  static const size_t varsPerLine = 10;
  bool endedOnNewline = false;
  BOOST_FOREACH(const Map::iterator& index_key, orderingIndex_) {
    if(index_key->second % varsPerLine != 0)
      cout << ", ";
    cout << index_key->second<< ":" << keyFormatter(index_key->first);
    if(index_key->second % varsPerLine == varsPerLine - 1) {
      cout << "\n";
      endedOnNewline = true;
    } else {
      endedOnNewline = false;
    }
  }
  if(!endedOnNewline)
    cout << "\n";
  cout.flush();
}

/* ************************************************************************* */
bool Ordering::equals(const Ordering& rhs, double tol) const {
  return order_ == rhs.order_;
}

/* ************************************************************************* */
Ordering::value_type Ordering::pop_back() {
  iterator lastItem = orderingIndex_.back(); // Get the map iterator to the highest-index entry
  value_type removed = *lastItem; // Save the key-index pair to return
  order_.erase(lastItem); // Erase the entry from the map
  orderingIndex_.pop_back(); // Erase the entry from the index
  return removed; // Return the removed item
}

/* ************************************************************************* */
void Unordered::print(const string& s) const {
  cout << s << " (" << size() << "):";
  BOOST_FOREACH(Index key, *this)
    cout << " " << key;
  cout << endl;
}

/* ************************************************************************* */
bool Unordered::equals(const Unordered &other, double tol) const {
  return *this == other;
}

}
