/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    Conditional.h
 * @brief   Base class for conditional densities
 * @author  Frank Dellaert
 */

// \callgraph

#pragma once

#include <iostream>
#include <boost/utility.hpp> // for noncopyable
#include <boost/foreach.hpp>
#include <boost/range/iterator_range.hpp>
#include <boost/serialization/nvp.hpp>
#include <gtsam/base/types.h>
#include <gtsam/base/Testable.h>
#include <gtsam/inference/Factor.h>
#include <gtsam/inference/Permutation.h>

namespace gtsam {

/**
 * Base class for conditional densities
 *
 * We make it noncopyable so we enforce the fact that factors are
 * kept in pointer containers. To be safe, you should make them
 * immutable, i.e., practicing functional programming.
 */
template<typename KEY>
class ConditionalBase: public gtsam::FactorBase<KEY>, boost::noncopyable, public Testable<ConditionalBase<KEY> > {

protected:

  /** The first nFrontal variables are frontal and the rest are parents. */
  size_t nrFrontals_;

public:

  typedef KEY Key;
  typedef ConditionalBase<Key> This;
  typedef gtsam::FactorBase<Key> Factor;
  typedef boost::shared_ptr<This> shared_ptr;
  typedef typename Factor::iterator iterator;
  typedef typename Factor::const_iterator const_iterator;
  typedef boost::iterator_range<const_iterator> Frontals;
  typedef boost::iterator_range<const_iterator> Parents;

  /** Empty Constructor to make serialization possible */
  ConditionalBase() : nrFrontals_(0) {}

  /** No parents */
  ConditionalBase(Key key) : Factor(key), nrFrontals_(1) {}

  /** Single parent */
  ConditionalBase(Key key, Key parent) : Factor(key, parent), nrFrontals_(1) {}

  /** Two parents */
  ConditionalBase(Key key, Key parent1, Key parent2) : Factor(key, parent1, parent2), nrFrontals_(1) {}

  /** Three parents */
  ConditionalBase(Key key, Key parent1, Key parent2, Key parent3) : Factor(key, parent1, parent2, parent3), nrFrontals_(1) {}

  /** Constructor from a frontal variable and a vector of parents */
  ConditionalBase(Key key, const std::vector<Key>& parents) : nrFrontals_(1) {
    Factor::keys_.resize(1 + parents.size());
    *(beginFrontals()) = key;
    std::copy(parents.begin(), parents.end(), beginParents());
  }

  /** Constructor from a frontal variable and an iterator range of parents */
  template<class DERIVED, typename ITERATOR>
  static typename DERIVED::shared_ptr FromRange(Key key, ITERATOR firstParent, ITERATOR lastParent) {
    typename DERIVED::shared_ptr conditional(new DERIVED);
    conditional->nrFrontals_ = 1;
    conditional->keys_.push_back(key);
    std::copy(firstParent, lastParent, back_inserter(conditional->keys_));
    return conditional;
  }

  /** Named constructor from any number of frontal variables and parents */
  template<typename DERIVED, typename ITERATOR>
  static typename DERIVED::shared_ptr FromRange(ITERATOR firstKey, ITERATOR lastKey, size_t nrFrontals) {
    typename DERIVED::shared_ptr conditional(new DERIVED);
    conditional->nrFrontals_ = nrFrontals;
    std::copy(firstKey, lastKey, back_inserter(conditional->keys_));
    return conditional;
  }

  /** check equality */
  template<class DERIVED>
  bool equals(const DERIVED& c, double tol = 1e-9) const {
    return nrFrontals_ == c.nrFrontals_ && Factor::equals(c, tol); }

	/** return the number of frontals */
	size_t nrFrontals() const { return nrFrontals_; }

	/** return the number of parents */
	size_t nrParents() const { return Factor::keys_.size() - nrFrontals_; }

	/** Special accessor when there is only one frontal variable. */
	Key key() const { assert(nrFrontals_==1); return Factor::keys_[0]; }

  /** Iterators over frontal and parent variables. */
  const_iterator beginFrontals() const { return Factor::keys_.begin(); }
  const_iterator endFrontals() const { return Factor::keys_.begin()+nrFrontals_; }
  const_iterator beginParents() const { return Factor::keys_.begin()+nrFrontals_; }
  const_iterator endParents() const { return Factor::keys_.end(); }

  /** Mutable iterators and accessors */
  iterator beginFrontals() { return Factor::keys_.begin(); }
  iterator endFrontals() { return Factor::keys_.begin()+nrFrontals_; }
  iterator beginParents() { return Factor::keys_.begin()+nrFrontals_; }
  iterator endParents() { return Factor::keys_.end(); }
  boost::iterator_range<iterator> frontals() { return boost::make_iterator_range(beginFrontals(), endFrontals()); }
  boost::iterator_range<iterator> parents() { return boost::make_iterator_range(beginParents(), endParents()); }

  /** return a view of the frontal keys */
  Frontals frontals() const {
    return boost::make_iterator_range(beginFrontals(), endFrontals()); }

	/** return a view of the parent keys */
	Parents parents() const {
	  return boost::make_iterator_range(beginParents(), endParents()); }

  /** print */
  void print(const std::string& s = "Conditional") const {
    std::cout << s << " P(";
    BOOST_FOREACH(Key key, frontals()) std::cout << " " << key;
    if (nrParents()>0) std::cout << " |";
    BOOST_FOREACH(Key parent, parents()) std::cout << " " << parent;
    std::cout << ")" << std::endl;
  }

  /** Permute the variables when only separator variables need to be permuted.
   * Returns true if any reordered variables appeared in the separator and
   * false if not.
   */
  bool permuteSeparatorWithInverse(const Permutation& inversePermutation) {
#ifndef NDEBUG
    BOOST_FOREACH(Key key, frontals()) { assert(key == inversePermutation[key]); }
#endif
    bool parentChanged = false;
    BOOST_FOREACH(Key& parent, parents()) {
      Key newParent = inversePermutation[parent];
      if(parent != newParent) {
        parentChanged = true;
        parent = newParent;
      }
    }
    return parentChanged;
  }

  /**
   * Permutes the Conditional, but for efficiency requires the permutation
   * to already be inverted.
   */
  void permuteWithInverse(const Permutation& inversePermutation) {
    // The permutation may not move the separators into the frontals
#ifndef NDEBUG
    BOOST_FOREACH(const Key frontal, this->frontals()) {
      BOOST_FOREACH(const Key separator, this->parents()) {
        assert(inversePermutation[frontal] < inversePermutation[separator]);
      }
    }
#endif
    Factor::permuteWithInverse(inversePermutation);
  }

protected:
  /** Debugging invariant that the keys should be in order, including that the
   * conditioned variable is numbered lower than the parents.
   */
  void assertInvariants() const;

private:
	/** Serialization function */
	friend class boost::serialization::access;
	template<class ARCHIVE>
	void serialize(ARCHIVE & ar, const unsigned int version) {
    ar & BOOST_SERIALIZATION_NVP(nrFrontals_);
	}
};

}
