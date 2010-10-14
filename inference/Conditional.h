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
class Conditional: public Factor, boost::noncopyable, public Testable<Conditional> {

protected:

  /** The first nFrontal variables are frontal and the rest are parents. */
  size_t nrFrontals_;

public:

  /** convenience typename for a shared pointer to this class */
  typedef gtsam::Factor FactorType;
  typedef boost::shared_ptr<Conditional> shared_ptr;
  typedef Factor::iterator iterator;
  typedef Factor::const_iterator const_iterator;
  typedef boost::iterator_range<const_iterator> Frontals;
  typedef boost::iterator_range<const_iterator> Parents;

  /** Empty Constructor to make serialization possible */
  Conditional() : nrFrontals_(0) {}

  /** No parents */
  Conditional(Index key) : Factor(key), nrFrontals_(1) {}

  /** Single parent */
  Conditional(Index key, Index parent) : Factor(key, parent), nrFrontals_(1) {}

  /** Two parents */
  Conditional(Index key, Index parent1, Index parent2) : Factor(key, parent1, parent2), nrFrontals_(1) {}

  /** Three parents */
  Conditional(Index key, Index parent1, Index parent2, Index parent3) : Factor(key, parent1, parent2, parent3), nrFrontals_(1) {}

  /** Constructor from a frontal variable and a vector of parents */
  Conditional(Index key, const std::vector<Index>& parents) : nrFrontals_(1) {
    keys_.resize(1 + parents.size());
    *(beginFrontals()) = key;
    std::copy(parents.begin(), parents.end(), beginParents());
  }

  /** Constructor from a frontal variable and an iterator range of parents */
  template<typename Iterator>
  static Conditional::shared_ptr FromRange(Index key, Iterator firstParent, Iterator lastParent) {
    Conditional::shared_ptr conditional(new Conditional);
    conditional->nrFrontals_ = 1;
    conditional->keys_.push_back(key);
    std::copy(firstParent, lastParent, back_inserter(conditional->keys_));
    return conditional;
  }

  /** Named constructor from any number of frontal variables and parents */
  template<typename Iterator>
  static Conditional::shared_ptr FromRange(Iterator firstKey, Iterator lastKey, size_t nrFrontals) {
    Conditional::shared_ptr conditional(new Conditional);
    conditional->nrFrontals_ = nrFrontals;
    std::copy(firstKey, lastKey, back_inserter(conditional->keys_));
    return conditional;
  }

  /** check equality */
  bool equals(const Conditional& c, double tol = 1e-9) const {
    return nrFrontals_ == c.nrFrontals_ && Factor::equals(c, tol); }

	/** return the number of frontals */
	size_t nrFrontals() const { return nrFrontals_; }

	/** return the number of parents */
	size_t nrParents() const { return keys_.size() - nrFrontals_; }

	/** Special accessor when there is only one frontal variable. */
	Index key() const { assert(nrFrontals_==1); return keys_[0]; }

  /** Iterators over frontal and parent variables. */
  const_iterator beginFrontals() const { return keys_.begin(); }
  const_iterator endFrontals() const { return keys_.begin()+nrFrontals_; }
  const_iterator beginParents() const { return keys_.begin()+nrFrontals_; }
  const_iterator endParents() const { return keys_.end(); }

  /** Mutable iterators and accessors */
  iterator beginFrontals() { return keys_.begin(); }
  iterator endFrontals() { return keys_.begin()+nrFrontals_; }
  iterator beginParents() { return keys_.begin()+nrFrontals_; }
  iterator endParents() { return keys_.end(); }
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
    BOOST_FOREACH(Index key, frontals()) std::cout << " " << key;
    if (nrParents()>0) std::cout << " |";
    BOOST_FOREACH(Index parent, parents()) std::cout << " " << parent;
    std::cout << ")" << std::endl;
  }

  /** Permute the variables when only separator variables need to be permuted.
   * Returns true if any reordered variables appeared in the separator and
   * false if not.
   */
  bool permuteSeparatorWithInverse(const Permutation& inversePermutation) {
#ifndef NDEBUG
    BOOST_FOREACH(Index key, frontals()) { assert(key == inversePermutation[key]); }
#endif
    bool parentChanged = false;
    BOOST_FOREACH(Index& parent, parents()) {
      Index newParent = inversePermutation[parent];
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
    BOOST_FOREACH(const Index frontal, this->frontals()) {
      BOOST_FOREACH(const Index separator, this->parents()) {
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

  friend class Factor;

private:
	/** Serialization function */
	friend class boost::serialization::access;
	template<class Archive>
	void serialize(Archive & ar, const unsigned int version) {
    ar & BOOST_SERIALIZATION_NVP(nrFrontals_);
	}
};

}
