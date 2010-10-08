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
class Conditional: boost::noncopyable, public Testable<Conditional> {

protected:
  /** Conditional just uses an internal Factor for storage (a conditional is
   * really just a special factor anyway, but we do this instead of inherit
   * to prevent "diamond" inheritance with GaussianFactor and
   * GaussianConditional.
   */
  Factor factor_;

  /** The first nFrontal variables are frontal and the rest are parents. */
  size_t nFrontal_;

  ValueWithDefault<bool, true> permuted_;

public:

  /** convenience typename for a shared pointer to this class */
  typedef gtsam::Factor FactorType;
  typedef boost::shared_ptr<Conditional> shared_ptr;
  typedef Factor::iterator iterator;
  typedef Factor::const_iterator const_iterator;
  typedef boost::iterator_range<const_iterator> Frontals;
  typedef boost::iterator_range<const_iterator> Parents;

  /** Empty Constructor to make serialization possible */
  Conditional(){}

  /** No parents */
  Conditional(varid_t key) : factor_(key), nFrontal_(1) {}

  /** Single parent */
  Conditional(varid_t key, varid_t parent) : factor_(key, parent), nFrontal_(1) {}

  /** Two parents */
  Conditional(varid_t key, varid_t parent1, varid_t parent2) : factor_(key, parent1, parent2), nFrontal_(1) {}

  /** Three parents */
  Conditional(varid_t key, varid_t parent1, varid_t parent2, varid_t parent3) : factor_(key, parent1, parent2, parent3), nFrontal_(1) {}

  /** Constructor from a frontal variable and a vector of parents */
  Conditional(varid_t key, const std::vector<varid_t>& parents) : nFrontal_(1) {
    factor_.keys_.resize(1+parents.size());
    *(beginFrontals()) = key;
    std::copy(parents.begin(), parents.end(), beginParents()); }

  /** Constructor from any number of frontal variables and parents */
  template<typename Iterator>
  Conditional(Iterator firstKey, Iterator lastKey, size_t nFrontals) : factor_(firstKey, lastKey), nFrontal_(nFrontals) {}

	/** Special accessor when there is only one frontal variable. */
	varid_t key() const { assert(nFrontal_==1); return factor_.keys_[0]; }

  /**
   * Permutes the Conditional, but for efficiency requires the permutation
   * to already be inverted.
   */
  void permuteWithInverse(const Permutation& inversePermutation) {
    factor_.permuteWithInverse(inversePermutation); }

  /** Permute the variables when only separator variables need to be permuted.
   * Returns true if any reordered variables appeared in the separator and
   * false if not.
   */
  bool permuteSeparatorWithInverse(const Permutation& inversePermutation) {
#ifndef NDEBUG
    BOOST_FOREACH(varid_t key, frontals()) { assert(key == inversePermutation[key]); }
#endif
    bool parentChanged = false;
    BOOST_FOREACH(varid_t& parent, parents()) {
      varid_t newParent = inversePermutation[parent];
      if(parent != newParent) {
        parentChanged = true;
        parent = newParent;
      }
    }
    return parentChanged;
  }

  /** return a const reference to all keys */
  const std::vector<varid_t>& keys() const { return factor_.keys(); }

  /** return a view of the frontal keys */
  Frontals frontals() const {
    return boost::make_iterator_range(beginFrontals(), endFrontals()); }

	/** return a view of the parent keys */
	Parents parents() const {
	  return boost::make_iterator_range(beginParents(), endParents()); }

	/** return the number of frontals */
	size_t nrFrontals() const { return nFrontal_; }

	/** return the number of parents */
	size_t nrParents() const { return factor_.keys_.size() - nFrontal_; }

  /** print */
  void print(const std::string& s = "Conditional") const {
    std::cout << s << " P(";
    BOOST_FOREACH(varid_t key, frontals()) std::cout << " " << key;
    if (nrParents()>0) std::cout << " |";
    BOOST_FOREACH(varid_t parent, parents()) std::cout << " " << parent;
    std::cout << ")" << std::endl;
  }

  /** check equality */
  bool equals(const Conditional& c, double tol = 1e-9) const {
    return nFrontal_ == c.nFrontal_ && factor_.equals(c.factor_, tol); }

  /** Iterators over frontal and parent variables. */
  const_iterator beginFrontals() const { return factor_.keys_.begin(); }
  const_iterator endFrontals() const { return factor_.keys_.begin()+nFrontal_; }
  const_iterator beginParents() const { return factor_.keys_.begin()+nFrontal_; }
  const_iterator endParents() const { return factor_.keys_.end(); }

  /** Mutable iterators and accessors */
  iterator beginFrontals() { return factor_.keys_.begin(); }
  iterator endFrontals() { return factor_.keys_.begin()+nFrontal_; }
  iterator beginParents() { return factor_.keys_.begin()+nFrontal_; }
  iterator endParents() { return factor_.keys_.end(); }
  boost::iterator_range<iterator> frontals() { return boost::make_iterator_range(beginFrontals(), endFrontals()); }
  boost::iterator_range<iterator> parents() { return boost::make_iterator_range(beginParents(), endParents()); }

protected:
  /** Debugging invariant that the keys should be in order, including that the
   * conditioned variable is numbered lower than the parents.
   */
  void checkSorted() const;

  friend class Factor;

private:
	/** Serialization function */
	friend class boost::serialization::access;
	template<class Archive>
	void serialize(Archive & ar, const unsigned int version) {
		ar & BOOST_SERIALIZATION_NVP(factor_);
    ar & BOOST_SERIALIZATION_NVP(nFrontal_);
	}
};

}
