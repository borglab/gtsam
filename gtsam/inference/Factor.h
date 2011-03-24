/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    Factor.h
 * @brief   The base class for all factors
 * @author  Kai Ni
 * @author  Frank Dellaert
 * @author  Richard Roberts
 */

// \callgraph

#pragma once

#include <map>
#include <set>
#include <vector>
#include <boost/utility.hpp> // for noncopyable
#include <boost/serialization/nvp.hpp>
#include <boost/foreach.hpp>
#include <gtsam/base/Testable.h>
#include <gtsam/base/FastMap.h>

namespace gtsam {

template<class KEY> class Conditional;

/**
 * This is the base class for all factor types.  It is templated on a KEY type,
 * though currently only IndexFactor and IndexConditional derive from this
 * class, using Index keys.  This class does not store any data other than its
 * keys.  Derived classes store data such as matrices and probability tables.
 *
 * todo:  Make NonlinearFactor derive from this too, which requires moving
 * Combine, eliminate*, permute* and the sorted key invariant to IndexFactor.
 *
 * Note that derived classes *must* redefine the ConditionalType and shared_ptr
 * typedefs to refer to the associated conditional and shared_ptr types of the
 * derived class.  See IndexFactor, JacobianFactor, etc. for examples.
 */
template<typename KEY>
class Factor : public Testable<Factor<KEY> > {

public:

  typedef KEY Key;
  typedef Factor<Key> This;

  /**
   * Typedef to the conditional type obtained by eliminating this factor.
   * Derived classes must redefine this.
   */
  typedef Conditional<Key> ConditionalType;

  /** A shared_ptr to this class.  Derived classes must redefine this. */
  typedef boost::shared_ptr<Factor> shared_ptr;

  /** Iterator over keys */
  typedef typename std::vector<Key>::iterator iterator;

  /** Const iterator over keys */
  typedef typename std::vector<Key>::const_iterator const_iterator;

protected:

  // The keys involved in this factor
  std::vector<Key> keys_;

  // Internal check to make sure keys are unique.  If NDEBUG is defined, this
  // is empty and optimized out.
  void assertInvariants() const;

public:

  /** Copy constructor */
  Factor(const This& f);

  /** Construct from derived type */
  Factor(const ConditionalType& c);

  /** Constructor from a collection of keys */
  template<class KEYITERATOR> Factor(KEYITERATOR beginKey, KEYITERATOR endKey) :
        keys_(beginKey, endKey) { assertInvariants(); }

  /** Default constructor for I/O */
  Factor() {}

  /** Construct unary factor */
  Factor(Key key) : keys_(1) {
    keys_[0] = key; assertInvariants(); }

  /** Construct binary factor */
  Factor(Key key1, Key key2) : keys_(2) {
    keys_[0] = key1; keys_[1] = key2; assertInvariants(); }

  /** Construct ternary factor */
  Factor(Key key1, Key key2, Key key3) : keys_(3) {
    keys_[0] = key1; keys_[1] = key2; keys_[2] = key3; assertInvariants(); }

  /** Construct 4-way factor */
  Factor(Key key1, Key key2, Key key3, Key key4) : keys_(4) {
    keys_[0] = key1; keys_[1] = key2; keys_[2] = key3; keys_[3] = key4; assertInvariants(); }

  /** Construct n-way factor */
  Factor(const std::set<Key>& keys) {
  	BOOST_FOREACH(const Key& key, keys)
  			keys_.push_back(key);
  	assertInvariants(); }

#ifdef TRACK_ELIMINATE
  /**
   * eliminate the first variable involved in this factor
   * @return a conditional on the eliminated variable
   */
  template<class CONDITIONAL>
  typename CONDITIONAL::shared_ptr eliminateFirst();

  /**
   * eliminate the first nrFrontals frontal variables.
   */
  template<class CONDITIONAL>
  typename BayesNet<CONDITIONAL>::shared_ptr eliminate(size_t nrFrontals = 1);
#endif

  /** iterators */
  const_iterator begin() const { return keys_.begin(); }
  const_iterator end() const { return keys_.end(); }

  /** mutable iterators */
  iterator begin() { return keys_.begin(); }
  iterator end() { return keys_.end(); }

  /** First key*/
  Key front() const { return keys_.front(); }

  /** Last key */
  Key back() const { return keys_.back(); }

  /** find */
  const_iterator find(Key key) const { return std::find(begin(), end(), key); }

  /** print */
  void print(const std::string& s = "Factor") const;

  /** check equality */
  bool equals(const This& other, double tol = 1e-9) const;

  /**
   * @return keys involved in this factor
   */
  const std::vector<Key>& keys() const { return keys_; }

  /**
   * @return the number of variables involved in this factor
   */
  size_t size() const { return keys_.size(); }

protected:

  /** Serialization function */
  friend class boost::serialization::access;
  template<class Archive>
  void serialize(Archive & ar, const unsigned int version) {
    ar & BOOST_SERIALIZATION_NVP(keys_);
  }
};

}
