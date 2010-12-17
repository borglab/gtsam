/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    Factor.h
 * @brief   A simple factor class to use in a factor graph
 * @brief   factor
 * @author  Kai Ni
 * @author  Frank Dellaert
 */

// \callgraph

#pragma once

#include <vector>
#include <map>
#include <boost/utility.hpp> // for noncopyable
#include <boost/serialization/nvp.hpp>
#include <boost/foreach.hpp>
#include <gtsam/base/types.h>
#include <gtsam/base/Testable.h>
#include <gtsam/base/FastMap.h>
#include <gtsam/inference/inference.h>

namespace gtsam {

template<class KEY> class ConditionalBase;

/**
 * A simple factor class to use in a factor graph.
 *
 * We make it noncopyable so we enforce the fact that factors are
 * kept in pointer containers. To be safe, you should make them
 * immutable, i.e., practicing functional programming. However, this
 * conflicts with efficiency as well, esp. in the case of incomplete
 * QR factorization. A solution is still being sought.
 *
 * A Factor is templated on a Values, for example VectorValues is a values structure of
 * labeled vectors. This way, we can have factors that might be defined on discrete
 * variables, continuous ones, or a combination of both. It is up to the config to
 * provide the appropriate values at the appropriate time.
 */
template<typename KEY>
class FactorBase : public Testable<FactorBase<KEY> > {

public:

  typedef KEY Key;
  typedef FactorBase<Key> This;
  typedef gtsam::ConditionalBase<Key> Conditional;
  typedef boost::shared_ptr<FactorBase> shared_ptr;
  typedef std::vector<Index>::iterator iterator;
  typedef std::vector<Index>::const_iterator const_iterator;

protected:

  std::vector<Key> keys_;

  /** Internal check to make sure keys are sorted.
   * If NDEBUG is defined, this is empty and optimized out. */
  void assertInvariants() const;

public:

  /** Copy constructor */
  FactorBase(const This& f);

  /** Construct from derived type */
  FactorBase(const Conditional& c);

  /** Constructor from a collection of keys */
  template<class KEYITERATOR> FactorBase(KEYITERATOR beginKey, KEYITERATOR endKey) :
        keys_(beginKey, endKey) { assertInvariants(); }

  /** Default constructor for I/O */
  FactorBase() {}

  /** Construct unary factor */
  FactorBase(Key key) : keys_(1) {
    keys_[0] = key; assertInvariants(); }

  /** Construct binary factor */
  FactorBase(Key key1, Key key2) : keys_(2) {
    keys_[0] = key1; keys_[1] = key2; assertInvariants(); }

  /** Construct ternary factor */
  FactorBase(Key key1, Key key2, Key key3) : keys_(3) {
    keys_[0] = key1; keys_[1] = key2; keys_[2] = key3; assertInvariants(); }

  /** Construct 4-way factor */
  FactorBase(Key key1, Key key2, Key key3, Key key4) : keys_(4) {
    keys_[0] = key1; keys_[1] = key2; keys_[2] = key3; keys_[3] = key4; assertInvariants(); }

  /** Construct n-way factor */
  FactorBase(std::set<Key> keys) {
  	BOOST_FOREACH(const Key& key, keys)
  			keys_.push_back(key);
  	assertInvariants(); }

  /** Create a combined joint factor (new style for EliminationTree). */
  template<class DERIVED>
  static typename DERIVED::shared_ptr Combine(const FactorGraph<DERIVED>& factors, const FastMap<Key, std::vector<Key> >& variableSlots);

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

  /**
   * Permutes the GaussianFactor, but for efficiency requires the permutation
   * to already be inverted.
   */
  void permuteWithInverse(const Permutation& inversePermutation);

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
//  template<class DERIVED>
  bool equals(const This& other, double tol = 1e-9) const;

  /**
   * return keys in order as created
   */
  const std::vector<Key>& keys() const { return keys_; }

  /**
   * @return the number of nodes the factor connects
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
