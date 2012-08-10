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

#include <set>
#include <vector>
#include <boost/serialization/nvp.hpp>
#include <boost/foreach.hpp>
#include <boost/function/function1.hpp>
#include <boost/lexical_cast.hpp>
#include <gtsam/base/types.h>
#include <gtsam/base/FastMap.h>

namespace gtsam {

template<class KEY> class Conditional;

/**
 * This is the base class for all factor types.  It is templated on a KEY type,
 * which will be the type used to label variables.  Key types currently in use
 * in gtsam are Index with symbolic (IndexFactor, SymbolicFactorGraph) and
 * Gaussian factors (GaussianFactor, JacobianFactor, HessianFactor, GaussianFactorGraph),
 * and Key with nonlinear factors (NonlinearFactor, NonlinearFactorGraph).
 * though currently only IndexFactor and IndexConditional derive from this
 * class, using Index keys.  This class does not store any data other than its
 * keys.  Derived classes store data such as matrices and probability tables.
 *
 * Note that derived classes *must* redefine the ConditionalType and shared_ptr
 * typedefs to refer to the associated conditional and shared_ptr types of the
 * derived class.  See IndexFactor, JacobianFactor, etc. for examples.
 *
 * This class is \b not virtual for performance reasons - derived symbolic classes,
 * IndexFactor and IndexConditional, need to be created and destroyed quickly
 * during symbolic elimination.  GaussianFactor and NonlinearFactor are virtual.
 * \nosubgrouping
 */
template<typename KEY>
class Factor {

public:

  typedef KEY KeyType; ///< The KEY template parameter
  typedef Factor<KeyType> This; ///< This class

  /**
   * Typedef to the conditional type obtained by eliminating this factor,
   * derived classes must redefine this.
   */
  typedef Conditional<KeyType> ConditionalType;

  /// A shared_ptr to this class, derived classes must redefine this.
  typedef boost::shared_ptr<Factor> shared_ptr;

  /// Iterator over keys
  typedef typename std::vector<KeyType>::iterator iterator;

  /// Const iterator over keys
  typedef typename std::vector<KeyType>::const_iterator const_iterator;

protected:

  /// The keys involved in this factor
  std::vector<KeyType> keys_;

public:

	/// @name Standard Constructors
	/// @{

  /** Copy constructor */
  Factor(const This& f);

  /** Construct from conditional, calls ConditionalType::toFactor() */
  Factor(const ConditionalType& c);

  /** Default constructor for I/O */
  Factor() {}

  /** Construct unary factor */
  Factor(KeyType key) : keys_(1) {
    keys_[0] = key; assertInvariants(); }

  /** Construct binary factor */
  Factor(KeyType key1, KeyType key2) : keys_(2) {
    keys_[0] = key1; keys_[1] = key2; assertInvariants(); }

  /** Construct ternary factor */
  Factor(KeyType key1, KeyType key2, KeyType key3) : keys_(3) {
    keys_[0] = key1; keys_[1] = key2; keys_[2] = key3; assertInvariants(); }

  /** Construct 4-way factor */
  Factor(KeyType key1, KeyType key2, KeyType key3, KeyType key4) : keys_(4) {
    keys_[0] = key1; keys_[1] = key2; keys_[2] = key3; keys_[3] = key4; assertInvariants(); }

  /** Construct 5-way factor */
  Factor(KeyType key1, KeyType key2, KeyType key3, KeyType key4, KeyType key5) : keys_(5) {
    keys_[0] = key1; keys_[1] = key2; keys_[2] = key3; keys_[3] = key4; keys_[4] = key5; assertInvariants(); }

  /** Construct 6-way factor */
  Factor(KeyType key1, KeyType key2, KeyType key3, KeyType key4, KeyType key5, KeyType key6) : keys_(6) {
    keys_[0] = key1; keys_[1] = key2; keys_[2] = key3; keys_[3] = key4; keys_[4] = key5; keys_[5] = key6; assertInvariants(); }

	/// @}
	/// @name Advanced Constructors
	/// @{

  /** Construct n-way factor */
	Factor(const std::set<KeyType>& keys) {
		BOOST_FOREACH(const KeyType& key, keys) keys_.push_back(key);
		assertInvariants();
	}

	/** Construct n-way factor */
	Factor(const std::vector<KeyType>& keys) : keys_(keys) {
		assertInvariants();
	}

  /** Constructor from a collection of keys */
  template<class KEYITERATOR> Factor(KEYITERATOR beginKey, KEYITERATOR endKey) :
        keys_(beginKey, endKey) { assertInvariants(); }

	/// @}

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

	/// @name Standard Interface
	/// @{

  /// First key
  KeyType front() const { return keys_.front(); }

  /// Last key
  KeyType back() const { return keys_.back(); }

  /// find
  const_iterator find(KeyType key) const { return std::find(begin(), end(), key); }

  ///TODO: comment
  const std::vector<KeyType>& keys() const { return keys_; }

  /** iterators */
  const_iterator begin() const { return keys_.begin(); }	///TODO: comment
  const_iterator end() const { return keys_.end(); }			///TODO: comment

  /**
   * @return the number of variables involved in this factor
   */
  size_t size() const { return keys_.size(); }

	/// @}
	/// @name Testable
	/// @{

  /// print
  void print(const std::string& s = "Factor",
  		const IndexFormatter& formatter = DefaultIndexFormatter) const;

  /// print only keys
  void printKeys(const std::string& s = "Factor",
  		const IndexFormatter& formatter = DefaultIndexFormatter) const;

  /// check equality
  bool equals(const This& other, double tol = 1e-9) const;

	/// @}
	/// @name Advanced Interface
	/// @{

  /**
   * @return keys involved in this factor
   */
  std::vector<KeyType>& keys() { return keys_; }

  /** mutable iterators */
  iterator begin() { return keys_.begin(); }	///TODO: comment
  iterator end() { return keys_.end(); }			///TODO: comment

protected:
	friend class JacobianFactor;
	friend class HessianFactor;

	/// Internal consistency check that is run frequently when in debug mode.
	/// If NDEBUG is defined, this is empty and optimized out.
	void assertInvariants() const;

private:
  /** Serialization function */
  friend class boost::serialization::access;
  template<class Archive>
  void serialize(Archive & ar, const unsigned int version) {
    ar & BOOST_SERIALIZATION_NVP(keys_);
  }

	/// @}

};

}
