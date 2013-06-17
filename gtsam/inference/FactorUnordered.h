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

#include <vector>
#include <boost/serialization/nvp.hpp>

#include <gtsam/base/types.h>
#include <gtsam/inference/Key.h>

namespace gtsam {

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
class GTSAM_EXPORT FactorUnordered {

private:
  typedef FactorUnordered This; ///< This class

  /// A shared_ptr to this class, derived classes must redefine this.
  typedef boost::shared_ptr<FactorUnordered> shared_ptr;

public:
  /// Iterator over keys
  typedef std::vector<Key>::iterator iterator;

  /// Const iterator over keys
  typedef std::vector<Key>::const_iterator const_iterator;

protected:

  /// The keys involved in this factor
  std::vector<Key> keys_;

  /// @name Standard Constructors
  /// @{

  /** Default constructor for I/O */
  FactorUnordered() {}

  /** Construct unary factor */
  FactorUnordered(Key key) : keys_(1) {
    keys_[0] = key; }

  /** Construct binary factor */
  FactorUnordered(Key key1, Key key2) : keys_(2) {
    keys_[0] = key1; keys_[1] = key2; }

  /** Construct ternary factor */
  FactorUnordered(Key key1, Key key2, Key key3) : keys_(3) {
    keys_[0] = key1; keys_[1] = key2; keys_[2] = key3; }

  /** Construct 4-way factor */
  FactorUnordered(Key key1, Key key2, Key key3, Key key4) : keys_(4) {
    keys_[0] = key1; keys_[1] = key2; keys_[2] = key3; keys_[3] = key4; }

  /** Construct 5-way factor */
  FactorUnordered(Key key1, Key key2, Key key3, Key key4, Key key5) : keys_(5) {
    keys_[0] = key1; keys_[1] = key2; keys_[2] = key3; keys_[3] = key4; keys_[4] = key5; }

  /** Construct 6-way factor */
  FactorUnordered(Key key1, Key key2, Key key3, Key key4, Key key5, Key key6) : keys_(6) {
    keys_[0] = key1; keys_[1] = key2; keys_[2] = key3; keys_[3] = key4; keys_[4] = key5; keys_[5] = key6; }

  /// @}
  

  /// @name Advanced Constructors
  /// @{

  /** Construct n-way factor from iterator over keys. */
  template<typename ITERATOR> static FactorUnordered FromIterator(ITERATOR first, ITERATOR last) {
    FactorUnordered result;
    result.keys_.assign(first, last);
    return result; }
  
  /** Construct n-way factor from container of keys. */
  template<class CONTAINER>
  static FactorUnordered FromKeys(const CONTAINER& keys) { return FromIterator(keys.begin(), keys.end()); }

  /// @}

public:
  /// @name Standard Interface
  /// @{

  /// First key
  Key front() const { return keys_.front(); }

  /// Last key
  Key back() const { return keys_.back(); }

  /// find
  const_iterator find(Key key) const { return std::find(begin(), end(), key); }

  /// Access the factor's involved variable keys
  const std::vector<Key>& keys() const { return keys_; }

  /** Iterator at beginning of involved variable keys */
  const_iterator begin() const { return keys_.begin(); }

  /** Iterator at end of involved variable keys */
  const_iterator end() const { return keys_.end(); }

  /**
   * @return the number of variables involved in this factor
   */
  size_t size() const { return keys_.size(); }

  /// @}
  

  /// @name Testable
  /// @{

  /// print
  void print(const std::string& s = "Factor", const KeyFormatter& formatter = DefaultKeyFormatter) const;

  /// print only keys
  void printKeys(const std::string& s = "Factor", const KeyFormatter& formatter = DefaultKeyFormatter) const;

  /// check equality
  bool equals(const This& other, double tol = 1e-9) const;

  /// @}
  

  /// @name Advanced Interface
  /// @{

  /** @return keys involved in this factor */
  std::vector<Key>& keys() { return keys_; }

  /** Iterator at beginning of involved variable keys */
  iterator begin() { return keys_.begin(); }

  /** Iterator at end of involved variable keys */
  iterator end() { return keys_.end(); }

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
