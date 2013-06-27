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
  class GTSAM_EXPORT FactorUnordered
  {

  private:
    // These typedefs are private because they must be overridden in derived classes.
    typedef FactorUnordered This; ///< This class
    typedef boost::shared_ptr<FactorUnordered> shared_ptr; ///< A shared_ptr to this class.

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

    /** Construct factor from container of keys.  This constructor is used internally from derived factor
    *  constructors, either from a container of keys or from a boost::assign::list_of. */
    template<typename CONTAINER>
    FactorUnordered(const CONTAINER& keys) : keys_(keys.begin(), keys.end()) {}

    /** Construct factor from iterator keys.  This constructor may be used internally from derived
    *  factor constructors, although our code currently does not use this. */
    template<typename ITERATOR>
    FactorUnordered(ITERATOR first, ITERATOR last) : keys_(first, last) {}

    /** Construct factor from container of keys.  This is called internally from derived factor static
    *  factor methods, as a workaround for not being able to call the protected constructors above. */
    template<typename CONTAINER>
    static FactorUnordered FromKeys(const CONTAINER& keys) {
      return FactorUnordered(keys.begin(), keys.end()); }

    /** Construct factor from iterator keys.  This is called internally from derived factor static
    *  factor methods, as a workaround for not being able to call the protected constructors above. */
    template<typename ITERATOR>
    static FactorUnordered FromIterators(ITERATOR first, ITERATOR last) {
      return FactorUnordered(first, last); }

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
