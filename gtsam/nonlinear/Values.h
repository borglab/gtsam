/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file Values.h
 * @author Richard Roberts
 *
 * @brief A non-templated config holding any types of Manifold-group elements
 *
 *  Detailed story:
 *  A values structure is a map from keys to values. It is used to specify the value of a bunch
 *  of variables in a factor graph. A Values is a values structure which can hold variables that
 *  are elements on manifolds, not just vectors. It then, as a whole, implements a aggregate type
 *  which is also a manifold element, and hence supports operations dim, retract, and localCoordinates.
 */

#pragma once

#include <string>
#include <utility>

#include <boost/pool/pool_alloc.hpp>
#include <boost/ptr_container/ptr_map.hpp>
#include <boost/iterator/transform_iterator.hpp>
#include <boost/function.hpp>
#include <boost/bind.hpp>
#include <boost/ptr_container/serialize_ptr_map.hpp>

#include <gtsam/base/Value.h>
#include <gtsam/base/FastMap.h>
#include <gtsam/linear/VectorValues.h>
#include <gtsam/nonlinear/Key.h>
#include <gtsam/nonlinear/Ordering.h>

namespace gtsam {

  // Forward declarations
  class ValueCloneAllocator;
  class ValueAutomaticCasting;

/**
  * A non-templated config holding any types of Manifold-group elements.  A
  * values structure is a map from keys to values. It is used to specify the
  * value of a bunch of variables in a factor graph. A Values is a values
  * structure which can hold variables that are elements on manifolds, not just
  * vectors. It then, as a whole, implements a aggregate type which is also a
  * manifold element, and hence supports operations dim, retract, and
  * localCoordinates.
  */
  class Values {

  private:

    // Internally we store a boost ptr_map, with a ValueCloneAllocator (defined
    // below) to clone and deallocate the Value objects, and a boost
    // fast_pool_allocator to allocate map nodes.  In this way, all memory is
    // allocated in a boost memory pool.
    typedef boost::ptr_map<
        Symbol,
        Value,
        std::less<Symbol>,
        ValueCloneAllocator,
        boost::fast_pool_allocator<std::pair<const Symbol, void*> > > KeyValueMap;

    // The member to store the values, see just above
    KeyValueMap values_;

    // Types obtained by iterating
    typedef KeyValueMap::const_iterator::value_type ConstKeyValuePtrPair;
    typedef KeyValueMap::iterator::value_type KeyValuePtrPair;

  public:

    /// A shared_ptr to this class
    typedef boost::shared_ptr<Values> shared_ptr;

    /// A pair of const references to the key and value, the dereferenced type of the const_iterator and const_reverse_iterator
    struct ConstKeyValuePair {
      const Symbol& first;
      const Value& second;
      ConstKeyValuePair(const Symbol& key, const Value& value) : first(key), second(value) {}
    };

    /// A pair of references to the key and value, the dereferenced type of the iterator and reverse_iterator
    struct KeyValuePair {
      const Symbol& first;
      Value& second;
      KeyValuePair(const Symbol& key, Value& value) : first(key), second(value) {}
    };

    /// Mutable forward iterator, with value type KeyValuePair
    typedef boost::transform_iterator<
        boost::function1<KeyValuePair, const KeyValuePtrPair&>, KeyValueMap::iterator> iterator;

    /// Const forward iterator, with value type ConstKeyValuePair
    typedef boost::transform_iterator<
        boost::function1<ConstKeyValuePair, const ConstKeyValuePtrPair&>, KeyValueMap::const_iterator> const_iterator;

    /// Mutable reverse iterator, with value type KeyValuePair
    typedef boost::transform_iterator<
        boost::function1<KeyValuePair, const KeyValuePtrPair&>, KeyValueMap::reverse_iterator> reverse_iterator;

    /// Const reverse iterator, with value type ConstKeyValuePair
    typedef boost::transform_iterator<
        boost::function1<ConstKeyValuePair, const ConstKeyValuePtrPair&>, KeyValueMap::const_reverse_iterator> const_reverse_iterator;

    /** Default constructor creates an empty Values class */
    Values() {}

    /** Copy constructor duplicates all keys and values */
    Values(const Values& other);

    /// @name Testable
    /// @{

    /** print method for testing and debugging */
    void print(const std::string& str = "") const;

    /** Test whether the sets of keys and values are identical */
    bool equals(const Values& other, double tol=1e-9) const;

    /// @}

    /** Retrieve a variable by key \c j.  The type of the value associated with
     * this key is supplied as a template argument to this function.
     * @param j Retrieve the value associated with this key
     * @tparam Value The type of the value stored with this key, this method
     * throws DynamicValuesIncorrectType if this requested type is not correct.
     * @return A const reference to the stored value
     */
    template<typename ValueType>
    const ValueType& at(const Symbol& j) const;

    /** Retrieve a variable by key \c j.  This non-templated version returns a
     * special ValueAutomaticCasting object that may be assigned to the proper
     * value.
     * @param j Retrieve the value associated with this key
     * @return A ValueAutomaticCasting object that may be assignmed to a Value
     * of the proper type.
     */
    ValueAutomaticCasting at(const Symbol& j) const;

    /** Retrieve a variable using a special key (typically TypedSymbol), which
     * contains the type of the value associated with the key, and which must
     * be conversion constructible to a Symbol, e.g.
     * <tt>Symbol(const TypedKey&)</tt>.  Throws DynamicValuesKeyDoesNotExist
     * the key is not found, and DynamicValuesIncorrectType if the value type
     * associated with the requested key does not match the stored value type.
     */
    template<class TypedKey>
    const typename TypedKey::Value& at(const TypedKey& j) const;

    /** operator[] syntax for at(const TypedKey& j) */
    template<class TypedKey>
    const typename TypedKey::Value& operator[](const TypedKey& j) const {
      return at(j); }

    /** operator[] syntax for at(const Symbol& j) */
    ValueAutomaticCasting operator[](const Symbol& j) const;

    /** Check if a value exists with key \c j.  See exists<>(const Symbol& j)
     * and exists(const TypedKey& j) for versions that return the value if it
     * exists. */
    bool exists(const Symbol& j) const;

    /** Check if a value with key \c j exists, returns the value with type
     * \c Value if the key does exist, or boost::none if it does not exist.
     * Throws DynamicValuesIncorrectType if the value type associated with the
     * requested key does not match the stored value type. */
    template<typename ValueType>
    boost::optional<const ValueType&> exists(const Symbol& j) const;

    /** Check if a value with key \c j exists, returns the value with type
     * \c Value if the key does exist, or boost::none if it does not exist.
     * Uses a special key (typically TypedSymbol), which contains the type of
     * the value associated with the key, and which must be conversion
     * constructible to a Symbol, e.g. <tt>Symbol(const TypedKey&)</tt>. Throws
     * DynamicValuesIncorrectType if the value type associated with the
     * requested key does not match the stored value type.
     */
    template<class TypedKey>
    boost::optional<const typename TypedKey::Value&> exists(const TypedKey& j) const;

    /** The number of variables in this config */
    size_t size() const { return values_.size(); }

    /** whether the config is empty */
    bool empty() const { return values_.empty(); }

    /** Get a zero VectorValues of the correct structure */
    VectorValues zeroVectors(const Ordering& ordering) const;

  private:
    static ConstKeyValuePair make_const_deref_pair(const KeyValueMap::const_iterator::value_type& key_value) {
      return ConstKeyValuePair(key_value.first, *key_value.second); }
    static KeyValuePair make_deref_pair(const KeyValueMap::iterator::value_type& key_value) {
      return KeyValuePair(key_value.first, *key_value.second); }

  public:
    const_iterator begin() const { return boost::make_transform_iterator(values_.begin(), &make_const_deref_pair); }
    const_iterator end() const { return boost::make_transform_iterator(values_.end(), &make_const_deref_pair); }
    iterator begin() { return boost::make_transform_iterator(values_.begin(), &make_deref_pair); }
    iterator end() { return boost::make_transform_iterator(values_.end(), &make_deref_pair); }
    const_reverse_iterator rbegin() const { return boost::make_transform_iterator(values_.rbegin(), &make_const_deref_pair); }
    const_reverse_iterator rend() const { return boost::make_transform_iterator(values_.rend(), &make_const_deref_pair); }
    reverse_iterator rbegin() { return boost::make_transform_iterator(values_.rbegin(), &make_deref_pair); }
    reverse_iterator rend() { return boost::make_transform_iterator(values_.rend(), &make_deref_pair); }

    /// @name Manifold Operations
    /// @{

    /** Add a delta config to current config and returns a new config */
    Values retract(const VectorValues& delta, const Ordering& ordering) const;

    /** Get a delta config about a linearization point c0 (*this) */
    VectorValues localCoordinates(const Values& cp, const Ordering& ordering) const;

    /** Get a delta config about a linearization point c0 (*this) */
    void localCoordinates(const Values& cp, const Ordering& ordering, VectorValues& delta) const;

    ///@}

    /** Add a variable with the given j, throws KeyAlreadyExists<J> if j is already present */
    void insert(const Symbol& j, const Value& val);

    /** Add a set of variables, throws KeyAlreadyExists<J> if a key is already present */
    void insert(const Values& values);

    /** single element change of existing element */
    void update(const Symbol& j, const Value& val);

    /** update the current available values without adding new ones */
    void update(const Values& values);

    /** Remove a variable from the config, throws KeyDoesNotExist<J> if j is not present */
    void erase(const Symbol& j);

    /**
     * Returns a set of keys in the config
     * Note: by construction, the list is ordered
     */
    FastList<Symbol> keys() const;

    /** Replace all keys and variables */
    Values& operator=(const Values& rhs);

    /** Remove all variables from the config */
    void clear() { values_.clear(); }

    /** Create an array of variable dimensions using the given ordering */
    std::vector<size_t> dims(const Ordering& ordering) const;

    /**
     * Generate a default ordering, simply in key sort order.  To instead
     * create a fill-reducing ordering, use
     * NonlinearFactorGraph::orderingCOLAMD().  Alternatively, you may permute
     * this ordering yourself (as orderingCOLAMD() does internally).
     */
    Ordering::shared_ptr orderingArbitrary(Index firstVar = 0) const;

  private:
  	/** Serialization function */
  	friend class boost::serialization::access;
  	template<class ARCHIVE>
  	void serialize(ARCHIVE & ar, const unsigned int version) {
  		ar & BOOST_SERIALIZATION_NVP(values_);
  	}

  };

  /* ************************************************************************* */
  class ValuesKeyAlreadyExists : public std::exception {
  protected:
    const Symbol key_; ///< The key that already existed

  private:
    mutable std::string message_;

  public:
    /// Construct with the key-value pair attemped to be added
    ValuesKeyAlreadyExists(const Symbol& key) throw() :
      key_(key) {}

    virtual ~ValuesKeyAlreadyExists() throw() {}

    /// The duplicate key that was attemped to be added
    const Symbol& key() const throw() { return key_; }

    /// The message to be displayed to the user
    virtual const char* what() const throw();
  };

  /* ************************************************************************* */
  class ValuesKeyDoesNotExist : public std::exception {
  protected:
    const char* operation_; ///< The operation that attempted to access the key
    const Symbol key_; ///< The key that does not exist

  private:
    mutable std::string message_;

  public:
    /// Construct with the key that does not exist in the values
    ValuesKeyDoesNotExist(const char* operation, const Symbol& key) throw() :
      operation_(operation), key_(key) {}

    virtual ~ValuesKeyDoesNotExist() throw() {}

    /// The key that was attempted to be accessed that does not exist
    const Symbol& key() const throw() { return key_; }

    /// The message to be displayed to the user
    virtual const char* what() const throw();
  };

  /* ************************************************************************* */
  class ValuesIncorrectType : public std::exception {
  protected:
    const Symbol key_; ///< The key requested
    const std::type_info& storedTypeId_;
    const std::type_info& requestedTypeId_;

  private:
    mutable std::string message_;

  public:
    /// Construct with the key that does not exist in the values
    ValuesIncorrectType(const Symbol& key,
        const std::type_info& storedTypeId, const std::type_info& requestedTypeId) throw() :
      key_(key), storedTypeId_(storedTypeId), requestedTypeId_(requestedTypeId) {}

    virtual ~ValuesIncorrectType() throw() {}

    /// The key that was attempted to be accessed that does not exist
    const Symbol& key() const throw() { return key_; }

    /// The typeid of the value stores in the Values
    const std::type_info& storedTypeId() const { return storedTypeId_; }

    /// The requested typeid
    const std::type_info& requestedTypeId() const { return requestedTypeId_; }

    /// The message to be displayed to the user
    virtual const char* what() const throw();
  };

  /* ************************************************************************* */
  class DynamicValuesMismatched : public std::exception {

  public:
    DynamicValuesMismatched() throw() {}

    virtual ~DynamicValuesMismatched() throw() {}

    virtual const char* what() const throw() {
      return "The Values 'this' and the argument passed to Values::localCoordinates have mismatched keys and values";
    }
  };

}

#include <gtsam/nonlinear/Values-inl.h>
