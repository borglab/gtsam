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

#include <gtsam/base/Value.h>
#include <gtsam/base/FastMap.h>
#include <gtsam/linear/VectorValues.h>
#include <gtsam/nonlinear/Key.h>
#include <gtsam/nonlinear/Ordering.h>

namespace gtsam {

  // Forward declarations
  class ValueCloneAllocator;

  struct _ValuesDimensionCollector {
    const Ordering& ordering;
    std::vector<size_t> dimensions;
    _ValuesDimensionCollector(const Ordering& _ordering) : ordering(_ordering), dimensions(_ordering.nVars()) {}
    template<typename I> void operator()(const I& key_value) {
      Index var;
      if(ordering.tryAt(key_value->first, var)) {
        assert(var < dimensions.size());
        dimensions[var] = key_value->second->dim();
      }
    }
  };

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

    // Type obtained by iterating
    typedef KeyValueMap::const_iterator::value_type KeyValuePair;

  public:

    typedef KeyValueMap::iterator iterator;
    typedef KeyValueMap::const_iterator const_iterator;
    typedef KeyValueMap::reverse_iterator reverse_iterator;
    typedef KeyValueMap::const_reverse_iterator const_reverse_iterator;

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

    const_iterator begin() const { return values_.begin(); }
    const_iterator end() const { return values_.end(); }
    iterator begin() { return values_.begin(); }
    iterator end() { return values_.end(); }
    const_reverse_iterator rbegin() const { return values_.rbegin(); }
    const_reverse_iterator rend() const { return values_.rend(); }
    reverse_iterator rbegin() { return values_.rbegin(); }
    reverse_iterator rend() { return values_.rend(); }

    /// @name Manifold Operations
    /// @{

    /** Add a delta config to current config and returns a new config */
    Values retract(const VectorValues& delta, const Ordering& ordering) const;

    /** Get a delta config about a linearization point c0 (*this) */
    VectorValues localCoordinates(const Values& cp, const Ordering& ordering) const;

    /** Get a delta config about a linearization point c0 (*this) */
    void localCoordinates(const Values& cp, const Ordering& ordering, VectorValues& delta) const;

    ///@}

    // imperative methods:

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

    /** Remove a variable from the config while returning the dimensionality of
     * the removed element (normally not needed by user code), throws
     * KeyDoesNotExist<J> if j is not present.
     */
    //void erase(const J& j, size_t& dim);

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
     * Apply a class with an application operator() to a const_iterator over
     * every <key,value> pair.  The operator must be able to handle such an
     * iterator for every type in the Values, (i.e. through templating).
     */
    template<typename A>
    void apply(A& operation) {
      for(iterator it = begin(); it != end(); ++it)
        operation(it);
    }

    template<typename A>
    void apply(A& operation) const {
      for(const_iterator it = begin(); it != end(); ++it)
        operation(it);
    }

    /**
     * Generate a default ordering, simply in key sort order.  To instead
     * create a fill-reducing ordering, use
     * NonlinearFactorGraph::orderingCOLAMD().  Alternatively, you may permute
     * this ordering yourself (as orderingCOLAMD() does internally).
     */
    Ordering::shared_ptr orderingArbitrary(Index firstVar = 0) const;

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
