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

#include <gtsam/base/FastDefaultAllocator.h>
#include <gtsam/base/GenericValue.h>
#include <gtsam/base/VectorSpace.h>
#include <gtsam/inference/Key.h>
#include <boost/ptr_container/serialize_ptr_map.hpp>
#include <boost/shared_ptr.hpp>
#ifdef GTSAM_ALLOW_DEPRECATED_SINCE_V42
#include <boost/iterator/transform_iterator.hpp>
#include <boost/iterator/filter_iterator.hpp>
#endif

#include <string>
#include <utility>

namespace gtsam {

  // Forward declarations / utilities
  class VectorValues;
  class ValueAutomaticCasting;
  template<typename T> static bool _truePredicate(const T&) { return true; }

  /* ************************************************************************* */
  class GTSAM_EXPORT ValueCloneAllocator {
  public:
    static Value* allocate_clone(const Value& a) { return a.clone_(); }
    static void deallocate_clone(const Value* a) { a->deallocate_(); }
    ValueCloneAllocator() {}
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
  class GTSAM_EXPORT Values {

  private:
    // Internally we store a boost ptr_map, with a ValueCloneAllocator (defined
    // below) to clone and deallocate the Value objects, and our compile-flag-
    // dependent FastDefaultAllocator to allocate map nodes.  In this way, the
    // user defines the allocation details (i.e. optimize for memory pool/arenas
    // concurrency).
    typedef internal::FastDefaultAllocator<typename std::pair<const Key, void*>>::type KeyValuePtrPairAllocator;
    typedef boost::ptr_map<
        Key,
        Value,
        std::less<Key>,
        ValueCloneAllocator,
        KeyValuePtrPairAllocator > KeyValueMap;

    // The member to store the values, see just above
    KeyValueMap values_;

  public:

    /// A shared_ptr to this class
    typedef boost::shared_ptr<Values> shared_ptr;

    /// A const shared_ptr to this class
    typedef boost::shared_ptr<const Values> const_shared_ptr;

    /// A key-value pair, which you get by dereferencing iterators
    struct GTSAM_EXPORT KeyValuePair {
      const Key key; ///< The key
      Value& value;  ///< The value

      KeyValuePair(Key _key, Value& _value) : key(_key), value(_value) {}
    };

    /// A key-value pair, which you get by dereferencing iterators
    struct GTSAM_EXPORT ConstKeyValuePair {
      const Key key; ///< The key
      const Value& value;  ///< The value

      ConstKeyValuePair(Key _key, const Value& _value) : key(_key), value(_value) {}
      ConstKeyValuePair(const KeyValuePair& kv) : key(kv.key), value(kv.value) {}
    };

    typedef KeyValuePair value_type;

    /** Default constructor creates an empty Values class */
    Values() {}

    /** Copy constructor duplicates all keys and values */
    Values(const Values& other);

    /** Move constructor */
    Values(Values&& other);
    
    /** Constructor from initializer list. Example usage:
     * \code
     * Values v = {{k1, genericValue(pose1)}, {k2, genericValue(point2)}};
     * \endcode
     */
    Values(std::initializer_list<ConstKeyValuePair> init);

    /** Construct from a Values and an update vector: identical to other.retract(delta) */
    Values(const Values& other, const VectorValues& delta);

    /// @name Testable
    /// @{

    /** print method for testing and debugging */
    void print(const std::string& str = "", const KeyFormatter& keyFormatter = DefaultKeyFormatter) const;

    /** Test whether the sets of keys and values are identical */
    bool equals(const Values& other, double tol=1e-9) const;

    /// @}

    /** Retrieve a variable by key \c j.  The type of the value associated with
     * this key is supplied as a template argument to this function.
     * @param j Retrieve the value associated with this key
     * @tparam ValueType The type of the value stored with this key, this method
     * Throws DynamicValuesIncorrectType if this requested type is not correct.
     * Dynamic matrices/vectors can be retrieved as fixed-size, but not vice-versa.
     * @return The stored value
     */
    template <typename ValueType>
    const ValueType at(Key j) const;

    /// version for double
    double atDouble(size_t key) const { return at<double>(key);}

    /** Retrieve a variable by key \c j.  This version returns a reference
     * to the base Value class, and needs to be casted before use.
     * @param j Retrieve the value associated with this key
     * @return A const reference to the stored value
     */
    const Value& at(Key j) const;

    /** Check if a value exists with key \c j.  See exists<>(Key j)
     * and exists(const TypedKey& j) for versions that return the value if it
     * exists. */
    bool exists(Key j) const;

    /** Check if a value with key \c j exists, returns the value with type
     * \c Value if the key does exist, or boost::none if it does not exist.
     * Throws DynamicValuesIncorrectType if the value type associated with the
     * requested key does not match the stored value type. */
    template<typename ValueType>
    boost::optional<const ValueType&> exists(Key j) const;

    /** The number of variables in this config */
    size_t size() const { return values_.size(); }

    /** whether the config is empty */
    bool empty() const { return values_.empty(); }

    /// @name Manifold Operations
    /// @{

    /** Add a delta config to current config and returns a new config */
    Values retract(const VectorValues& delta) const;

    /**
     * Retract, but only for Keys appearing in \c mask. In-place.
     * \param mask Mask on Keys where to apply retract.
     */
    void retractMasked(const VectorValues& delta, const KeySet& mask);

    /** Get a delta config about a linearization point c0 (*this) */
    VectorValues localCoordinates(const Values& cp) const;

    ///@}

    /** Add a variable with the given j, throws KeyAlreadyExists<J> if j is already present */
    void insert(Key j, const Value& val);

    /** Add a set of variables, throws KeyAlreadyExists<J> if a key is already present */
    void insert(const Values& values);

    /** Templated version to add a variable with the given j,
     * throws KeyAlreadyExists<J> if j is already present
     */
    template <typename ValueType>
    void insert(Key j, const ValueType& val);

    /// version for double
    void insertDouble(Key j, double c) { insert<double>(j,c); }

    /** single element change of existing element */
    void update(Key j, const Value& val);

    /** Templated version to update a variable with the given j,
      * throws KeyDoesNotExist<J> if j is not present.
      * If no chart is specified, the DefaultChart<ValueType> is used.
      */
    template <typename T>
    void update(Key j, const T& val);

    /** update the current available values without adding new ones */
    void update(const Values& values);

    /// If key j exists, update value, else perform an insert.
    void insert_or_assign(Key j, const Value& val);

    /**
     * Update a set of variables.
     * If any variable key doe not exist, then perform an insert.
     */
    void insert_or_assign(const Values& values);

    /// Templated version to insert_or_assign a variable with the given j.
    template <typename ValueType>
    void insert_or_assign(Key j, const ValueType& val);

    /** Remove a variable from the config, throws KeyDoesNotExist<J> if j is not present */
    void erase(Key j);

    /**
     * Returns a vector of keys in the config.
     * Note: by construction, the list is ordered
     */
    KeyVector keys() const;

    /**
     * Returns a set of keys in the config.
     */
    KeySet keySet() const;

    /** Replace all keys and variables */
    Values& operator=(const Values& rhs);

    /** Swap the contents of two Values without copying data */
    void swap(Values& other) { values_.swap(other.values_); }

    /** Remove all variables from the config */
    void clear() { values_.clear(); }

    /** Compute the total dimensionality of all values (\f$ O(n) \f$) */
    size_t dim() const;

    /** Return all dimensions in a map (\f$ O(n log n) \f$) */
    std::map<Key,size_t> dims() const;

    /** Return a VectorValues of zero vectors for each variable in this Values */
    VectorValues zeroVectors() const;

    // Count values of given type \c ValueType
    template<class ValueType>
    size_t count() const {
      size_t i = 0;
      for (const auto key_value : values_) {
        if (dynamic_cast<const GenericValue<ValueType>*>(key_value.second))
          ++i;
      }
      return i;
    }

    /**
     * Extract a subset of values of the given type \c ValueType.
     * 
     * In this templated version, only key-value pairs whose value matches the
     * template argument \c ValueType and whose key causes the function argument
     * \c filterFcn to return true are visited when iterating over the filtered
     * view. This replaces the fancier but very boost-dependent \c filter methods
     * that were previously available up to GTSAM 4.2.
     * 
     * @tparam ValueType The type that the value in a key-value pair must match
     * to be included in the filtered view.  Currently, base classes are not
     * resolved so the type must match exactly, except if ValueType = Value, in
     * which case no type filtering is done.
     * @param filterFcn The function that determines which key-value pairs are
     * included in the filtered view, for which this function returns \c true
     * on their keys (default:  always return true so that filter() only
     * filters by type, matching \c ValueType).
     * @return An Eigen aligned map on Key with the filtered values.
     */
    template <class ValueType>
    std::map<Key, ValueType> // , std::less<Key>, Eigen::aligned_allocator<ValueType>
    extract(const std::function<bool(Key)>& filterFcn = &_truePredicate<Key>) const;

#ifdef GTSAM_ALLOW_DEPRECATED_SINCE_V42
    // Types obtained by iterating
    typedef KeyValueMap::const_iterator::value_type ConstKeyValuePtrPair;
    typedef KeyValueMap::iterator::value_type KeyValuePtrPair;

    /** insert that mimics the STL map insert - if the value already exists, the map is not modified
     *  and an iterator to the existing value is returned, along with 'false'.  If the value did not
     *  exist, it is inserted and an iterator pointing to the new element, along with 'true', is
     *  returned. */
    std::pair<iterator, bool> tryInsert(Key j, const Value& value);

    /// Mutable forward iterator, with value type KeyValuePair
    typedef boost::transform_iterator<
        std::function<KeyValuePair(const KeyValuePtrPair&)>, KeyValueMap::iterator> iterator;

    /// Const forward iterator, with value type ConstKeyValuePair
    typedef boost::transform_iterator<
        std::function<ConstKeyValuePair(const ConstKeyValuePtrPair&)>, KeyValueMap::const_iterator> const_iterator;

    /// Mutable reverse iterator, with value type KeyValuePair
    typedef boost::transform_iterator<
        std::function<KeyValuePair(const KeyValuePtrPair&)>, KeyValueMap::reverse_iterator> reverse_iterator;

    /// Const reverse iterator, with value type ConstKeyValuePair
    typedef boost::transform_iterator<
        std::function<ConstKeyValuePair(const ConstKeyValuePtrPair&)>, KeyValueMap::const_reverse_iterator> const_reverse_iterator;

    static ConstKeyValuePair make_const_deref_pair(const KeyValueMap::const_iterator::value_type& key_value) {
      return ConstKeyValuePair(key_value.first, *key_value.second); }

    static KeyValuePair make_deref_pair(const KeyValueMap::iterator::value_type& key_value) {
      return KeyValuePair(key_value.first, *key_value.second); }

    const_iterator begin() const { return boost::make_transform_iterator(values_.begin(), &make_const_deref_pair); }
    const_iterator end() const { return boost::make_transform_iterator(values_.end(), &make_const_deref_pair); }
    iterator begin() { return boost::make_transform_iterator(values_.begin(), &make_deref_pair); }
    iterator end() { return boost::make_transform_iterator(values_.end(), &make_deref_pair); }
    const_reverse_iterator rbegin() const { return boost::make_transform_iterator(values_.rbegin(), &make_const_deref_pair); }
    const_reverse_iterator rend() const { return boost::make_transform_iterator(values_.rend(), &make_const_deref_pair); }
    reverse_iterator rbegin() { return boost::make_transform_iterator(values_.rbegin(), &make_deref_pair); }
    reverse_iterator rend() { return boost::make_transform_iterator(values_.rend(), &make_deref_pair); }

    /** Find an element by key, returning an iterator, or end() if the key was
     * not found. */
    iterator find(Key j) { return boost::make_transform_iterator(values_.find(j), &make_deref_pair); }

    /** Find an element by key, returning an iterator, or end() if the key was
     * not found. */
    const_iterator find(Key j) const { return boost::make_transform_iterator(values_.find(j), &make_const_deref_pair); }

    /** Find the element greater than or equal to the specified key. */
    iterator lower_bound(Key j) { return boost::make_transform_iterator(values_.lower_bound(j), &make_deref_pair); }

    /** Find the element greater than or equal to the specified key. */
    const_iterator lower_bound(Key j) const { return boost::make_transform_iterator(values_.lower_bound(j), &make_const_deref_pair); }

    /** Find the lowest-ordered element greater than the specified key. */
    iterator upper_bound(Key j) { return boost::make_transform_iterator(values_.upper_bound(j), &make_deref_pair); }

    /** Find the lowest-ordered element greater than the specified key. */
    const_iterator upper_bound(Key j) const { return boost::make_transform_iterator(values_.upper_bound(j), &make_const_deref_pair); }

    /** A filtered view of a Values, returned from Values::filter. */
    template <class ValueType = Value>
    class Filtered;

    /** A filtered view of a const Values, returned from Values::filter. */
    template <class ValueType = Value>
    class ConstFiltered;

    /** Constructor from a Filtered view copies out all values */
    template <class ValueType>
    Values(const Filtered<ValueType>& view);

    /** Constructor from a Filtered or ConstFiltered view */
    template <class ValueType>
    Values(const ConstFiltered<ValueType>& view);

    /// A filtered view of the original Values class.
    Filtered<Value> GTSAM_DEPRECATED
    filter(const std::function<bool(Key)>& filterFcn);

    /// A filtered view of the original Values class, also filter on type.
    template <class ValueType>
    Filtered<ValueType> GTSAM_DEPRECATED
    filter(const std::function<bool(Key)>& filterFcn = &_truePredicate<Key>);

    /// A filtered view of the original Values class, const version.
    ConstFiltered<Value> GTSAM_DEPRECATED
    filter(const std::function<bool(Key)>& filterFcn) const;

    /// A filtered view of the original Values class, also on type, const.
    template <class ValueType>
    ConstFiltered<ValueType> GTSAM_DEPRECATED filter(
        const std::function<bool(Key)>& filterFcn = &_truePredicate<Key>) const;
#endif

  private:
    // Filters based on ValueType (if not Value) and also based on the user-
    // supplied \c filter function.
    template<class ValueType>
    static bool filterHelper(const std::function<bool(Key)> filter, const ConstKeyValuePair& key_value) {
      BOOST_STATIC_ASSERT((!boost::is_same<ValueType, Value>::value));
      // Filter and check the type
      return filter(key_value.key) && (dynamic_cast<const GenericValue<ValueType>*>(&key_value.value));
    }

    /** Serialization function */
    friend class boost::serialization::access;
    template<class ARCHIVE>
    void serialize(ARCHIVE & ar, const unsigned int /*version*/) {
      ar & BOOST_SERIALIZATION_NVP(values_);
    }

  };

  /* ************************************************************************* */
  class ValuesKeyAlreadyExists : public std::exception {
  protected:
    const Key key_; ///< The key that already existed

  private:
    mutable std::string message_;

  public:
    /// Construct with the key-value pair attempted to be added
    ValuesKeyAlreadyExists(Key key) noexcept :
      key_(key) {}

    ~ValuesKeyAlreadyExists() noexcept override {}

    /// The duplicate key that was attempted to be added
    Key key() const noexcept { return key_; }

    /// The message to be displayed to the user
    GTSAM_EXPORT const char* what() const noexcept override;
  };

  /* ************************************************************************* */
  class ValuesKeyDoesNotExist : public std::exception {
  protected:
    const char* operation_; ///< The operation that attempted to access the key
    const Key key_; ///< The key that does not exist

  private:
    mutable std::string message_;

  public:
    /// Construct with the key that does not exist in the values
    ValuesKeyDoesNotExist(const char* operation, Key key) noexcept :
      operation_(operation), key_(key) {}

    ~ValuesKeyDoesNotExist() noexcept override {}

    /// The key that was attempted to be accessed that does not exist
    Key key() const noexcept { return key_; }

    /// The message to be displayed to the user
    GTSAM_EXPORT const char* what() const noexcept override;
  };

  /* ************************************************************************* */
  class ValuesIncorrectType : public std::exception {
  protected:
    const Key key_; ///< The key requested
    const std::type_info& storedTypeId_;
    const std::type_info& requestedTypeId_;

  private:
    mutable std::string message_;

  public:
    /// Construct with the key that does not exist in the values
    ValuesIncorrectType(Key key,
        const std::type_info& storedTypeId, const std::type_info& requestedTypeId) noexcept :
      key_(key), storedTypeId_(storedTypeId), requestedTypeId_(requestedTypeId) {}

    ~ValuesIncorrectType() noexcept override {}

    /// The key that was attempted to be accessed that does not exist
    Key key() const noexcept { return key_; }

    /// The typeid of the value stores in the Values
    const std::type_info& storedTypeId() const { return storedTypeId_; }

    /// The requested typeid
    const std::type_info& requestedTypeId() const { return requestedTypeId_; }

    /// The message to be displayed to the user
    GTSAM_EXPORT const char* what() const noexcept override;
  };

  /* ************************************************************************* */
  class DynamicValuesMismatched : public std::exception {

  public:
    DynamicValuesMismatched() noexcept {}

    ~DynamicValuesMismatched() noexcept override {}

    const char* what() const noexcept override {
      return "The Values 'this' and the argument passed to Values::localCoordinates have mismatched keys and values";
    }
  };

  /* ************************************************************************* */
  class NoMatchFoundForFixed: public std::exception {

  protected:
    const size_t M1_, N1_;
    const size_t M2_, N2_;

  private:
    mutable std::string message_;

  public:
    NoMatchFoundForFixed(size_t M1, size_t N1, size_t M2, size_t N2) noexcept :
        M1_(M1), N1_(N1), M2_(M2), N2_(N2) {
    }

    ~NoMatchFoundForFixed() noexcept override {
    }

    GTSAM_EXPORT const char* what() const noexcept override;
  };

  /* ************************************************************************* */
  /// traits
  template<>
  struct traits<Values> : public Testable<Values> {
  };

} //\ namespace gtsam


#include <gtsam/nonlinear/Values-inl.h>
