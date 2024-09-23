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

#include <gtsam/inference/Key.h>
#include <gtsam/base/FastDefaultAllocator.h>
#include <gtsam/base/GenericValue.h>
#include <gtsam/base/VectorSpace.h>

#ifdef GTSAM_ENABLE_BOOST_SERIALIZATION
#include <boost/serialization/unique_ptr.hpp>
#endif


#include <memory>
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
    using KeyValueMap =
        std::map<Key, std::unique_ptr<Value>, std::less<Key>,
                 std::allocator<std::pair<const Key, std::unique_ptr<Value>>>>;

    // The member to store the values, see just above
    KeyValueMap values_;

  public:

    /// A shared_ptr to this class
    typedef std::shared_ptr<Values> shared_ptr;

    /// A const shared_ptr to this class
    typedef std::shared_ptr<const Values> const_shared_ptr;

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

    /// @name Constructors
    /// @{

    /** Default constructor creates an empty Values class */
    Values() = default;

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

    /// @}
    /// @name Testable
    /// @{

    /** print method for testing and debugging */
    void print(const std::string& str = "", const KeyFormatter& keyFormatter = DefaultKeyFormatter) const;

    /** Test whether the sets of keys and values are identical */
    bool equals(const Values& other, double tol=1e-9) const;

    /// @}
    /// @name Standard Interface
    /// @{

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

    /** Check if a value with key \c j exists, returns a pointer to the const version of the value
     * \c Value if the key does exist, or nullptr if it does not exist.
     * Throws DynamicValuesIncorrectType if the value type associated with the
     * requested key does not match the stored value type. */
    template<typename ValueType>
    const ValueType * exists(Key j) const;

    /** The number of variables in this config */
    size_t size() const { return values_.size(); }

    /** whether the config is empty */
    bool empty() const { return values_.empty(); }

    /// @}
    /// @name Iterator
    /// @{

    struct deref_iterator {
      using const_iterator_type = typename KeyValueMap::const_iterator;
      const_iterator_type it_;
      deref_iterator(const_iterator_type it) : it_(it) {}
      ConstKeyValuePair operator*() const { return {it_->first, *(it_->second)}; }
      std::unique_ptr<ConstKeyValuePair> operator->() {
        return std::make_unique<ConstKeyValuePair>(it_->first, *(it_->second));
      }
      bool operator==(const deref_iterator& other) const {
        return it_ == other.it_;
      }
      bool operator!=(const deref_iterator& other) const { return it_ != other.it_; }
      deref_iterator& operator++() {
        ++it_;
        return *this;
      }
    };

    deref_iterator begin() const { return deref_iterator(values_.begin()); }
    deref_iterator end() const { return deref_iterator(values_.end()); }

    /** Find an element by key, returning an iterator, or end() if the key was
     * not found. */
    deref_iterator find(Key j) const { return deref_iterator(values_.find(j)); }

    /** Find the element greater than or equal to the specified key. */
    deref_iterator lower_bound(Key j) const { return deref_iterator(values_.lower_bound(j)); }
    
    /** Find the lowest-ordered element greater than the specified key. */
    deref_iterator upper_bound(Key j) const { return deref_iterator(values_.upper_bound(j)); }

    /// @}
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

    /** Partial specialization that allows passing a unary Eigen expression for val.
      *
      * A unary expression is an expression such as 2*a or -a, where a is a valid Vector or Matrix type.
      * The typical usage is for types Point2 (i.e. Eigen::Vector2d) or Point3 (i.e. Eigen::Vector3d).
      * For example, together with the partial specialization for binary operators, a user may call insert(j, 2*a + M*b - c),
      * where M is an appropriately sized matrix (such as a rotation matrix).
      * Thus, it isn't necessary to explicitly evaluate the Eigen expression, as in insert(j, (2*a + M*b - c).eval()),
      * nor is it necessary to first assign the expression to a separate variable.
      */
    template <typename UnaryOp, typename ValueType>
    void insert(Key j, const Eigen::CwiseUnaryOp<UnaryOp, const ValueType>& val);

    /** Partial specialization that allows passing a binary Eigen expression for val.
      *
      * A binary expression is an expression such as a + b, where a and b are valid Vector or Matrix
      * types of compatible size.
      * The typical usage is for types Point2 (i.e. Eigen::Vector2d) or Point3 (i.e. Eigen::Vector3d).
      * For example, together with the partial specialization for binary operators, a user may call insert(j, 2*a + M*b - c),
      * where M is an appropriately sized matrix (such as a rotation matrix).
      * Thus, it isn't necessary to explicitly evaluate the Eigen expression, as in insert(j, (2*a + M*b - c).eval()),
      * nor is it necessary to first assign the expression to a separate variable.
      */
    template <typename BinaryOp, typename ValueType1, typename ValueType2>
    void insert(Key j, const Eigen::CwiseBinaryOp<BinaryOp, const ValueType1, const ValueType2>& val);

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

    /** Partial specialization that allows passing a unary Eigen expression for val,
      * similar to the partial specialization for insert.
      */
    template <typename UnaryOp, typename ValueType>
    void update(Key j, const Eigen::CwiseUnaryOp<UnaryOp, const ValueType>& val);

    /** Partial specialization that allows passing a binary Eigen expression for val,
      * similar to the partial specialization for insert.
      */
    template <typename BinaryOp, typename ValueType1, typename ValueType2>
    void update(Key j, const Eigen::CwiseBinaryOp<BinaryOp, const ValueType1, const ValueType2>& val);

    /** update the current available values without adding new ones */
    void update(const Values& values);

    /// If key j exists, update value, else perform an insert.
    void insert_or_assign(Key j, const Value& val);

    /**
     * Update a set of variables.
     * If any variable key does not exist, then perform an insert.
     */
    void insert_or_assign(const Values& values);

    /// Templated version to insert_or_assign a variable with the given j.
    template <typename ValueType>
    void insert_or_assign(Key j, const ValueType& val);

    /** Partial specialization that allows passing a unary Eigen expression for val,
      * similar to the partial specialization for insert.
      */
    template <typename UnaryOp, typename ValueType>
    void insert_or_assign(Key j, const Eigen::CwiseUnaryOp<UnaryOp, const ValueType>& val);

    /** Partial specialization that allows passing a binary Eigen expression for val,
      * similar to the partial specialization for insert.
      */
    template <typename BinaryOp, typename ValueType1, typename ValueType2>
    void insert_or_assign(Key j, const Eigen::CwiseBinaryOp<BinaryOp, const ValueType1, const ValueType2>& val);

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
    template <class ValueType>
    size_t count() const;

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

  private:
    // Filters based on ValueType (if not Value) and also based on the user-
    // supplied \c filter function.
    template<class ValueType>
    static bool filterHelper(const std::function<bool(Key)> filter, const ConstKeyValuePair& key_value) {
      // static_assert if ValueType is type: Value
      static_assert(!std::is_same<Value, ValueType>::value, "ValueType must not be type: Value to use this filter");
      // Filter and check the type
      return filter(key_value.key) && (dynamic_cast<const GenericValue<ValueType>*>(&key_value.value));
    }

#ifdef GTSAM_ENABLE_BOOST_SERIALIZATION
    /** Serialization function */
    friend class boost::serialization::access;
    template<class ARCHIVE>
    void serialize(ARCHIVE & ar, const unsigned int /*version*/) {
      ar & BOOST_SERIALIZATION_NVP(values_);
    }
#endif

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
