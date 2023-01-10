/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    VectorValues.h
 * @brief   Factor Graph Values
 * @author  Richard Roberts
 */

#pragma once

#include <gtsam/linear/Scatter.h>
#include <gtsam/inference/Ordering.h>
#include <gtsam/base/Vector.h>
#include <gtsam/base/ConcurrentMap.h>
#include <gtsam/base/FastVector.h>
#include <gtsam/global_includes.h>

#include <boost/shared_ptr.hpp>


#include <map>
#include <string>
#include <iosfwd>

namespace gtsam {

  /**
   * VectorValues represents a collection of vector-valued variables associated
   * each with a unique integer index.  It is typically used to store the variables
   * of a GaussianFactorGraph.  Optimizing a GaussianFactorGraph or GaussianBayesNet
   * returns this class.
   *
   * For basic usage, such as receiving a linear solution from gtsam solving functions,
   * or creating this class in unit tests and examples where speed is not important,
   * you can use a simple interface:
   *  - The default constructor VectorValues() to create this class
   *  - insert(Key, const Vector&) to add vector variables
   *  - operator[](Key) for read and write access to stored variables
   *  - \ref exists (Key) to check if a variable is present
   *  - Other facilities like iterators, size(), dim(), etc.
   *
   * Example:
   * \code
     VectorValues values;
     values.emplace(3, Vector3(1.0, 2.0, 3.0));
     values.emplace(4, Vector2(4.0, 5.0));
     values.emplace(0, (Vector(4) << 6.0, 7.0, 8.0, 9.0).finished());

     // Prints [ 3.0 4.0 ]
     gtsam::print(values[1]);

     // Prints [ 8.0 9.0 ]
     values[1] = Vector2(8.0, 9.0);
     gtsam::print(values[1]);
     \endcode
   *
   * <h2>Advanced Interface and Performance Information</h2>
   *
   * Access is through the variable Key j, and returns a SubVector,
   * which is a view on the underlying data structure.
   *
   * This class is additionally used in gradient descent and dog leg to store the gradient.
   * @ingroup linear
   */
  class GTSAM_EXPORT VectorValues {
   protected:
    typedef VectorValues This;
    typedef ConcurrentMap<Key, Vector> Values;  ///< Collection of Vectors making up a VectorValues
    Values values_;                             ///< Vectors making up this VectorValues

   public:
    typedef Values::iterator iterator;              ///< Iterator over vector values
    typedef Values::const_iterator const_iterator;  ///< Const iterator over vector values
    typedef boost::shared_ptr<This> shared_ptr;     ///< shared_ptr to this class
    typedef Values::value_type value_type;          ///< Typedef to pair<Key, Vector>
    typedef value_type KeyValuePair;                ///< Typedef to pair<Key, Vector>
    typedef std::map<Key, size_t> Dims;             ///< Keyed vector dimensions

    /// @name Standard Constructors
    /// @{

    /// Default constructor creates an empty VectorValues.
    VectorValues() {}

    /// Construct from initializer list.
    VectorValues(std::initializer_list<std::pair<Key, Vector>> init)
        : values_(init.begin(), init.end()) {}

    /** Merge two VectorValues into one, this is more efficient than inserting
     * elements one by one. */
    VectorValues(const VectorValues& first, const VectorValues& second);

    /** Create from another container holding pair<Key,Vector>. */
    template<class CONTAINER>
    explicit VectorValues(const CONTAINER& c) : values_(c.begin(), c.end()) {}

    /** Implicit copy constructor to specialize the explicit constructor from any container. */
    VectorValues(const VectorValues& c) : values_(c.values_) {}

    /** Create from a pair of iterators over pair<Key,Vector>. */
    template<typename ITERATOR>
    VectorValues(ITERATOR first, ITERATOR last) : values_(first, last) {}

    /// Constructor from Vector, with Dims
    VectorValues(const Vector& c, const Dims& dims);

    /// Constructor from Vector, with Scatter
    VectorValues(const Vector& c, const Scatter& scatter);

    /** Create a VectorValues with the same structure as \c other, but filled with zeros. */
    static VectorValues Zero(const VectorValues& other);

    /// @}
    /// @name Standard Interface
    /// @{

    /** Number of variables stored. */
    size_t size() const { return values_.size(); }

    /** Return the dimension of variable \c j. */
    size_t dim(Key j) const { return at(j).rows(); }

    /** Check whether a variable with key \c j exists. */
    bool exists(Key j) const { return find(j) != end(); }

    /**
     * Read/write access to the vector value with key \c j, throws
     * std::out_of_range if \c j does not exist, identical to operator[](Key).
     */
    Vector& at(Key j) {
      iterator item = find(j);
      if (item == end())
        throw std::out_of_range(
        "Requested variable '" + DefaultKeyFormatter(j) + "' is not in this VectorValues.");
      else
        return item->second;
    }

    /**
     * Access the vector value with key \c j (const version), throws
     * std::out_of_range if \c j does not exist, identical to operator[](Key).
     */
    const Vector& at(Key j) const {
      const_iterator item = find(j);
      if (item == end())
        throw std::out_of_range(
        "Requested variable '" + DefaultKeyFormatter(j) + "' is not in this VectorValues.");
      else
        return item->second;
    }

    /** Read/write access to the vector value with key \c j, throws std::out_of_range if \c j does
    *   not exist, identical to at(Key). */
    Vector& operator[](Key j) { return at(j); }

    /** Access the vector value with key \c j (const version), throws std::out_of_range if \c j does
    *   not exist, identical to at(Key). */
    const Vector& operator[](Key j) const { return at(j); }

    /** For all key/value pairs in \c values, replace values with corresponding keys in this class
    *   with those in \c values.  Throws std::out_of_range if any keys in \c values are not present
    *   in this class. */
    VectorValues& update(const VectorValues& values);

    /** Insert a vector \c value with key \c j.  Throws an invalid_argument exception if the key \c
     *  j is already used.
     * @param value The vector to be inserted.
     * @param j The index with which the value will be associated. */
    iterator insert(const std::pair<Key, Vector>& key_value);

    /** Emplace a vector \c value with key \c j.  Throws an invalid_argument exception if the key \c
     *  j is already used.
     * @param value The vector to be inserted.
     * @param j The index with which the value will be associated. */
    template<class... Args>
    inline std::pair<VectorValues::iterator, bool> emplace(Key j, Args&&... args) {
#if ! defined(GTSAM_USE_TBB) || defined (TBB_GREATER_EQUAL_2020)
      return values_.emplace(std::piecewise_construct, std::forward_as_tuple(j), std::forward_as_tuple(args...));
#else
      return values_.insert(std::make_pair(j, Vector(std::forward<Args>(args)...)));
#endif
    }

    /** Insert a vector \c value with key \c j.  Throws an invalid_argument exception if the key \c
     *  j is already used.
     * @param value The vector to be inserted.
     * @param j The index with which the value will be associated. */
    iterator insert(Key j, const Vector& value) {
      return insert(std::make_pair(j, value));
    }

    /** Insert all values from \c values.  Throws an invalid_argument exception if any keys to be
     *  inserted are already used. */
    VectorValues& insert(const VectorValues& values);

    /** insert that mimics the STL map insert - if the value already exists, the map is not modified
     *  and an iterator to the existing value is returned, along with 'false'.  If the value did not
     *  exist, it is inserted and an iterator pointing to the new element, along with 'true', is
     *  returned. */
    inline std::pair<iterator, bool> tryInsert(Key j, const Vector& value) {
#ifdef TBB_GREATER_EQUAL_2020
      return values_.emplace(j, value);
#else
      return values_.insert(std::make_pair(j, value));
#endif
    }

    /** Erase the vector with the given key, or throw std::out_of_range if it does not exist */
    void erase(Key var) {
      if (values_.unsafe_erase(var) == 0)
        throw std::invalid_argument("Requested variable '" +
                                    DefaultKeyFormatter(var) +
                                    "', is not in this VectorValues.");
    }

    /** Set all values to zero vectors. */
    void setZero();

    iterator begin()             { return values_.begin(); }  ///< Iterator over variables
    const_iterator begin() const { return values_.begin(); }  ///< Iterator over variables
    iterator end()               { return values_.end(); }    ///< Iterator over variables
    const_iterator end() const   { return values_.end(); }    ///< Iterator over variables

    /**
     * Return the iterator corresponding to the requested key, or end() if no
     * variable is present with this key.
     */
    iterator find(Key j) { return values_.find(j); }

    /**
     * Return the iterator corresponding to the requested key, or end() if no
     * variable is present with this key.
     */
    const_iterator find(Key j) const { return values_.find(j); }

    /// overload operator << to print to stringstream
    GTSAM_EXPORT friend std::ostream& operator<<(std::ostream&, const VectorValues&);

    /** print required by Testable for unit testing */
    void print(const std::string& str = "VectorValues",
        const KeyFormatter& formatter = DefaultKeyFormatter) const;

    /** equals required by Testable for unit testing */
    bool equals(const VectorValues& x, double tol = 1e-9) const;

    /// @{
    /// @name Advanced Interface
    /// @{

    /** Retrieve the entire solution as a single vector */
    Vector vector() const;

    /** Access a vector that is a subset of relevant keys. */
    template <typename CONTAINER>
    Vector vector(const CONTAINER& keys) const {
      DenseIndex totalDim = 0;
      FastVector<const Vector*> items;
      items.reserve(keys.end() - keys.begin());
      for (Key key : keys) {
        const Vector* v = &at(key);
        totalDim += v->size();
        items.push_back(v);
      }

      Vector result(totalDim);
      DenseIndex pos = 0;
      for (const Vector* v : items) {
        result.segment(pos, v->size()) = *v;
        pos += v->size();
      }

      return result;
    }

    /** Access a vector that is a subset of relevant keys, dims version. */
    Vector vector(const Dims& dims) const;

    /** Swap the data in this VectorValues with another. */
    void swap(VectorValues& other);

    /** Check if this VectorValues has the same structure (keys and dimensions) as another */
    bool hasSameStructure(const VectorValues other) const;

    /// @}
    /// @name Linear algebra operations
    /// @{

    /** Dot product with another VectorValues, interpreting both as vectors of
    * their concatenated values.  Both VectorValues must have the
    * same structure (checked when NDEBUG is not defined). */
    double dot(const VectorValues& v) const;

    /** Vector L2 norm */
    double norm() const;

    /** Squared vector L2 norm */
    double squaredNorm() const;

    /** Element-wise addition, synonym for add().  Both VectorValues must have the same structure
     *  (checked when NDEBUG is not defined). */
    VectorValues operator+(const VectorValues& c) const;

    /** Element-wise addition, synonym for operator+().  Both VectorValues must have the same
     *  structure (checked when NDEBUG is not defined). */
    VectorValues add(const VectorValues& c) const;

    /** Element-wise addition in-place, synonym for operator+=().  Both VectorValues must have the
     * same structure (checked when NDEBUG is not defined). */
    VectorValues& operator+=(const VectorValues& c);

    /** Element-wise addition in-place, synonym for operator+=().  Both VectorValues must have the
     * same structure (checked when NDEBUG is not defined). */
    VectorValues& addInPlace(const VectorValues& c);

    /** Element-wise addition in-place, but allows for empty slots in *this. Slower */
    VectorValues& addInPlace_(const VectorValues& c);

    /** Element-wise subtraction, synonym for subtract().  Both VectorValues must have the same
     *  structure (checked when NDEBUG is not defined). */
    VectorValues operator-(const VectorValues& c) const;

    /** Element-wise subtraction, synonym for operator-().  Both VectorValues must have the same
     *  structure (checked when NDEBUG is not defined). */
    VectorValues subtract(const VectorValues& c) const;

    /** Element-wise scaling by a constant. */
    friend GTSAM_EXPORT VectorValues operator*(const double a, const VectorValues &v);

    /** Element-wise scaling by a constant. */
    VectorValues scale(const double a) const;

    /** Element-wise scaling by a constant in-place. */
    VectorValues& operator*=(double alpha);

    /** Element-wise scaling by a constant in-place. */
    VectorValues& scaleInPlace(double alpha);

    /// @}

    /// @name Wrapper support
    /// @{

    /**
     * @brief Output as a html table.
     *
     * @param keyFormatter function that formats keys.
     */
    std::string html(
        const KeyFormatter& keyFormatter = DefaultKeyFormatter) const;

    /// @}

  private:
    /** Serialization function */
    friend class boost::serialization::access;
    template<class ARCHIVE>
    void serialize(ARCHIVE & ar, const unsigned int /*version*/) {
      ar & BOOST_SERIALIZATION_NVP(values_);
    }
  }; // VectorValues definition

  /// traits
  template<>
  struct traits<VectorValues> : public Testable<VectorValues> {
  };

} // \namespace gtsam
