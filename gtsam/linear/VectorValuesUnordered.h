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

#include <gtsam/base/Vector.h>
#include <gtsam/base/FastMap.h>
#include <gtsam/global_includes.h>

#include <boost/shared_ptr.hpp>

namespace gtsam {

  /**
   * This class represents a collection of vector-valued variables associated
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
   * Indices can be non-consecutive and inserted out-of-order, but you should not
   * use indices that are larger than a reasonable array size because the indices
   * correspond to positions in an internal array.
   *
   * Example:
   * \code
     VectorValues values;
     values.insert(3, Vector_(3, 1.0, 2.0, 3.0));
     values.insert(4, Vector_(2, 4.0, 5.0));
     values.insert(0, Vector_(4, 6.0, 7.0, 8.0, 9.0));

     // Prints [ 3.0 4.0 ]
     gtsam::print(values[1]);

     // Prints [ 8.0 9.0 ]
     values[1] = Vector_(2, 8.0, 9.0);
     gtsam::print(values[1]);
     \endcode
   *
   * <h2>Advanced Interface and Performance Information</h2>
   *
   * Internally, all vector values are stored as part of one large vector.  In
   * gtsam this vector is always pre-allocated for efficiency, using the
   * advanced interface described below.  Accessing and modifying already-allocated
   * values is \f$ O(1) \f$.  Using the insert() function of the standard interface
   * is slow because it requires re-allocating the internal vector.
   *
   * For advanced usage, or where speed is important:
   *  - Allocate space ahead of time using a pre-allocating constructor
   *    (\ref AdvancedConstructors "Advanced Constructors"), Zero(),
   *    SameStructure(), resize(), or append().  Do not use
   *    insert(Key, const Vector&), which always has to re-allocate the
   *    internal vector.
   *  - The vector() function permits access to the underlying Vector, for
   *    doing mathematical or other operations that require all values.
   *  - operator[]() returns a SubVector view of the underlying Vector,
   *    without copying any data.
   *
   * Access is through the variable index j, and returns a SubVector,
   * which is a view on the underlying data structure.
   *
   * This class is additionally used in gradient descent and dog leg to store the gradient.
   * \nosubgrouping
   */
  class GTSAM_EXPORT VectorValuesUnordered {
  protected:
    typedef VectorValuesUnordered This;
    typedef FastMap<Key, Vector> Values; ///< Typedef for the collection of Vectors making up a VectorValues
    Values values_; ///< Collection of Vectors making up this VectorValues

  public:
    typedef Values::iterator iterator; ///< Iterator over vector values
    typedef Values::const_iterator const_iterator; ///< Const iterator over vector values
    typedef Values::reverse_iterator reverse_iterator; ///< Reverse iterator over vector values
    typedef Values::const_reverse_iterator const_reverse_iterator; ///< Const reverse iterator over vector values
    typedef boost::shared_ptr<VectorValuesUnordered> shared_ptr; ///< shared_ptr to this class
    typedef Values::value_type value_type; ///< Typedef to pair<Key, Vector>, a key-value pair
    typedef value_type KeyValuePair; ///< Typedef to pair<Key, Vector>, a key-value pair

    /// @name Standard Constructors
    /// @{

    /**
     * Default constructor creates an empty VectorValues.
     */
    VectorValuesUnordered() {}

    /** Merge two VectorValues into one, this is more efficient than inserting elements one by one. */
    VectorValuesUnordered(const VectorValuesUnordered& first, const VectorValuesUnordered& second);

    /** Create a VectorValues with the same structure as \c other, but filled with zeros. */
    static VectorValuesUnordered Zero(const VectorValuesUnordered& other);

    /// @}
    /// @name Standard Interface
    /// @{

    /** Number of variables stored. */
    Key size() const { return values_.size(); }

    /** Return the dimension of variable \c j. */
    size_t dim(Key j) const { return at(j).rows(); }

    /** Check whether a variable with key \c j exists. */
    bool exists(Key j) const { return find(j) != end(); }

    /** Read/write access to the vector value with key \c j, throws std::out_of_range if \c j does not exist, identical to operator[](Key). */
    Vector& at(Key j) {
      iterator item = find(j);
      if(item == end())
        throw std::out_of_range(
        "Requested variable '" + DefaultKeyFormatter(j) + "' is not in this VectorValues.");
      else
        return item->second;
    }

    /** Access the vector value with key \c j (const version), throws std::out_of_range if \c j does not exist, identical to operator[](Key). */
    const Vector& at(Key j) const {
      const_iterator item = find(j);
      if(item == end())
        throw std::out_of_range(
        "Requested variable '" + DefaultKeyFormatter(j) + "' is not in this VectorValues.");
      else
        return item->second;
    }

    /** Read/write access to the vector value with key \c j, throws std::out_of_range if \c j does not exist, identical to at(Key). */
    Vector& operator[](Key j) { return at(j); }

    /** Access the vector value with key \c j (const version), throws std::out_of_range if \c j does not exist, identical to at(Key). */
    const Vector& operator[](Key j) const { return at(j); }

    /** Insert a vector \c value with key \c j.  Throws an invalid_argument exception if the key \c j is already used.
     * @param value The vector to be inserted.
     * @param j The index with which the value will be associated.
     */
    void insert(Key j, const Vector& value) {
      if(!values_.insert(std::make_pair(j, value)).second)
        throw std::invalid_argument(
        "Requested to insert variable '" + DefaultKeyFormatter(j) + "' already in this VectorValues.");
    }

    /** Insert all values from \c values.  Throws an invalid_argument exception if any keys to be
     *  inserted are already used. */
    void insert(const VectorValuesUnordered& values);

    iterator begin()                      { return values_.begin(); }  ///< Iterator over variables
    const_iterator begin() const          { return values_.begin(); }  ///< Iterator over variables
    iterator end()                         { return values_.end(); }    ///< Iterator over variables
    const_iterator end() const            { return values_.end(); }    ///< Iterator over variables
    reverse_iterator rbegin()              { return values_.rbegin(); } ///< Reverse iterator over variables
    const_reverse_iterator rbegin() const { return values_.rbegin(); } ///< Reverse iterator over variables
    reverse_iterator rend()                { return values_.rend(); }   ///< Reverse iterator over variables
    const_reverse_iterator rend() const   { return values_.rend(); }   ///< Reverse iterator over variables

    /** Return the iterator corresponding to the requested key, or end() if no variable is present with this key. */
    iterator find(Key j) { return values_.find(j); }

    /** Return the iterator corresponding to the requested key, or end() if no variable is present with this key. */
    const_iterator find(Key j) const { return values_.find(j); }

    /** print required by Testable for unit testing */
    void print(const std::string& str = "VectorValues: ",
        const KeyFormatter& formatter = DefaultKeyFormatter) const;

    /** equals required by Testable for unit testing */
    bool equals(const VectorValuesUnordered& x, double tol = 1e-9) const;

    /// @{
    /// @name Advanced Interface
    /// @{

    /** Retrieve the entire solution as a single vector */
    const Vector asVector() const;

    /** Access a vector that is a subset of relevant keys. */
    const Vector vector(const std::vector<Key>& keys) const;

    /**
     * Swap the data in this VectorValues with another.
     */
    void swap(VectorValuesUnordered& other);

    /// @}
    /// @name Linear algebra operations
    /// @{

    /** Dot product with another VectorValues, interpreting both as vectors of
    * their concatenated values.  Both VectorValues must have the
    * same structure (checked when NDEBUG is not defined). */
    double dot(const VectorValuesUnordered& v) const;

    /** Vector L2 norm */
    double norm() const;

    /** Squared vector L2 norm */
    double squaredNorm() const;

    /**
     * + operator does element-wise addition.  Both VectorValues must have the
     * same structure (checked when NDEBUG is not defined).
     */
    VectorValuesUnordered operator+(const VectorValuesUnordered& c) const;

    /**
     * + operator does element-wise subtraction.  Both VectorValues must have the
     * same structure (checked when NDEBUG is not defined).
     */
    VectorValuesUnordered operator-(const VectorValuesUnordered& c) const;

    /**
     * += operator does element-wise addition.  Both VectorValues must have the
     * same structure (checked when NDEBUG is not defined).
     */
    VectorValuesUnordered& operator+=(const VectorValuesUnordered& c);

    /// @}

    /// @}
    /// @name Matlab syntactic sugar for linear algebra operations
    /// @{

    //inline VectorValuesUnordered add(const VectorValuesUnordered& c) const { return *this + c; }
    //inline VectorValuesUnordered scale(const double a, const VectorValuesUnordered& c) const { return a * (*this); }

    /// @}

    /**
     * scale a vector by a scalar
     */
    //friend VectorValuesUnordered operator*(const double a, const VectorValuesUnordered &v) {
    //  VectorValuesUnordered result = VectorValuesUnordered::SameStructure(v);
    //  for(Key j = 0; j < v.size(); ++j)
    //    result.values_[j] = a * v.values_[j];
    //  return result;
    //}

    // TODO: linear algebra interface seems to have been added for SPCG.
    friend void scal(double alpha, VectorValuesUnordered& x);

    //// TODO: linear algebra interface seems to have been added for SPCG.
    //friend void axpy(double alpha, const VectorValuesUnordered& x, VectorValuesUnordered& y) {
    //  if(x.size() != y.size())
    //    throw std::invalid_argument("axpy(VectorValues) called with different vector sizes");
    //  for(Key j = 0; j < x.size(); ++j)
    //    if(x.values_[j].size() == y.values_[j].size())
    //      y.values_[j] += alpha * x.values_[j];
    //    else
    //      throw std::invalid_argument("axpy(VectorValues) called with different vector sizes");
    //}
    //// TODO: linear algebra interface seems to have been added for SPCG.
    //friend void sqrt(VectorValuesUnordered &x) {
    //  for(Key j = 0; j < x.size(); ++j)
    //    x.values_[j] = x.values_[j].cwiseSqrt();
    //}

    //// TODO: linear algebra interface seems to have been added for SPCG.
    //friend void ediv(const VectorValuesUnordered& numerator, const VectorValuesUnordered& denominator, VectorValuesUnordered &result) {
    //  if(numerator.size() != denominator.size() || numerator.size() != result.size())
    //    throw std::invalid_argument("ediv(VectorValues) called with different vector sizes");
    //  for(Key j = 0; j < numerator.size(); ++j)
    //    if(numerator.values_[j].size() == denominator.values_[j].size() && numerator.values_[j].size() == result.values_[j].size())
    //      result.values_[j] = numerator.values_[j].cwiseQuotient(denominator.values_[j]);
    //    else
    //      throw std::invalid_argument("ediv(VectorValues) called with different vector sizes");
    //}

    //// TODO: linear algebra interface seems to have been added for SPCG.
    //friend void edivInPlace(VectorValuesUnordered& x, const VectorValuesUnordered& y) {
    //  if(x.size() != y.size())
    //    throw std::invalid_argument("edivInPlace(VectorValues) called with different vector sizes");
    //  for(Key j = 0; j < x.size(); ++j)
    //    if(x.values_[j].size() == y.values_[j].size())
    //      x.values_[j].array() /= y.values_[j].array();
    //    else
    //      throw std::invalid_argument("edivInPlace(VectorValues) called with different vector sizes");
    //}

  private:
    /** Serialization function */
    friend class boost::serialization::access;
    template<class ARCHIVE>
    void serialize(ARCHIVE & ar, const unsigned int version) {
      ar & BOOST_SERIALIZATION_NVP(values_);
    }
  }; // VectorValues definition

} // \namespace gtsam
