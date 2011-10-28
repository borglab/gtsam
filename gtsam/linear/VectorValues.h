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
#include <gtsam/base/types.h>

#include <boost/foreach.hpp>
#include <boost/shared_ptr.hpp>
#include <numeric>

namespace gtsam {

  /**
   * This class stores a collection of vector-valued variables, each referenced
   * by a unique variable index.  It is typically used to store the variables
   * of a GaussianFactorGraph.  Optimizing a GaussianFactorGraph or GaussianBayesNet
   * returns this class.
   *
   * For basic usage, such as receiving a linear solution from gtsam solving functions,
   * or creating this class in unit tests and examples where speed is not important,
   * you can use a simple interface:
   *  - The default constructor VectorValues() to create this class
   *  - insert(Index, const Vector&) to add vector variables
   *  - operator[](Index) for read and write access to stored variables
   *  - \ref exists (Index) to check if a variable is present
   *  - Other facilities like iterators, size(), dim(), etc.
   *
   * Example:
   * \code
     VectorValues values;
     values.insert(0, Vector_(3, 5.0, 6.0, 7.0));
     values.insert(1, Vector_(2, 3.0, 4.0));

     // Prints [ 3.0 4.0 ]
     gtsam::print(values[1]);

     // Prints [ 8.0 9.0 ]
     values[1] = Vector_(2, 8.0, 9.0);
     gtsam::print(values[1]);
     \endcode
   *
   * Internally, this class stores all vectors as part of one large vector.  This is
   * necessary for performance, and the gtsam linear solving code exploits it by
   * only allocating one large vector to store the solution.  For advanced usage,
   * or where speed is important, be aware of the following:
   *  - It is faster to allocate space ahead of time using a pre-allocating constructor
   *    or the resize() and append() functions, than to use insert(Index, const Vector&),
   *    which always has to re-allocate the internal vector.
   *  - The vector() function permits access to the underlying Vector.
   *  - operator[]() returns a SubVector view of the underlying Vector.
   *
   * Access is through the variable index j, and returns a SubVector,
   * which is a view on the underlying data structure.
   *
   * This class is additionally used in gradient descent and dog leg to store the gradient.
   * \nosubgrouping
   */
  class VectorValues {
  protected:
    Vector values_; ///< The underlying vector storing the values
    typedef std::vector<SubVector> ValueMaps; ///< Collection of SubVector s
    ValueMaps maps_; ///< SubVector s referencing each vector variable in values_

  public:
    typedef ValueMaps::iterator iterator; ///< Iterator over vector values
    typedef ValueMaps::const_iterator const_iterator; ///< Const iterator over vector values
    typedef ValueMaps::reverse_iterator reverse_iterator; ///< Reverse iterator over vector values
    typedef ValueMaps::const_reverse_iterator const_reverse_iterator; ///< Const reverse iterator over vector values
    typedef boost::shared_ptr<VectorValues> shared_ptr; ///< shared_ptr to this class

    /// @name Standard constructors
    /// @{

    /**
     * Default constructor creates an empty VectorValues.
     */
    VectorValues() {} //

    /** Copy constructor */
    VectorValues(const VectorValues &other); //

    /** Named constructor to create a VectorValues of the same structure of the
     * specifed one, but filled with zeros.
     * @return
     */
    static VectorValues Zero(const VectorValues& model);

    /// @}
    /// @name Standard interface
    /// @{

    /** Number of variables stored, always 1 more than the highest variable index,
     * even if some variables with lower indices are not present. */
    Index size() const { return maps_.size(); } //

    /** Return the dimension of variable \c j. */
    size_t dim(Index j) const { checkExists(j); return (*this)[j].rows(); } //

    /** Return the summed dimensionality of all variables. */
    size_t dim() const { return values_.rows(); } //

    /** Check whether a variable exists by index. */
    bool exists(Index j) const { return j < size() && maps_[j].rows() > 0; } //

    /** Reference a variable by index. */
    SubVector& operator[](Index j) { checkExists(j); return maps_[j]; } //

    /** Reference a variable by index. */
    const SubVector& operator[](Index j) const { checkExists(j); return maps_[j]; } //

    /** Insert a vector \c value with index \c j.
     * Causes reallocation. Can be used to insert values in any order, but
     * throws an invalid_argument exception if the index \j is already used.
     * @param value The vector to be inserted.
     * @param j The index with which the value will be associated.
     */
    void insert(Index j, const Vector& value); //

    /** Assignment */
    VectorValues& operator=(const VectorValues& rhs); //

    iterator begin()                      { chk(); return maps_.begin(); }  ///< Iterator over variables
    const_iterator begin() const          { chk(); return maps_.begin(); }  ///< Iterator over variables
    iterator end()                        { chk(); return maps_.end(); }    ///< Iterator over variables
    const_iterator end() const            { chk(); return maps_.end(); }    ///< Iterator over variables
    reverse_iterator rbegin()             { chk(); return maps_.rbegin(); } ///< Iterator over variables
    const_reverse_iterator rbegin() const { chk(); return maps_.rbegin(); } ///< Iterator over variables
    reverse_iterator rend()               { chk(); return maps_.rend(); }   ///< Iterator over variables
    const_reverse_iterator rend() const   { chk(); return maps_.rend(); }   ///< Iterator over variables

    /** print required by Testable for unit testing */
    void print(const std::string& str = "VectorValues: ") const; //

    /** equals required by Testable for unit testing */
    bool equals(const VectorValues& x, double tol = 1e-9) const; //

    /// @}
    /// @name Advanced constructors
    /// @{

    /** Construct from a container of variable dimensions (in variable order). */
    template<class CONTAINER>
    VectorValues(const CONTAINER& dimensions) { append(dimensions); } //

    /** Construct to hold nVars vectors of varDim dimension each. */
    VectorValues(Index nVars, size_t varDim) { resize(nVars, varDim); } //

    /** Named constructor to create a VectorValues that matches the structure of
     * the specified VectorValues, but do not initialize the new values. */
    static VectorValues SameStructure(const VectorValues& other); //

    /** Named constructor to create a VectorValues from a container of variable
     * dimensions that is filled with zeros. */
    template<class CONTAINER>
    static VectorValues Zero(const CONTAINER& dimensions);

    /// @}
    /// @name Advanced interface
    /// @{

    /** Resize this VectorValues to have identical structure to other, leaving
     * this VectorValues with uninitialized values.
     * @param other The VectorValues whose structure to copy
     */
    void resizeLike(const VectorValues& other); //

    /** Resize the VectorValues to hold \c nVars variables, each of dimension
     * \c varDim.  This function does not preserve any data, after calling
     * it all variables will be uninitialized.
     * @param nVars The number of variables to create
     * @param varDim The dimension of each variable
     */
    void resize(Index nVars, size_t varDim);

    /** Resize the VectorValues to contain variables of the dimensions stored
     * in \c dimensions.  The new variables are uninitialized, but this function
     * is used to pre-allocate space for performance.  This function does not
     * preserve any data, after calling it all variables will be uninitialized.
     * @param dimensions A container of the dimension of each variable to create.
     */
    template<class CONTAINER>
    void resize(const CONTAINER& dimensions);

    /** Append to the VectorValues to additionally contain variables of the
     * dimensions stored in \c dimensions.  The new variables are uninitialized,
     * but this function is used to pre-allocate space for performance.  This
     * function preserves the original data, so all previously-existing variables
     * are left unchanged.
     * @param dimensions A container of the dimension of each variable to create.
     */
    template<class CONTAINER>
    void append(const CONTAINER& dimensions); //

    /** Reference the entire solution vector (const version). */
    const Vector& vector() const { chk(); return values_; } //

    /** Reference the entire solution vector. */
    Vector& vector() { chk(); return values_; } //

    /** Check whether this VectorValues has the same structure, meaning has the
     * same number of variables and that all variables are of the same dimension,
     * as another VectorValues
     * @param other The other VectorValues with which to compare structure
     * @return \c true if the structure is the same, \c false if not.
     */
    bool hasSameStructure(const VectorValues& other) const;

    /** Dot product with another VectorValues, interpreting both as vectors of
     * their concatenated values. */
    double dot(const VectorValues& V) const {
      return gtsam::dot(this->values_, V.values_);
    }

    /**
     * + operator does element-wise addition.  Both VectorValues must have the
     * same structure (checked when NDEBUG is not defined).
     */
    VectorValues operator+(const VectorValues& c) const;

    /**
     * += operator does element-wise addition.  Both VectorValues must have the
     * same structure (checked when NDEBUG is not defined).
     */
    void operator+=(const VectorValues& c);

    /// @}

  private:
    // Verifies that the underlying Vector is consistent with the collection of SubVectors
    void chk() const;

    // Throw an exception if j does not exist
    void checkExists(Index j) const {
      chk();
      if(!exists(j))
        throw std::out_of_range("VectorValues: requested variable index is not in this VectorValues.");
    }

    // Resize
    void copyStructureFrom(const VectorValues& other);

  public:
    /// TODO: linear algebra interface seems to have been added for SPCG.
    friend size_t dim(const VectorValues& V) {
      return V.dim();
    }
    /// TODO: linear algebra interface seems to have been added for SPCG.
    friend double dot(const VectorValues& V1, const VectorValues& V2) {
      return gtsam::dot(V1.values_, V2.values_);
    }
    /// TODO: linear algebra interface seems to have been added for SPCG.
    friend void scal(double alpha, VectorValues& x) {
      gtsam::scal(alpha, x.values_);
    }
    /// TODO: linear algebra interface seems to have been added for SPCG.
    friend void axpy(double alpha, const VectorValues& x, VectorValues& y) {
      gtsam::axpy(alpha, x.values_, y.values_);
    }
    /// TODO: linear algebra interface seems to have been added for SPCG.
    friend void sqrt(VectorValues &x) {
      Vector y = gtsam::esqrt(x.values_);
      x.values_ = y;
    }

    /// TODO: linear algebra interface seems to have been added for SPCG.
    friend void ediv(const VectorValues& numerator,
        const VectorValues& denominator, VectorValues &result) {
      assert(
          numerator.dim() == denominator.dim() && denominator.dim() == result.dim());
      const size_t sz = result.dim();
      for (size_t i = 0; i < sz; ++i)
        result.values_[i] = numerator.values_[i] / denominator.values_[i];
    }

    /// TODO: linear algebra interface seems to have been added for SPCG.
    friend void edivInPlace(VectorValues& x, const VectorValues& y) {
      assert(x.dim() == y.dim());
      const size_t sz = x.dim();
      for (size_t i = 0; i < sz; ++i)
        x.values_[i] /= y.values_[i];
    }

  private:
    /** Serialization function */
    friend class boost::serialization::access;
    template<class ARCHIVE>
    void save(ARCHIVE & ar, const unsigned int version) const {
      // The maps_ stores pointers, so we serialize dimensions instead
      std::vector<size_t> dimensions(size());
      for(size_t j=0; j<maps_.size(); ++j)
        dimensions[j] = maps_[j].rows();
      ar & BOOST_SERIALIZATION_NVP(dimensions);
      ar & BOOST_SERIALIZATION_NVP(values_);
    }
    template<class ARCHIVE>
    void load(ARCHIVE & ar, const unsigned int version) {
      std::vector<size_t> dimensions;
      ar & BOOST_SERIALIZATION_NVP(dimensions); // Load dimensions
      resize(dimensions); // Allocate space for everything
      ar & BOOST_SERIALIZATION_NVP(values_); // Load values
      chk();
    }
    BOOST_SERIALIZATION_SPLIT_MEMBER()
  }; // VectorValues definition

  // Implementations of template and inline functions

  /* ************************************************************************* */
  template<class CONTAINER>
  void VectorValues::resize(const CONTAINER& dimensions) {
    maps_.clear();
    values_.resize(0);
    append(dimensions);
  }

  /* ************************************************************************* */
  template<class CONTAINER>
  void VectorValues::append(const CONTAINER& dimensions) {
    chk();
    int newDim = std::accumulate(dimensions.begin(), dimensions.end(), 0); // Sum of dimensions
    values_.conservativeResize(dim() + newDim);
    // Relocate existing maps
    int varStart = 0;
    for(size_t j = 0; j < maps_.size(); ++j) {
      new (&maps_[j]) SubVector(values_.segment(varStart, maps_[j].rows()));
      varStart += maps_[j].rows();
    }
    maps_.reserve(maps_.size() + dimensions.size());
    BOOST_FOREACH(size_t dim, dimensions) {
      maps_.push_back(values_.segment(varStart, dim));
      varStart += dim; // varStart is continued from first for loop
    }
  }

  /* ************************************************************************* */
  template<class CONTAINER>
  VectorValues VectorValues::Zero(const CONTAINER& dimensions) {
    VectorValues ret(dimensions);
    ret.vector() = Vector::Zero(ret.dim());
    return ret;
  }

  /* ************************************************************************* */
  inline void VectorValues::chk() const {
#ifndef NDEBUG
    // Check that the first SubVector points to the beginning of the Vector
    if(maps_.size() > 0) {
      assert(values_.data() == maps_[0].data());
      // Check that the end of the last SubVector points to the end of the Vector
      assert(values_.rows() == maps_.back().data() + maps_.back().rows() - maps_.front().data());
    }
#endif
  }

} // \namespace gtsam
