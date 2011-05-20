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

#include <gtsam/base/Testable.h>
#include <gtsam/base/Vector.h>
#include <gtsam/base/types.h>

#include <boost/foreach.hpp>
#include <boost/shared_ptr.hpp>

#include <vector>

namespace gtsam {

class VectorValues : public Testable<VectorValues> {
protected:
  Vector values_;
  std::vector<size_t> varStarts_; // start at 0 with size nVars + 1

public:
  template<class C> class _impl_iterator;  // Forward declaration of iterator implementation
  typedef boost::shared_ptr<VectorValues> shared_ptr;
  typedef _impl_iterator<VectorValues> iterator;
  typedef _impl_iterator<const VectorValues> const_iterator;
  typedef SubVector value_reference_type;
  typedef ConstSubVector const_value_reference_type;
  typedef SubVector mapped_type;
  typedef ConstSubVector const_mapped_type;

  /**
   * Default constructor creates an empty VectorValues.  reserve(...) must be
   * called to allocate space before any values can be added.  This prevents
   * slow reallocation of space at runtime.
   */
  VectorValues() : varStarts_(1,0) {}
  VectorValues(const VectorValues &V) : values_(V.values_), varStarts_(V.varStarts_) {}

  /** Construct from a container of variable dimensions (in variable order). */
  template<class CONTAINER>
  VectorValues(const CONTAINER& dimensions);

  /** Construct to hold nVars vectors of varDim dimension each. */
  VectorValues(Index nVars, size_t varDim);

  /** Construct from a container of variable dimensions in variable order and
   * a combined Vector of all of the variables in order.
   */
  VectorValues(const std::vector<size_t>& dimensions, const Vector& values);

  /** Construct forom the variable dimensions in varaible order and a double array that contains actual values */
  VectorValues(const std::vector<size_t>& dimensions, const double* values);

  /** Named constructor to create a VectorValues that matches the structure of
   * the specified VectorValues, but do not initialize the new values.
   */
  static VectorValues SameStructure(const VectorValues& otherValues);

  /** Element access */
  mapped_type operator[](Index variable);
  const_mapped_type operator[](Index variable) const;

  /** Number of elements */
  Index size() const { return varStarts_.size()-1; }

  /** Total dimensionality used (could be smaller than what has been allocated
   * with reserve(...) ).
   */
  size_t dim() const { return varStarts_.back(); }

  /* dot product */
  double dot(const VectorValues& V) const { return gtsam::dot(this->values_, V.values_) ; }

  /** Total dimensions capacity allocated */
  size_t dimCapacity() const { return values_.size(); }

  /** Iterator access */
  iterator begin() { return _impl_iterator<VectorValues>(*this, 0); }
  const_iterator begin() const { return _impl_iterator<const VectorValues>(*this, 0); }
  iterator end() { return _impl_iterator<VectorValues>(*this, varStarts_.size()-1); }
  const_iterator end() const { return _impl_iterator<const VectorValues>(*this, varStarts_.size()-1); }

  /** Reference the entire solution vector (const version). */
  const Vector& vector() const { return values_; }

  /** Reference the entire solution vector. */
  Vector& vector() { return values_; }

  /** Reserve space for a total number of variables and dimensionality */
  void reserve(Index nVars, size_t totalDims) { values_.resize(totalDims); varStarts_.reserve(nVars+1); }

  /**
   * Append a variable using the next variable ID, and return that ID.  Space
   * must have been allocated ahead of time using reserve(...).
   */
  Index push_back_preallocated(const Vector& vector) {
    Index var = varStarts_.size()-1;
    varStarts_.push_back(varStarts_.back()+vector.size());
    this->operator[](var) = vector;  // This will assert that values_ has enough allocated space.
    return var;
  }

  /** Set all elements to zero */
  void makeZero() { values_.setZero(); }

  /** print required by Testable for unit testing */
  void print(const std::string& str = "VectorValues: ") const {
    std::cout << str << ": " << varStarts_.size()-1 << " elements\n";
    for(Index var=0; var<size(); ++var) {
      std::cout << "  " << var << " " << operator[](var) << "\n";
    }
    std::cout.flush();
  }

  /** equals required by Testable for unit testing */
  bool equals(const VectorValues& x, double tol=1e-9) const {
  	return varStarts_ == x.varStarts_ && equal_with_abs_tol(values_, x.values_, tol);
  }

  /** + operator simply adds Vectors.  This checks for structural equivalence
   * when NDEBUG is not defined.
   */
  VectorValues operator+(const VectorValues& c) const {
    assert(varStarts_ == c.varStarts_);
    VectorValues result;
    result.varStarts_ = varStarts_;
    result.values_ = values_.head(varStarts_.back()) + c.values_.head(varStarts_.back());
//    result.values_ = boost::numeric::ublas::project(values_, boost::numeric::ublas::range(0, varStarts_.back())) +
//    				 boost::numeric::ublas::project(c.values_, boost::numeric::ublas::range(0, c.varStarts_.back()));
    return result;
  }

  void operator+=(const VectorValues& c) {
    assert(varStarts_ == c.varStarts_);
    this->values_ += c.values_.head(varStarts_.back());
//    this->values_ = boost::numeric::ublas::project(this->values_, boost::numeric::ublas::range(0, varStarts_.back())) +
//    				boost::numeric::ublas::project(c.values_, boost::numeric::ublas::range(0, c.varStarts_.back()));
  }


  /**
   * Iterator (handles both iterator and const_iterator depending on whether
   * the template type is const.
   */
  template<class C>
  class _impl_iterator {
  protected:
    C& config_;
    Index curVariable_;

    _impl_iterator(C& config, Index curVariable) : config_(config), curVariable_(curVariable) {}
    void checkCompat(const _impl_iterator<C>& r) { assert(&config_ == &r.config_); }
    friend class VectorValues;

  public:
    typedef typename const_selector<C, VectorValues, VectorValues::mapped_type, VectorValues::const_mapped_type>::type value_type;
    _impl_iterator<C>& operator++() { ++curVariable_; return *this; }
    _impl_iterator<C>& operator--() { --curVariable_; return *this; }
    _impl_iterator<C>& operator++(int) { throw std::runtime_error("Use prefix ++ operator"); }
    _impl_iterator<C>& operator--(int) { throw std::runtime_error("Use prefix -- operator"); }
    _impl_iterator<C>& operator+=(ptrdiff_t step) { curVariable_ += step; return *this; }
    _impl_iterator<C>& operator-=(ptrdiff_t step) { curVariable_ += step; return *this; }
    ptrdiff_t operator-(const _impl_iterator<C>& r) { checkCompat(r); return curVariable_ - r.curVariable_; }
    bool operator==(const _impl_iterator<C>& r) { checkCompat(r); return curVariable_ == r.curVariable_; }
    bool operator!=(const _impl_iterator<C>& r) { checkCompat(r); return curVariable_ != r.curVariable_; }
    value_type operator*() { return config_[curVariable_]; }
  };

  static VectorValues zero(const VectorValues& x) {
  	VectorValues cloned(x);
  	cloned.makeZero();
  	return cloned;
  }

protected:
  void checkVariable(Index variable) const { assert(variable < varStarts_.size()-1); }


public:
  friend size_t dim(const VectorValues& V) { return V.varStarts_.back(); }
  friend double dot(const VectorValues& V1, const VectorValues& V2) { return gtsam::dot(V1.values_, V2.values_) ; }
  friend void scal(double alpha, VectorValues& x) {	gtsam::scal(alpha, x.values_) ; }
  friend void axpy(double alpha, const VectorValues& x, VectorValues& y) { gtsam::axpy(alpha, x.values_, y.values_) ; }
  friend void sqrt(VectorValues &x) { Vector y = gtsam::esqrt(x.values_); x.values_ = y; }

  friend void ediv(const VectorValues& numerator, const VectorValues& denominator, VectorValues &result) {
    assert(numerator.dim() == denominator.dim() && denominator.dim() == result.dim()) ;
    const size_t sz = result.dim();
    for ( size_t i = 0 ; i < sz ; ++i ) result.values_[i] = numerator.values_[i]/denominator.values_[i] ;
  }

  friend void edivInPlace(VectorValues& x, const VectorValues& y) {
    assert(x.dim() == y.dim());
    const size_t sz = x.dim();
    for ( size_t i = 0 ; i < sz ; ++i ) x.values_[i] /= y.values_[i] ;
  }

  // check whether there's a zero in the vector
  friend bool anyZero(const VectorValues& x, double tol=1e-5) {
	  bool flag = false ;
	  size_t i=0;
	  for (const double *v = x.values_.data(); i< (size_t) x.values_.size(); ++v) {
//	  BOOST_FOREACH(const double &v, x.values_) {
//	  	double v = x(i);
		  if ( *v < tol && *v > -tol) {
			  flag = true ;
			  break;
		  }
		  ++i;
	  }
	  return flag;
  }
};

/// Implementations of functions

template<class CONTAINER>
inline VectorValues::VectorValues(const CONTAINER& dimensions) : varStarts_(dimensions.size()+1) {
  varStarts_[0] = 0;
  size_t varStart = 0;
  Index var = 0;
  BOOST_FOREACH(size_t dim, dimensions) {
    varStarts_[++var] = (varStart += dim);
  }
  values_.resize(varStarts_.back());
}

inline VectorValues::VectorValues(Index nVars, size_t varDim) : varStarts_(nVars+1) {
  varStarts_[0] = 0;
  size_t varStart = 0;
  for(Index j=1; j<=nVars; ++j)
    varStarts_[j] = (varStart += varDim);
  values_.resize(varStarts_.back());
}

inline VectorValues::VectorValues(const std::vector<size_t>& dimensions, const Vector& values) :
    values_(values), varStarts_(dimensions.size()+1) {
  varStarts_[0] = 0;
  size_t varStart = 0;
  Index var = 0;
  BOOST_FOREACH(size_t dim, dimensions) {
    varStarts_[++var] = (varStart += dim);
  }
  assert(varStarts_.back() == (size_t) values.size());
}

inline VectorValues::VectorValues(const std::vector<size_t>& dimensions, const double* values) :
		varStarts_(dimensions.size()+1) {
	varStarts_[0] = 0;
	size_t varStart = 0;
	Index var = 0;
	BOOST_FOREACH(size_t dim, dimensions) {
		varStarts_[++var] = (varStart += dim);
	}
	values_ = Vector_(varStart, values);
}

inline VectorValues VectorValues::SameStructure(const VectorValues& otherValues) {
  VectorValues ret;
  ret.varStarts_ = otherValues.varStarts_;
  ret.values_.resize(ret.varStarts_.back(), false);
  return ret;
}

inline VectorValues::mapped_type VectorValues::operator[](Index variable) {
  checkVariable(variable);
  const size_t start = varStarts_[variable], n = varStarts_[variable+1] - start;
  return values_.segment(start, n);
//  return boost::numeric::ublas::project(values_,
//      boost::numeric::ublas::range(varStarts_[variable], varStarts_[variable+1]));
}

inline VectorValues::const_mapped_type VectorValues::operator[](Index variable) const {
  checkVariable(variable);
  const size_t start = varStarts_[variable], n = varStarts_[variable+1] - start;
  return values_.segment(start, n);
//  return boost::numeric::ublas::project(values_,
//      boost::numeric::ublas::range(varStarts_[variable], varStarts_[variable+1]));
}

struct DimSpec: public std::vector<size_t> {

  typedef std::vector<size_t> Base ;
  typedef boost::shared_ptr<DimSpec> shared_ptr;

  DimSpec ():Base() {}
  DimSpec (size_t n):Base(n) {}
  DimSpec (size_t n, size_t init) : Base(n,init) {}
  DimSpec (const VectorValues &V) : Base(V.size()) {
    const size_t n = V.size() ;
    for ( size_t i = 0 ; i < n ; ++i ) {
      (*this)[i] = V[i].size() ;
    }
  }
} ;



}
