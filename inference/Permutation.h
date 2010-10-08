/**
 * @file    Permutation.h
 * @brief   
 * @author  Richard Roberts
 * @created Sep 12, 2010
 */

#pragma once

#include <gtsam/base/types.h>

#include <vector>
#include <iostream>
#include <boost/shared_ptr.hpp>

namespace gtsam {

class Inference;

/**
 * A permutation reorders variables, for example to reduce fill-in during
 * elimination.  To save computation, the permutation can be applied to
 * the necessary data structures only once, then multiple computations
 * performed on the permuted problem.  For example, in an iterative
 * non-linear setting, a permutation can be created from the symbolic graph
 * structure and applied to the ordering of nonlinear variables once, so
 * that linearized factor graphs are already correctly ordered and need
 * not be permuted.
 *
 * For convenience, there is also a helper class "Permuted" that transforms
 * arguments supplied through the square-bracket [] operator through the
 * permutation.  Note that this helper class stores a reference to the original
 * container.
 */
class Permutation {
protected:
  std::vector<varid_t> rangeIndices_;

public:
  typedef boost::shared_ptr<Permutation> shared_ptr;
  typedef std::vector<varid_t>::const_iterator const_iterator;
  typedef std::vector<varid_t>::iterator iterator;

  /**
   * Create an empty permutation.  This cannot do anything, but you can later
   * assign to it.
   */
  Permutation() {}

  /**
   * Create an uninitialized permutation.  You must assign all values using the
   * square bracket [] operator or they will be undefined!
   */
  Permutation(varid_t nVars) : rangeIndices_(nVars) {}

  /**
   * Permute the given variable, i.e. determine it's new index after the
   * permutation.
   */
  varid_t operator[](varid_t variable) const { check(variable); return rangeIndices_[variable]; }
  varid_t& operator[](varid_t variable) { check(variable); return rangeIndices_[variable]; }

  /**
   * The number of variables in the range of this permutation, i.e. the output
   * space.
   */
  varid_t size() const { return rangeIndices_.size(); }

  /** Whether the permutation contains any entries */
  bool empty() const { return rangeIndices_.empty(); }

  /**
   * Resize the permutation.  You must fill in the new entries if new new size
   * is larger than the old one.  If the new size is smaller, entries from the
   * end are discarded.
   */
  void resize(size_t newSize) { rangeIndices_.resize(newSize); }

  /**
   * Return an identity permutation.
   */
  static Permutation Identity(varid_t nVars);

  iterator begin() { return rangeIndices_.begin(); }
  const_iterator begin() const { return rangeIndices_.begin(); }
  iterator end() { return rangeIndices_.end(); }
  const_iterator end() const { return rangeIndices_.end(); }

  /** Print for debugging */
  void print(const std::string& str = "Permutation: ") const;

  /** Equals */
  bool equals(const Permutation& rhs) const { return rangeIndices_ == rhs.rangeIndices_; }

  /**
   * Syntactic sugar for accessing another container through a permutation.
   * Allows the syntax:
   *   Permuted<Container> permuted(permutation, container);
   *   permuted[index1];
   *   permuted[index2];
   * which is equivalent to:
   *   container[permutation[index1]];
   *   container[permutation[index2]];
   * but more concise.
   */

  /**
   * Permute the permutation, p1.permute(p2)[i] is equivalent to p2[p1[i]].
   */
  Permutation::shared_ptr permute(const Permutation& permutation) const;

  /**
   * A partial permutation, reorders the variables selected by selector through
   * partialPermutation.  selector and partialPermutation should have the same
   * size, this is checked if NDEBUG is not defined.
   */
  Permutation::shared_ptr partialPermutation(const Permutation& selector, const Permutation& partialPermutation) const;

  /**
   * Return the inverse permutation.  This is only possible if this is a non-
   * reducing permutation, that is, (*this)[i] < this->size() for all i<size().
   * If NDEBUG is not defined, this conditional will be checked.
   */
  Permutation::shared_ptr inverse() const;

protected:
  void check(varid_t variable) const { assert(variable < rangeIndices_.size()); }

  friend class Inference;
};


/**
 * Definition of Permuted class, see above comment for details.
 */
template<typename Container, typename ValueType = typename Container::value_reference_type>
class Permuted {
  Permutation permutation_;
  Container& container_;
public:
  typedef ValueType value_type;

  /** Construct as a permuted view on the Container.  The permutation is copied
   * but only a reference to the container is stored.
   */
  Permuted(const Permutation& permutation, Container& container) : permutation_(permutation), container_(container) {}

  /** Construct as a view on the Container with an identity permutation.  Only
   * a reference to the container is stored.
   */
  Permuted(Container& container) : permutation_(Permutation::Identity(container.size())), container_(container) {}

  /** Access the container through the permutation */
  value_type operator[](size_t index) const { return container_[permutation_[index]]; }

//  /** Access the container through the permutation (const version) */
//  const_value_type operator[](size_t index) const;

  /** Permute this view by applying a permutation to the underlying permutation */
  void permute(const Permutation& permutation) { assert(permutation.size() == this->size()); permutation_ = *permutation_.permute(permutation); }

  /** Access the underlying container */
  Container* operator->() { return &container_; }

  /** Access the underlying container (const version) */
  const Container* operator->() const { return &container_; }

  /** Size of the underlying container */
  size_t size() const { return container_.size(); }

  /** Access to the underlying container */
  Container& container() { return container_; }

  /** Access to the underlying container (const version) */
  const Container& container() const { return container_; }

  /** Access the underlying permutation */
  Permutation& permutation() { return permutation_; }
  const Permutation& permutation() const { return permutation_; }
};


/* ************************************************************************* */
inline Permutation Permutation::Identity(varid_t nVars) {
  Permutation ret(nVars);
  for(varid_t i=0; i<nVars; ++i)
    ret[i] = i;
  return ret;
}

/* ************************************************************************* */
inline Permutation::shared_ptr Permutation::permute(const Permutation& permutation) const {
  const size_t nVars = permutation.size();
  Permutation::shared_ptr result(new Permutation(nVars));
  for(size_t j=0; j<nVars; ++j) {
    assert(permutation[j] < rangeIndices_.size());
    (*result)[j] = operator[](permutation[j]);
  }
  return result;
}

/* ************************************************************************* */
inline Permutation::shared_ptr Permutation::partialPermutation(
    const Permutation& selector, const Permutation& partialPermutation) const {
  assert(selector.size() == partialPermutation.size());
  Permutation::shared_ptr result(new Permutation(*this));

  for(varid_t subsetPos=0; subsetPos<selector.size(); ++subsetPos)
    (*result)[selector[subsetPos]] = (*this)[selector[partialPermutation[subsetPos]]];

  return result;
}

/* ************************************************************************* */
inline Permutation::shared_ptr Permutation::inverse() const {
  Permutation::shared_ptr result(new Permutation(this->size()));
  for(varid_t i=0; i<this->size(); ++i) {
    assert((*this)[i] < this->size());
    (*result)[(*this)[i]] = i;
  }
  return result;
}

/* ************************************************************************* */
inline void Permutation::print(const std::string& str) const {
  std::cout << str;
  BOOST_FOREACH(varid_t s, rangeIndices_) { std::cout << s << " "; }
  std::cout << std::endl;
}

}
