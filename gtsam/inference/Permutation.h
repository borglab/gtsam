/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    Permutation.h
 * @brief   
 * @author  Richard Roberts
 * @created Sep 12, 2010
 */

#pragma once

#include <gtsam/base/types.h>

#include <vector>
#include <string>
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
  std::vector<Index> rangeIndices_;

public:
  typedef boost::shared_ptr<Permutation> shared_ptr;
  typedef std::vector<Index>::const_iterator const_iterator;
  typedef std::vector<Index>::iterator iterator;

  /**
   * Create an empty permutation.  This cannot do anything, but you can later
   * assign to it.
   */
  Permutation() {}

  /**
   * Create an uninitialized permutation.  You must assign all values using the
   * square bracket [] operator or they will be undefined!
   */
  Permutation(Index nVars) : rangeIndices_(nVars) {}

  /**
   * Permute the given variable, i.e. determine it's new index after the
   * permutation.
   */
  Index operator[](Index variable) const { check(variable); return rangeIndices_[variable]; }
  Index& operator[](Index variable) { check(variable); return rangeIndices_[variable]; }

  /**
   * The number of variables in the range of this permutation, i.e. the output
   * space.
   */
  Index size() const { return rangeIndices_.size(); }

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
  static Permutation Identity(Index nVars);

  /**
   * Create a permutation that pulls the given variables to the front while
   * pushing the rest to the back.
   */
  static Permutation PullToFront(const std::vector<Index>& toFront, size_t size, bool filterDuplicates = false);

  /**
   * Create a permutation that pulls the given variables to the front while
   * pushing the rest to the back.
   */
  static Permutation PushToBack(const std::vector<Index>& toBack, size_t size, bool filterDuplicates = false);

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
  void check(Index variable) const { assert(variable < rangeIndices_.size()); }

  friend class Inference;
};


/**
 * Definition of Permuted class, see above comment for details.
 */
template<typename CONTAINER, typename VALUETYPE = typename CONTAINER::value_reference_type>
class Permuted {
  Permutation permutation_;
  CONTAINER& container_;
public:
  typedef VALUETYPE value_type;

  /** Construct as a permuted view on the Container.  The permutation is copied
   * but only a reference to the container is stored.
   */
  Permuted(const Permutation& permutation, CONTAINER& container) : permutation_(permutation), container_(container) {}

  /** Construct as a view on the Container with an identity permutation.  Only
   * a reference to the container is stored.
   */
  Permuted(CONTAINER& container) : permutation_(Permutation::Identity(container.size())), container_(container) {}

  /** Access the container through the permutation */
  value_type operator[](size_t index) const { return container_[permutation_[index]]; }

//  /** Access the container through the permutation (const version) */
//  const_value_type operator[](size_t index) const;

  /** Permute this view by applying a permutation to the underlying permutation */
  void permute(const Permutation& permutation) { assert(permutation.size() == this->size()); permutation_ = *permutation_.permute(permutation); }

  /** Access the underlying container */
  CONTAINER* operator->() { return &container_; }

  /** Access the underlying container (const version) */
  const CONTAINER* operator->() const { return &container_; }

  /** Size of the underlying container */
  size_t size() const { return container_.size(); }

  /** Access to the underlying container */
  CONTAINER& container() { return container_; }

  /** Access to the underlying container (const version) */
  const CONTAINER& container() const { return container_; }

  /** Access the underlying permutation */
  Permutation& permutation() { return permutation_; }
  const Permutation& permutation() const { return permutation_; }
};


}
