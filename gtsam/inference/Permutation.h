/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    Permutation.h
 * @author  Richard Roberts
 * @date    Sep 12, 2010
 */

#pragma once

#include <vector>
#include <string>
#include <iostream>
#include <boost/shared_ptr.hpp>

#include <gtsam/base/types.h>

namespace gtsam {

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
 * Permutations can be considered to a 1-1 mapping from an original set of indices
 * to a different set of indices.  Permutations can be composed and inverted
 * in order to create new indexing for a structure.
 * \nosubgrouping
 */
class Permutation {
protected:
  std::vector<Index> rangeIndices_;

public:
  typedef boost::shared_ptr<Permutation> shared_ptr;
  typedef std::vector<Index>::const_iterator const_iterator;
  typedef std::vector<Index>::iterator iterator;

	/// @name Standard Constructors
	/// @{

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

	/// @}
	/// @name Testable
	/// @{

  /** Print */
  void print(const std::string& str = "Permutation: ") const;

  /** Check equality */
  bool equals(const Permutation& rhs, double tol=0.0) const { return rangeIndices_ == rhs.rangeIndices_; }


	/// @}
	/// @name Standard Interface
	/// @{

  /**
   * Return the new index of the supplied variable after the permutation
   */
  Index operator[](Index variable) const { check(variable); return rangeIndices_[variable]; }

  /**
   * Return the new index of the supplied variable after the permutation. This version allows modification.
   */
  Index& operator[](Index variable) { check(variable); return rangeIndices_[variable]; }

  /**
   * Return the new index of the supplied variable after the permutation. Synonym for operator[](Index).
   */
  Index at(Index variable) const { return operator[](variable); }

  /**
   * Return the new index of the supplied variable after the permutation. This version allows modification.   Synonym for operator[](Index).
   */
  Index& at(Index variable) { return operator[](variable); }

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
   * Create a permutation that pushes the given variables to the back while
   * pulling the rest to the front.
   */
  static Permutation PushToBack(const std::vector<Index>& toBack, size_t size, bool filterDuplicates = false);


  /**
   * Permute the permutation, p1.permute(p2)[i] is equivalent to p1[p2[i]].
   */
  Permutation::shared_ptr permute(const Permutation& permutation) const;

  /**
   * Return the inverse permutation.  This is only possible if this is a non-
   * reducing permutation, that is, (*this)[i] < this->size() for all i<size().
   * If NDEBUG is not defined, this conditional will be checked.
   */
  Permutation::shared_ptr inverse() const;

  const_iterator begin() const { return rangeIndices_.begin(); }	///< Iterate through the indices
  const_iterator end() const { return rangeIndices_.end(); }			///< Iterate through the indices

	/** Apply the permutation to a collection, which must have operator[] defined.
	 * Note that permutable gtsam data structures typically have their own
	 * permute function to apply a permutation.  Permutation::applyToCollection is
	 * a generic function, e.g. for STL classes.
	 * @param input The input collection.
	 * @param output The preallocated output collection, which is assigned output[i] = input[permutation[i]]
	 */
	template<typename INPUT_COLLECTION, typename OUTPUT_COLLECTION>
	void applyToCollection(OUTPUT_COLLECTION& output, const INPUT_COLLECTION& input) const {
		for(size_t i = 0; i < output.size(); ++i) output[i] = input[(*this)[i]]; }


	/// @}
	/// @name Advanced Interface
	/// @{


  /**
   * A partial permutation, reorders the variables selected by selector through
   * partialPermutation.  selector and partialPermutation should have the same
   * size, this is checked if NDEBUG is not defined.
   */
  Permutation::shared_ptr partialPermutation(const Permutation& selector, const Permutation& partialPermutation) const;

  iterator begin() { return rangeIndices_.begin(); }	///< Iterate through the indices
  iterator end() { return rangeIndices_.end(); }			///< Iterate through the indices

protected:
  void check(Index variable) const { assert(variable < rangeIndices_.size()); }

	/// @}
};

}
