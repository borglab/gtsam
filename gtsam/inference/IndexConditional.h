/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    IndexConditional.h
 * @author  Richard Roberts
 * @date    Oct 17, 2010
 */

#pragma once

#include <gtsam/base/types.h>
#include <gtsam/inference/Conditional.h>
#include <gtsam/inference/IndexFactor.h>
#include <gtsam/inference/Permutation.h>

namespace gtsam {

  /**
   * IndexConditional serves two purposes.  It is the base class for
   * GaussianConditional, and also functions as a symbolic conditional with
   * Index keys, produced by symbolic elimination of IndexFactor.
   *
   * It derives from Conditional with a key type of Index, which is an
   * unsigned integer.
   * \nosubgrouping
   */
  class IndexConditional : public Conditional<Index> {

  protected:

    // Checks that frontal indices are sorted and lower than parent indices
    void assertInvariants() const;

  public:

    typedef IndexConditional This;
    typedef Conditional<Index> Base;
    typedef IndexFactor FactorType;
    typedef boost::shared_ptr<IndexConditional> shared_ptr;

  	/// @name Standard Constructors
  	/// @{

    /** Empty Constructor to make serialization possible */
    IndexConditional() { assertInvariants(); }

    /** No parents */
    IndexConditional(Index j) : Base(j) { assertInvariants(); }

    /** Single parent */
    IndexConditional(Index j, Index parent) : Base(j, parent) { assertInvariants(); }

    /** Two parents */
    IndexConditional(Index j, Index parent1, Index parent2) : Base(j, parent1, parent2) { assertInvariants(); }

    /** Three parents */
    IndexConditional(Index j, Index parent1, Index parent2, Index parent3) : Base(j, parent1, parent2, parent3) { assertInvariants(); }

  	/// @}
  	/// @name Advanced Constructors
  	/// @{

    /** Constructor from a frontal variable and a vector of parents */
		IndexConditional(Index j, const std::vector<Index>& parents) : Base(j, parents) {
			assertInvariants();
		}

    /** Constructor from keys and nr of frontal variables */
		IndexConditional(const std::vector<Index>& keys, size_t nrFrontals) :
			Base(keys, nrFrontals) {
			assertInvariants();
		}

		/// @}
		/// @name Standard Interface
		/// @{

		/** Named constructor directly returning a shared pointer */
		template<class KEYS>
		static shared_ptr FromKeys(const KEYS& keys, size_t nrFrontals) {
		  shared_ptr conditional(new IndexConditional());
		  conditional->keys_.assign(keys.begin(), keys.end());
		  conditional->nrFrontals_ = nrFrontals;
		  return conditional;
		}

    /** Convert to a factor */
    IndexFactor::shared_ptr toFactor() const {
      return IndexFactor::shared_ptr(new IndexFactor(*this));
    }

  	/// @}
  	/// @name Advanced Interface
  	/// @{

    /** Permute the variables when only separator variables need to be permuted.
     * Returns true if any reordered variables appeared in the separator and
     * false if not.
     */
    bool permuteSeparatorWithInverse(const Permutation& inversePermutation);

    /**
     * Permutes the Conditional, but for efficiency requires the permutation
     * to already be inverted.
     */
    void permuteWithInverse(const Permutation& inversePermutation);

  private:
  	/** Serialization function */
  	friend class boost::serialization::access;
  	template<class Archive>
  	void serialize(Archive & ar, const unsigned int version) {
  		ar & BOOST_SERIALIZATION_BASE_OBJECT_NVP(Base);
  	}

  	/// @}

  };

}
