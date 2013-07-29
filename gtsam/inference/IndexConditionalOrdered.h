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

#include <gtsam/global_includes.h>
#include <gtsam/inference/ConditionalOrdered.h>
#include <gtsam/inference/IndexFactorOrdered.h>
#include <gtsam/inference/PermutationOrdered.h>

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
  class IndexConditionalOrdered : public ConditionalOrdered<Index> {

  protected:

    // Checks that frontal indices are sorted and lower than parent indices
    GTSAM_EXPORT void assertInvariants() const;

  public:

    typedef IndexConditionalOrdered This;
    typedef ConditionalOrdered<Index> Base;
    typedef IndexFactorOrdered FactorType;
    typedef boost::shared_ptr<IndexConditionalOrdered> shared_ptr;

    /// @name Standard Constructors
    /// @{

    /** Empty Constructor to make serialization possible */
    IndexConditionalOrdered() { assertInvariants(); }

    /** No parents */
    IndexConditionalOrdered(Index j) : Base(j) { assertInvariants(); }

    /** Single parent */
    IndexConditionalOrdered(Index j, Index parent) : Base(j, parent) { assertInvariants(); }

    /** Two parents */
    IndexConditionalOrdered(Index j, Index parent1, Index parent2) : Base(j, parent1, parent2) { assertInvariants(); }

    /** Three parents */
    IndexConditionalOrdered(Index j, Index parent1, Index parent2, Index parent3) : Base(j, parent1, parent2, parent3) { assertInvariants(); }

    /// @}
    /// @name Advanced Constructors
    /// @{

    /** Constructor from a frontal variable and a vector of parents */
    IndexConditionalOrdered(Index j, const std::vector<Index>& parents) : Base(j, parents) {
      assertInvariants();
    }

    /** Constructor from keys and nr of frontal variables */
    IndexConditionalOrdered(const std::vector<Index>& keys, size_t nrFrontals) :
      Base(keys, nrFrontals) {
      assertInvariants();
    }

    /** Constructor from keys and nr of frontal variables */
    IndexConditionalOrdered(const std::list<Index>& keys, size_t nrFrontals) :
      Base(keys, nrFrontals) {
      assertInvariants();
    }

    /// @}
    /// @name Standard Interface
    /// @{

    /** Named constructor directly returning a shared pointer */
    template<class KEYS>
    static shared_ptr FromKeys(const KEYS& keys, size_t nrFrontals) {
      shared_ptr conditional(new IndexConditionalOrdered());
      conditional->keys_.assign(keys.begin(), keys.end());
      conditional->nrFrontals_ = nrFrontals;
      return conditional;
    }

    /** Convert to a factor */
    IndexFactorOrdered::shared_ptr toFactor() const {
      return IndexFactorOrdered::shared_ptr(new IndexFactorOrdered(*this));
    }

    /// @}
    /// @name Advanced Interface
    /// @{

    /** Permute the variables when only separator variables need to be permuted.
     * Returns true if any reordered variables appeared in the separator and
     * false if not.
     */
    GTSAM_EXPORT bool reduceSeparatorWithInverse(const internal::Reduction& inverseReduction);

    /**
     * Permutes the Conditional, but for efficiency requires the permutation
     * to already be inverted.
     */
    GTSAM_EXPORT void permuteWithInverse(const Permutation& inversePermutation);

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
