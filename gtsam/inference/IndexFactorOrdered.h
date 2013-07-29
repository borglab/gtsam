/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    IndexFactor.h
 * @author  Richard Roberts
 * @date    Oct 17, 2010
 */

#pragma once

#include <gtsam/inference/FactorOrdered.h>
#include <gtsam/inference/PermutationOrdered.h>

namespace gtsam {

  // Forward declaration of IndexConditional
  class IndexConditionalOrdered;

  /**
   * IndexFactor serves two purposes.  It is the base class for all linear
   * factors (GaussianFactor, JacobianFactor, HessianFactor), and also
   * functions as a symbolic factor with Index keys, used to do symbolic
   * elimination by JunctionTree.
   *
   * It derives from Factor with a key type of Index, an unsigned integer.
   * \nosubgrouping
   */
  class IndexFactorOrdered: public FactorOrdered<Index> {

  protected:

    /// @name Advanced Interface
    /// @{

    /// Internal function for checking class invariants (unique keys for this factor)
    GTSAM_EXPORT void assertInvariants() const;

    /// @}

  public:

    typedef IndexFactorOrdered This;
    typedef FactorOrdered<Index> Base;

    /** Elimination produces an IndexConditional */
    typedef IndexConditionalOrdered ConditionalType;

    /** Overriding the shared_ptr typedef */
    typedef boost::shared_ptr<IndexFactorOrdered> shared_ptr;

    /// @name Standard Interface
    /// @{

    /** Copy constructor */
    IndexFactorOrdered(const This& f) :
      Base(f) {
      assertInvariants();
    }

    /** Construct from derived type */
    GTSAM_EXPORT IndexFactorOrdered(const IndexConditionalOrdered& c);

    /** Default constructor for I/O */
    IndexFactorOrdered() {
      assertInvariants();
    }

    /** Construct unary factor */
    IndexFactorOrdered(Index j) :
      Base(j) {
      assertInvariants();
    }

    /** Construct binary factor */
    IndexFactorOrdered(Index j1, Index j2) :
      Base(j1, j2) {
      assertInvariants();
    }

    /** Construct ternary factor */
    IndexFactorOrdered(Index j1, Index j2, Index j3) :
      Base(j1, j2, j3) {
      assertInvariants();
    }

    /** Construct 4-way factor */
    IndexFactorOrdered(Index j1, Index j2, Index j3, Index j4) :
      Base(j1, j2, j3, j4) {
      assertInvariants();
    }

    /** Construct 5-way factor */
    IndexFactorOrdered(Index j1, Index j2, Index j3, Index j4, Index j5) :
      Base(j1, j2, j3, j4, j5) {
      assertInvariants();
    }

    /** Construct 6-way factor */
    IndexFactorOrdered(Index j1, Index j2, Index j3, Index j4, Index j5, Index j6) :
      Base(j1, j2, j3, j4, j5, j6) {
      assertInvariants();
    }

    /// @}
    /// @name Advanced Constructors
    /// @{

    /** Construct n-way factor */
    IndexFactorOrdered(const std::set<Index>& js) :
      Base(js) {
      assertInvariants();
    }

    /** Construct n-way factor */
    IndexFactorOrdered(const std::vector<Index>& js) :
      Base(js) {
      assertInvariants();
    }

    /** Constructor from a collection of keys */
    template<class KeyIterator> IndexFactorOrdered(KeyIterator beginKey,
        KeyIterator endKey) :
      Base(beginKey, endKey) {
      assertInvariants();
    }

    /// @}

#ifdef TRACK_ELIMINATE
    /**
     * eliminate the first variable involved in this factor
     * @return a conditional on the eliminated variable
     */
    GTSAM_EXPORT boost::shared_ptr<ConditionalType> eliminateFirst();

    /** eliminate the first nrFrontals frontal variables.*/
    GTSAM_EXPORT boost::shared_ptr<BayesNetOrdered<ConditionalType> > eliminate(size_t nrFrontals =
        1);
#endif

    /// @name Advanced Interface
    /// @{

    /**
     * Permutes the factor, but for efficiency requires the permutation
     * to already be inverted.
     */
    GTSAM_EXPORT void permuteWithInverse(const Permutation& inversePermutation);

    /**
     * Apply a reduction, which is a remapping of variable indices.
     */
    GTSAM_EXPORT void reduceWithInverse(const internal::Reduction& inverseReduction);

    virtual ~IndexFactorOrdered() {
    }

  private:
    /** Serialization function */
    friend class boost::serialization::access;
    template<class ARCHIVE>
    void serialize(ARCHIVE & ar, const unsigned int version) {
      ar & BOOST_SERIALIZATION_BASE_OBJECT_NVP(Base);
    }

    /// @}

  }; // IndexFactor

}
