/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file Potentials.h
 * @date March 24, 2011
 * @author Frank Dellaert
 */

#pragma once

#include <gtsam/discrete/AlgebraicDecisionTree.h>
#include <gtsam/discrete/DiscreteKey.h>
#include <gtsam/inference/Key.h>

#include <boost/shared_ptr.hpp>
#include <set>

namespace gtsam {

  /**
   * A base class for both DiscreteFactor and DiscreteConditional
   */
  class Potentials: public AlgebraicDecisionTree<Key> {

  public:

    typedef AlgebraicDecisionTree<Key> ADT;

  protected:

    /// Cardinality for each key, used in combine
    std::map<Key,size_t> cardinalities_;

    /** Constructor from ColumnIndex, and ADT */
    Potentials(const ADT& potentials) :
        ADT(potentials) {
    }

    // Safe division for probabilities
    GTSAM_EXPORT static double safe_div(const double& a, const double& b);

//    // Apply either a permutation or a reduction
//    template<class P>
//    void remapIndices(const P& remapping);

  public:

    /** Default constructor for I/O */
    GTSAM_EXPORT Potentials();

    /** Constructor from Indices and ADT */
    GTSAM_EXPORT Potentials(const DiscreteKeys& keys, const ADT& decisionTree);

    /** Constructor from Indices and (string or doubles) */
    template<class SOURCE>
    Potentials(const DiscreteKeys& keys, SOURCE table) :
        ADT(keys, table), cardinalities_(keys.cardinalities()) {
    }

    // Testable
    GTSAM_EXPORT bool equals(const Potentials& other, double tol = 1e-9) const;
    GTSAM_EXPORT void print(const std::string& s = "Potentials: ",
        const KeyFormatter& formatter = DefaultKeyFormatter) const;

    size_t cardinality(Key j) const { return cardinalities_.at(j);}

//    /**
//     * @brief Permutes the keys in Potentials
//     *
//     * This permutes the Indices and performs necessary re-ordering of ADD.
//     * This is virtual so that derived types e.g. DecisionTreeFactor can
//     * re-implement it.
//     */
//    GTSAM_EXPORT virtual void permuteWithInverse(const Permutation& inversePermutation);
//
//    /**
//     * Apply a reduction, which is a remapping of variable indices.
//     */
//    GTSAM_EXPORT virtual void reduceWithInverse(const internal::Reduction& inverseReduction);

  }; // Potentials

// traits
template<> struct traits<Potentials> : public Testable<Potentials> {};
template<> struct traits<Potentials::ADT> : public Testable<Potentials::ADT> {};


} // namespace gtsam
