/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    BayesTreeCliqueBase.h
 * @brief   Base class for cliques of a BayesTree
 * @author  Richard Roberts and Frank Dellaert
 */

#pragma once

#include <gtsam/base/types.h>
#include <gtsam/inference/FactorGraphUnordered.h>
#include <gtsam/inference/BayesNetUnordered.h>

namespace gtsam {
  template<class CLIQUE> class BayesTreeUnordered;
}

namespace gtsam {

  /**
   * This is the base class for BayesTree cliques.  The default and standard derived type is
   * BayesTreeClique, but some algorithms, like iSAM2, use a different clique type in order to store
   * extra data along with the clique.
   *
   * This class is templated on the derived class (i.e. the curiously recursive template pattern).
   * The advantage of this over using virtual classes is that it avoids the need for casting to get
   * the derived type.  This is possible because all cliques in a BayesTree are the same type - if
   * they were not then we'd need a virtual class.
   *
   * @tparam DERIVED The derived clique type.
   * @tparam CONDITIONAL The conditional type.
   * \nosubgrouping */
  template<class DERIVED, class FACTORGRAPH, class BAYESNET>
  struct BayesTreeCliqueBaseUnordered {

  private:
    typedef BayesTreeCliqueBaseUnordered<DERIVED, FACTORGRAPH, BAYESNET> This;
    typedef DERIVED DerivedType;
    typedef boost::shared_ptr<This> shared_ptr;
    typedef boost::weak_ptr<This> weak_ptr;
    typedef boost::shared_ptr<DerivedType> derived_ptr;
    typedef boost::weak_ptr<DerivedType> derived_weak_ptr;

  public:
    typedef FACTORGRAPH FactorGraphType;
    typedef BAYESNET BayesNetType;
    typedef typename BayesNetType::ConditionalType ConditionalType;
    typedef boost::shared_ptr<ConditionalType> sharedConditional;
    typedef typename FactorGraphType::FactorType FactorType;
    typedef typename FactorGraphType::Eliminate Eliminate;

  protected:

    /// @name Standard Constructors
    /// @{

    /** Default constructor */
    BayesTreeCliqueBaseUnordered() {}

    /** Construct from a conditional, leaving parent and child pointers uninitialized */
    BayesTreeCliqueBaseUnordered(const sharedConditional& conditional) : conditional_(conditional) {}

    /// @}

    /// This stores the Cached separator margnal P(S)
    mutable boost::optional<FactorGraphType> cachedSeparatorMarginal_;

  public:
    sharedConditional conditional_;
    derived_weak_ptr parent_;
    std::vector<derived_ptr> children;

    /// @name Testable
    /// @{

    /** check equality */
    bool equals(const DERIVED& other, double tol = 1e-9) const {
      return (!conditional_ && !other.conditional())
          || conditional_->equals(*other.conditional(), tol);
    }

    /** print this node */
    void print(const std::string& s = "", const KeyFormatter& keyFormatter = DefaultKeyFormatter) const;

    /// @}
    /// @name Standard Interface
    /// @{

    /** Access the conditional */
    const sharedConditional& conditional() const { return conditional_; }

    /** is this the root of a Bayes tree ? */
    inline bool isRoot() const { return parent_.expired(); }

    /** The size of subtree rooted at this clique, i.e., nr of Cliques */
    size_t treeSize() const;

    /** Collect number of cliques with cached separator marginals */
    size_t numCachedSeparatorMarginals() const;

    /** return a shared_ptr to the parent clique */
    derived_ptr parent() const { return parent_.lock(); }

    /// @}
    /// @name Advanced Interface
    /// @{

    ///** return the conditional P(S|Root) on the separator given the root */
    //BayesNetType shortcut(derived_ptr root, Eliminate function) const;

    ///** return the marginal P(S) on the separator */
    //FactorGraphType separatorMarginal(derived_ptr root, Eliminate function) const;

    ///** return the marginal P(C) of the clique, using marginal caching */
    //FactorGraphType marginal2(derived_ptr root, Eliminate function) const;

    /**
     * This deletes the cached shortcuts of all cliques (subtree) below this clique.
     * This is performed when the bayes tree is modified.
     */
    void deleteCachedShortcuts();

    const boost::optional<FactorGraphType>& cachedSeparatorMarginal() const {
      return cachedSeparatorMarginal_; }

    friend class BayesTreeUnordered<DerivedType>;

  protected:

    ///// Calculate set \f$ S \setminus B \f$ for shortcut calculations
    //std::vector<Key> separator_setminus_B(derived_ptr B) const;

    ///// Calculate set \f$ S_p \cap B \f$ for shortcut calculations
    //std::vector<Key> parent_separator_intersection_B(derived_ptr B) const;

    ///**
    // * Determine variable indices to keep in recursive separator shortcut calculation
    // * The factor graph p_Cp_B has keys from the parent clique Cp and from B.
    // * But we only keep the variables not in S union B.
    // */
    //std::vector<Key> shortcut_indices(derived_ptr B, const FactorGraphType& p_Cp_B) const;

    /** Non-recursive delete cached shortcuts and marginals - internal only. */
    void deleteCachedShortcutsNonRecursive() { cachedSeparatorMarginal_ = boost::none; }

  private:

    /** Serialization function */
    friend class boost::serialization::access;
    template<class ARCHIVE>
    void serialize(ARCHIVE & ar, const unsigned int version) {
      ar & BOOST_SERIALIZATION_NVP(conditional_);
      ar & BOOST_SERIALIZATION_NVP(parent_);
      ar & BOOST_SERIALIZATION_NVP(children_);
    }

    /// @}

  };

}
