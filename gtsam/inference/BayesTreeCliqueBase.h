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

#include <gtsam/inference/Key.h>
#include <gtsam/inference/Ordering.h>
#include <gtsam/base/types.h>
#include <gtsam/base/FastVector.h>

#include <string>
#include <mutex>
#include <optional>

namespace gtsam {

  // Forward declarations
  template<class CLIQUE> class BayesTree;
  template<class GRAPH> struct EliminationTraits;

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
  template<class DERIVED, class FACTORGRAPH>
  class BayesTreeCliqueBase
  {
  private:
    typedef BayesTreeCliqueBase<DERIVED, FACTORGRAPH> This;
    typedef DERIVED DerivedType;
    typedef EliminationTraits<FACTORGRAPH> EliminationTraitsType;
    typedef std::shared_ptr<This> shared_ptr;
    typedef std::weak_ptr<This> weak_ptr;
    typedef std::shared_ptr<DerivedType> derived_ptr;
    typedef std::weak_ptr<DerivedType> derived_weak_ptr;

  public:
    typedef FACTORGRAPH FactorGraphType;
    typedef typename EliminationTraitsType::BayesNetType BayesNetType;
    typedef typename BayesNetType::ConditionalType ConditionalType;
    typedef std::shared_ptr<ConditionalType> sharedConditional;
    typedef typename FactorGraphType::FactorType FactorType;
    typedef typename FactorGraphType::Eliminate Eliminate;

  protected:

    /// @name Standard Constructors
    /// @{

    /// Default constructor
    BayesTreeCliqueBase() : problemSize_(1) {}

    /// Construct from a conditional, leaving parent and child pointers
    /// uninitialized.
    BayesTreeCliqueBase(const sharedConditional& conditional)
        : conditional_(conditional), problemSize_(1) {}

    /// Shallow copy constructor.
    BayesTreeCliqueBase(const BayesTreeCliqueBase& c)
        : conditional_(c.conditional_),
          parent_(c.parent_),
          children(c.children),
          problemSize_(c.problemSize_),
          is_root(c.is_root) {}

    /// Shallow copy assignment constructor
    BayesTreeCliqueBase& operator=(const BayesTreeCliqueBase& c) {
      conditional_ = c.conditional_;
      parent_ = c.parent_;
      children = c.children;
      problemSize_ = c.problemSize_;
      is_root = c.is_root;
      return *this;
    }

    // Virtual destructor.
    virtual ~BayesTreeCliqueBase() {}

    /// @}

    /// This stores the Cached separator marginal P(S)
    mutable std::optional<FactorGraphType> cachedSeparatorMarginal_;
    /// This protects Cached seperator marginal P(S) from concurrent read/writes
    /// as many the functions which access it are const (hence the mutable)
    /// leading to the false impression that these const functions are thread-safe
    /// which is not true due to these mutable values. This is fixed by applying this mutex.
    mutable std::mutex cachedSeparatorMarginalMutex_;

  public:
    sharedConditional conditional_;
    derived_weak_ptr parent_;
    FastVector<derived_ptr> children;
    int problemSize_;

    bool is_root = false;

    /// Fill the elimination result produced during elimination.  Here this just stores the
    /// conditional and ignores the remaining factor, but this is overridden in ISAM2Clique
    /// to also cache the remaining factor.
    void setEliminationResult(const typename FactorGraphType::EliminationResult& eliminationResult);

    /// @name Testable
    /// @{

    /** check equality */
    bool equals(const DERIVED& other, double tol = 1e-9) const;

    /** print this node */
    virtual void print(
        const std::string& s = "",
        const KeyFormatter& keyFormatter = DefaultKeyFormatter) const;

    /// @}
    /// @name Standard Interface
    /// @{

    /** Access the conditional */
    const sharedConditional& conditional() const { return conditional_; }

    /// Return true if this clique is the root of a Bayes tree. 
    inline bool isRoot() const { return parent_.expired(); }

    /// Return the number of children.
    size_t nrChildren() const { return children.size(); }

    /// Return the child at index i.
    const derived_ptr operator[](size_t i) const { return children.at(i); }

    /** The size of subtree rooted at this clique, i.e., nr of Cliques */
    size_t treeSize() const;

    /** Collect number of cliques with cached separator marginals */
    size_t numCachedSeparatorMarginals() const;

    /** return a shared_ptr to the parent clique */
    derived_ptr parent() const { return parent_.lock(); }

    /** Problem size (used for parallel traversal) */
    int problemSize() const { return problemSize_; }

    /// @}
    /// @name Advanced Interface
    /// @{

    /** return the conditional P(S|Root) on the separator given the root */
    BayesNetType shortcut(const derived_ptr& root, Eliminate function = EliminationTraitsType::DefaultEliminate) const;

    /** return the marginal P(S) on the separator */
    FactorGraphType separatorMarginal(Eliminate function = EliminationTraitsType::DefaultEliminate) const;

    /** return the marginal P(C) of the clique, using marginal caching */
    FactorGraphType marginal2(Eliminate function = EliminationTraitsType::DefaultEliminate) const;

    /**
     * This deletes the cached shortcuts of all cliques (subtree) below this clique.
     * This is performed when the bayes tree is modified.
     */
    void deleteCachedShortcuts();

    const std::optional<FactorGraphType>& cachedSeparatorMarginal() const {
      std::lock_guard<std::mutex> marginalLock(cachedSeparatorMarginalMutex_);
      return cachedSeparatorMarginal_; 
    }

    friend class BayesTree<DerivedType>;

  protected:

    /// Calculate set \f$ S \setminus B \f$ for shortcut calculations
    KeyVector separator_setminus_B(const derived_ptr& B) const;

    /** Determine variable indices to keep in recursive separator shortcut calculation The factor
     *  graph p_Cp_B has keys from the parent clique Cp and from B. But we only keep the variables
     *  not in S union B. */
    KeyVector shortcut_indices(const derived_ptr& B, const FactorGraphType& p_Cp_B) const;

    /** Non-recursive delete cached shortcuts and marginals - internal only. */
    void deleteCachedShortcutsNonRecursive() { 
      std::lock_guard<std::mutex> marginalLock(cachedSeparatorMarginalMutex_);
      cachedSeparatorMarginal_ = {}; 
    }

  private:

#ifdef GTSAM_ENABLE_BOOST_SERIALIZATION
    /** Serialization function */
    friend class boost::serialization::access;
    template<class ARCHIVE>
    void serialize(ARCHIVE & ar, const unsigned int /*version*/) {
      if(!parent_.lock()) {
        is_root = true;
      }
      ar & BOOST_SERIALIZATION_NVP(is_root);
      ar & BOOST_SERIALIZATION_NVP(conditional_);
      if (!is_root) { // TODO(fan): Workaround for boost/serialization #119
        ar & BOOST_SERIALIZATION_NVP(parent_);
      }
      ar & BOOST_SERIALIZATION_NVP(children);
    }
#endif

    /// @}

  };

}
