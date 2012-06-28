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

#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>
#include <boost/weak_ptr.hpp>

#include <gtsam/base/types.h>
#include <gtsam/inference/FactorGraph.h>
#include <gtsam/inference/BayesNet.h>

namespace gtsam { template<class CONDITIONAL, class CLIQUE> class BayesTree; }

namespace gtsam {

  /**
   * This is the base class for BayesTree cliques.  The default and standard
   * derived type is BayesTreeClique, but some algorithms, like iSAM2, use a
   * different clique type in order to store extra data along with the clique.
   *
   * This class is templated on the derived class (i.e. the curiously recursive
   * template pattern).  The advantage of this over using virtual classes is
   * that it avoids the need for casting to get the derived type.  This is
   * possible because all cliques in a BayesTree are the same type - if they
   * were not then we'd need a virtual class.
   *
   * @tparam DERIVED The derived clique type.
   * @tparam CONDITIONAL The conditional type.
   * \nosubgrouping
   */
  template<class DERIVED, class CONDITIONAL>
  struct BayesTreeCliqueBase {

  public:
    typedef BayesTreeCliqueBase<DERIVED,CONDITIONAL> This;
    typedef DERIVED DerivedType;
    typedef CONDITIONAL ConditionalType;
    typedef boost::shared_ptr<ConditionalType> sharedConditional;
    typedef boost::shared_ptr<This> shared_ptr;
    typedef boost::weak_ptr<This> weak_ptr;
    typedef boost::shared_ptr<DerivedType> derived_ptr;
    typedef boost::weak_ptr<DerivedType> derived_weak_ptr;
    typedef typename ConditionalType::FactorType FactorType;
    typedef typename FactorGraph<FactorType>::Eliminate Eliminate;

  protected:

  	/// @name Standard Constructors
  	/// @{

    /** Default constructor */
    BayesTreeCliqueBase() {}

    /** Construct from a conditional, leaving parent and child pointers uninitialized */
    BayesTreeCliqueBase(const sharedConditional& conditional);

    /** Construct from an elimination result, which is a pair<CONDITIONAL,FACTOR> */
    BayesTreeCliqueBase(const std::pair<sharedConditional, boost::shared_ptr<typename ConditionalType::FactorType> >& result);

   	/// @}

    /// This stores the Cached Shortcut value
    mutable boost::optional<BayesNet<ConditionalType> > cachedShortcut_;

  public:
    sharedConditional conditional_;
    derived_weak_ptr parent_;
    std::list<derived_ptr> children_;

  	/// @name Testable
  	/// @{

    /** check equality */
    bool equals(const This& other, double tol=1e-9) const {
      return (!conditional_ && !other.conditional()) ||
          conditional_->equals(*other.conditional(), tol);
    }

    /** print this node */
    void print(const std::string& s = "", const IndexFormatter& indexFormatter = DefaultIndexFormatter ) const;

    /** print this node and entire subtree below it */
    void printTree(const std::string& indent="", const IndexFormatter& indexFormatter = DefaultIndexFormatter ) const;

  	/// @}
  	/// @name Standard Interface
  	/// @{

    /** Access the conditional */
    const sharedConditional& conditional() const { return conditional_; }

    /** is this the root of a Bayes tree ? */
    inline bool isRoot() const { return parent_.expired(); }

    /** The size of subtree rooted at this clique, i.e., nr of Cliques */
    size_t treeSize() const;

    /** The arrow operator accesses the conditional */
    const ConditionalType* operator->() const { return conditional_.get(); }

    /** return the const reference of children */
    const std::list<derived_ptr>& children() const { return children_; }

    /** return a shared_ptr to the parent clique */
    derived_ptr parent() const { return parent_.lock(); }

  	/// @}
  	/// @name Advanced Interface
  	/// @{

    /** The arrow operator accesses the conditional */
    ConditionalType* operator->() { return conditional_.get(); }

    /** return the reference of children non-const version*/
    std::list<derived_ptr>& children() { return children_; }

    /** Construct shared_ptr from a conditional, leaving parent and child pointers uninitialized */
    static derived_ptr Create(const sharedConditional& conditional) { return boost::make_shared<DerivedType>(conditional); }

    /** Construct shared_ptr from a FactorGraph<FACTOR>::EliminationResult.  In this class
     * the conditional part is kept and the factor part is ignored, but in derived clique
     * types, such as ISAM2Clique, the factor part is kept as a cached factor.
     * @param result An elimination result, which is a pair<CONDITIONAL,FACTOR>
     */
     static derived_ptr Create(const std::pair<sharedConditional, boost::shared_ptr<typename ConditionalType::FactorType> >& result) { return boost::make_shared<DerivedType>(result); }

     /** Returns a new clique containing a copy of the conditional but without
      * the parent and child clique pointers.
      */
     derived_ptr clone() const { return Create(sharedConditional(new ConditionalType(*conditional_))); }

    /** Permute the variables in the whole subtree rooted at this clique */
    void permuteWithInverse(const Permutation& inversePermutation);

    /** Permute variables when they only appear in the separators.  In this
     * case the running intersection property will be used to prevent always
     * traversing the whole tree.  Returns whether any separator variables in
     * this subtree were reordered.
     */
    bool permuteSeparatorWithInverse(const Permutation& inversePermutation);

    /** return the conditional P(S|Root) on the separator given the root */
    BayesNet<ConditionalType> shortcut(derived_ptr root, Eliminate function) const;

    /** return the marginal P(C) of the clique */
    FactorGraph<FactorType> marginal(derived_ptr root, Eliminate function) const;

    /** return the joint P(C1,C2), where C1==this. TODO: not a method? */
    FactorGraph<FactorType> joint(derived_ptr C2, derived_ptr root, Eliminate function) const;

    /**
     * This deletes the cached shortcuts of all cliques (subtree) below this clique.
     * This is performed when the bayes tree is modified.
     */
    void deleteCachedShorcuts();

    /** return cached shortcut of the clique */
    const boost::optional<BayesNet<ConditionalType> > cachedShortcut() const { return cachedShortcut_; }

    friend class BayesTree<ConditionalType, DerivedType>;

  protected:

    ///TODO: comment
    void assertInvariants() const;

    /// Reset the computed shortcut of this clique. Used by friend BayesTree
    void resetCachedShortcut() { cachedShortcut_ = boost::none; }

  private:

    /** Cliques cannot be copied except by the clone() method, which does not
     * copy the parent and child pointers.
     */
    BayesTreeCliqueBase(const This& other) { assert(false); }

    /** Cliques cannot be copied except by the clone() method, which does not
     * copy the parent and child pointers.
     */
    This& operator=(const This& other) { assert(false); return *this; }

    /** Serialization function */
    friend class boost::serialization::access;
    template<class ARCHIVE>
    void serialize(ARCHIVE & ar, const unsigned int version) {
      ar & BOOST_SERIALIZATION_NVP(conditional_);
      ar & BOOST_SERIALIZATION_NVP(parent_);
      ar & BOOST_SERIALIZATION_NVP(children_);
    }

  	/// @}

  }; // \struct Clique

  template<class DERIVED, class CONDITIONAL>
  const DERIVED* asDerived(const BayesTreeCliqueBase<DERIVED,CONDITIONAL>* base) {
    return static_cast<const DERIVED*>(base);
  }

  template<class DERIVED, class CONDITIONAL>
  DERIVED* asDerived(BayesTreeCliqueBase<DERIVED,CONDITIONAL>* base) {
    return static_cast<DERIVED*>(base);
  }

}
