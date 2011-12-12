/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    BayesTreeCliqueBase
 * @brief   Base class for cliques of a BayesTree
 * @author  Richard Roberts and Frank Dellaert
 */

#pragma once

#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>

#include <gtsam/base/types.h>
#include <gtsam/inference/FactorGraph.h>
#include <gtsam/inference/BayesNet.h>

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
   * @tparam The derived clique type.
   */
  template<class DERIVED>
  struct BayesTreeCliqueBase {

  protected:
    void assertInvariants() const;

    /** Default constructor */
    BayesTreeCliqueBase() {}

    /** Construct from a conditional, leaving parent and child pointers uninitialized */
    BayesTreeCliqueBase(const sharedConditional& conditional);

  public:
    typedef BayesTreeClique<DERIVED> This;
    typedef DERIVED DerivedType;
    typedef typename DERIVED::ConditionalType ConditionalType;
    typedef boost::shared_ptr<ConditionalType> sharedConditional;
    typedef typename boost::shared_ptr<This> shared_ptr;
    typedef typename boost::weak_ptr<This> weak_ptr;
    typedef typename boost::shared_ptr<DerivedType> derived_ptr;
    typedef typename boost::weak_ptr<DerivedType> derived_weak_ptr;
    typedef typename ConditionalType::FactorType FactorType;
    typedef typename FactorGraph<FactorType>::Eliminate Eliminate;

    sharedConditional conditional_;
    derived_weak_ptr parent_;
    std::list<derived_ptr> children_;

    /** Construct shared_ptr from a conditional, leaving parent and child pointers uninitialized */
    static derived_ptr Create(const sharedConditional& conditional) { return boost::make_shared<DerivedType>(conditional); }

    /** Construct shared_ptr from a FactorGraph<FACTOR>::EliminationResult.  In this class
     * the conditional part is kept and the factor part is ignored, but in derived clique
     * types, such as ISAM2Clique, the factor part is kept as a cached factor.
     * @param An elimination result, which is a pair<CONDITIONAL,FACTOR>
     */
    static derived_ptr Create(const std::pair<sharedConditional, boost::shared_ptr<typename ConditionalType::FactorType> >& result) { return boost::make_shared<DerivedType>(result); }

    void cloneToBayesTree(BayesTree& newTree, shared_ptr parent_clique = shared_ptr()) const {
      sharedConditional newConditional = sharedConditional(new CONDITIONAL(*conditional_));
      sharedClique newClique = newTree.addClique(newConditional, parent_clique);
      if (cachedFactor_)
        newClique->cachedFactor_ = cachedFactor_->clone();
      else newClique->cachedFactor_ = typename FactorType::shared_ptr();
      if (!parent_clique) {
        newTree.root_ = newClique;
      }
      BOOST_FOREACH(const shared_ptr& childClique, children_) {
        childClique->cloneToBayesTree(newTree, newClique);
      }
    }

    /** print this node */
    void print(const std::string& s = "") const;

    /** The arrow operator accesses the conditional */
    const ConditionalType* operator->() const { return conditional_.get(); }

    /** The arrow operator accesses the conditional */
    ConditionalType* operator->() { return conditional_.get(); }

    /** Access the conditional */
    const sharedConditional& conditional() const { return conditional_; }

    /** is this the root of a Bayes tree ? */
    inline bool isRoot() const { return parent_.expired(); }

    /** return the const reference of children */
    std::list<derived_ptr>& children() { return children_; }
    const std::list<derived_ptr>& children() const { return children_; }

    /** The size of subtree rooted at this clique, i.e., nr of Cliques */
    size_t treeSize() const;

    /** print this node and entire subtree below it */
    void printTree(const std::string& indent="") const;

    /** Permute the variables in the whole subtree rooted at this clique */
    void permuteWithInverse(const Permutation& inversePermutation);

    /** Permute variables when they only appear in the separators.  In this
     * case the running intersection property will be used to prevent always
     * traversing the whole tree.  Returns whether any separator variables in
     * this subtree were reordered.
     */
    bool permuteSeparatorWithInverse(const Permutation& inversePermutation);

    /** return the conditional P(S|Root) on the separator given the root */
    // TODO: create a cached version
    BayesNet<ConditionalType> shortcut(shared_ptr root, Eliminate function);

    /** return the marginal P(C) of the clique */
    FactorGraph<FactorType> marginal(shared_ptr root, Eliminate function);

    /** return the joint P(C1,C2), where C1==this. TODO: not a method? */
    FactorGraph<FactorType> joint(shared_ptr C2, shared_ptr root, Eliminate function);

    bool equals(const This& other, double tol=1e-9) const {
      return (!conditional_ && !other.conditional()) ||
          conditional_->equals(*(other.conditional()), tol);
    }

  private:
    /** Serialization function */
    friend class boost::serialization::access;
    template<class ARCHIVE>
    void serialize(ARCHIVE & ar, const unsigned int version) {
      ar & BOOST_SERIALIZATION_NVP(conditional_);
      ar & BOOST_SERIALIZATION_NVP(parent_);
      ar & BOOST_SERIALIZATION_NVP(children_);
    }

  }; // \struct Clique

  template<class DERIVED>
  typename BayesTreeCliqueBase<DERIVED>::derived_ptr asDerived(const BayesTreeCliqueBase<DERIVED>& base) {
#ifndef NDEBUG
    return boost::dynamic_pointer_cast<DERIVED>(base);
#else
    return boost::static_pointer_cast<DERIVED>(base);
#endif
  }

#include <gtsam/inference/BayesTreeCliqueBase-inl.h>

}
