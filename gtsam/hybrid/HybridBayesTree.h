/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    HybridBayesTree.h
 * @brief   Hybrid Bayes Tree, the result of eliminating a
 * HybridJunctionTree
 * @date Mar 11, 2022
 * @author  Fan Jiang
 */

#pragma once

#include <gtsam/hybrid/HybridBayesNet.h>
#include <gtsam/hybrid/HybridGaussianFactorGraph.h>
#include <gtsam/inference/BayesTree.h>
#include <gtsam/inference/BayesTreeCliqueBase.h>
#include <gtsam/inference/Conditional.h>
#include <gtsam/linear/GaussianBayesTree.h>

#include <string>

namespace gtsam {

// Forward declarations
class HybridConditional;
class VectorValues;

/* ************************************************************************* */
/**
 * @brief A clique in a HybridBayesTree
 * which is a HybridConditional internally.
 *
 * @ingroup hybrid
 */
class GTSAM_EXPORT HybridBayesTreeClique
    : public BayesTreeCliqueBase<HybridBayesTreeClique,
                                 HybridGaussianFactorGraph> {
 public:
  typedef HybridBayesTreeClique This;
  typedef BayesTreeCliqueBase<HybridBayesTreeClique, HybridGaussianFactorGraph>
      Base;
  typedef std::shared_ptr<This> shared_ptr;
  typedef std::weak_ptr<This> weak_ptr;
  HybridBayesTreeClique() {}
  HybridBayesTreeClique(const std::shared_ptr<HybridConditional>& conditional)
      : Base(conditional) {}
  ///< Copy constructor
  HybridBayesTreeClique(const HybridBayesTreeClique& clique) : Base(clique) {}
};

/* ************************************************************************* */
/** A Bayes tree representing a Hybrid density */
class GTSAM_EXPORT HybridBayesTree : public BayesTree<HybridBayesTreeClique> {
 private:
  typedef BayesTree<HybridBayesTreeClique> Base;

 public:
  typedef HybridBayesTree This;
  typedef std::shared_ptr<This> shared_ptr;

  /// @name Standard interface
  /// @{
  /** Default constructor, creates an empty Bayes tree */
  HybridBayesTree() = default;

  /** Check equality */
  bool equals(const This& other, double tol = 1e-9) const;

  /**
   * @brief Get the Gaussian Bayes Tree which corresponds to a specific discrete
   * value assignment.
   *
   * @param assignment The discrete value assignment for the discrete keys.
   * @return GaussianBayesTree
   */
  GaussianBayesTree choose(const DiscreteValues& assignment) const;

  /** Error for all conditionals. */
  double error(const HybridValues& values) const {
    return HybridGaussianFactorGraph(*this).error(values);
  }

  /**
   * @brief Helper function to add a clique of hybrid conditionals to the passed
   * in GaussianBayesNetTree. Operates recursively on the clique in a bottom-up
   * fashion, adding the children first.
   *
   * @param clique The
   * @param result
   * @return GaussianBayesNetTree&
   */
  GaussianBayesNetTree& addCliqueToTree(const sharedClique& clique,
                                        GaussianBayesNetTree& result) const;

  /**
   * @brief Assemble a DecisionTree of (GaussianBayesTree, double) leaves for
   * each discrete assignment.
   * The included double value is used to make
   * constructing the model selection term cleaner and more efficient.
   *
   * @return GaussianBayesNetValTree
   */
  GaussianBayesNetValTree assembleTree() const;

  /*
    Compute L(M;Z), the likelihood of the discrete model M
    given the measurements Z.
    This is called the model selection term.

    To do so, we perform the integration of L(M;Z) ∝ L(X;M,Z)P(X|M).

    By Bayes' rule, P(X|M,Z) ∝ L(X;M,Z)P(X|M),
    hence L(X;M,Z)P(X|M) is the unnormalized probabilty of
    the joint Gaussian distribution.

    This can be computed by multiplying all the exponentiated errors
    of each of the conditionals.

    Return a tree where each leaf value is L(M_i;Z).
  */
  AlgebraicDecisionTree<Key> modelSelection() const;

  /**
   * @brief Optimize the hybrid Bayes tree by computing the MPE for the current
   * set of discrete variables and using it to compute the best continuous
   * update delta.
   *
   * @return HybridValues
   */
  HybridValues optimize() const;

  /**
   * @brief Recursively optimize the BayesTree to produce a vector solution.
   *
   * @param assignment The discrete values assignment to select the Gaussian
   * mixtures.
   * @return VectorValues
   */
  VectorValues optimize(const DiscreteValues& assignment) const;

  /**
   * @brief Prune the underlying Bayes tree.
   *
   * @param maxNumberLeaves The max number of leaf nodes to keep.
   */
  void prune(const size_t maxNumberLeaves);

  /// @}

 private:
#ifdef GTSAM_ENABLE_BOOST_SERIALIZATION
  /** Serialization function */
  friend class boost::serialization::access;
  template <class ARCHIVE>
  void serialize(ARCHIVE& ar, const unsigned int /*version*/) {
    ar& BOOST_SERIALIZATION_BASE_OBJECT_NVP(Base);
  }
#endif
};

/// traits
template <>
struct traits<HybridBayesTreeClique> : public Testable<HybridBayesTreeClique> {
};

template <>
struct traits<HybridBayesTree> : public Testable<HybridBayesTree> {};

/**
 * @brief Class for Hybrid Bayes tree orphan subtrees.
 *
 * This object stores parent keys in our base type factor so that
 * eliminating those parent keys will pull this subtree into the
 * elimination.
 *
 * This is a template instantiation for hybrid Bayes tree cliques, storing both
 * the regular keys *and* discrete keys in the HybridConditional.
 */
template <>
class BayesTreeOrphanWrapper<HybridBayesTreeClique> : public HybridConditional {
 public:
  typedef HybridBayesTreeClique CliqueType;
  typedef HybridConditional Base;

  std::shared_ptr<CliqueType> clique;

  /**
   * @brief Construct a new Bayes Tree Orphan Wrapper object.
   *
   * @param clique Bayes tree clique.
   */
  BayesTreeOrphanWrapper(const std::shared_ptr<CliqueType>& clique)
      : clique(clique) {
    // Store parent keys in our base type factor so that eliminating those
    // parent keys will pull this subtree into the elimination.
    this->keys_.assign(clique->conditional()->beginParents(),
                       clique->conditional()->endParents());
    this->discreteKeys_.assign(clique->conditional()->discreteKeys().begin(),
                               clique->conditional()->discreteKeys().end());
  }

  /// print utility
  void print(
      const std::string& s = "",
      const KeyFormatter& formatter = DefaultKeyFormatter) const override {
    clique->print(s + "stored clique", formatter);
  }
};

}  // namespace gtsam
