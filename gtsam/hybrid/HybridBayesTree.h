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

  virtual ~HybridBayesTreeClique() {}
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
