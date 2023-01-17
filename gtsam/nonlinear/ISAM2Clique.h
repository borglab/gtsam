/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    ISAM2Clique.h
 * @brief   Specialized iSAM2 Clique
 * @author  Michael Kaess, Richard Roberts
 */

// \callgraph

#pragma once

#include <gtsam/inference/BayesTreeCliqueBase.h>
#include <gtsam/inference/Key.h>
#include <gtsam/linear/GaussianBayesNet.h>
#include <gtsam/linear/GaussianConditional.h>
#include <gtsam/linear/GaussianFactorGraph.h>
#include <string>

namespace gtsam {

/**
 * Specialized Clique structure for ISAM2, incorporating caching and gradient
 * contribution
 * TODO: more documentation
 */
class GTSAM_EXPORT ISAM2Clique
    : public BayesTreeCliqueBase<ISAM2Clique, GaussianFactorGraph> {
 public:
  typedef ISAM2Clique This;
  typedef BayesTreeCliqueBase<This, GaussianFactorGraph> Base;
  typedef std::shared_ptr<This> shared_ptr;
  typedef std::weak_ptr<This> weak_ptr;
  typedef GaussianConditional ConditionalType;
  typedef ConditionalType::shared_ptr sharedConditional;

  Base::FactorType::shared_ptr cachedFactor_;
  Vector gradientContribution_;
#ifdef USE_BROKEN_FAST_BACKSUBSTITUTE
  mutable FastMap<Key, VectorValues::iterator> solnPointers_;
#endif

  /// Default constructor
  ISAM2Clique() : Base() {}
  virtual ~ISAM2Clique() = default;

  /// Copy constructor, does *not* copy solution pointers as these are invalid
  /// in different trees.
  ISAM2Clique(const ISAM2Clique& other)
      : Base(other),
        cachedFactor_(other.cachedFactor_),
        gradientContribution_(other.gradientContribution_) {}

  /// Assignment operator, does *not* copy solution pointers as these are
  /// invalid in different trees.
  ISAM2Clique& operator=(const ISAM2Clique& other) {
    Base::operator=(other);
    cachedFactor_ = other.cachedFactor_;
    gradientContribution_ = other.gradientContribution_;
    return *this;
  }

  /// Overridden to also store the remaining factor and gradient contribution
  void setEliminationResult(
      const FactorGraphType::EliminationResult& eliminationResult);

  /** Access the cached factor */
  Base::FactorType::shared_ptr& cachedFactor() { return cachedFactor_; }

  /// Access the gradient contribution
  const Vector& gradientContribution() const { return gradientContribution_; }

  /// Recursively add gradient at zero to g
  void addGradientAtZero(VectorValues* g) const;

  bool equals(const This& other, double tol = 1e-9) const;

  /** print this node */
  void print(const std::string& s = "",
             const KeyFormatter& formatter = DefaultKeyFormatter) const override;

  void optimizeWildfire(const KeySet& replaced, double threshold,
                        KeySet* changed, VectorValues* delta,
                        size_t* count) const;

  bool optimizeWildfireNode(const KeySet& replaced, double threshold,
                            KeySet* changed, VectorValues* delta,
                            size_t* count) const;

  /**
   * Starting from the root, add up entries of frontal and conditional matrices
   * of each conditional
   */
  void nnz_internal(size_t* result) const;
  size_t calculate_nnz() const;

  /**
   * Recursively search this clique and its children for marked keys appearing
   * in the separator, and add the *frontal* keys of any cliques whose
   * separator contains any marked keys to the set \c keys.  The purpose of
   * this is to discover the cliques that need to be redone due to information
   * propagating to them from cliques that directly contain factors being
   * relinearized.
   *
   * The original comment says this finds all variables directly connected to
   * the marked ones by measurements.  Is this true, because it seems like this
   * would also pull in variables indirectly connected through other frontal or
   * separator variables?
   *
   * Alternatively could we trace up towards the root for each variable here?
   */
  void findAll(const KeySet& markedMask, KeySet* keys) const;

 private:
  /**
   * Check if clique was replaced, or if any parents were changed above the
   * threshold or themselves replaced.
   */
  bool isDirty(const KeySet& replaced, const KeySet& changed) const;

  /**
   * Back-substitute - special version stores solution pointers in cliques for
   * fast access.
   */
  void fastBackSubstitute(VectorValues* delta) const;

  /*
   * Check whether the values changed above a threshold, or always true if the
   * clique was replaced.
   */
  bool valuesChanged(const KeySet& replaced, const Vector& originalValues,
                     const VectorValues& delta, double threshold) const;

  /// Set changed flag for each frontal variable
  void markFrontalsAsChanged(KeySet* changed) const;

  /// Restore delta to original values, guided by frontal keys.
  void restoreFromOriginals(const Vector& originalValues,
                            VectorValues* delta) const;

  /** Serialization function */
  friend class boost::serialization::access;
  template <class ARCHIVE>
  void serialize(ARCHIVE& ar, const unsigned int /*version*/) {
    ar& BOOST_SERIALIZATION_BASE_OBJECT_NVP(Base);
    ar& BOOST_SERIALIZATION_NVP(cachedFactor_);
    ar& BOOST_SERIALIZATION_NVP(gradientContribution_);
  }
};  // \struct ISAM2Clique

/**
 * Optimize the BayesTree, starting from the root.
 * @param threshold The maximum change against the PREVIOUS delta for
 * non-replaced variables that can be ignored, ie. the old delta entry is kept
 * and recursive backsubstitution might eventually stop if none of the changed
 * variables are contained in the subtree.
 * @param replaced Needs to contain all variables that are contained in the top
 * of the Bayes tree that has been redone.
 * @return The number of variables that were solved for.
 * @param delta The current solution, an offset from the linearization point.
 */
size_t optimizeWildfire(const ISAM2Clique::shared_ptr& root, double threshold,
                        const KeySet& replaced, VectorValues* delta);

size_t optimizeWildfireNonRecursive(const ISAM2Clique::shared_ptr& root,
                                    double threshold, const KeySet& replaced,
                                    VectorValues* delta);

}  // namespace gtsam
