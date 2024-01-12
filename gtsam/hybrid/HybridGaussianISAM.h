/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file HybridGaussianISAM.h
 * @date March 31, 2022
 * @author Fan Jiang
 * @author Frank Dellaert
 * @author Richard Roberts
 */

#pragma once

#include <gtsam/base/Testable.h>
#include <gtsam/hybrid/HybridBayesTree.h>
#include <gtsam/hybrid/HybridGaussianFactorGraph.h>
#include <gtsam/inference/ISAM.h>

#include <optional>

namespace gtsam {

/**
 * @brief Incremental Smoothing and Mapping (ISAM) algorithm
 * for hybrid factor graphs.
 *
 * @ingroup hybrid
 */
class GTSAM_EXPORT HybridGaussianISAM : public ISAM<HybridBayesTree> {
 public:
  typedef ISAM<HybridBayesTree> Base;
  typedef HybridGaussianISAM This;
  typedef std::shared_ptr<This> shared_ptr;

  /// @name Standard Constructors
  /// @{

  /** Create an empty Bayes Tree */
  HybridGaussianISAM();

  /** Copy constructor */
  HybridGaussianISAM(const HybridBayesTree& bayesTree);

  /// @}

 private:
  /// Internal method that performs the ISAM update.
  void updateInternal(
      const HybridGaussianFactorGraph& newFactors,
      HybridBayesTree::Cliques* orphans,
      const std::optional<size_t>& maxNrLeaves = {},
      const std::optional<Ordering>& ordering = {},
      const HybridBayesTree::Eliminate& function =
          HybridBayesTree::EliminationTraitsType::DefaultEliminate);

 public:
  /**
   * @brief Perform update step with new factors.
   *
   * @param newFactors Factor graph of new factors to add and eliminate.
   * @param maxNrLeaves The maximum number of leaves to keep after pruning.
   * @param ordering Custom elimination ordering.
   * @param function Elimination function.
   */
  void update(const HybridGaussianFactorGraph& newFactors,
              const std::optional<size_t>& maxNrLeaves = {},
              const std::optional<Ordering>& ordering = {},
              const HybridBayesTree::Eliminate& function =
                  HybridBayesTree::EliminationTraitsType::DefaultEliminate);

  /**
   * @brief Helper method to get an ordering given the existing factors and any
   * new factors added.
   *
   * @param factors The existing factors in the BayesTree.
   * @param newFactors New factors added during the update step.
   * @return Ordering
   */
  static Ordering GetOrdering(HybridGaussianFactorGraph& factors,
                              const HybridGaussianFactorGraph& newFactors);
};

/// traits
template <>
struct traits<HybridGaussianISAM> : public Testable<HybridGaussianISAM> {};

}  // namespace gtsam
