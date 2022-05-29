/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file HybridISAM.h
 * @date March 31, 2022
 * @author Fan Jiang
 * @author Frank Dellaert
 * @author Richard Roberts
 */

#pragma once

#include <gtsam/base/Testable.h>
#include <gtsam/hybrid/GaussianHybridFactorGraph.h>
#include <gtsam/hybrid/HybridBayesTree.h>
#include <gtsam/inference/ISAM.h>

namespace gtsam {

class GTSAM_EXPORT HybridISAM : public ISAM<HybridBayesTree> {
 public:
  typedef ISAM<HybridBayesTree> Base;
  typedef HybridISAM This;
  typedef boost::shared_ptr<This> shared_ptr;

  /// @name Standard Constructors
  /// @{

  /** Create an empty Bayes Tree */
  HybridISAM();

  /** Copy constructor */
  HybridISAM(const HybridBayesTree& bayesTree);

  /// @}

 private:
  /// Internal method that performs the ISAM update.
  void updateInternal(
      const GaussianHybridFactorGraph& newFactors,
      HybridBayesTree::Cliques* orphans,
      const HybridBayesTree::Eliminate& function =
          HybridBayesTree::EliminationTraitsType::DefaultEliminate);

 public:
  /**
   * @brief Perform update step with new factors.
   *
   * @param newFactors Factor graph of new factors to add and eliminate.
   * @param function Elimination function.
   */
  void update(const GaussianHybridFactorGraph& newFactors,
              const HybridBayesTree::Eliminate& function =
                  HybridBayesTree::EliminationTraitsType::DefaultEliminate);
};

/// traits
template <>
struct traits<HybridISAM> : public Testable<HybridISAM> {};

}  // namespace gtsam