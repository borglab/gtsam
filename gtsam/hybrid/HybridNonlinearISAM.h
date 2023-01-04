/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file HybridNonlinearISAM.h
 * @date Sep 12, 2022
 * @author Varun Agrawal
 */

#pragma once

#include <gtsam/hybrid/HybridGaussianISAM.h>
#include <gtsam/hybrid/HybridNonlinearFactorGraph.h>

namespace gtsam {
/**
 * Wrapper class to manage ISAM in a nonlinear context
 */
class GTSAM_EXPORT HybridNonlinearISAM {
 protected:
  /** The internal iSAM object */
  gtsam::HybridGaussianISAM isam_;

  /** The current linearization point */
  Values linPoint_;

  /// The discrete assignment
  DiscreteValues assignment_;

  /** The original factors, used when relinearizing */
  HybridNonlinearFactorGraph factors_;

  /** The reordering interval and counter */
  int reorderInterval_;
  int reorderCounter_;

  /** The elimination function */
  HybridGaussianFactorGraph::Eliminate eliminationFunction_;

 public:
  /// @name Standard Constructors
  /// @{

  /**
   * Periodically reorder and relinearize
   * @param reorderInterval is the number of updates between reorderings,
   *   0 never reorders (and is dangerous for memory consumption)
   *  1 (default) reorders every time, in worse case is batch every update
   *  typical values are 50 or 100
   */
  HybridNonlinearISAM(
      int reorderInterval = 1,
      const HybridGaussianFactorGraph::Eliminate& eliminationFunction =
          HybridGaussianFactorGraph::EliminationTraitsType::DefaultEliminate)
      : reorderInterval_(reorderInterval),
        reorderCounter_(0),
        eliminationFunction_(eliminationFunction) {}

  /// @}
  /// @name Standard Interface
  /// @{

  /** Return the current solution estimate */
  Values estimate();

  // /** find the marginal covariance for a single variable */
  // Matrix marginalCovariance(Key key) const;

  // access

  /** access the underlying bayes tree */
  const HybridGaussianISAM& bayesTree() const { return isam_; }

  /**
   * @brief Prune the underlying Bayes tree.
   *
   * @param maxNumberLeaves The max number of leaf nodes to keep.
   */
  void prune(const size_t maxNumberLeaves) { isam_.prune(maxNumberLeaves); }

  /** Return the current linearization point */
  const Values& getLinearizationPoint() const { return linPoint_; }

  /** Return the current discrete assignment */
  const DiscreteValues& assignment() const { return assignment_; }

  /** get underlying nonlinear graph */
  const HybridNonlinearFactorGraph& getFactorsUnsafe() const {
    return factors_;
  }

  /** get counters */
  int reorderInterval() const { return reorderInterval_; }  ///< TODO: comment
  int reorderCounter() const { return reorderCounter_; }    ///< TODO: comment

  /** prints out all contents of the system */
  void print(const std::string& s = "",
             const KeyFormatter& keyFormatter = DefaultKeyFormatter) const;

  /** prints out clique statistics */
  void printStats() const;

  /** saves the Tree to a text file in GraphViz format */
  void saveGraph(const std::string& s,
                 const KeyFormatter& keyFormatter = DefaultKeyFormatter) const;

  /// @}
  /// @name Advanced Interface
  /// @{

  /** Add new factors along with their initial linearization points */
  void update(const HybridNonlinearFactorGraph& newFactors,
              const Values& initialValues,
              const boost::optional<size_t>& maxNrLeaves = boost::none,
              const boost::optional<Ordering>& ordering = boost::none);

  /** Relinearization and reordering of variables */
  void reorder_relinearize();

  /// @}
};

}  // namespace gtsam
