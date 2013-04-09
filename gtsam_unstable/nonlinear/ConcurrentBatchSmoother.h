/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    ConcurrentBatchSmoother.h
 * @brief   A Levenberg-Marquardt Batch Smoother that implements the
 *          Concurrent Filtering and Smoothing interface.
 * @author  Stephen Williams
 */

// \callgraph
#pragma once

#include <gtsam_unstable/nonlinear/ConcurrentFilteringAndSmoothing.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <queue>

namespace gtsam {

/**
 * A Levenberg-Marquardt Batch Smoother that implements the Concurrent Filtering and Smoother interface.
 */
class GTSAM_UNSTABLE_EXPORT ConcurrentBatchSmoother : public ConcurrentSmoother {

public:

  typedef ConcurrentSmoother Base; ///< typedef for base class

  /**
   * Meta information returned about the update
   */
  // TODO: Think of some more things to put here
  struct Result {
    size_t iterations; ///< The number of optimizer iterations performed
    size_t nonlinearVariables; ///< The number of variables that can be relinearized
    size_t linearVariables; ///< The number of variables that must keep a constant linearization point
    double error; ///< The final factor graph error
    Result() : iterations(0), nonlinearVariables(0), linearVariables(0), error(0) {};
  };

  /** Default constructor */
  ConcurrentBatchSmoother(const LevenbergMarquardtParams& parameters) : parameters_(parameters) {};

  /** Default destructor */
  virtual ~ConcurrentBatchSmoother() {};

  // Implement a GTSAM standard 'print' function
  void print(const std::string& s = "Concurrent Batch Smoother:\n", const KeyFormatter& keyFormatter = DefaultKeyFormatter) const;

  /** Compute the current best estimate of all variables and return a full Values structure.
   * If only a single variable is needed, it may be faster to call calculateEstimate(const KEY&).
   */
  Values calculateEstimate() const {
    return theta_;
  }

  /** Compute the current best estimate of a single variable. This is generally faster than
   * calling the no-argument version of calculateEstimate if only specific variables are needed.
   * @param key
   * @return
   */
  template<class VALUE>
  VALUE calculateEstimate(const Key key) const {
    return theta_.at<VALUE>(key);
  }

  /**
   * Add new factors and variables to the smoother.
   *
   * Add new measurements, and optionally new variables, to the smoother.
   * This runs a full step of the ISAM2 algorithm, relinearizing and updating
   * the solution as needed, according to the wildfire and relinearize
   * thresholds.
   *
   * @param newFactors The new factors to be added to the smoother
   * @param newTheta Initialization points for new variables to be added to the smoother
   * You must include here all new variables occuring in newFactors (which were not already
   * in the smoother).  There must not be any variables here that do not occur in newFactors,
   * and additionally, variables that were already in the system must not be included here.
   */
  Result update(const NonlinearFactorGraph& newFactors = NonlinearFactorGraph(), const Values& newTheta = Values());


protected:

  /** A typedef defining an Key-Factor mapping **/
  typedef std::map<Key, std::set<Index> > FactorIndex;

  LevenbergMarquardtParams parameters_;  ///< LM parameters
  NonlinearFactorGraph graph_;  ///< The graph of all the smoother factors
  Values theta_;  ///< Current linearization point
  Ordering ordering_; ///< The current ordering used to generate the deltas
  VectorValues delta_;  ///< Current set of offsets from the linearization point
  Values separatorValues_;  ///< The set of keys to be kept in the root and their linearization points
  std::queue<size_t> availableSlots_; ///< The set of available factor graph slots caused by deleting factors
  FactorIndex factorIndex_; ///< A cross-reference structure to allow efficient factor lookups by key
  std::vector<size_t> filterSummarizationSlots_;  ///< The slots in graph for the last set of filter summarized factors
  NonlinearFactorGraph smootherSummarization_; ///< A temporary holding place for calculated smoother summarization

  /**
   * Perform any required operations before the synchronization process starts.
   * Called by 'synchronize'
   */
  virtual void presync();

  /**
   * Populate the provided containers with factors that constitute the smoother branch summarization
   * needed by the filter.
   *
   * @param summarizedFactors The summarized factors for the filter branch
   */
  virtual void getSummarizedFactors(NonlinearFactorGraph& summarizedFactors);

  /**
   * Apply the new smoother factors sent by the filter, and the updated version of the filter
   * branch summarized factors.
   *
   * @param smootherFactors A set of new factors added to the smoother from the filter
   * @param smootherValues Linearization points for any new variables
   * @param summarizedFactors An updated version of the filter branch summarized factors
   * @param rootValues The linearization point of the root variables
   */
  virtual void synchronize(const NonlinearFactorGraph& smootherFactors, const Values& smootherValues,
      const NonlinearFactorGraph& summarizedFactors, const Values& rootValues);

  /**
   * Perform any required operations after the synchronization process finishes.
   * Called by 'synchronize'
   */
  virtual void postsync();

  /** Augment the graph with a new factor
   *
   * @param factors The factor to add to the graph
   * @return The slot in the graph it was inserted into
   */
  size_t insertFactor(const NonlinearFactor::shared_ptr& factor);

  /** Remove a factor from the graph by slot index */
  void removeFactor(size_t slot);

  /** Optimize the graph using a modified version of L-M */
  void optimize();

  /** Find all of the nonlinear factors that contain any of the provided keys */
  std::set<size_t> findFactorsWithAny(const std::set<Key>& keys) const;

  /** Find all of the nonlinear factors that contain only the provided keys */
  std::set<size_t> findFactorsWithOnly(const std::set<Key>& keys) const;

  /** Create a linearized factor from the information remaining after marginalizing out the requested keys */
  NonlinearFactor::shared_ptr marginalizeKeysFromFactor(const NonlinearFactor::shared_ptr& factor, const std::set<Key>& keysToKeep, const Values& theta) const;

private:
  /** Some printing functions for debugging */

  static void PrintNonlinearFactor(const gtsam::NonlinearFactor::shared_ptr& factor,
      const std::string& indent = "", const gtsam::KeyFormatter& keyFormatter = gtsam::DefaultKeyFormatter);

  static void PrintLinearFactor(const gtsam::GaussianFactor::shared_ptr& factor, const gtsam::Ordering& ordering,
      const std::string& indent = "", const gtsam::KeyFormatter& keyFormatter = gtsam::DefaultKeyFormatter);

//  static void PrintSingleClique(const gtsam::ISAM2Clique::shared_ptr& clique, const gtsam::Ordering& ordering,
//      const std::string& indent = "", const gtsam::KeyFormatter& keyFormatter = gtsam::DefaultKeyFormatter);
//
//  static void PrintRecursiveClique(const gtsam::ISAM2Clique::shared_ptr& clique, const gtsam::Ordering& ordering,
//      const std::string& indent = "", const gtsam::KeyFormatter& keyFormatter = gtsam::DefaultKeyFormatter);
//
//  static void PrintBayesTree(const gtsam::ISAM2& bayesTree, const gtsam::Ordering& ordering,
//      const std::string& indent = "", const gtsam::KeyFormatter& keyFormatter = gtsam::DefaultKeyFormatter);

//  typedef BayesTree<GaussianConditional,ISAM2Clique>::sharedClique Clique;
//  static void SymbolicPrintTree(const Clique& clique, const Ordering& ordering, const std::string indent = "");

}; // ConcurrentBatchSmoother

}/// namespace gtsam
