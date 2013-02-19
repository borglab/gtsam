/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    ConcurrentBatchFilter.h
 * @brief   A Levenberg-Marquardt Batch Filter that implements the
 *          Concurrent Filtering and Smoothing interface.
 * @author  Stephen Williams
 */

// \callgraph
#pragma once

#include "ConcurrentFilteringAndSmoothing.h"
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <map>
#include <queue>

namespace gtsam {

/**
 * A Levenberg-Marquardt Batch Filter that implements the Concurrent Filtering and Smoother interface.
 */
class ConcurrentBatchFilter : public ConcurrentFilter {

public:

  typedef ConcurrentFilter Base; ///< typedef for base class
  typedef std::map<Key, double> KeyTimestampMap; ///< Typedef for a Key-Timestamp map/database
  typedef std::multimap<double, Key> TimestampKeyMap;///< Typedef for a Timestamp-Key map/database

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
  ConcurrentBatchFilter(const LevenbergMarquardtParams& parameters, double lag) : parameters_(parameters), lag_(lag) {};

  /** Default destructor */
  virtual ~ConcurrentBatchFilter() {};

  // Implement a GTSAM standard 'print' function
  void print(const std::string& s = "Concurrent Batch Filter:\n", const KeyFormatter& keyFormatter = DefaultKeyFormatter) const;

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
   * Add new factors and variables to the filter.
   *
   * Add new measurements, and optionally new variables, to the filter.
   * This runs a full update step of the derived filter algorithm
   *
   * @param newFactors The new factors to be added to the smoother
   * @param newTheta Initialization points for new variables to be added to the filter
   * You must include here all new variables occurring in newFactors that were not already
   * in the filter.
   */
  Result update(const NonlinearFactorGraph& newFactors = NonlinearFactorGraph(), const Values& newTheta = Values(),
      const KeyTimestampMap& timestamps = KeyTimestampMap());

protected:

  /** A typedef defining an Key-Factor mapping **/
  typedef std::map<Key, std::set<Index> > FactorIndex;

  LevenbergMarquardtParams parameters_;  ///< LM parameters
  double lag_; ///< Time before keys are transitioned from the filter to the smoother
  NonlinearFactorGraph graph_;  ///< The graph of all the smoother factors
  Values theta_;  ///< Current solution
  TimestampKeyMap timestampKeyMap_; ///< The set of keys associated with each timestamp
  KeyTimestampMap keyTimestampMap_; ///< The timestamp associated with each key
  std::queue<size_t> availableSlots_; ///< The set of available factor graph slots caused by deleting factors
  FactorIndex factorIndex_; ///< A cross-reference structure to allow efficient factor lookups by key

  std::vector<size_t> smootherSummarizationSlots_;  ///< The slots in graph for the last set of smoother summarized factors
  Values separatorValues_;

  // Storage for information to be sent to the smoother
  NonlinearFactorGraph filterSummarization_; ///< A temporary holding place for calculated filter summarization factors to be sent to the smoother
  Values rootValues_;  ///< The set of keys to be kept in the root and their linearization points
  NonlinearFactorGraph smootherFactors_;  ///< A temporary holding place for the set of full nonlinear factors being sent to the smoother
  Values smootherValues_; ///< A temporary holding place for the linearization points of all keys being sent to the smoother

  /**
   * Perform any required operations before the synchronization process starts.
   * Called by 'synchronize'
   */
  virtual void presync();

  /**
   * Populate the provided containers with factors that constitute the filter branch summarization
   * needed by the smoother. Also, linearization points for the new root clique must be provided.
   *
   * @param summarizedFactors The summarized factors for the filter branch
   * @param rootValues The linearization points of the root clique variables
   */
  virtual void getSummarizedFactors(NonlinearFactorGraph& summarizedFactors, Values& rootValues);

  /**
   * Populate the provided containers with factors being sent to the smoother from the filter. These
   * may be original nonlinear factors, or factors encoding a summarization of the filter information.
   * The specifics will be implementation-specific for a given filter.
   *
   * @param smootherFactors The new factors to be added to the smoother
   * @param smootherValues The linearization points of any new variables
   */
  virtual void getSmootherFactors(NonlinearFactorGraph& smootherFactors, Values& smootherValues);

  /**
   * Apply the updated version of the smoother branch summarized factors.
   *
   * @param summarizedFactors An updated version of the smoother branch summarized factors
   */
  virtual void synchronize(const NonlinearFactorGraph& summarizedFactors);

  /**
   * Perform any required operations after the synchronization process finishes.
   * Called by 'synchronize'
   */
  virtual void postsync();


  /** Augment the graph with a new factor
   *
   * @param factor The new factor to add to the graph
   * @return The slot in the graph where the factor was inserted
   */
  size_t insertFactor(const NonlinearFactor::shared_ptr& factor);

  /** Remove a factor from the graph by slot index */
  void removeFactor(size_t slot);

  /** Remove the specified key from all data structures */
  void removeKey(Key key);

  /** Update the Timestamps associated with the keys */
  void updateKeyTimestampMap(const KeyTimestampMap& newTimestamps);

  /** Find the most recent timestamp of the system */
  double getCurrentTimestamp() const;

  /** Find all of the keys associated with timestamps before the provided time */
  std::set<Key> findKeysBefore(double timestamp) const;

  /** Find all of the keys associated with timestamps after the provided time */
  std::set<Key> findKeysAfter(double timestamp) const;

  /** Find all of the nonlinear factors that contain any of the provided keys */
  std::set<size_t> findFactorsWithAny(const std::set<Key>& keys) const;

  /** Find all of the nonlinear factors that contain only the provided keys */
  std::set<size_t> findFactorsWithOnly(const std::set<Key>& keys) const;

  /** Create linearized factors from any factors remaining after marginalizing out the requested keys */
  NonlinearFactor::shared_ptr marginalizeKeysFromFactor(const NonlinearFactor::shared_ptr& factor, const std::set<Key>& remainingKeys) const;

private:
  typedef BayesTree<GaussianConditional,ISAM2Clique>::sharedClique Clique;
  static void SymbolicPrintTree(const Clique& clique, const Ordering& ordering, const std::string indent = "");

}; // ConcurrentBatchFilter

}/// namespace gtsam
