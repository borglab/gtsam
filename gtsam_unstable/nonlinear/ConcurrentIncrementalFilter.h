/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    ConcurrentBatchFilter.h
 * @brief   An iSAM2-based Filter that implements the
 *          Concurrent Filtering and Smoothing interface.
 * @author  Stephen Williams
 */

// \callgraph
#pragma once

#include <gtsam_unstable/nonlinear/ConcurrentFilteringAndSmoothing.h>
#include <gtsam/nonlinear/ISAM2.h>

namespace gtsam {

/**
 * An iSAM2-based Batch Filter that implements the Concurrent Filtering and Smoother interface.
 */
class GTSAM_UNSTABLE_EXPORT ConcurrentIncrementalFilter : public virtual ConcurrentFilter {

public:

  typedef boost::shared_ptr<ConcurrentIncrementalFilter> shared_ptr;
  typedef ConcurrentFilter Base; ///< typedef for base class

  /** Meta information returned about the update */
  struct Result {
    size_t iterations; ///< The number of optimizer iterations performed
    size_t nonlinearVariables; ///< The number of variables that can be relinearized
    size_t linearVariables; ///< The number of variables that must keep a constant linearization point
    size_t variablesReeliminated;
    size_t variablesRelinearized;

    /** The indices of the newly-added factors, in 1-to-1 correspondence with the
     * factors passed as \c newFactors update().  These indices may be
     * used later to refer to the factors in order to remove them.
     */
    FactorIndices newFactorsIndices;

    double error; ///< The final factor graph error

    /// Constructor
    Result() : iterations(0), nonlinearVariables(0), linearVariables(0), error(0) {};

    /// Getter methods
    size_t getIterations() const { return iterations; }
    size_t getNonlinearVariables() const { return nonlinearVariables; }
    size_t getLinearVariables() const { return linearVariables; }
    double getError() const { return error; }
  };

  /** Default constructor */
  ConcurrentIncrementalFilter(const ISAM2Params& parameters = ISAM2Params()) : isam2_(parameters) {};

  /** Default destructor */
  ~ConcurrentIncrementalFilter() override {};

  /** Implement a GTSAM standard 'print' function */
  void print(const std::string& s = "Concurrent Incremental Filter:\n", const KeyFormatter& keyFormatter = DefaultKeyFormatter) const override;

  /** Check if two Concurrent Filters are equal */
  bool equals(const ConcurrentFilter& rhs, double tol = 1e-9) const override;

  /** Access the current set of factors */
  const NonlinearFactorGraph& getFactors() const {
    return isam2_.getFactorsUnsafe();
  }

  /** Access the current linearization point */
  const ISAM2& getISAM2() const {
    return isam2_;
  }

  /** Access the current linearization point */
  const Values& getLinearizationPoint() const {
    return isam2_.getLinearizationPoint();
  }

  /** Access the current set of deltas to the linearization point */
  const VectorValues& getDelta() const {
    return isam2_.getDelta();
  }

  /** Compute the current best estimate of all variables and return a full Values structure.
   * If only a single variable is needed, it may be faster to call calculateEstimate(const KEY&).
   */
  Values calculateEstimate() const {
    return isam2_.calculateEstimate();
  }

  /** Compute the current best estimate of a single variable. This is generally faster than
   * calling the no-argument version of calculateEstimate if only specific variables are needed.
   * @param key
   * @return
   */
  template<class VALUE>
  VALUE calculateEstimate(Key key) const {
    return isam2_.calculateEstimate<VALUE>(key);
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
   * @param keysToMove An optional set of keys to move from the filter to the smoother
   * @param removeFactorIndices An optional set of indices corresponding to the factors you want to remove from the graph
   */
  Result update(const NonlinearFactorGraph& newFactors = NonlinearFactorGraph(), const Values& newTheta = Values(),
      const std::optional<FastList<Key> >& keysToMove = {},
      const std::optional< FactorIndices >& removeFactorIndices = {});

  /**
   * Perform any required operations before the synchronization process starts.
   * Called by 'synchronize'
   */
  void presync() override;

  /**
   * Populate the provided containers with factors that constitute the filter branch summarization
   * needed by the smoother. Also, linearization points for the new root clique must be provided.
   *
   * @param summarizedFactors The summarized factors for the filter branch
   * @param rootValues The linearization points of the root clique variables
   */
  void getSummarizedFactors(NonlinearFactorGraph& filterSummarization, Values& filterSummarizationValues) override;

  /**
   * Populate the provided containers with factors being sent to the smoother from the filter. These
   * may be original nonlinear factors, or factors encoding a summarization of the filter information.
   * The specifics will be implementation-specific for a given filter.
   *
   * @param smootherFactors The new factors to be added to the smoother
   * @param smootherValues The linearization points of any new variables
   */
  void getSmootherFactors(NonlinearFactorGraph& smootherFactors, Values& smootherValues) override;

  /**
   * Apply the updated version of the smoother branch summarized factors.
   *
   * @param summarizedFactors An updated version of the smoother branch summarized factors
   */
  void synchronize(const NonlinearFactorGraph& smootherSummarization, const Values& smootherSummarizationValues) override;

  /**
   * Perform any required operations after the synchronization process finishes.
   * Called by 'synchronize'
   */
  void postsync() override;

protected:

  ISAM2 isam2_; ///< The iSAM2 inference engine

  // ???
  NonlinearFactorGraph previousSmootherSummarization_; ///< The smoother summarization on the old separator sent by the smoother during the last synchronization
  FactorIndices currentSmootherSummarizationSlots_;  ///< The slots in factor graph that correspond to the current smoother summarization on the current separator
  NonlinearFactorGraph smootherShortcut_; ///< A set of conditional factors from the old separator to the current separator (recursively calculated during each filter update)

  // Storage for information to be sent to the smoother
  NonlinearFactorGraph smootherFactors_;  ///< A temporary holding place for the set of full nonlinear factors being sent to the smoother
  Values smootherValues_; ///< A temporary holding place for the linearization points of all keys being sent to the smoother

private:

  /** Traverse the iSAM2 Bayes Tree, inserting all descendants of the provided index/key into 'additionalKeys' */
  static void RecursiveMarkAffectedKeys(const Key& key, const ISAM2Clique::shared_ptr& clique, std::set<Key>& additionalKeys);

  /** Find the set of iSAM2 factors adjacent to 'keys' */
  static FactorIndices FindAdjacentFactors(const ISAM2& isam2, const FastList<Key>& keys, const FactorIndices& factorsToIgnore);

  /** Update the shortcut marginal between the current separator keys and the previous separator keys */
  // TODO: Make this a static function
  void updateShortcut(const NonlinearFactorGraph& removedFactors);

  /** Calculate marginal factors on the current separator variables using just the information in the filter */
  // TODO: Make this a static function
  NonlinearFactorGraph calculateFilterSummarization() const;

}; // ConcurrentBatchFilter

/// Typedef for Matlab wrapping
typedef ConcurrentIncrementalFilter::Result ConcurrentIncrementalFilterResult;

/// traits
template<>
struct traits<ConcurrentIncrementalFilter> : public Testable<ConcurrentIncrementalFilter> {
};

} //\ namespace gtsam
