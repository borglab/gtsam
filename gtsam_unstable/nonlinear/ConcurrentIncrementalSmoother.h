/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    ConcurrentIncrementalSmoother.h
 * @brief   An iSAM2-based Smoother that implements the
 *          Concurrent Filtering and Smoothing interface.
 * @author  Stephen Williams
 */

// \callgraph
#pragma once

#include <gtsam_unstable/nonlinear/ConcurrentFilteringAndSmoothing.h>
#include <gtsam/nonlinear/ISAM2.h>

namespace gtsam {

/**
 * A Levenberg-Marquardt Batch Smoother that implements the Concurrent Filtering and Smoother interface.
 */
class GTSAM_UNSTABLE_EXPORT ConcurrentIncrementalSmoother : public virtual ConcurrentSmoother {

public:
  typedef boost::shared_ptr<ConcurrentIncrementalSmoother> shared_ptr;
  typedef ConcurrentSmoother Base; ///< typedef for base class

  /** Meta information returned about the update */
  struct Result {
    size_t iterations; ///< The number of optimizer iterations performed
    size_t nonlinearVariables; ///< The number of variables that can be relinearized
    size_t linearVariables; ///< The number of variables that must keep a constant linearization point
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
  ConcurrentIncrementalSmoother(const ISAM2Params& parameters = ISAM2Params()) : isam2_(parameters) {};

  /** Default destructor */
  ~ConcurrentIncrementalSmoother() override {};

  /** Implement a GTSAM standard 'print' function */
  void print(const std::string& s = "Concurrent Incremental Smoother:\n", const KeyFormatter& keyFormatter = DefaultKeyFormatter) const override;

  /** Check if two Concurrent Smoothers are equal */
  bool equals(const ConcurrentSmoother& rhs, double tol = 1e-9) const override;

  /** Access the current set of factors */
  const NonlinearFactorGraph& getFactors() const {
    return isam2_.getFactorsUnsafe();
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
  Result update(const NonlinearFactorGraph& newFactors = NonlinearFactorGraph(), const Values& newTheta = Values(),
      const boost::optional<FactorIndices>& removeFactorIndices = boost::none);

  /**
   * Perform any required operations before the synchronization process starts.
   * Called by 'synchronize'
   */
  void presync() override;

  /**
   * Populate the provided containers with factors that constitute the smoother branch summarization
   * needed by the filter.
   *
   * @param summarizedFactors The summarized factors for the filter branch
   */
  void getSummarizedFactors(NonlinearFactorGraph& summarizedFactors, Values& separatorValues) override;

  /**
   * Apply the new smoother factors sent by the filter, and the updated version of the filter
   * branch summarized factors.
   *
   * @param smootherFactors A set of new factors added to the smoother from the filter
   * @param smootherValues Linearization points for any new variables
   * @param summarizedFactors An updated version of the filter branch summarized factors
   * @param rootValues The linearization point of the root variables
   */
  void synchronize(const NonlinearFactorGraph& smootherFactors, const Values& smootherValues,
      const NonlinearFactorGraph& summarizedFactors, const Values& separatorValues) override;

  /**
   * Perform any required operations after the synchronization process finishes.
   * Called by 'synchronize'
   */
  void postsync() override;

protected:

  ISAM2 isam2_; ///< iSAM2 inference engine

  // Storage for information received from the filter during the last synchronization
  NonlinearFactorGraph smootherFactors_; ///< New factors to be added to the smoother during the next update
  Values smootherValues_; ///< New variables to be added to the smoother during the next update
  NonlinearFactorGraph filterSummarizationFactors_; ///< New filter summarization factors to replace the existing filter summarization during the next update
  Values separatorValues_; ///< The linearization points of the separator variables. These should not be changed during optimization.
  FactorIndices filterSummarizationSlots_;  ///< The slots in factor graph that correspond to the current filter summarization factors
  bool synchronizationUpdatesAvailable_; ///< Flag indicating the currently stored synchronization updates have not been applied yet

  // Storage for information to be sent to the filter
  NonlinearFactorGraph smootherSummarization_; ///< A temporary holding place for calculated smoother summarization

private:

  /** Calculate the smoother marginal factors on the separator variables */
  void updateSmootherSummarization();

}; // ConcurrentBatchSmoother

/// Typedef for Matlab wrapping
typedef ConcurrentIncrementalSmoother::Result ConcurrentIncrementalSmootherResult;

/// traits
template<>
struct traits<ConcurrentIncrementalSmoother> : public Testable<ConcurrentIncrementalSmoother> {
};

} // \ namespace gtsam
