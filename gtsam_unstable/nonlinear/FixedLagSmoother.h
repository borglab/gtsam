/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    FixedLagSmoother.h
 * @brief   Base class for a fixed-lag smoother. This mimics the basic interface to iSAM2.
 *
 * @author  Stephen Williams
 * @date    Feb 27, 2013
 */

// \callgraph
#pragma once

#include <gtsam_unstable/dllexport.h>
#include <gtsam/inference/Key.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>

#include <map>
#include <vector>

namespace gtsam {

class GTSAM_UNSTABLE_EXPORT FixedLagSmoother {

public:

  /// Typedef for a shared pointer to an Incremental Fixed-Lag Smoother
  typedef boost::shared_ptr<FixedLagSmoother> shared_ptr;

  /// Typedef for a Key-Timestamp map/database
  typedef std::map<Key, double> KeyTimestampMap;
  typedef std::multimap<double, Key> TimestampKeyMap;

  /**
   * Meta information returned about the update
   */
  // TODO: Think of some more things to put here
  struct Result {
    size_t iterations; ///< The number of optimizer iterations performed
    size_t intermediateSteps; ///< The number of intermediate steps performed within the optimization. For L-M, this is the number of lambdas tried.
    size_t nonlinearVariables; ///< The number of variables that can be relinearized
    size_t linearVariables; ///< The number of variables that must keep a constant linearization point
    double error; ///< The final factor graph error
    Result() : iterations(0), intermediateSteps(0), nonlinearVariables(0), linearVariables(0), error(0) {};

    /// Getter methods
    size_t getIterations() const { return iterations; }
    size_t getIntermediateSteps() const { return intermediateSteps; }
    size_t getNonlinearVariables() const { return nonlinearVariables; }
    size_t getLinearVariables() const { return linearVariables; }
    double getError() const { return error; }
    void print() const;
  };

  /** default constructor */
  FixedLagSmoother(double smootherLag = 0.0) : smootherLag_(smootherLag) { }

  /** destructor */
  virtual ~FixedLagSmoother() { }

  /** Print the factor for debugging and testing (implementing Testable) */
  virtual void print(
      const std::string& s = "FixedLagSmoother:\n",
      const KeyFormatter& keyFormatter = DefaultKeyFormatter) const;

  /** Check if two IncrementalFixedLagSmoother Objects are equal */
  virtual bool equals(const FixedLagSmoother& rhs, double tol = 1e-9) const;

  /** read the current smoother lag */
  double smootherLag() const {
    return smootherLag_;
  }

  /** write to the current smoother lag */
  double& smootherLag() {
    return smootherLag_;
  }

  /** Access the current set of timestamps associated with each variable */
  const KeyTimestampMap& timestamps() const {
    return keyTimestampMap_;
  }

  /** Add new factors, updating the solution and relinearizing as needed. */
  virtual Result update(const NonlinearFactorGraph& newFactors = NonlinearFactorGraph(),
                        const Values& newTheta = Values(),
                        const KeyTimestampMap& timestamps = KeyTimestampMap(),
                        const FactorIndices& factorsToRemove = FactorIndices()) = 0;

  /** Compute an estimate from the incomplete linear delta computed during the last update.
   * This delta is incomplete because it was not updated below wildfire_threshold.  If only
   * a single variable is needed, it is faster to call calculateEstimate(const KEY&).
   */
  virtual Values calculateEstimate() const  = 0;


protected:

  /** The length of the smoother lag. Any variable older than this amount will be marginalized out. */
  double smootherLag_;

  /** The current timestamp associated with each tracked key */
  TimestampKeyMap timestampKeyMap_;
  KeyTimestampMap keyTimestampMap_;

  /** Update the Timestamps associated with the keys */
  void updateKeyTimestampMap(const KeyTimestampMap& newTimestamps);

  /** Erase keys from the Key-Timestamps database */
  void eraseKeyTimestampMap(const KeyVector& keys);

  /** Find the most recent timestamp of the system */
  double getCurrentTimestamp() const;

  /** Find all of the keys associated with timestamps before the provided time */
  KeyVector findKeysBefore(double timestamp) const;

  /** Find all of the keys associated with timestamps before the provided time */
  KeyVector findKeysAfter(double timestamp) const;

}; // FixedLagSmoother

/// Typedef for matlab wrapping
typedef FixedLagSmoother::KeyTimestampMap FixedLagSmootherKeyTimestampMap;
typedef FixedLagSmootherKeyTimestampMap::value_type FixedLagSmootherKeyTimestampMapValue;
typedef FixedLagSmoother::Result FixedLagSmootherResult;

} /// namespace gtsam
