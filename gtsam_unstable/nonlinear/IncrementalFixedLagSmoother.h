/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    IncrementalFixedLagSmoother.h
 * @brief   An iSAM2-based fixed-lag smoother.
 *
 * @author  Michael Kaess, Stephen Williams
 * @date    Oct 14, 2012
 */

// \callgraph
#pragma once

#include <gtsam_unstable/nonlinear/FixedLagSmoother.h>
#include <gtsam/nonlinear/ISAM2.h>

namespace gtsam {

/**
 * This is a base class for the various HMF2 implementations. The HMF2 eliminates the factor graph
 * such that the active states are placed in/near the root. This base class implements a function
 * to calculate the ordering, and an update function to incorporate new factors into the HMF.
 */
class GTSAM_UNSTABLE_EXPORT IncrementalFixedLagSmoother: public FixedLagSmoother {

public:

  /// Typedef for a shared pointer to an Incremental Fixed-Lag Smoother
  typedef boost::shared_ptr<IncrementalFixedLagSmoother> shared_ptr;

  /** default constructor */
  IncrementalFixedLagSmoother(double smootherLag = 0.0,
      const ISAM2Params& parameters = DefaultISAM2Params()) :
      FixedLagSmoother(smootherLag), isam_(parameters) {
  }

  /** destructor */
  ~IncrementalFixedLagSmoother() override {
  }

  /** Print the factor for debugging and testing (implementing Testable) */
  void print(const std::string& s = "IncrementalFixedLagSmoother:\n",
      const KeyFormatter& keyFormatter = DefaultKeyFormatter) const override;

  /** Check if two IncrementalFixedLagSmoother Objects are equal */
  bool equals(const FixedLagSmoother& rhs, double tol = 1e-9) const override;

  /**
   * Add new factors, updating the solution and re-linearizing as needed.
   * @param newFactors new factors on old and/or new variables
   * @param newTheta new values for new variables only
   * @param timestamps an (optional) map from keys to real time stamps
   * @param factorsToRemove an (optional) list of factors to remove.
   */
  Result update(const NonlinearFactorGraph& newFactors = NonlinearFactorGraph(),
                const Values& newTheta = Values(), //
                const KeyTimestampMap& timestamps = KeyTimestampMap(),
                const FactorIndices& factorsToRemove = FactorIndices()) override;

  /** Compute an estimate from the incomplete linear delta computed during the last update.
   * This delta is incomplete because it was not updated below wildfire_threshold.  If only
   * a single variable is needed, it is faster to call calculateEstimate(const KEY&).
   */
  Values calculateEstimate() const override {
    return isam_.calculateEstimate();
  }

  /** Compute an estimate for a single variable using its incomplete linear delta computed
   * during the last update.  This is faster than calling the no-argument version of
   * calculateEstimate, which operates on all variables.
   * @param key
   * @return
   */
  template<class VALUE>
  VALUE calculateEstimate(Key key) const {
    return isam_.calculateEstimate<VALUE>(key);
  }

  /** return the current set of iSAM2 parameters */
  const ISAM2Params& params() const {
    return isam_.params();
  }

  /** Access the current set of factors */
  const NonlinearFactorGraph& getFactors() const {
    return isam_.getFactorsUnsafe();
  }

  /** Access the current linearization point */
  const Values& getLinearizationPoint() const {
    return isam_.getLinearizationPoint();
  }

  /** Access the current set of deltas to the linearization point */
  const VectorValues& getDelta() const {
    return isam_.getDelta();
  }

  /// Calculate marginal covariance on given variable
  Matrix marginalCovariance(Key key) const {
    return isam_.marginalCovariance(key);
  }

  /// Get results of latest isam2 update
  const ISAM2Result& getISAM2Result() const{ return isamResult_; }

protected:

  /** Create default parameters */
  static ISAM2Params DefaultISAM2Params() {
    ISAM2Params params;
    params.findUnusedFactorSlots = true;
    return params;
  }

  /** An iSAM2 object used to perform inference. The smoother lag is controlled
   * by what factors are removed each iteration */
  ISAM2 isam_;

  /** Store results of latest isam2 update */
  ISAM2Result isamResult_;

  /** Erase any keys associated with timestamps before the provided time */
  void eraseKeysBefore(double timestamp);

  /** Fill in an iSAM2 ConstrainedKeys structure such that the provided keys are eliminated before all others */
  void createOrderingConstraints(const KeyVector& marginalizableKeys,
      boost::optional<FastMap<Key, int> >& constrainedKeys) const;

private:
  /** Private methods for printing debug information */
  static void PrintKeySet(const std::set<Key>& keys, const std::string& label =
      "Keys:");
  static void PrintSymbolicFactor(const GaussianFactor::shared_ptr& factor);
  static void PrintSymbolicGraph(const GaussianFactorGraph& graph,
      const std::string& label = "Factor Graph:");
  static void PrintSymbolicTree(const gtsam::ISAM2& isam,
      const std::string& label = "Bayes Tree:");
  static void PrintSymbolicTreeHelper(
      const gtsam::ISAM2Clique::shared_ptr& clique, const std::string indent =
          "");

};
// IncrementalFixedLagSmoother

}/// namespace gtsam
