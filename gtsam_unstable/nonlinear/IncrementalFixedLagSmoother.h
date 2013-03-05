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
class IncrementalFixedLagSmoother : public FixedLagSmoother {

public:

  /// Typedef for a shared pointer to an Incremental Fixed-Lag Smoother
  typedef boost::shared_ptr<IncrementalFixedLagSmoother> shared_ptr;

  /** default constructor */
  IncrementalFixedLagSmoother(double smootherLag = 0.0, const ISAM2Params& parameters = ISAM2Params()) :
    FixedLagSmoother(smootherLag), isam_(parameters) { };

  /** destructor */
  virtual ~IncrementalFixedLagSmoother() { };

  /** Print the factor for debugging and testing (implementing Testable) */
  virtual void print(const std::string& s = "IncrementalFixedLagSmoother:\n", const KeyFormatter& keyFormatter = DefaultKeyFormatter) const;

  /** Check if two IncrementalFixedLagSmoother Objects are equal */
  virtual bool equals(const FixedLagSmoother& rhs, double tol = 1e-9) const;

  /** Add new factors, updating the solution and relinearizing as needed. */
  Result update(const NonlinearFactorGraph& newFactors = NonlinearFactorGraph(), const Values& newTheta = Values(),
      const KeyTimestampMap& timestamps = KeyTimestampMap());

  /** Compute an estimate from the incomplete linear delta computed during the last update.
   * This delta is incomplete because it was not updated below wildfire_threshold.  If only
   * a single variable is needed, it is faster to call calculateEstimate(const KEY&).
   */
  Values calculateEstimate() const {
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

protected:
  /** An iSAM2 object used to perform inference. The smoother lag is controlled
   * by what factors are removed each iteration */
  ISAM2 isam_;

  /** Erase any keys associated with timestamps before the provided time */
  void eraseKeysBefore(double timestamp);

  /** Fill in an iSAM2 ConstrainedKeys structure such that the provided keys are eliminated before all others */
  void createOrderingConstraints(const std::set<Key>& marginalizableKeys, boost::optional<FastMap<Key,int> >& constrainedKeys) const;

private:
  /** Private methods for printing debug information */
  static void PrintKeySet(const std::set<Key>& keys, const std::string& label = "Keys:");
  static void PrintSymbolicFactor(const GaussianFactor::shared_ptr& factor, const gtsam::Ordering& ordering);
  static void PrintSymbolicGraph(const GaussianFactorGraph& graph, const gtsam::Ordering& ordering, const std::string& label = "Factor Graph:");
  static void PrintSymbolicTree(const gtsam::ISAM2& isam, const std::string& label = "Bayes Tree:");
  static void PrintSymbolicTreeHelper(const gtsam::ISAM2Clique::shared_ptr& clique, const gtsam::Ordering& ordering, const std::string indent = "");

}; // IncrementalFixedLagSmoother

} /// namespace gtsam
