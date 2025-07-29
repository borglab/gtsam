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

#include <gtsam/nonlinear/FixedLagSmoother.h>
#include <gtsam/nonlinear/ISAM2.h>
#include "gtsam/dllexport.h"

namespace gtsam {

/**
 * This is a base class for the various HMF2 implementations. The HMF2 eliminates the factor graph
 * such that the active states are placed in/near the root. This base class implements a function
 * to calculate the ordering, and an update function to incorporate new factors into the HMF.
 */
class GTSAM_EXPORT IncrementalFixedLagSmoother: public FixedLagSmoother {

 public:

  /// Typedef for a shared pointer to an Incremental Fixed-Lag Smoother
  typedef std::shared_ptr<IncrementalFixedLagSmoother> shared_ptr;

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

  /** Check equality and get result */
  std::string equalsDetail(const FixedLagSmoother& rhs, double tol = 1e-9);

  /**
   * Add new factors, updating the solution and re-linearizing as needed.
   * @param newFactors new factors on old and/or new variables
   * @param newTheta new values for new variables only
   * @param timestamps an (optional) map from keys to real time stamps
   * @param factorsToRemove an (optional) list of factors to remove.
   * @param adaptiveSmootherLag an (optional) smoother lag.
   */
  Result update(const NonlinearFactorGraph& newFactors = NonlinearFactorGraph(),
                const Values& newTheta = Values(),  //
                const KeyTimestampMap& timestamps = KeyTimestampMap(),
                const FactorIndices& factorsToRemove = FactorIndices(),
                const double adaptiveSmootherLag = -1.0) override;

  /**
   * replace the Values with a new one, according to the keys in the Values.
   * @param newTheta new Values
   */
  void replaceValues(const Values& newTheta = Values());

  /**
   * Marginalize using a smaller lag based on the original isam2 solution, keep the origin one.
   * @param adaptiveSmootherLag a customized smoother lag
   * @param isamParam isam2 paramter for a temperal optimizer.
   */
  Values calculateSubEstimate(
      const double adaptiveSmootherLag = -1.0,
      const ISAM2Params& isamParam = DefaultISAM2Params());

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

  /// Get the iSAM2 object which is used for the inference internally
  const ISAM2& getISAM2() const { return isam_; }

  /// Get the sub iSAM2 object which is used for the inference internally
  const ISAM2& getSubISAM2() const { return subIsam_; }

  /// Get the initial value when calling update()
  const gtsam::Values& getInitialTheta() const { return initialTheta_; }

  /** Deep clone the current Smoother 
   * @param rewrite deep clone the internal ISAM2 object, default true
   */
  const IncrementalFixedLagSmoother deepClone(const bool rewrite = true);

  /// force relinearize the internal ISAM2 object.
  const void forceRelinearize();

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

  /** An iSAM2 object used to perform inference for sub inference*/
  ISAM2 subIsam_;

  /** Store results of latest isam2 update */
  ISAM2Result isamResult_;

  /** Store initial value when calling update() */
  gtsam::Values initialTheta_;

  /** Store serialization string */
  std::string smootherString_;

  /** Smoother name for serialization */
  std::string smootherName_ = "IncrementalSmoother";

  /** Erase any keys associated with timestamps before the provided time */
  void eraseKeysBefore(double timestamp);

  /** Fill in an iSAM2 ConstrainedKeys structure such that the provided keys are eliminated before all others */
  void createOrderingConstraints(const KeyVector& marginalizableKeys,
      std::optional<FastMap<Key, int> >& constrainedKeys) const;

  void setISAM2(ISAM2 isam) { isam_ = isam; }
  void setInitialTheta(gtsam::Values initialTheta) {
    initialTheta_ = initialTheta;
  }
  void setKeyTimestampMap(KeyTimestampMap keyTimestampMap,
                          TimestampKeyMap timestampKeyMap) {
    keyTimestampMap_ = keyTimestampMap;
    timestampKeyMap_ = timestampKeyMap;
  }
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
#if GTSAM_ENABLE_BOOST_SERIALIZATION
  /// Serialization function
  friend class boost::serialization::access;
  template <class Archive>
  void serialize(Archive& ar, const unsigned int version) {

    // ar& boost::serialization::base_object<FixedLagSmoother>(*this);
    ar& BOOST_SERIALIZATION_BASE_OBJECT_NVP(FixedLagSmoother);
    ar& BOOST_SERIALIZATION_NVP(smootherLag_);
    ar& BOOST_SERIALIZATION_NVP(timestampKeyMap_);
    ar& BOOST_SERIALIZATION_NVP(keyTimestampMap_);
    ar& BOOST_SERIALIZATION_NVP(isam_);
    ar& BOOST_SERIALIZATION_NVP(subIsam_);
    ar& BOOST_SERIALIZATION_NVP(isamResult_);
    ar& BOOST_SERIALIZATION_NVP(initialTheta_);
    ar& BOOST_SERIALIZATION_NVP(smootherString_);
    ar& BOOST_SERIALIZATION_NVP(smootherName_);
  }
#endif
};
// IncrementalFixedLagSmoother

}/// namespace gtsam

