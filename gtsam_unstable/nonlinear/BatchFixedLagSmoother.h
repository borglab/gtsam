/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    BatchFixedLagSmoother.h
 * @brief   An LM-based fixed-lag smoother. To the extent possible, this class mimics the iSAM2
 * interface. However, additional parameters, such as the smoother lag and the timestamp associated
 * with each variable are needed.
 *
 * @author  Michael Kaess, Stephen Williams
 * @date    Oct 14, 2012
 */

// \callgraph
#pragma once

#include <gtsam_unstable/nonlinear/LinearizedFactor.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>

#include <queue>

namespace gtsam {

class BatchFixedLagSmoother {

public:

  /// Typedef for a shared pointer to an Incremental Fixed-Lag Smoother
  typedef boost::shared_ptr<BatchFixedLagSmoother> shared_ptr;

  /// Typedef for a Key-Timestamp map/database
  typedef std::map<Key, double> KeyTimestampMap;
  typedef std::multimap<double, Key> TimestampKeyMap;

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


  /** default constructor */
  BatchFixedLagSmoother(const LevenbergMarquardtParams& parameters = LevenbergMarquardtParams(), double smootherLag = 0.0, bool enforceConsistency = false);

  /** destructor */
  virtual ~BatchFixedLagSmoother() { };

  // TODO: Create a better 'print'
  /** Print the factor for debugging and testing (implementing Testable) */
  virtual void print(const std::string& s = "BatchFixedLagSmoother:\n", const KeyFormatter& keyFormatter = DefaultKeyFormatter) const;

  /** Check if two IncrementalFixedLagSmoother Objects are equal */
  virtual bool equals(const BatchFixedLagSmoother& rhs, double tol = 1e-9) const;

  /**
   * Add new factors, updating the solution and relinearizing as needed.
   */
  Result update(const NonlinearFactorGraph& newFactors = NonlinearFactorGraph(), const Values& newTheta = Values(), const KeyTimestampMap& timestamps = KeyTimestampMap());

  /** Access the current set of timestamps associated with each variable */
  const KeyTimestampMap& getTimestamps() const {
    return keyTimestampMap_;
  }

  /** Access the current linearization point */
  const Values& getLinearizationPoint() const {
    return theta_;
  }

  /** Compute an estimate from the incomplete linear delta computed during the last update.
   * This delta is incomplete because it was not updated below wildfire_threshold.  If only
   * a single variable is needed, it is faster to call calculateEstimate(const KEY&).
   */
  Values calculateEstimate() const {
    //return theta_.retract(delta_, ordering_);
    return theta_;
  }

  /** Compute an estimate for a single variable using its incomplete linear delta computed
   * during the last update.  This is faster than calling the no-argument version of
   * calculateEstimate, which operates on all variables.
   * @param key
   * @return
   */
  template<class VALUE>
  VALUE calculateEstimate(Key key) const {
    //const Index index = ordering_.at(key);
    //const SubVector delta = delta_.at(index);
    //return theta_.at<VALUE>(key).retract(delta);
    return theta_.at<VALUE>(key);
  }

  /** Compute an estimate using a complete delta computed by a full back-substitution.
   */
  Values calculateBestEstimate() const {
    return calculateEstimate();
  }

  /** Return the final/best estimate of each variable encountered, including those that have been marginalized out previously.
   */
  Values calculateFinalEstimate() const {
    Values final(thetaFinal_);
    final.insert(theta_);
    return final;
  }


  /** Access the set of nonlinear factors */
  const NonlinearFactorGraph& getFactorsUnsafe() const {
    return factors_;
  }

  /** return the current set of iSAM2 parameters */
  const LevenbergMarquardtParams& params() const {
    return parameters_;
  }

  /** return the current smoother lag */
  double smootherLag() const {
    return smootherLag_;
  }



  /** return the marginal square root information matrix associated with a specific variable */
  Matrix marginalR(Key key) const;

  /** return the marginal information matrix associated with a specific variable */
  Matrix marginalInformation(Key key) const;

  /** return the marginal covariance associated with a specific variable */
  Matrix marginalCovariance(Key key) const;



protected:

  /** The nonlinear factors **/
  NonlinearFactorGraph factors_;

  /** The current linearization point **/
  Values theta_;

  /** The final estimate of each variable **/
  Values thetaFinal_;

  /** The L-M optimization parameters **/
  LevenbergMarquardtParams parameters_;

  /** The length of the smoother lag. Any variable older than this amount will be marginalized out. */
  double smootherLag_;

  /** A flag indicating if the optimizer should enforce probabilistic consistency by maintaining the
   * linearization point of all variables involved in linearized/marginal factors at the edge of the
   * smoothing window. This idea is from ??? TODO: Look up paper reference **/
  bool enforceConsistency_;

  /** The current timestamp associated with each tracked key */
  TimestampKeyMap timestampKeyMap_;
  KeyTimestampMap keyTimestampMap_;

  /** A typedef defining an Key-Factor mapping **/
  typedef std::map<Key, std::set<Index> > FactorIndex;

  /** A cross-reference structure to allow efficient factor lookups by key **/
  FactorIndex factorIndex_;

  /** The set of keys involved in current linearized factors. These keys should not be relinearized. **/
  //std::set<Key> linearizedKeys_;
  Values linearizedKeys_;

  /** The set of available factor graph slots. These occur because we are constantly deleting factors, leaving holes. **/
  std::queue<size_t> availableSlots_;


  /** Augment the list of factors with a set of new factors */
  void updateFactors(const NonlinearFactorGraph& newFactors);

  /** Remove factors from the list of factors by slot index */
  void removeFactors(const std::set<size_t>& deleteFactors);

  /** Update the Timestamps associated with the keys */
  void updateKeyTimestampMap(const KeyTimestampMap& newTimestamps);

  /** Find the most recent timestamp of the system */
  double getCurrentTimestamp() const;

  /** Find all of the keys associated with timestamps before the provided time */
  std::set<Key> findKeysBefore(double timestamp) const;

  /** Erase any keys associated with timestamps before the provided time */
  void eraseKeys(const std::set<Key>& keys);

  /** Marginalize out selected variables */
  void marginalizeKeys(const std::set<Key>& marginalizableKeys);


private:

  static void PrintKeySet(const std::set<Key>& keys, const std::string& label);

  static void PrintSymbolicFactor(const GaussianFactor::shared_ptr& factor, const Ordering& ordering);

  static void PrintSymbolicGraph(const GaussianFactorGraph& graph, const Ordering& ordering, const std::string& label);

  static void PrintSymbolicFactor(const NonlinearFactor::shared_ptr& factor);

  static void PrintSymbolicGraph(const NonlinearFactorGraph& graph, const std::string& label);

}; // BatchFixedLagSmoother

} /// namespace gtsam
