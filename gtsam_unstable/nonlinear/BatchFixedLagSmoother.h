/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    BatchFixedLagSmoother.h
 * @brief   An LM-based fixed-lag smoother.
 *
 * @author  Michael Kaess, Stephen Williams
 * @date    Oct 14, 2012
 */

// \callgraph
#pragma once

#include <gtsam_unstable/nonlinear/FixedLagSmoother.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <queue>

namespace gtsam {

class GTSAM_UNSTABLE_EXPORT BatchFixedLagSmoother : public FixedLagSmoother {

public:

  /// Typedef for a shared pointer to an Incremental Fixed-Lag Smoother
  typedef boost::shared_ptr<BatchFixedLagSmoother> shared_ptr;

  /** default constructor */
  BatchFixedLagSmoother(double smootherLag = 0.0, const LevenbergMarquardtParams& parameters = LevenbergMarquardtParams(), bool enforceConsistency = true) :
    FixedLagSmoother(smootherLag), parameters_(parameters), enforceConsistency_(enforceConsistency) { };

  /** destructor */
  virtual ~BatchFixedLagSmoother() { };

  /** Print the factor for debugging and testing (implementing Testable) */
  virtual void print(const std::string& s = "BatchFixedLagSmoother:\n", const KeyFormatter& keyFormatter = DefaultKeyFormatter) const;

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
    return theta_.retract(delta_, ordering_);
  }

  /** Compute an estimate for a single variable using its incomplete linear delta computed
   * during the last update.  This is faster than calling the no-argument version of
   * calculateEstimate, which operates on all variables.
   * @param key
   * @return
   */
  template<class VALUE>
  VALUE calculateEstimate(Key key) const {
    const Index index = ordering_.at(key);
    const Vector delta = delta_.at(index);
    return theta_.at<VALUE>(key).retract(delta);
  }

  /** read the current set of optimizer parameters */
  const LevenbergMarquardtParams& params() const {
    return parameters_;
  }

  /** update the current set of optimizer parameters */
  LevenbergMarquardtParams& params() {
    return parameters_;
  }

  /** Access the current set of factors */
  const NonlinearFactorGraph& getFactors() const {
    return factors_;
  }

  /** Access the current linearization point */
  const Values& getLinearizationPoint() const {
    return theta_;
  }

  /** Access the current ordering */
  const OrderingOrdered& getOrdering() const {
    return ordering_;
  }

  /** Access the current set of deltas to the linearization point */
  const VectorValuesOrdered& getDelta() const {
    return delta_;
  }

protected:

  /** A typedef defining an Key-Factor mapping **/
  typedef std::map<Key, std::set<Index> > FactorIndex;

  /** The L-M optimization parameters **/
  LevenbergMarquardtParams parameters_;

  /** A flag indicating if the optimizer should enforce probabilistic consistency by maintaining the
   * linearization point of all variables involved in linearized/marginal factors at the edge of the
   * smoothing window. This idea is from ??? TODO: Look up paper reference **/
  bool enforceConsistency_;

  /** The nonlinear factors **/
  NonlinearFactorGraph factors_;

  /** The current linearization point **/
  Values theta_;

  /** The set of keys involved in current linear factors. These keys should not be relinearized. **/
  Values linearKeys_;

  /** The current ordering */
  OrderingOrdered ordering_;

  /** The current set of linear deltas */
  VectorValuesOrdered delta_;

  /** The set of available factor graph slots. These occur because we are constantly deleting factors, leaving holes. **/
  std::queue<size_t> availableSlots_;

  /** A cross-reference structure to allow efficient factor lookups by key **/
  FactorIndex factorIndex_;



  /** Augment the list of factors with a set of new factors */
  void insertFactors(const NonlinearFactorGraph& newFactors);

  /** Remove factors from the list of factors by slot index */
  void removeFactors(const std::set<size_t>& deleteFactors);

  /** Erase any keys associated with timestamps before the provided time */
  void eraseKeys(const std::set<Key>& keys);

  /** Use colamd to update into an efficient ordering */
  void reorder(const std::set<Key>& marginalizeKeys = std::set<Key>());

  /** Optimize the current graph using a modified version of L-M */
  Result optimize();

  /** Marginalize out selected variables */
  void marginalize(const std::set<Key>& marginalizableKeys);


  // A custom elimination tree that supports forests and partial elimination
  class EliminationForest {
  public:
    typedef boost::shared_ptr<EliminationForest> shared_ptr; ///< Shared pointer to this class

  private:
    typedef FastList<GaussianFactorOrdered::shared_ptr> Factors;
    typedef FastList<shared_ptr> SubTrees;
    typedef std::vector<GaussianConditionalOrdered::shared_ptr> Conditionals;

    Index key_; ///< index associated with root
    Factors factors_; ///< factors associated with root
    SubTrees subTrees_; ///< sub-trees

    /** default constructor, private, as you should use Create below */
    EliminationForest(Index key = 0) : key_(key) {}

    /**
     * Static internal function to build a vector of parent pointers using the
     * algorithm of Gilbert et al., 2001, BIT.
     */
    static std::vector<Index> ComputeParents(const VariableIndexOrdered& structure);

    /** add a factor, for Create use only */
    void add(const GaussianFactorOrdered::shared_ptr& factor) { factors_.push_back(factor); }

    /** add a subtree, for Create use only */
    void add(const shared_ptr& child) { subTrees_.push_back(child); }

  public:

    /** return the key associated with this tree node */
    Index key() const { return key_; }

    /** return the const reference of children */
    const SubTrees& children() const { return subTrees_; }

    /** return the const reference to the factors */
    const Factors& factors() const { return factors_; }

    /** Create an elimination tree from a factor graph */
    static std::vector<shared_ptr> Create(const GaussianFactorGraphOrdered& factorGraph, const VariableIndexOrdered& structure);

    /** Recursive routine that eliminates the factors arranged in an elimination tree */
    GaussianFactorOrdered::shared_ptr eliminateRecursive(GaussianFactorGraphOrdered::Eliminate function);

    /** Recursive function that helps find the top of each tree */
    static void removeChildrenIndices(std::set<Index>& indices, const EliminationForest::shared_ptr& tree);
  };

private:
  /** Private methods for printing debug information */
  static void PrintKeySet(const std::set<Key>& keys, const std::string& label);
  static void PrintSymbolicFactor(const NonlinearFactor::shared_ptr& factor);
  static void PrintSymbolicFactor(const GaussianFactorOrdered::shared_ptr& factor, const OrderingOrdered& ordering);
  static void PrintSymbolicGraph(const NonlinearFactorGraph& graph, const std::string& label);
  static void PrintSymbolicGraph(const GaussianFactorGraphOrdered& graph, const OrderingOrdered& ordering, const std::string& label);
}; // BatchFixedLagSmoother

} /// namespace gtsam
