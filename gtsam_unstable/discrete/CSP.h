/*
 * CSP.h
 * @brief Constraint Satisfaction Problem class
 * @date Feb 6, 2012
 * @author Frank Dellaert
 */

#pragma once

#include <gtsam/discrete/DiscreteFactorGraph.h>
#include <gtsam_unstable/discrete/AllDiff.h>
#include <gtsam_unstable/discrete/SingleValue.h>

namespace gtsam {

/**
 * Constraint Satisfaction Problem class
 * A specialization of a DiscreteFactorGraph.
 * It knows about CSP-specific constraints and algorithms
 */
class GTSAM_UNSTABLE_EXPORT CSP : public DiscreteFactorGraph {
 public:
  /** A map from keys to values */
  typedef KeyVector Indices;
  typedef Assignment<Key> Values;
  typedef boost::shared_ptr<Values> sharedValues;

 public:
  //    /// Constructor
  //    CSP() {
  //    }

  /// Add a unary constraint, allowing only a single value
  void addSingleValue(const DiscreteKey& dkey, size_t value) {
    boost::shared_ptr<SingleValue> factor(new SingleValue(dkey, value));
    push_back(factor);
  }

  /// Add a binary AllDiff constraint
  void addAllDiff(const DiscreteKey& key1, const DiscreteKey& key2) {
    boost::shared_ptr<BinaryAllDiff> factor(new BinaryAllDiff(key1, key2));
    push_back(factor);
  }

  /// Add a general AllDiff constraint
  void addAllDiff(const DiscreteKeys& dkeys) {
    boost::shared_ptr<AllDiff> factor(new AllDiff(dkeys));
    push_back(factor);
  }

  //    /** return product of all factors as a single factor */
  //    DecisionTreeFactor product() const {
  //      DecisionTreeFactor result;
  //      for(const sharedFactor& factor: *this)
  //        if (factor) result = (*factor) * result;
  //      return result;
  //    }

  /// Find the best total assignment - can be expensive
  sharedValues optimalAssignment() const;

  /// Find the best total assignment - can be expensive
  sharedValues optimalAssignment(const Ordering& ordering) const;

  //    /*
  //     * Perform loopy belief propagation
  //     * True belief propagation would check for each value in domain
  //     * whether any satisfying separator assignment can be found.
  //     * This corresponds to hyper-arc consistency in CSP speak.
  //     * This can be done by creating a mini-factor graph and search.
  //     * For a nine-by-nine Sudoku, the search tree will be 8+6+6=20 levels
  //     deep.
  //     * It will be very expensive to exclude values that way.
  //     */
  //     void applyBeliefPropagation(size_t nrIterations = 10) const;

  /*
   * Apply arc-consistency ~ Approximate loopy belief propagation
   * We need to give the domains to a constraint, and it returns
   * a domain whose values don't conflict in the arc-consistency way.
   * TODO: should get cardinality from Indices
   */
  void runArcConsistency(size_t cardinality, size_t nrIterations = 10,
                         bool print = false) const;
};  // CSP

}  // namespace gtsam
