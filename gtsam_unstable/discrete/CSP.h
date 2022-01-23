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
  using Values = DiscreteValues; ///< backwards compatibility

  /// Add a unary constraint, allowing only a single value
  void addSingleValue(const DiscreteKey& dkey, size_t value) {
    emplace_shared<SingleValue>(dkey, value);
  }

  /// Add a binary AllDiff constraint
  void addAllDiff(const DiscreteKey& key1, const DiscreteKey& key2) {
    emplace_shared<BinaryAllDiff>(key1, key2);
  }

  /// Add a general AllDiff constraint
  void addAllDiff(const DiscreteKeys& dkeys) { emplace_shared<AllDiff>(dkeys); }

  //    /** return product of all factors as a single factor */
  //    DecisionTreeFactor product() const {
  //      DecisionTreeFactor result;
  //      for(const sharedFactor& factor: *this)
  //        if (factor) result = (*factor) * result;
  //      return result;
  //    }

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
  //     void applyBeliefPropagation(size_t maxIterations = 10) const;

  /*
   * Apply arc-consistency ~ Approximate loopy belief propagation
   * We need to give the domains to a constraint, and it returns
   * a domain whose values don't conflict in the arc-consistency way.
   * TODO: should get cardinality from DiscreteKeys
   */
  Domains runArcConsistency(size_t cardinality,
                            size_t maxIterations = 10) const;

  /// Run arc consistency for all variables, return true if any domain changed.
  bool runArcConsistency(const VariableIndex& index, Domains* domains) const;

  /*
   * Create a new CSP, applying the given Domain constraints.
   */
  CSP partiallyApply(const Domains& domains) const;
};  // CSP

}  // namespace gtsam
