/*
 * AllDiff.h
 * @brief General "all-different" constraint
 * @date Feb 6, 2012
 * @author Frank Dellaert
 */

#pragma once

#include <gtsam/discrete/BinaryAllDiff.h>
#include <gtsam/discrete/DiscreteKey.h>
#include <gtsam/discrete/TableFactor.h>

namespace gtsam {

/**
 * General AllDiff constraint.
 * Returns 1 if values for all keys are different, 0 otherwise.
 */
class GTSAM_EXPORT AllDiff : public Constraint {
  DiscreteKey discreteKey(size_t i) const {
    Key j = keys_[i];
    return DiscreteKey(j, cardinalities_.at(j));
  }

 public:
  /// Construct from keys.
  AllDiff(const DiscreteKeys& dkeys);

  /// Print the constraint.
  void print(const std::string& s = "", const KeyFormatter& formatter =
                                            DefaultKeyFormatter) const override;

  /// Return whether another factor is an equal constraint.
  bool equals(const DiscreteFactor& other, double tol) const override {
    if (!dynamic_cast<const AllDiff*>(&other))
      return false;
    else {
      const AllDiff& f(static_cast<const AllDiff&>(other));
      return cardinalities_.size() == f.cardinalities_.size() &&
             std::equal(cardinalities_.begin(), cardinalities_.end(),
                        f.cardinalities_.begin());
    }
  }

  /// Calculate value = expensive !
  double evaluate(const Assignment<Key>& values) const override;

  /// Convert into a decisiontree, can be *very* expensive !
  DecisionTreeFactor toDecisionTreeFactor() const override;

  /// Convert directly into a sparse table, enumerating only valid assignments.
  TableFactor toTableFactor() const override;

  /// Multiply into a decisiontree
  DecisionTreeFactor operator*(const DecisionTreeFactor& f) const override;

  /**
   * Ensure Arc-consistency by checking every possible value of domain j.
   * @param j domain to be checked
   * @param (in/out) domains all domains, but only domains->at(j) will be
   * checked.
   * @return true if domains->at(j) was changed, false otherwise.
   */
  bool ensureArcConsistency(Key j, Domains* domains) const override;

  /// Partially apply known values
  Constraint::shared_ptr partiallyApply(const DiscreteValues&) const override;

  /// Partially apply known values, domain version
  Constraint::shared_ptr partiallyApply(const Domains&) const override;
};

}  // namespace gtsam
