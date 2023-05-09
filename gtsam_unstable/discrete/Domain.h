/*
 * Domain.h
 * @brief Domain restriction constraint
 * @date Feb 13, 2012
 * @author Frank Dellaert
 */

#pragma once

#include <gtsam/discrete/DiscreteKey.h>
#include <gtsam_unstable/discrete/Constraint.h>

namespace gtsam {

/**
 * Domain restriction constraint
 */
class GTSAM_UNSTABLE_EXPORT Domain : public Constraint {
  size_t cardinality_;       /// Cardinality
  std::set<size_t> values_;  /// allowed values

 public:
  typedef boost::shared_ptr<Domain> shared_ptr;

  // Constructor on Discrete Key initializes an "all-allowed" domain
  Domain(const DiscreteKey& dkey)
      : Constraint(dkey.first), cardinality_(dkey.second) {
    for (size_t v = 0; v < cardinality_; v++) values_.insert(v);
  }

  // Constructor on Discrete Key with single allowed value
  // Consider SingleValue constraint
  Domain(const DiscreteKey& dkey, size_t v)
      : Constraint(dkey.first), cardinality_(dkey.second) {
    values_.insert(v);
  }

  /// Constructor
  Domain(const Domain& other)
      : Constraint(other.keys_[0]), values_(other.values_) {}

  /// insert a value, non const :-(
  void insert(size_t value) { values_.insert(value); }

  /// erase a value, non const :-(
  void erase(size_t value) { values_.erase(value); }

  size_t nrValues() const { return values_.size(); }

  bool isSingleton() const { return nrValues() == 1; }

  size_t firstValue() const { return *values_.begin(); }

  // print
  void print(const std::string& s = "", const KeyFormatter& formatter =
                                            DefaultKeyFormatter) const override;

  /// equals
  bool equals(const DiscreteFactor& other, double tol) const override {
    if (!dynamic_cast<const Domain*>(&other))
      return false;
    else {
      const Domain& f(static_cast<const Domain&>(other));
      return (cardinality_ == f.cardinality_) && (values_ == f.values_);
    }
  }

  bool contains(size_t value) const { return values_.count(value) > 0; }

  /// Calculate value
  double operator()(const Values& values) const override;

  /// Convert into a decisiontree
  DecisionTreeFactor toDecisionTreeFactor() const override;

  /// Multiply into a decisiontree
  DecisionTreeFactor operator*(const DecisionTreeFactor& f) const override;

  /*
   * Ensure Arc-consistency
   * @param j domain to be checked
   * @param domains all other domains
   */
  bool ensureArcConsistency(size_t j,
                            std::vector<Domain>& domains) const override;

  /**
   *  Check for a value in domain that does not occur in any other connected
   * domain. If found, we make this a singleton... Called in
   * AllDiff::ensureArcConsistency
   *  @param keys connected domains through alldiff
   */
  bool checkAllDiff(const KeyVector keys, std::vector<Domain>& domains);

  /// Partially apply known values
  Constraint::shared_ptr partiallyApply(const Values& values) const override;

  /// Partially apply known values, domain version
  Constraint::shared_ptr partiallyApply(
      const std::vector<Domain>& domains) const override;
};

}  // namespace gtsam
