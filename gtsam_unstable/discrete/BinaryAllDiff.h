/*
 * BinaryAllDiff.h
 * @brief Binary "all-different" constraint
 * @date Feb 6, 2012
 * @author Frank Dellaert
 */

#pragma once

#include <gtsam/discrete/DecisionTreeFactor.h>
#include <gtsam_unstable/discrete/Constraint.h>
#include <gtsam_unstable/discrete/Domain.h>

namespace gtsam {

/**
 * Binary AllDiff constraint
 * Returns 1 if values for two keys are different, 0 otherwise
 * DiscreteFactors are all awkward in that they have to store two types of keys:
 * for each variable we have a Index and an Index. In this factor, we
 * keep the Indices locally, and the Indices are stored in IndexFactor.
 */
class BinaryAllDiff : public Constraint {
  size_t cardinality0_, cardinality1_;  /// cardinality

 public:
  /// Constructor
  BinaryAllDiff(const DiscreteKey& key1, const DiscreteKey& key2)
      : Constraint(key1.first, key2.first),
        cardinality0_(key1.second),
        cardinality1_(key2.second) {}

  // print
  void print(
      const std::string& s = "",
      const KeyFormatter& formatter = DefaultKeyFormatter) const override {
    std::cout << s << "BinaryAllDiff on " << formatter(keys_[0]) << " and "
              << formatter(keys_[1]) << std::endl;
  }

  /// equals
  bool equals(const DiscreteFactor& other, double tol) const override {
    if (!dynamic_cast<const BinaryAllDiff*>(&other))
      return false;
    else {
      const BinaryAllDiff& f(static_cast<const BinaryAllDiff&>(other));
      return (cardinality0_ == f.cardinality0_) &&
             (cardinality1_ == f.cardinality1_);
    }
  }

  /// Calculate value
  double operator()(const Values& values) const override {
    return (double)(values.at(keys_[0]) != values.at(keys_[1]));
  }

  /// Convert into a decisiontree
  DecisionTreeFactor toDecisionTreeFactor() const override {
    DiscreteKeys keys;
    keys.push_back(DiscreteKey(keys_[0], cardinality0_));
    keys.push_back(DiscreteKey(keys_[1], cardinality1_));
    std::vector<double> table;
    for (size_t i1 = 0; i1 < cardinality0_; i1++)
      for (size_t i2 = 0; i2 < cardinality1_; i2++) table.push_back(i1 != i2);
    DecisionTreeFactor converted(keys, table);
    return converted;
  }

  /// Multiply into a decisiontree
  DecisionTreeFactor operator*(const DecisionTreeFactor& f) const override {
    // TODO: can we do this more efficiently?
    return toDecisionTreeFactor() * f;
  }

  /*
   * Ensure Arc-consistency
   * @param j domain to be checked
   * @param domains all other domains
   */
  bool ensureArcConsistency(size_t j,
                            std::vector<Domain>& domains) const override {
    //      throw std::runtime_error(
    //          "BinaryAllDiff::ensureArcConsistency not implemented");
    return false;
  }

  /// Partially apply known values
  Constraint::shared_ptr partiallyApply(const Values&) const override {
    throw std::runtime_error("BinaryAllDiff::partiallyApply not implemented");
  }

  /// Partially apply known values, domain version
  Constraint::shared_ptr partiallyApply(
      const std::vector<Domain>&) const override {
    throw std::runtime_error("BinaryAllDiff::partiallyApply not implemented");
  }
};

}  // namespace gtsam
