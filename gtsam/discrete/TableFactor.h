/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file TableFactor.h
 * @date May 4, 2023
 * @author Yoonwoo Kim, Varun Agrawal
 */

#pragma once

#include <gtsam/discrete/DecisionTreeFactor.h>
#include <gtsam/discrete/DiscreteFactor.h>
#include <gtsam/discrete/DiscreteKey.h>
#include <gtsam/discrete/Ring.h>

#include <Eigen/Sparse>
#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#if GTSAM_ENABLE_BOOST_SERIALIZATION
#include <gtsam/base/MatrixSerialization.h>

#include <boost/serialization/nvp.hpp>
#endif

namespace gtsam {

class DiscreteConditional;
class HybridValues;

/**
 * A discrete probabilistic factor optimized for sparsity.
 * Uses sparse_table_ to store only the nonzero probabilities.
 * Computes the assigned value for the key using the ordering which the
 * nonzero probabilties are stored in. (lazy cartesian product)
 *
 * @ingroup discrete
 */
class GTSAM_EXPORT TableFactor : public DiscreteFactor {
 protected:
  /// SparseVector of nonzero probabilities.
  Eigen::SparseVector<double> sparse_table_;

 private:
  /// Map of Keys and their denominators used in keyValueForIndex.
  std::map<Key, size_t> denominators_;
  /// Sorted DiscreteKeys to use internally.
  DiscreteKeys sorted_dkeys_;

  /**
   * Return the discrete value assigned to `target_key` at a linear table
   * index.
   *
   * A key's discrete value is the integer identifying one of its alternatives,
   * ranging from zero to its cardinality minus one. It is distinct from the
   * factor value stored at a table index. Keys use the order in which they
   * were supplied to the factor, and the last key varies fastest.
   * `denominators_[target_key]` is the product of the cardinalities of all
   * keys after `target_key`, so the discrete value is
   * `(index / denominators_[target_key]) % cardinality(target_key)`.
   *
   * `index` is the linear index in the full table, even when the table is
   * sparse; it is not the ordinal number of a nonzero entry. For example, let
   * `v0` have cardinality 2 and `v1` cardinality 3:
   *
   *   index | v0 discrete value | v1 discrete value | factor value
   *   ------|-------------------|-------------------|-------------
   *       0 |                 0 |                 0 |          10
   *       1 |                 0 |                 1 |          21
   *       2 |                 0 |                 2 |          32
   *       3 |                 1 |                 0 |          43
   *       4 |                 1 |                 1 |          54
   *       5 |                 1 |                 2 |          65
   *
   * At index 2, `keyValueForIndex(v1, 2)` returns the discrete value 2; the
   * factor value in that row is 32. At index 5, the discrete value for `v1`
   * is also 2, but the factor value is 65. Meanwhile,
   * `keyValueForIndex(v0, 2)` returns 0. With three keys `(x, y, z)` of
   * cardinalities `(2, 3, 2)`, `z` changes every index, `y` every 2 indices,
   * and `x` every 6 indices. Index 8 represents the discrete values
   * `(x, y, z) = (1, 1, 0)`.
   *
   * @param target_key Key whose discrete value to decode.
   * @param index Linear index in the full Cartesian-product table.
   * @return The discrete value of `target_key` at `index`.
   */
  size_t keyValueForIndex(Key target_key, uint64_t index) const;

  /**
   * @brief Return ith key in keys_ as a DiscreteKey
   * @param i ith key in keys_
   * @return DiscreteKey
   */
  DiscreteKey discreteKey(size_t i) const {
    return DiscreteKey(keys_[i], cardinalities_.at(keys_[i]));
  }

 public:
  /**
   * Convert probability table given as doubles to SparseVector.
   * Example: {0, 1, 1, 0, 0, 1, 0} -> values: {1, 1, 1}, indices: {1, 2, 5}
   */
  static Eigen::SparseVector<double> Convert(const DiscreteKeys& keys,
                                             const std::vector<double>& table);

  /// Convert probability table given as string to SparseVector.
  static Eigen::SparseVector<double> Convert(const DiscreteKeys& keys,
                                             const std::string& table);

  // typedefs needed to play nice with gtsam
  typedef TableFactor This;
  typedef DiscreteFactor Base;  ///< Typedef to base class
  typedef std::shared_ptr<TableFactor> shared_ptr;
  typedef Eigen::SparseVector<double>::InnerIterator SparseIt;
  typedef std::vector<std::pair<DiscreteValues, double>> AssignValList;

  /// @name Standard Constructors
  /// @{

  /** Default constructor for I/O */
  TableFactor();

  /** Constructor from DiscreteKeys and TableFactor */
  TableFactor(const DiscreteKeys& keys, const TableFactor& potentials);

  /** Constructor from sparse_table */
  TableFactor(const DiscreteKeys& keys,
              const Eigen::SparseVector<double>& table);

  /** Constructor from doubles */
  TableFactor(const DiscreteKeys& keys, const std::vector<double>& table)
      : TableFactor(keys, Convert(keys, table)) {}

  /** Constructor from string */
  TableFactor(const DiscreteKeys& keys, const std::string& table)
      : TableFactor(keys, Convert(keys, table)) {}

  /// Single-key specialization
  template <class SOURCE>
  TableFactor(const DiscreteKey& key, SOURCE table)
      : TableFactor(DiscreteKeys{key}, table) {}

  /// Single-key specialization, with vector of doubles.
  TableFactor(const DiscreteKey& key, const std::vector<double>& row)
      : TableFactor(DiscreteKeys{key}, row) {}

  /// Constructor from DecisionTreeFactor
  TableFactor(const DiscreteKeys& keys, const DecisionTreeFactor& dtf);
  TableFactor(const DecisionTreeFactor& dtf);

  /// Constructor from DecisionTree<Key, double>/AlgebraicDecisionTree
  TableFactor(const DiscreteKeys& keys, const DecisionTree<Key, double>& dtree);

  /** Construct from a DiscreteConditional type */
  explicit TableFactor(const DiscreteConditional& c);

  /// @}
  /// @name Testable
  /// @{

  /// equality
  bool equals(const DiscreteFactor& other, double tol = 1e-9) const override;

  // print
  void print(
      const std::string& s = "TableFactor:\n",
      const KeyFormatter& formatter = DefaultKeyFormatter) const override;

  // /// @}
  // /// @name Standard Interface
  // /// @{

  /// Getter for the underlying sparse vector
  Eigen::SparseVector<double> sparseTable() const { return sparse_table_; }

  /// Evaluate probability distribution, is just look up in TableFactor.
  double evaluate(const Assignment<Key>& values) const override;

  /// Calculate error for DiscreteValues `x`, is -log(probability).
  double error(const DiscreteValues& values) const override;

  /// multiply with a scalar
  DiscreteFactor::shared_ptr operator*(double s) const override {
    return std::make_shared<TableFactor>(
        apply([s](const double& a) { return Ring::mul(a, s); }));
  }

  /// multiply two TableFactors
  TableFactor operator*(const TableFactor& f) const {
    return apply(f, Ring::mul);
  };

  /// multiply with DecisionTreeFactor
  DecisionTreeFactor operator*(const DecisionTreeFactor& f) const override;

  /**
   * @brief Multiply factors, DiscreteFactor::shared_ptr edition.
   *
   * This method accepts `DiscreteFactor::shared_ptr` and uses dynamic
   * dispatch and specializations to perform the most efficient
   * multiplication.
   *
   * While converting a DecisionTreeFactor to a TableFactor is efficient, the
   * reverse is not.
   * Hence we specialize the code to return a TableFactor always.
   *
   * @param f The factor to multiply with.
   * @return DiscreteFactor::shared_ptr
   */
  virtual DiscreteFactor::shared_ptr multiply(
      const DiscreteFactor::shared_ptr& f) const override;

  static double safe_div(const double& a, const double& b);

  /// divide by factor f (safely)
  TableFactor operator/(const TableFactor& f) const {
    return apply(f, safe_div);
  }

  /// divide by DiscreteFactor::shared_ptr f (safely)
  DiscreteFactor::shared_ptr operator/(
      const DiscreteFactor::shared_ptr& f) const override;

  /// Convert into a decisiontree
  DecisionTreeFactor toDecisionTreeFactor() const override;

  /// Return this sparse table representation.
  TableFactor toTableFactor() const override { return *this; }

  /// Create a TableFactor that is a subset of this TableFactor
  TableFactor choose(const DiscreteValues parentAssignments,
                     DiscreteKeys parent_keys) const;

  /// Create new factor by summing all values with the same separator values
  DiscreteFactor::shared_ptr sum(size_t nrFrontals) const override;

  /// Create new factor by summing all values with the same separator values
  DiscreteFactor::shared_ptr sum(const Ordering& keys) const override;

  /// Find the maximum value in the factor.
  double max() const override;

  /// Create new factor by maximizing over all values with the same separator.
  DiscreteFactor::shared_ptr max(size_t nrFrontals) const override;

  /// Create new factor by maximizing over all values with the same separator.
  DiscreteFactor::shared_ptr max(const Ordering& keys) const override;

  /// @}
  /// @name Advanced Interface
  /// @{

  /**
   * Apply unary operator `op(*this)` where `op` accepts the discrete value.
   * @param op a unary operator that operates on TableFactor
   */
  TableFactor apply(Unary op) const;
  /**
   * Apply unary operator `op(*this)` where `op` accepts the discrete assignment
   * and the value at that assignment.
   * @param op a unary operator that operates on TableFactor
   */
  TableFactor apply(UnaryAssignment op) const;

  /**
   * Apply binary operator (*this) "op" f
   * @param f the second argument for op
   * @param op a binary operator that operates on TableFactor
   */
  TableFactor apply(const TableFactor& f, Binary op) const;

  /// Create unique representation of union modes.
  uint64_t unionRep(const DiscreteKeys& keys, const DiscreteValues& assign,
                    const uint64_t idx) const;

  /// Create a hash map of input factor with assignment of contract modes as
  /// keys and vector of hashed assignment of free modes and value as values.
  std::unordered_map<uint64_t, AssignValList> createMap(
      const DiscreteKeys& contract, const DiscreteKeys& free) const;

  /// Create unique representation
  uint64_t uniqueRep(const DiscreteKeys& keys, const uint64_t idx) const;

  /// Create unique representation with DiscreteValues
  uint64_t uniqueRep(const DiscreteValues& assignments) const;

  /// Find DiscreteValues for corresponding index.
  DiscreteValues findAssignments(const uint64_t idx) const;

  /// Find value for corresponding DiscreteValues.
  double findValue(const DiscreteValues& values) const;

  /**
   * Combine frontal variables using binary operator "op"
   * @param nrFrontals nr. of frontal to combine variables in this factor
   * @param op a binary operator that operates on TableFactor
   * @return shared pointer to newly created TableFactor
   */
  shared_ptr combine(size_t nrFrontals, Binary op) const;

  /**
   * Combine frontal variables in an Ordering using binary operator "op"
   * @param nrFrontals nr. of frontal to combine variables in this factor
   * @param op a binary operator that operates on TableFactor
   * @return shared pointer to newly created TableFactor
   */
  shared_ptr combine(const Ordering& keys, Binary op) const;

  /// Enumerate all values into a map from values to double.
  std::vector<std::pair<DiscreteValues, double>> enumerate() const;

  /**
   * @brief Prune the decision tree of discrete variables.
   *
   * Pruning will set the values to be "pruned" to 0 indicating a 0
   * probability. An assignment is pruned if it is not in the top
   * `maxNrAssignments` values.
   *
   * A violation can occur if there are more
   * duplicate values than `maxNrAssignments`. A violation here is the need to
   * un-prune the decision tree (e.g. all assignment values are 1.0). We could
   * have another case where some subset of duplicates exist (e.g. for a tree
   * with 8 assignments we have 1, 1, 1, 1, 0.8, 0.7, 0.6, 0.5), but this is
   * not a violation since the for `maxNrAssignments=5` the top values are (1,
   * 0.8).
   *
   * @param maxNrAssignments The maximum number of assignments to keep.
   * @return TableFactor
   */
  TableFactor prune(size_t maxNrAssignments) const;

  /**
   * Get the number of non-zero values contained in this factor.
   * It could be much smaller than `prod_{key}(cardinality(key))`.
   */
  uint64_t nrValues() const override { return sparse_table_.nonZeros(); }

  /// Restrict the factor to the given assignment.
  DiscreteFactor::shared_ptr restrict(
      const DiscreteValues& assignment) const override;

  /// @}
  /// @name Wrapper support
  /// @{

  /**
   * @brief Render as markdown table
   *
   * @param keyFormatter GTSAM-style Key formatter.
   * @param names optional, category names corresponding to choices.
   * @return std::string a markdown string.
   */
  std::string markdown(const KeyFormatter& keyFormatter = DefaultKeyFormatter,
                       const Names& names = {}) const override;

  /**
   * @brief Render as html table
   *
   * @param keyFormatter GTSAM-style Key formatter.
   * @param names optional, category names corresponding to choices.
   * @return std::string a html string.
   */
  std::string html(const KeyFormatter& keyFormatter = DefaultKeyFormatter,
                   const Names& names = {}) const override;

  /// @}
  /// @name HybridValues methods.
  /// @{

  /**
   * Calculate error for HybridValues `x`, is -log(probability)
   * Simply dispatches to DiscreteValues version.
   */
  double error(const HybridValues& values) const override;

  /// @}

#ifdef GTSAM_ALLOW_DEPRECATED_SINCE_V43
  /// @name Deprecated scope helpers
  /// @{

  /// @deprecated Sort both discreteKeys() results, then use
  /// std::set_intersection instead.
  DiscreteKeys contractDkeys(const TableFactor& factor) const;

  /// @deprecated Sort both discreteKeys() results, then use std::set_difference
  /// instead.
  DiscreteKeys freeDkeys(const TableFactor& factor) const;

  /// @deprecated Sort both discreteKeys() results, then use std::set_union
  /// instead.
  DiscreteKeys unionDkeys(const TableFactor& factor) const;

  /// @}
#endif

 private:
#if GTSAM_ENABLE_BOOST_SERIALIZATION
  /** Serialization function */
  friend class boost::serialization::access;
  template <class ARCHIVE>
  void serialize(ARCHIVE& ar, const unsigned int /*version*/) {
    ar& BOOST_SERIALIZATION_BASE_OBJECT_NVP(Base);
    ar& BOOST_SERIALIZATION_NVP(sparse_table_);
    ar& BOOST_SERIALIZATION_NVP(denominators_);
    ar& BOOST_SERIALIZATION_NVP(sorted_dkeys_);
  }
#endif
};

// traits
template <>
struct traits<TableFactor> : public Testable<TableFactor> {};
}  // namespace gtsam
