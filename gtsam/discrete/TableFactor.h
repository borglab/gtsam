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

#include <gtsam/discrete/DiscreteFactor.h>
#include <gtsam/discrete/DiscreteKey.h>
#include <gtsam/inference/Ordering.h>

#include <Eigen/Sparse>
#include <algorithm>
#include <map>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

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
   * @brief Uses lazy cartesian product to find nth entry in the cartesian
   * product of arrays in O(1)
   * Example)
   *   v0 | v1 | val
   *    0 |  0 |  10
   *    0 |  1 |  21
   *    1 |  0 |  32
   *    1 |  1 |  43
   *   keyValueForIndex(v1, 2) = 0
   * @param target_key nth entry's key to find out its assigned value
   * @param index nth entry in the sparse vector
   * @return TableFactor
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

  /// Convert probability table given as doubles to SparseVector.
  /// Example) {0, 1, 1, 0, 0, 1, 0} -> values: {1, 1, 1}, indices: {1, 2, 5}
  static Eigen::SparseVector<double> Convert(const std::vector<double>& table);

  /// Convert probability table given as string to SparseVector.
  static Eigen::SparseVector<double> Convert(const std::string& table);

 public:
  // typedefs needed to play nice with gtsam
  typedef TableFactor This;
  typedef DiscreteFactor Base;  ///< Typedef to base class
  typedef std::shared_ptr<TableFactor> shared_ptr;
  typedef Eigen::SparseVector<double>::InnerIterator SparseIt;
  typedef std::vector<std::pair<DiscreteValues, double>> AssignValList;
  using Binary = std::function<double(const double, const double)>;

 public:
  /** The Real ring with addition and multiplication */
  struct Ring {
    static inline double zero() { return 0.0; }
    static inline double one() { return 1.0; }
    static inline double add(const double& a, const double& b) { return a + b; }
    static inline double max(const double& a, const double& b) {
      return std::max(a, b);
    }
    static inline double mul(const double& a, const double& b) { return a * b; }
    static inline double div(const double& a, const double& b) {
      return (a == 0 || b == 0) ? 0 : (a / b);
    }
    static inline double id(const double& x) { return x; }
  };

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
      : TableFactor(keys, Convert(table)) {}

  /** Constructor from string */
  TableFactor(const DiscreteKeys& keys, const std::string& table)
      : TableFactor(keys, Convert(table)) {}

  /// Single-key specialization
  template <class SOURCE>
  TableFactor(const DiscreteKey& key, SOURCE table)
      : TableFactor(DiscreteKeys{key}, table) {}

  /// Single-key specialization, with vector of doubles.
  TableFactor(const DiscreteKey& key, const std::vector<double>& row)
      : TableFactor(DiscreteKeys{key}, row) {}

  /// Constructor from DecisionTreeFactor
  TableFactor(const DiscreteKeys& keys, const DecisionTreeFactor& dtf);

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

  /// Calculate probability for given values `x`,
  /// is just look up in TableFactor.
  double evaluate(const DiscreteValues& values) const {
    return operator()(values);
  }

  /// Evaluate probability distribution, sugar.
  double operator()(const DiscreteValues& values) const override;

  /// Calculate error for DiscreteValues `x`, is -log(probability).
  double error(const DiscreteValues& values) const;

  /// multiply two TableFactors
  TableFactor operator*(const TableFactor& f) const {
    return apply(f, Ring::mul);
  };

  /// multiply with DecisionTreeFactor
  DecisionTreeFactor operator*(const DecisionTreeFactor& f) const override;

  static double safe_div(const double& a, const double& b);

  /// divide by factor f (safely)
  TableFactor operator/(const TableFactor& f) const {
    return apply(f, safe_div);
  }

  /// Convert into a decisiontree
  DecisionTreeFactor toDecisionTreeFactor() const override;

  /// Create a TableFactor that is a subset of this TableFactor
  TableFactor choose(const DiscreteValues assignments,
                     DiscreteKeys parent_keys) const;

  /// Create new factor by summing all values with the same separator values
  shared_ptr sum(size_t nrFrontals) const {
    return combine(nrFrontals, Ring::add);
  }

  /// Create new factor by summing all values with the same separator values
  shared_ptr sum(const Ordering& keys) const {
    return combine(keys, Ring::add);
  }

  /// Create new factor by maximizing over all values with the same separator.
  shared_ptr max(size_t nrFrontals) const {
    return combine(nrFrontals, Ring::max);
  }

  /// Create new factor by maximizing over all values with the same separator.
  shared_ptr max(const Ordering& keys) const {
    return combine(keys, Ring::max);
  }

  /// @}
  /// @name Advanced Interface
  /// @{

  /**
   * Apply binary operator (*this) "op" f
   * @param f the second argument for op
   * @param op a binary operator that operates on TableFactor
   */
  TableFactor apply(const TableFactor& f, Binary op) const;

  /// Return keys in contract mode.
  DiscreteKeys contractDkeys(const TableFactor& f) const;

  /// Return keys in free mode.
  DiscreteKeys freeDkeys(const TableFactor& f) const;

  /// Return union of DiscreteKeys in two factors.
  DiscreteKeys unionDkeys(const TableFactor& f) const;

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
};

// traits
template <>
struct traits<TableFactor> : public Testable<TableFactor> {};
}  // namespace gtsam
