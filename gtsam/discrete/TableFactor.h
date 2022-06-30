/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file TableFactor.h
 * @date May 12, 2022
 * @author Yoonwoo Kim
 */

#pragma once

#include <gtsam/base/Testable.h>
#include <gtsam/discrete/DecisionTreeFactor.h>
#include <gtsam/discrete/DiscreteFactor.h>
#include <gtsam/discrete/DiscreteKey.h>
#include <gtsam/inference/Ordering.h>

#include <Eigen/Sparse>
#include <algorithm>
#include <boost/assign/std/map.hpp>
#include <boost/make_shared.hpp>
#include <boost/shared_ptr.hpp>
#include <cmath>  // for floor
#include <map>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace gtsam {

/**
 * A discrete probabilistic factor optimized for sparsity.
 */
class GTSAM_EXPORT TableFactor : public DiscreteFactor {
 private:
  Eigen::SparseVector<double> sparse_table_;
  std::map<Key, size_t> cardinalities_;
  std::map<Key, size_t> denominator_;

  DiscreteKey discreteKey(size_t i) const {
    Key j = keys_[i];
    return DiscreteKey(j, cardinalities_.at(j));
  }

  /// Convert probability table given as doubles to SparseVector.
  static Eigen::SparseVector<double> Convert(const std::vector<double>& table);

  /// Convert probability table given as string to SparseVector.
  static Eigen::SparseVector<double> Convert(const std::string& table);

  /// Finds value for the key at index
  // TODO: add some example doxygen block
  size_t keyValueForIndex(Key target_key, size_t index) const;

  /// Create union of keys
  DiscreteKeys unionKeys(const TableFactor& f) const;

 public:
  // typedefs needed to play nice with gtsam
  typedef TableFactor This;
  typedef DiscreteFactor Base;  ///< Typedef to base class
  typedef boost::shared_ptr<TableFactor> shared_ptr;
  typedef Eigen::SparseVector<double>::InnerIterator sparse_it;

  /// @name Constructors
  /// @{

  /// Default constructo for I/O.
  TableFactor();

  /// Constructor from Eigen::SparseVector.
  TableFactor(const DiscreteKeys& keys,
              const Eigen::SparseVector<double>& table);

  /// Constructor from doubles.
  TableFactor(const DiscreteKeys& keys, const std::vector<double>& table)
      : TableFactor(keys, Convert(table)) {}

  /// Constructor from string.
  TableFactor(const DiscreteKeys& keys, const std::string& table)
      : TableFactor(keys, Convert(table)) {}

  /// Constructor for single-key.
  template <class SOURCE>
  TableFactor(const DiscreteKey& key, SOURCE table)
      : TableFactor(DiscreteKeys{key}, table) {}

  /// Constructor for single-key, with vector of doubles.
  TableFactor(const DiscreteKey& key, const std::vector<double>& row)
      : TableFactor(DiscreteKeys{key}, row) {}

  /// @}
  /// @name Testable
  /// @{

  /// equals
  bool equals(const DiscreteFactor& other, double tol = 1e-9) const override;

  /// print
  void print(
      const std::string& s = "TableFactor\n",
      const KeyFormatter& formatter = DefaultKeyFormatter) const override;

  /// @}
  /// @name Standard Interface
  /// @{

  /// Find value for given assignment of values to variables
  double operator()(const DiscreteValues& values) const override;

  /// Multiply with DecisionTreeFactor
  DecisionTreeFactor operator*(const DecisionTreeFactor& f) const override;

  /// Multiply with TableFactor
  TableFactor operator*(const TableFactor& f) const override;

  /// Get the cardinality of key j
  size_t cardinality(Key j) const { return cardinalities_.at(j); }

  /// Generate DecisionTreeFactor from TableFactor
  DecisionTreeFactor toDecisionTreeFactor() const override;

  /// Create new factor by summing all values with the same separator values
  TableFactor sum(size_t nrFrontals) const;

  /// Create new factor by summing all values with the same separator values
  TableFactor sum(const Ordering& frontalKeys) const;

  /// Create new factor by maximizing over all values with the same separator
  TableFactor max(size_t nrFrontals) const;

  /// Create new factor by maximizing over all values with the same separator
  TableFactor max(const Ordering& frontalKeys) const;

  /// @}
  /// @name Advanced Interface
  /// @{

  /// Generate TableFactor from DecisionTreeFactor
  TableFactor fromDecisionTreeFactor(const DecisionTreeFactor& f) const;

  /// Find assignments with maximum value
  DiscreteValues maxAssignment() const;

  /// Find index for given assignment of values
  size_t findIndex(const DiscreteValues& assignment) const;

  /**
   * @brief Project assignment onto the union of assignments
   *
   * ex) if f1(v0 & v1, "1, 2, 3, 4"), and the given assignment, assignment_f
   * is (v0 = 0) the result of f1.project(assignment_f) is going to be
   * vector of DiscreteValues where [(v0 = 0, v1 = 0, (v0 = 0, v1 = 1))]
   *
   * @param assignment_f The maximum number of assignments to keep.
   * @return vector<DiscreteValues>
   */
  std::vector<DiscreteValues> project(const DiscreteValues& assignment_f) const;

  /// Return all the discrete keys associated with this factor.
  DiscreteKeys discreteKeys() const;

  /// Enumerate all values into a map from values to double.
  std::vector<std::pair<DiscreteValues, double>> enumerate() const;

  /// @}
  /// @name Wrapper support
  /// @{

  /// Translation table from values to strings.
  using Names = DiscreteValues::Names;

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
};

// traits
template <>
struct traits<TableFactor> : public Testable<TableFactor> {};

}  // namespace gtsam
