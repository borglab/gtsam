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

class GTSAM_EXPORT TableFactor : public DiscreteFactor {
 private:
  Eigen::SparseVector<double> sparse_table_;
  std::map<Key, size_t> cardinalities_;
  std::map<Key, size_t> denominator_;

  DiscreteKey discreteKey(size_t i) const {
    Key j = keys_[i];
    return DiscreteKey(j, cardinalities_.at(j));
  }

  /** Convert table to SparseVector */
  static Eigen::SparseVector<double> Convert(const std::vector<double>& table);

  /** Convert table to SparseVector */
  static Eigen::SparseVector<double> Convert(const std::string& table);

 public:
  // typedefs needed to play nice with gtsam
  typedef TableFactor This;
  typedef DiscreteFactor Base;  ///< Typedef to base class
  typedef boost::shared_ptr<TableFactor> shared_ptr;

  /// @name Constructors
  /// @{

  /** Default constructo for I/O */
  TableFactor();

  /** Constructor from Eigen::SparseVector */
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
  TableFactor operator*(const TableFactor& f) const;

  /// Get the cardinality of key j
  size_t cardinality(Key j) const { return cardinalities_.at(j); }

  /// Generate DecisionTreeFactor from TableFactor
  DecisionTreeFactor toDecisionTreeFactor() const override;

  /// Create new factor by summing all values with the same separator values
  shared_ptr sum(size_t nrFrontals) const;

  /// Create new TableFactor where the input key is eliminated
  TableFactor eliminate(const Key key) const;

  /// Create new factor by maximizing over all values with the same separator
  TableFactor max(size_t nrFrontals) const;

  /// Create new factor by maximizing over all values with the same separator
  TableFactor max(const Ordering& frontalKeys) const;

  /// @}
  /// @name Advanced Interface
  /// @{

  /// Generate TableFactor from DecisionTreeFactor
  TableFactor fromDecisionTreeFactor(const DecisionTreeFactor& f) const;

  /// Finds value for the key at index
  size_t lazy_cp(Key target_key, size_t index) const;

  /// Find assignments with maximum value
  DiscreteValues maxAssignment() const;

  /// Find index for given assignment of values
  size_t findIndex(const DiscreteValues& assignment) const;

  /// Create union of keys
  DiscreteKeys unionKeys(const TableFactor& f) const;

  /// Project assignment that agrees with the given assignment
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
