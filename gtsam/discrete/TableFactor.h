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

 public:
  // typedefs needed to play nice with gtsam
  typedef TableFactor This;
  typedef DiscreteFactor Base;  ///< Typedef to base class
  typedef boost::shared_ptr<TableFactor> shared_ptr;

  /// @}
  /// @name Constructors
  /// @{

  /** Default constructo for I/O */
  TableFactor(){};

  /** Constructor from Eigen::SparseVector */
  TableFactor(const DiscreteKeys& keys,
              const Eigen::SparseVector<double>& table)
      : DiscreteFactor(keys.indices()), sparse_table_(table.size()) {
    sparse_table_ = table;
    size_t denom = table.size();
    for (const DiscreteKey& dkey : keys) {
      cardinalities_.insert(dkey);
      denom /= dkey.second;
      denominator_.insert(std::pair<Key, size_t>(dkey.first, denom));
    }
  }

  /** Convert table to SparseVector */
  static Eigen::SparseVector<double> Convert(const std::vector<double>& table) {
    Eigen::SparseVector<double> sparse_table(table.size());
    // counting number of nonzero elements in table and reserving the space
    const size_t nnz = std::count_if(table.begin(), table.end(),
                                     [](size_t i) { return i != 0; });
    sparse_table.reserve(nnz);
    for (size_t i = 0; i < table.size(); i++) {
      if (table[i] != 0) sparse_table.insert(i) = table[i];
    }
    return sparse_table;
  }

  /** Constructor from doubles */
  TableFactor(const DiscreteKeys& keys, const std::vector<double>& table)
      : TableFactor(keys, Convert(table)) {}

  /** Convert table to SparseVector */
  static Eigen::SparseVector<double> Convert(const std::string& table) {
    // Convert string to doubles
    std::vector<double> ys;
    std::istringstream iss(table);
    std::copy(std::istream_iterator<double>(iss),
              std::istream_iterator<double>(), std::back_inserter(ys));
    return Convert(ys);
  }

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
  bool equals(const DiscreteFactor& other, double tol = 1e-9) const override {
    if (!dynamic_cast<const TableFactor*>(&other)) {
      return false;
    } else {
      const auto& f(static_cast<const TableFactor&>(other));
      return sparse_table_ == f.sparse_table_;
    }
  }

  /// print
  void print(
      const std::string& s = "TableFactor\n",
      const KeyFormatter& formatter = DefaultKeyFormatter) const override {
    Base::print(s, formatter);
  }

  /// @}
  /// @name Standard Interface
  /// @{

  /// Find value for given assignment of values to variables
  double operator()(const DiscreteValues& values) const override {
    // Find index of value in O(N) where N is number of keys
    size_t idx = findIndex(values);
    return sparse_table_.coeff(idx);
  }

  /// Multiply with DecisionTreeFactor
  DecisionTreeFactor operator*(const DecisionTreeFactor& f) const override {
    return toDecisionTreeFactor() * f;
  }

  /// Multiply with TableFactor
  TableFactor operator*(const TableFactor& f) const {
    DiscreteKeys keys = unionKeys(f);
    size_t cardinality = 1;
    // Create new TableFactor
    for (DiscreteKey key : keys) cardinality *= key.second;
    Eigen::SparseVector<double> new_sparse_table_(cardinality);
    new_sparse_table_.reserve(sparse_table_.nonZeros() *
                              (cardinality / sparse_table_.size()));
    TableFactor multiplied_f(keys, new_sparse_table_);
    for (Eigen::SparseVector<double>::InnerIterator it(f.sparse_table_); it;
         ++it) {
      DiscreteValues assignment_f;
      for (Key key : f.keys_) {
        assignment_f[key] = f.lazy_cp(key, it.index());
      }
      std::vector<DiscreteValues> projections = project(assignment_f);
      for (DiscreteValues projection : projections) {
        size_t m_index = multiplied_f.findIndex(projection);
        multiplied_f.sparse_table_.insert(m_index) =
            it.value() * operator()(projection);
      }
    }
    return multiplied_f;
  }

  /// Get the cardinality of key j
  size_t cardinality(Key j) const { return cardinalities_.at(j); }

  /// Generate DecisionTreeFactor from TableFactor
  DecisionTreeFactor toDecisionTreeFactor() const override {
    DiscreteKeys dkeys = discreteKeys();
    std::vector<double> table;
    for (size_t i = 0; i < sparse_table_.size(); i++) {
      table.push_back(sparse_table_.coeff(i));
    }
    DecisionTreeFactor f(dkeys, table);
    return f;
  }

  /// Create new factor by summing all values with the same separator values
  shared_ptr sum(size_t nrFrontals) const {
    // Create map to keep track of summed values
    std::map<size_t, double> cs;
    size_t cardinality = cardinalities_.at(nrFrontals);
    for (size_t i = 0; i < cardinality; i++) cs[i] = 0;
    DiscreteKey target_key = discreteKey(nrFrontals);
    for (Eigen::SparseVector<double>::InnerIterator it(sparse_table_); it;
         ++it) {
      cs[lazy_cp(target_key.first, it.index())] += it.value();
    }
    std::vector<double> result;
    for (std::map<size_t, double>::iterator it = cs.begin(); it != cs.end();
         ++it) {
      result.push_back(it->second);
    }
    return boost::make_shared<TableFactor>(target_key, result);
  }

  /// Create new factor by maximizing over all values with the same separator
  shared_ptr max(size_t nrFrontals) const {
    // Create map to keep track of summed values
    std::map<size_t, double> cs;
    size_t cardinality = cardinalities_.at(nrFrontals);
    for (size_t i = 0; i < cardinality; i++) cs[i] = 0;
    DiscreteKey target_key = discreteKey(nrFrontals);
    for (Eigen::SparseVector<double>::InnerIterator it(sparse_table_); it;
         ++it) {
      double val = lazy_cp(target_key.first, it.index());
      if (cs[val] < it.value()) cs[val] = it.value();
    }
    std::vector<double> result;
    for (std::map<size_t, double>::iterator it = cs.begin(); it != cs.end();
         ++it) {
      result.push_back(it->second);
    }
    return boost::make_shared<TableFactor>(target_key, result);
  }

  /// @}
  /// @name Advanced Interface
  /// @{

  /// Generate TableFactor from DecisionTreeFactor
  TableFactor fromDecisionTreeFactor(const DecisionTreeFactor& f) {
    DiscreteKeys tree_dkeys = f.discreteKeys();
    std::vector<std::pair<DiscreteValues, double>> enumerated = f.enumerate();
    std::vector<double> table;
    for (size_t i = 0; i < enumerated.size(); i++) {
      table.push_back(enumerated[i].second);
    }
    TableFactor table_f(tree_dkeys, table);
    return table_f;
  }

  // Finds value for the key at index
  size_t lazy_cp(Key target_key, size_t index) const {
    return (index / denominator_.at(target_key)) %
           cardinalities_.at(target_key);
  }

  // Find index for given assignment of values
  size_t findIndex(const DiscreteValues& assignment) const {
    // a b c d => D * (C * (B * (a) + b) + c) + d
    // index = a
    // for (i in bcd) :
    //     index = index * I + i
    size_t index = 0;
    for (auto&& key : keys_) {
      if (index) {
        index *= cardinalities_.at(key);
      }
      index += assignment.at(key);
    }
    return index;
  }

  // Create union of keys
  DiscreteKeys unionKeys(const TableFactor& f) const {
    // new cardinalities
    std::map<Key, size_t> cs;
    // make unique key-cardinality map
    for (Key j : keys()) cs[j] = cardinality(j);
    for (Key j : f.keys()) cs[j] = f.cardinality(j);
    // Convert map into keys
    DiscreteKeys keys;
    for (const std::pair<const Key, size_t>& key : cs) {
      keys.emplace_back(key);
    }
    // Sort
    std::sort(keys.begin(), keys.end());
    return keys;
  }

  // Project assignment that agrees with the given assignment
  std::vector<DiscreteValues> project(
      const DiscreteValues& assignment_f) const {
    std::vector<DiscreteValues> projected_assignments;
    for (Eigen::SparseVector<double>::InnerIterator it(sparse_table_); it;
         ++it) {
      DiscreteValues union_assignments;
      bool flag = true;
      // check if there is an overlapping key
      for (auto a_it = assignment_f.begin(); a_it != assignment_f.end() && flag;
           a_it++) {
        union_assignments.insert(*a_it);
        // if there is an overlapping key check the assigned value for that key
        if (find(a_it->first) != keys_.end()) {
          flag = lazy_cp(a_it->first, it.index()) == a_it->second;
        }
      }
      // If there is no overlapping key, or if the overlapped key's assigned
      // value is same with TableFactor's value project the assignment.
      if (flag) {
        for (Key key : keys_) {
          union_assignments.insert(
              std::pair<Key, size_t>(key, lazy_cp(key, it.index())));
        }
        projected_assignments.push_back(union_assignments);
      }
    }
    return projected_assignments;
  }

  DiscreteKeys discreteKeys() const {
    DiscreteKeys result;
    for (Key key : keys_) {
      DiscreteKey dkey(key, cardinality(key));
      if (std::find(result.begin(), result.end(), dkey) == result.end()) {
        result.push_back(dkey);
      }
    }
    return result;
  }

  /// Enumerate all values into a map from values to double.
  std::vector<std::pair<DiscreteValues, double>> enumerate() const {
    // Get all possible assignments
    std::vector<std::pair<Key, size_t>> pairs = discreteKeys();
    // Reverse to make cartesian product output a more natural ordering.
    std::vector<std::pair<Key, size_t>> rpairs(pairs.rbegin(), pairs.rend());
    const auto assignments = DiscreteValues::CartesianProduct(rpairs);

    // Construct unordered_map with values
    std::vector<std::pair<DiscreteValues, double>> result;
    for (const auto& assignment : assignments) {
      result.emplace_back(assignment, operator()(assignment));
    }
    return result;
  }

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
                       const Names& names = {}) const override {
    return "";
  }

  /**
   * @brief Render as html table
   *
   * @param keyFormatter GTSAM-style Key formatter.
   * @param names optional, category names corresponding to choices.
   * @return std::string a html string.
   */
  std::string html(const KeyFormatter& keyFormatter = DefaultKeyFormatter,
                   const Names& names = {}) const override {
    return "";
  }

  /// @}
};

// traits
template <>
struct traits<TableFactor> : public Testable<TableFactor> {};

}  // namespace gtsam