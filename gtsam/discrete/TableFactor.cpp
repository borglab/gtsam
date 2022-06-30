/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file TableFactor.cpp
 * @brief disrete factor
 * @date May 27, 2022
 * @author Yoonwoo Kim
 */

#include <gtsam/discrete/TableFactor.h>

#include <boost/format.hpp>
#include <boost/make_shared.hpp>
#include <utility>

using namespace std;
namespace gtsam {

/* ************************************************************************ */
TableFactor::TableFactor(){};

/* ************************************************************************ */
TableFactor::TableFactor(const DiscreteKeys& keys,
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

/* ************************************************************************ */
Eigen::SparseVector<double> TableFactor::Convert(
    const std::vector<double>& table) {
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

/* ************************************************************************ */
Eigen::SparseVector<double> TableFactor::Convert(const std::string& table) {
  // Convert string to doubles
  std::vector<double> ys;
  std::istringstream iss(table);
  std::copy(std::istream_iterator<double>(iss), std::istream_iterator<double>(),
            std::back_inserter(ys));
  return Convert(ys);
}

/* ************************************************************************ */
bool TableFactor::equals(const DiscreteFactor& other, double tol) const {
  if (!dynamic_cast<const TableFactor*>(&other)) {
    return false;
  } else {
    const auto& f(static_cast<const TableFactor&>(other));
    return (sparse_table_ - f.sparse_table_).sum() == 0;
  }
}

/* ************************************************************************ */
void TableFactor::print(const string& s, const KeyFormatter& formatter) const {
  cout << s;
  cout << " f[";
  for (auto&& key : keys())
    cout << boost::format(" (%1%,%2%),") % formatter(key) % cardinality(key);
  cout << " ]" << endl;
}

/* ************************************************************************ */
double TableFactor::operator()(const DiscreteValues& values) const {
  // Find index of value in O(N) where N is number of keys
  size_t idx = findIndex(values);
  return sparse_table_.coeff(idx);
}

/* ************************************************************************ */
DecisionTreeFactor TableFactor::operator*(const DecisionTreeFactor& f) const {
  return toDecisionTreeFactor() * f;
}

/* ************************************************************************ */
TableFactor TableFactor::operator*(const TableFactor& f) const {
  DiscreteKeys keys = unionKeys(f);
  size_t cardinality = 1;
  // Create new TableFactor
  for (DiscreteKey key : keys) cardinality *= key.second;
  Eigen::SparseVector<double> new_sparse_table_(cardinality);
  new_sparse_table_.reserve(sparse_table_.nonZeros() *
                            (cardinality / sparse_table_.size()));
  TableFactor multiplied_f(keys, new_sparse_table_);
  for (sparse_it it(f.sparse_table_); it; ++it) {
    DiscreteValues assignment_f;
    for (Key key : f.keys_) {
      assignment_f[key] = f.keyValueForIndex(key, it.index());
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

/* ************************************************************************ */
DecisionTreeFactor TableFactor::toDecisionTreeFactor() const {
  DiscreteKeys dkeys = discreteKeys();
  std::vector<double> table;
  for (size_t i = 0; i < sparse_table_.size(); i++) {
    table.push_back(sparse_table_.coeff(i));
  }
  DecisionTreeFactor f(dkeys, table);
  return f;
}

/* ************************************************************************ */
TableFactor TableFactor::sum(size_t nrFrontals) const {
  if (nrFrontals > size())
    throw invalid_argument(
        (boost::format("TableFactor::max: invalid number of frontal "
                       "keys %d, nr.keys=%d") %
         nrFrontals % size())
            .str());
  // Find all remaining keys
  DiscreteKeys dkeys;
  size_t cardinality = 1;
  for (size_t i = 0; i < keys_.size(); i++) {
    if (i >= nrFrontals) {
      dkeys.push_back(discreteKey(keys_[i]));
      cardinality *= cardinalities_.at(keys_[i]);
    }
  }
  // Create a new TableFactor with remaining keys
  Eigen::SparseVector<double> new_sparse_table_(cardinality);
  if (sparse_table_.nonZeros() < cardinality)
    new_sparse_table_.reserve(sparse_table_.nonZeros());
  TableFactor eliminated_f(dkeys, new_sparse_table_);
  // Populate the new TableFactor
  for (sparse_it it(sparse_table_); it; ++it) {
    DiscreteValues assignments;
    for (DiscreteKey dkey : dkeys) {
      assignments[dkey.first] = keyValueForIndex(dkey.first, it.index());
    }
    size_t index = eliminated_f.findIndex(assignments);
    // store the summed value
    eliminated_f.sparse_table_.coeffRef(index) += it.value();
  }
  return eliminated_f;
}

/* ************************************************************************ */
TableFactor TableFactor::sum(const Ordering& frontalKeys) const {
  if (frontalKeys.size() > size())
    throw invalid_argument(
        (boost::format("TableFactor::max: invalid number of frontal "
                       "keys %d, nr.keys=%d") %
         frontalKeys.size() % size())
            .str());
  // Find all remaining keys
  DiscreteKeys dkeys;
  size_t cardinality = 1;
  for (Key k : keys_) {
    if (std::find(frontalKeys.begin(), frontalKeys.end(), k) ==
        frontalKeys.end()) {
      dkeys.push_back(discreteKey(k));
      cardinality *= cardinalities_.at(k);
    }
  }
  // Create a new TableFactor with remaining keys
  Eigen::SparseVector<double> new_sparse_table_(cardinality);
  if (sparse_table_.nonZeros() < cardinality)
    new_sparse_table_.reserve(sparse_table_.nonZeros());
  TableFactor eliminated_f(dkeys, new_sparse_table_);
  // Populate the new TableFactor
  for (sparse_it it(sparse_table_); it; ++it) {
    DiscreteValues assignments;
    for (DiscreteKey dkey : dkeys) {
      assignments[dkey.first] = keyValueForIndex(dkey.first, it.index());
    }
    size_t index = eliminated_f.findIndex(assignments);
    // store the summed value
    eliminated_f.sparse_table_.coeffRef(index) += it.value();
  }
  return eliminated_f;
}


/* ************************************************************************ */
TableFactor TableFactor::max(size_t nrFrontals) const {
  if (nrFrontals > size())
    throw invalid_argument(
        (boost::format("TableFactor::max: invalid number of frontal "
                       "keys %d, nr.keys=%d") %
         nrFrontals % size())
            .str());
  // Find all remaining keys
  DiscreteKeys dkeys;
  size_t cardinality = 1;
  for (size_t i = 0; i < keys_.size(); i++) {
    if (i >= nrFrontals) {
      dkeys.push_back(discreteKey(keys_[i]));
      cardinality *= cardinalities_.at(keys_[i]);
    }
  }
  // Create a new TableFactor with remaining keys
  Eigen::SparseVector<double> new_sparse_table_(cardinality);
  if (sparse_table_.nonZeros() < cardinality)
    new_sparse_table_.reserve(sparse_table_.nonZeros());
  TableFactor eliminated_f(dkeys, new_sparse_table_);
  // Populate the new TableFactor
  for (sparse_it it(sparse_table_); it; ++it) {
    DiscreteValues assignments;
    for (DiscreteKey dkey : dkeys) {
      assignments[dkey.first] = keyValueForIndex(dkey.first, it.index());
    }
    size_t index = eliminated_f.findIndex(assignments);
    // swap values to store the maximum
    if (it.value() > eliminated_f.sparse_table_.coeff(index)) {
      eliminated_f.sparse_table_.coeffRef(index) = it.value();
    }
  }
  return eliminated_f;
}

/* ************************************************************************ */
TableFactor TableFactor::max(const Ordering& frontalKeys) const {
  if (frontalKeys.size() > size())
    throw invalid_argument(
        (boost::format("TableFactor::max: invalid number of frontal "
                       "keys %d, nr.keys=%d") %
         frontalKeys.size() % size())
            .str());
  // Find all remaining keys
  DiscreteKeys dkeys;
  size_t cardinality = 1;
  for (Key k : keys_) {
    if (std::find(frontalKeys.begin(), frontalKeys.end(), k) ==
        frontalKeys.end()) {
      dkeys.push_back(discreteKey(k));
      cardinality *= cardinalities_.at(k);
    }
  }
  // Create a new TableFactor with remaining keys
  Eigen::SparseVector<double> new_sparse_table_(cardinality);
  if (sparse_table_.nonZeros() < cardinality)
    new_sparse_table_.reserve(sparse_table_.nonZeros());
  TableFactor eliminated_f(dkeys, new_sparse_table_);
  // Populate the new TableFactor
  for (sparse_it it(sparse_table_); it; ++it) {
    DiscreteValues assignments;
    for (DiscreteKey dkey : dkeys) {
      assignments[dkey.first] = keyValueForIndex(dkey.first, it.index());
    }
    size_t index = eliminated_f.findIndex(assignments);
    // swap values to store the maximum
    if (it.value() > eliminated_f.sparse_table_.coeff(index)) {
      eliminated_f.sparse_table_.coeffRef(index) = it.value();
    }
  }
  return eliminated_f;
}

/* ************************************************************************ */
TableFactor TableFactor::fromDecisionTreeFactor(
    const DecisionTreeFactor& f) const {
  auto enumerated = f.enumerate();
  std::vector<double> table;
  for (size_t i = 0; i < enumerated.size(); i++) {
    table.push_back(enumerated[i].second);
  }
  DiscreteKeys tree_dkeys = f.discreteKeys();
  TableFactor table_f(tree_dkeys, table);
  return table_f;
}

/* ************************************************************************ */
size_t TableFactor::keyValueForIndex(Key target_key, size_t index) const {
  // TODO: give a link to lazy cartesian product algorithm
  return (index / denominator_.at(target_key)) % cardinalities_.at(target_key);
}

/* ************************************************************************ */
size_t TableFactor::findIndex(const DiscreteValues& assignment) const {
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

/* ************************************************************************ */
DiscreteValues TableFactor::maxAssignment() const {
  DiscreteValues assignments;
  Eigen::Index nnz = sparse_table_.nonZeros();
  Eigen::Index maxRowIdx;
  Eigen::VectorXd::Map(sparse_table_.valuePtr(), nnz).maxCoeff(&maxRowIdx);
  for (Key key : keys_) {
    assignments[key] =
        keyValueForIndex(key, sparse_table_.innerIndexPtr()[maxRowIdx]);
  }
  return assignments;
}

/* ************************************************************************ */
DiscreteKeys TableFactor::unionKeys(const TableFactor& f) const {
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

/* ************************************************************************ */
std::vector<DiscreteValues> TableFactor::project(
    const DiscreteValues& assignment_f) const {
  // project assignment_f onto union of assignments
  // ex) project (v0, v2) onto the union (v0, v1, v2)
  std::vector<DiscreteValues> projected_assignments;
  for (sparse_it it(sparse_table_); it; ++it) {
    DiscreteValues union_assignments;
    bool flag = true;
    // check if there is an overlapping key
    for (auto a_it = assignment_f.begin(); a_it != assignment_f.end() && flag;
         a_it++) {
      union_assignments.insert(*a_it);
      // if there is an overlapping key check the assigned value for that key
      if (find(a_it->first) != keys_.end()) {
        flag = keyValueForIndex(a_it->first, it.index()) == a_it->second;
      }
    }
    // If there is no overlapping key, or if the overlapped key's assigned
    // value is same with TableFactor's value project the assignment.
    if (flag) {
      for (Key key : keys_) {
        union_assignments.insert(
            std::pair<Key, size_t>(key, keyValueForIndex(key, it.index())));
      }
      projected_assignments.push_back(union_assignments);
    }
  }
  return projected_assignments;
}

/* ************************************************************************ */
std::vector<std::pair<DiscreteValues, double>> TableFactor::enumerate() const {
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

/* ************************************************************************ */
DiscreteKeys TableFactor::discreteKeys() const {
  DiscreteKeys result;
  for (Key key : keys_) {
    DiscreteKey dkey(key, cardinality(key));
    if (std::find(result.begin(), result.end(), dkey) == result.end()) {
      result.push_back(dkey);
    }
  }
  return result;
}

// Print out header.
/* ************************************************************************ */
string TableFactor::markdown(const KeyFormatter& keyFormatter,
                             const Names& names) const {
  stringstream ss;

  // Print out header.
  ss << "|";
  for (auto& key : keys()) {
    ss << keyFormatter(key) << "|";
  }
  ss << "value|\n";

  // Print out separator with alignment hints.
  ss << "|";
  for (size_t j = 0; j < size(); j++) ss << ":-:|";
  ss << ":-:|\n";

  // Print out all rows.
  auto rows = enumerate();
  for (const auto& kv : rows) {
    ss << "|";
    auto assignment = kv.first;
    for (auto& key : keys()) {
      size_t index = assignment.at(key);
      ss << DiscreteValues::Translate(names, key, index) << "|";
    }
    ss << kv.second << "|\n";
  }
  return ss.str();
}

/* ************************************************************************ */
string TableFactor::html(const KeyFormatter& keyFormatter,
                         const Names& names) const {
  stringstream ss;

  // Print out preamble.
  ss << "<div>\n<table class='TableFactor'>\n  <thead>\n";

  // Print out header row.
  ss << "    <tr>";
  for (auto& key : keys()) {
    ss << "<th>" << keyFormatter(key) << "</th>";
  }
  ss << "<th>value</th></tr>\n";

  // Finish header and start body.
  ss << "  </thead>\n  <tbody>\n";

  // Print out all rows.
  auto rows = enumerate();
  for (const auto& kv : rows) {
    ss << "    <tr>";
    auto assignment = kv.first;
    for (auto& key : keys()) {
      size_t index = assignment.at(key);
      ss << "<th>" << DiscreteValues::Translate(names, key, index) << "</th>";
    }
    ss << "<td>" << kv.second << "</td>";  // value
    ss << "</tr>\n";
  }
  ss << "  </tbody>\n</table>\n</div>";
  return ss.str();
}

/* ************************************************************************ */
}  // namespace gtsam
