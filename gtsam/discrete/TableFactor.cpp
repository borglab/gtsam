/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file TableFactor.cpp
 * @brief discrete factor
 * @date May 4, 2023
 * @author Yoonwoo Kim, Varun Agrawal
 */

#include <gtsam/base/FastSet.h>
#include <gtsam/discrete/DecisionTreeFactor.h>
#include <gtsam/discrete/DiscreteConditional.h>
#include <gtsam/discrete/TableFactor.h>
#include <gtsam/hybrid/HybridValues.h>

#include <utility>

using namespace std;

namespace gtsam {

/* ************************************************************************ */
TableFactor::TableFactor() {}

/* ************************************************************************ */
TableFactor::TableFactor(const DiscreteKeys& dkeys,
                         const TableFactor& potentials)
    : DiscreteFactor(dkeys.indices(), dkeys.cardinalities()) {
  sparse_table_ = potentials.sparse_table_;
  denominators_ = potentials.denominators_;
  sorted_dkeys_ = discreteKeys();
  sort(sorted_dkeys_.begin(), sorted_dkeys_.end());
}

/* ************************************************************************ */
TableFactor::TableFactor(const DiscreteKeys& dkeys,
                         const Eigen::SparseVector<double>& table)
    : DiscreteFactor(dkeys.indices(), dkeys.cardinalities()),
      sparse_table_(table.size()) {
  sparse_table_ = table;
  double denom = table.size();
  for (const DiscreteKey& dkey : dkeys) {
    denom /= dkey.second;
    denominators_.insert(std::pair<Key, double>(dkey.first, denom));
  }
  sorted_dkeys_ = discreteKeys();
  sort(sorted_dkeys_.begin(), sorted_dkeys_.end());
}

/* ************************************************************************ */
TableFactor::TableFactor(const DiscreteConditional& c)
    : TableFactor(c.discreteKeys(), c.probabilities()) {}

/* ************************************************************************ */
Eigen::SparseVector<double> TableFactor::Convert(
    const std::vector<double>& table) {
  Eigen::SparseVector<double> sparse_table(table.size());
  // Count number of nonzero elements in table and reserving the space.
  const uint64_t nnz = std::count_if(table.begin(), table.end(),
                                     [](uint64_t i) { return i != 0; });
  sparse_table.reserve(nnz);
  for (uint64_t i = 0; i < table.size(); i++) {
    if (table[i] != 0) sparse_table.insert(i) = table[i];
  }
  sparse_table.pruned();
  sparse_table.data().squeeze();
  return sparse_table;
}

/* ************************************************************************ */
Eigen::SparseVector<double> TableFactor::Convert(const std::string& table) {
  // Convert string to doubles.
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
    return sparse_table_.isApprox(f.sparse_table_, tol);
  }
}

/* ************************************************************************ */
double TableFactor::operator()(const DiscreteValues& values) const {
  // a b c d => D * (C * (B * (a) + b) + c) + d
  uint64_t idx = 0, card = 1;
  for (auto it = sorted_dkeys_.rbegin(); it != sorted_dkeys_.rend(); ++it) {
    if (values.find(it->first) != values.end()) {
      idx += card * values.at(it->first);
    }
    card *= it->second;
  }
  return sparse_table_.coeff(idx);
}

/* ************************************************************************ */
double TableFactor::findValue(const DiscreteValues& values) const {
  // a b c d => D * (C * (B * (a) + b) + c) + d
  uint64_t idx = 0, card = 1;
  for (auto it = keys_.rbegin(); it != keys_.rend(); ++it) {
    if (values.find(*it) != values.end()) {
      idx += card * values.at(*it);
    }
    card *= cardinality(*it);
  }
  return sparse_table_.coeff(idx);
}

/* ************************************************************************ */
double TableFactor::error(const DiscreteValues& values) const {
  return -log(evaluate(values));
}

/* ************************************************************************ */
double TableFactor::error(const HybridValues& values) const {
  return error(values.discrete());
}

/* ************************************************************************ */
DecisionTreeFactor TableFactor::operator*(const DecisionTreeFactor& f) const {
  return toDecisionTreeFactor() * f;
}

/* ************************************************************************ */
DecisionTreeFactor TableFactor::toDecisionTreeFactor() const {
  DiscreteKeys dkeys = discreteKeys();
  std::vector<double> table;
  for (auto i = 0; i < sparse_table_.size(); i++) {
    table.push_back(sparse_table_.coeff(i));
  }
  DecisionTreeFactor f(dkeys, table);
  return f;
}

/* ************************************************************************ */
TableFactor TableFactor::choose(const DiscreteValues parent_assign,
                                DiscreteKeys parent_keys) const {
  if (parent_keys.empty()) return *this;

  // Unique representation of parent values.
  uint64_t unique = 0;
  uint64_t card = 1;
  for (auto it = keys_.rbegin(); it != keys_.rend(); ++it) {
    if (parent_assign.find(*it) != parent_assign.end()) {
      unique += parent_assign.at(*it) * card;
      card *= cardinality(*it);
    }
  }

  // Find child DiscreteKeys
  DiscreteKeys child_dkeys;
  std::sort(parent_keys.begin(), parent_keys.end());
  std::set_difference(sorted_dkeys_.begin(), sorted_dkeys_.end(),
                      parent_keys.begin(), parent_keys.end(),
                      std::back_inserter(child_dkeys));

  // Create child sparse table to populate.
  uint64_t child_card = 1;
  for (const DiscreteKey& child_dkey : child_dkeys)
    child_card *= child_dkey.second;
  Eigen::SparseVector<double> child_sparse_table_(child_card);
  child_sparse_table_.reserve(child_card);

  // Populate child sparse table.
  for (SparseIt it(sparse_table_); it; ++it) {
    // Create unique representation of parent keys
    uint64_t parent_unique = uniqueRep(parent_keys, it.index());
    // Populate the table
    if (parent_unique == unique) {
      uint64_t idx = uniqueRep(child_dkeys, it.index());
      child_sparse_table_.insert(idx) = it.value();
    }
  }

  child_sparse_table_.pruned();
  child_sparse_table_.data().squeeze();
  return TableFactor(child_dkeys, child_sparse_table_);
}

/* ************************************************************************ */
double TableFactor::safe_div(const double& a, const double& b) {
  // The use for safe_div is when we divide the product factor by the sum
  // factor. If the product or sum is zero, we accord zero probability to the
  // event.
  return (a == 0 || b == 0) ? 0 : (a / b);
}

/* ************************************************************************ */
void TableFactor::print(const string& s, const KeyFormatter& formatter) const {
  cout << s;
  cout << " f[";
  for (auto&& key : keys())
    cout << " (" << formatter(key) << "," << cardinality(key) << "),";
  cout << " ]" << endl;
  for (SparseIt it(sparse_table_); it; ++it) {
    DiscreteValues assignment = findAssignments(it.index());
    for (auto&& kv : assignment) {
      cout << "(" << formatter(kv.first) << ", " << kv.second << ")";
    }
    cout << " | " << it.value() << " | " << it.index() << endl;
  }
  cout << "number of nnzs: " << sparse_table_.nonZeros() << endl;
}

/* ************************************************************************ */
TableFactor TableFactor::apply(const TableFactor& f, Binary op) const {
  if (keys_.empty() && sparse_table_.nonZeros() == 0)
    return f;
  else if (f.keys_.empty() && f.sparse_table_.nonZeros() == 0)
    return *this;
  // 1. Identify keys for contract and free modes.
  DiscreteKeys contract_dkeys = contractDkeys(f);
  DiscreteKeys f_free_dkeys = f.freeDkeys(*this);
  DiscreteKeys union_dkeys = unionDkeys(f);
  // 2. Create hash table for input factor f
  unordered_map<uint64_t, AssignValList> map_f =
      f.createMap(contract_dkeys, f_free_dkeys);
  // 3. Initialize multiplied factor.
  uint64_t card = 1;
  for (auto u_dkey : union_dkeys) card *= u_dkey.second;
  Eigen::SparseVector<double> mult_sparse_table(card);
  mult_sparse_table.reserve(card);
  // 3. Multiply.
  for (SparseIt it(sparse_table_); it; ++it) {
    uint64_t contract_unique = uniqueRep(contract_dkeys, it.index());
    if (map_f.find(contract_unique) == map_f.end()) continue;
    for (auto assignVal : map_f[contract_unique]) {
      uint64_t union_idx = unionRep(union_dkeys, assignVal.first, it.index());
      mult_sparse_table.insert(union_idx) = op(it.value(), assignVal.second);
    }
  }
  // 4. Free unused memory.
  mult_sparse_table.pruned();
  mult_sparse_table.data().squeeze();
  // 5. Create union keys and return.
  return TableFactor(union_dkeys, mult_sparse_table);
}

/* ************************************************************************ */
DiscreteKeys TableFactor::contractDkeys(const TableFactor& f) const {
  // Find contract modes.
  DiscreteKeys contract;
  set_intersection(sorted_dkeys_.begin(), sorted_dkeys_.end(),
                   f.sorted_dkeys_.begin(), f.sorted_dkeys_.end(),
                   back_inserter(contract));
  return contract;
}

/* ************************************************************************ */
DiscreteKeys TableFactor::freeDkeys(const TableFactor& f) const {
  // Find free modes.
  DiscreteKeys free;
  set_difference(sorted_dkeys_.begin(), sorted_dkeys_.end(),
                 f.sorted_dkeys_.begin(), f.sorted_dkeys_.end(),
                 back_inserter(free));
  return free;
}

/* ************************************************************************ */
DiscreteKeys TableFactor::unionDkeys(const TableFactor& f) const {
  // Find union modes.
  DiscreteKeys union_dkeys;
  set_union(sorted_dkeys_.begin(), sorted_dkeys_.end(), f.sorted_dkeys_.begin(),
            f.sorted_dkeys_.end(), back_inserter(union_dkeys));
  return union_dkeys;
}

/* ************************************************************************ */
uint64_t TableFactor::unionRep(const DiscreteKeys& union_keys,
                               const DiscreteValues& f_free,
                               const uint64_t idx) const {
  uint64_t union_idx = 0, card = 1;
  for (auto it = union_keys.rbegin(); it != union_keys.rend(); it++) {
    if (f_free.find(it->first) == f_free.end()) {
      union_idx += keyValueForIndex(it->first, idx) * card;
    } else {
      union_idx += f_free.at(it->first) * card;
    }
    card *= it->second;
  }
  return union_idx;
}

/* ************************************************************************ */
unordered_map<uint64_t, TableFactor::AssignValList> TableFactor::createMap(
    const DiscreteKeys& contract, const DiscreteKeys& free) const {
  // 1. Initialize map.
  unordered_map<uint64_t, AssignValList> map_f;
  // 2. Iterate over nonzero elements.
  for (SparseIt it(sparse_table_); it; ++it) {
    // 3. Create unique representation of contract modes.
    uint64_t unique_rep = uniqueRep(contract, it.index());
    // 4. Create assignment for free modes.
    DiscreteValues free_assignments;
    for (auto& key : free)
      free_assignments[key.first] = keyValueForIndex(key.first, it.index());
    // 5. Populate map.
    if (map_f.find(unique_rep) == map_f.end()) {
      map_f[unique_rep] = {make_pair(free_assignments, it.value())};
    } else {
      map_f[unique_rep].push_back(make_pair(free_assignments, it.value()));
    }
  }
  return map_f;
}

/* ************************************************************************ */
uint64_t TableFactor::uniqueRep(const DiscreteKeys& dkeys,
                                const uint64_t idx) const {
  if (dkeys.empty()) return 0;
  uint64_t unique_rep = 0, card = 1;
  for (auto it = dkeys.rbegin(); it != dkeys.rend(); it++) {
    unique_rep += keyValueForIndex(it->first, idx) * card;
    card *= it->second;
  }
  return unique_rep;
}

/* ************************************************************************ */
uint64_t TableFactor::uniqueRep(const DiscreteValues& assignments) const {
  if (assignments.empty()) return 0;
  uint64_t unique_rep = 0, card = 1;
  for (auto it = assignments.rbegin(); it != assignments.rend(); it++) {
    unique_rep += it->second * card;
    card *= cardinalities_.at(it->first);
  }
  return unique_rep;
}

/* ************************************************************************ */
DiscreteValues TableFactor::findAssignments(const uint64_t idx) const {
  DiscreteValues assignment;
  for (Key key : keys_) {
    assignment[key] = keyValueForIndex(key, idx);
  }
  return assignment;
}

/* ************************************************************************ */
TableFactor::shared_ptr TableFactor::combine(size_t nrFrontals,
                                             Binary op) const {
  if (nrFrontals > size()) {
    throw invalid_argument(
        "TableFactor::combine: invalid number of frontal "
        "keys " +
        to_string(nrFrontals) + ", nr.keys=" + std::to_string(size()));
  }
  // Find remaining keys.
  DiscreteKeys remain_dkeys;
  uint64_t card = 1;
  for (auto i = nrFrontals; i < keys_.size(); i++) {
    remain_dkeys.push_back(discreteKey(i));
    card *= cardinality(keys_[i]);
  }
  // Create combined table.
  Eigen::SparseVector<double> combined_table(card);
  combined_table.reserve(sparse_table_.nonZeros());
  // Populate combined table.
  for (SparseIt it(sparse_table_); it; ++it) {
    uint64_t idx = uniqueRep(remain_dkeys, it.index());
    double new_val = op(combined_table.coeff(idx), it.value());
    combined_table.coeffRef(idx) = new_val;
  }
  // Free unused memory.
  combined_table.pruned();
  combined_table.data().squeeze();
  return std::make_shared<TableFactor>(remain_dkeys, combined_table);
}

/* ************************************************************************ */
TableFactor::shared_ptr TableFactor::combine(const Ordering& frontalKeys,
                                             Binary op) const {
  if (frontalKeys.size() > size()) {
    throw invalid_argument(
        "TableFactor::combine: invalid number of frontal "
        "keys " +
        std::to_string(frontalKeys.size()) +
        ", nr.keys=" + std::to_string(size()));
  }
  // Find remaining keys.
  DiscreteKeys remain_dkeys;
  uint64_t card = 1;
  for (Key key : keys_) {
    if (std::find(frontalKeys.begin(), frontalKeys.end(), key) ==
        frontalKeys.end()) {
      remain_dkeys.emplace_back(key, cardinality(key));
      card *= cardinality(key);
    }
  }
  // Create combined table.
  Eigen::SparseVector<double> combined_table(card);
  combined_table.reserve(sparse_table_.nonZeros());
  // Populate combined table.
  for (SparseIt it(sparse_table_); it; ++it) {
    uint64_t idx = uniqueRep(remain_dkeys, it.index());
    double new_val = op(combined_table.coeff(idx), it.value());
    combined_table.coeffRef(idx) = new_val;
  }
  // Free unused memory.
  combined_table.pruned();
  combined_table.data().squeeze();
  return std::make_shared<TableFactor>(remain_dkeys, combined_table);
}

/* ************************************************************************ */
size_t TableFactor::keyValueForIndex(Key target_key, uint64_t index) const {
  // http://phrogz.net/lazy-cartesian-product
  return (index / denominators_.at(target_key)) % cardinality(target_key);
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
  for (SparseIt it(sparse_table_); it; ++it) {
    DiscreteValues assignment = findAssignments(it.index());
    ss << "|";
    for (auto& key : keys()) {
      size_t index = assignment.at(key);
      ss << DiscreteValues::Translate(names, key, index) << "|";
    }
    ss << it.value() << "|\n";
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
  for (SparseIt it(sparse_table_); it; ++it) {
    DiscreteValues assignment = findAssignments(it.index());
    ss << "    <tr>";
    for (auto& key : keys()) {
      size_t index = assignment.at(key);
      ss << "<th>" << DiscreteValues::Translate(names, key, index) << "</th>";
    }
    ss << "<td>" << it.value() << "</td>";  // value
    ss << "</tr>\n";
  }
  ss << "  </tbody>\n</table>\n</div>";
  return ss.str();
}

/* ************************************************************************ */
TableFactor TableFactor::prune(size_t maxNrAssignments) const {
  const size_t N = maxNrAssignments;

  // Get the probabilities in the TableFactor so we can threshold.
  vector<pair<Eigen::Index, double>> probabilities;

  // Store non-zero probabilities along with their indices in a vector.
  for (SparseIt it(sparse_table_); it; ++it) {
    probabilities.emplace_back(it.index(), it.value());
  }

  // The number of probabilities can be lower than max_leaves.
  if (probabilities.size() <= N) return *this;

  // Sort the vector in descending order based on the element values.
  sort(probabilities.begin(), probabilities.end(),
       [](const std::pair<Eigen::Index, double>& a,
          const std::pair<Eigen::Index, double>& b) {
         return a.second > b.second;
       });

  // Keep the largest N probabilities in the vector.
  if (probabilities.size() > N) probabilities.resize(N);

  // Create pruned sparse vector.
  Eigen::SparseVector<double> pruned_vec(sparse_table_.size());
  pruned_vec.reserve(probabilities.size());

  // Populate pruned sparse vector.
  for (const auto& prob : probabilities) {
    pruned_vec.insert(prob.first) = prob.second;
  }

  // Create pruned decision tree factor and return.
  return TableFactor(this->discreteKeys(), pruned_vec);
}

/* ************************************************************************ */
}  // namespace gtsam
