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
#include <gtsam/inference/Ordering.h>

#include <algorithm>
#include <functional>
#include <stdexcept>
#include <utility>
#include <vector>

using namespace std;

namespace gtsam {

namespace {

/// Keys in one portion of a factor scope and their joint cardinality.
struct Scope {
  DiscreteKeys keys;
  uint64_t cardinality = 1;

  Scope& operator+=(const DiscreteKey& key) {
    keys.push_back(key);
    cardinality *= key.second;
    return *this;
  }
};

/// Scope partition needed to combine two table factors.
struct ScopePartition {
  Scope contractScope;
  Scope thisFreeScope;
  Scope factorFreeScope;
  Scope unionScope;

  /// Partition two sorted scopes and compute their cardinalities in one pass.
  ScopePartition(const DiscreteKeys& thisKeys, const DiscreteKeys& factorKeys) {
    contractScope.keys.reserve(std::min(thisKeys.size(), factorKeys.size()));
    thisFreeScope.keys.reserve(thisKeys.size());
    factorFreeScope.keys.reserve(factorKeys.size());
    unionScope.keys.reserve(thisKeys.size() + factorKeys.size());

    auto thisIterator = thisKeys.begin();
    auto factorIterator = factorKeys.begin();
    while (thisIterator != thisKeys.end() ||
           factorIterator != factorKeys.end()) {
      if (factorIterator == factorKeys.end() ||
          (thisIterator != thisKeys.end() && *thisIterator < *factorIterator)) {
        thisFreeScope += *thisIterator;
        unionScope += *thisIterator;
        ++thisIterator;
      } else if (thisIterator == thisKeys.end() ||
                 *factorIterator < *thisIterator) {
        factorFreeScope += *factorIterator;
        unionScope += *factorIterator;
        ++factorIterator;
      } else {
        contractScope += *thisIterator;
        unionScope += *thisIterator;
        ++thisIterator;
        ++factorIterator;
      }
    }
  }

  /// Upper bound on result nonzeros for the given sparse factor sizes.
  uint64_t reserveSize(uint64_t thisNonzeroCount,
                       uint64_t factorNonzeroCount) const {
    if (thisFreeScope.keys.empty() && factorFreeScope.keys.empty()) {
      // The scopes are identical, so multiplication can only remove entries.
      return std::min(thisNonzeroCount, factorNonzeroCount);
    } else if (factorFreeScope.keys.empty()) {
      // The other scope is contained in this and can only mask this table.
      return thisNonzeroCount;
    } else if (thisFreeScope.keys.empty()) {
      // This scope is contained in the other and can only mask its table.
      return factorNonzeroCount;
    }

    // Both factors add dimensions. Bound each possible sparse expansion and
    // return the smaller bound instead of the full dense cardinality.
    const auto boundedProduct = [unionCardinality = unionScope.cardinality](
                                    uint64_t nonzeroCount,
                                    uint64_t expansionCardinality) {
      if (nonzeroCount > unionCardinality / expansionCardinality) {
        return unionCardinality;
      }
      return nonzeroCount * expansionCardinality;
    };
    const uint64_t thisExpansion =
        boundedProduct(thisNonzeroCount, factorFreeScope.cardinality);
    const uint64_t factorExpansion =
        boundedProduct(factorNonzeroCount, thisFreeScope.cardinality);
    return std::min(thisExpansion, factorExpansion);
  }
};

/// One mixed-radix digit, or a contiguous group of digits, in two table layouts.
struct IndexProjection {
  uint64_t sourceStride;
  uint64_t cardinality;
  uint64_t targetStride;
};

/**
 * Cache the strides needed to project a source index onto a subset of its keys.
 * Adjacent digits can be decoded together when they are contiguous in both
 * layouts: a * c + b = (index / stride) % (cardinality_a * c). For example,
 * projecting onto a suffix of the source keys needs one group instead of one
 * integer division and remainder per key. Use actual strides because the
 * metadata-only TableFactor constructor can reorder keys without moving data.
 */
std::vector<IndexProjection> makeSubsetProjection(
    const DiscreteKeys& keys, const std::map<Key, size_t>& sourceStrides,
    const std::map<Key, size_t>& targetStrides) {
  std::vector<IndexProjection> projection;
  projection.reserve(keys.size());
  for (const auto& [key, cardinality] : keys) {
    if (cardinality == 1) continue;  // This digit is always zero.
    const IndexProjection term{sourceStrides.at(key), cardinality,
                               targetStrides.at(key)};
    if (!projection.empty() &&
        projection.back().sourceStride == term.sourceStride * cardinality &&
        projection.back().targetStride == term.targetStride * cardinality) {
      auto& previous = projection.back();
      previous = {term.sourceStride, previous.cardinality * cardinality,
                  term.targetStride};
    } else {
      projection.push_back(term);
    }
  }
  return projection;
}

}  // namespace

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
TableFactor::TableFactor(const DiscreteKeys& dkeys, const SparseVector& table)
    : DiscreteFactor(dkeys.indices(), dkeys.cardinalities()),
      sparse_table_(table.size()) {
  sparse_table_ = table;
  // Use integer arithmetic since `double` cannot represent every index above
  // 2^53 due to precision limitations.
  uint64_t denominator = table.size();
  for (const DiscreteKey& dkey : dkeys) {
    denominator /= dkey.second;
    denominators_.emplace(dkey.first, denominator);
  }
  sorted_dkeys_ = discreteKeys();
  sort(sorted_dkeys_.begin(), sorted_dkeys_.end());
}

/* ************************************************************************ */
TableFactor::TableFactor(const DiscreteKeys& dkeys,
                         const DecisionTree<Key, double>& dtree)
    : TableFactor(dkeys, DecisionTreeFactor(dkeys, dtree)) {}

/**
 * @brief Compute the indexing of the leaves in the decision tree based on the
 * assignment and add the (index, leaf) pair to a SparseVector.
 *
 * We visit each leaf in the tree, and using the cardinalities of the keys,
 * compute the correct index to add the leaf to a SparseVector which
 *  is then used to create the TableFactor.
 *
 * @param dt The DecisionTree
 * @return SparseVector
 */
static SparseVector ComputeSparseTable(const DiscreteKeys& dkeys,
                                       const DecisionTreeFactor& dt) {
  // SparseVector needs to know the maximum possible index,
  // so we compute the product of cardinalities.
  size_t cardinalityProduct = 1;
  for (auto&& [_, c] : dt.cardinalities()) {
    cardinalityProduct *= c;
  }
  SparseVector sparseTable(cardinalityProduct);
  size_t nrValues = 0;
  dt.visit([&nrValues](double x) {
    if (x > 0) nrValues += 1;
  });
  sparseTable.reserve(nrValues);

  KeySet allKeys(dt.keys().begin(), dt.keys().end());

  // Compute denominators to be used in computing sparse table indices
  std::map<Key, size_t> denominators;
  uint64_t denominator = sparseTable.size();
  for (const DiscreteKey& dkey : dkeys) {
    denominator /= dkey.second;
    denominators.emplace(dkey.first, denominator);
  }

  /**
   * @brief Functor which is called by the DecisionTree for each leaf.
   * For each leaf value, we use the corresponding assignment to compute a
   * corresponding index into a SparseVector. We then populate sparseTable with
   * the value at the computed index.
   *
   * Takes advantage of the sparsity of the DecisionTree to be efficient. When
   * merged branches are encountered, we enumerate over the missing keys.
   *
   */
  auto op = [&](const Assignment<Key>& assignment, double p) {
    // Check if greater than a threshold because we consider
    // smaller than that as numerically 0
    if (p > 1e-150) {
      // Get all the keys involved in this assignment
      KeySet assignmentKeys;
      for (auto&& [k, _] : assignment) {
        assignmentKeys.insert(k);
      }

      // Find the keys missing in the assignment
      KeyVector diff;
      std::set_difference(allKeys.begin(), allKeys.end(),
                          assignmentKeys.begin(), assignmentKeys.end(),
                          std::back_inserter(diff));

      // Generate all assignments using the missing keys
      DiscreteKeys extras;
      for (auto&& key : diff) {
        extras.push_back({key, dt.cardinality(key)});
      }
      auto&& extraAssignments = DiscreteValues::CartesianProduct(extras);

      for (auto&& extra : extraAssignments) {
        // Create new assignment using the extra assignment
        DiscreteValues updatedAssignment(assignment);
        updatedAssignment.insert(extra);

        // Generate index and add to the sparse vector.
        Eigen::Index idx = 0;
        // We go in reverse since a DecisionTree has the highest label first
        for (auto&& it = updatedAssignment.rbegin();
             it != updatedAssignment.rend(); it++) {
          idx += it->second * denominators.at(it->first);
        }
        sparseTable.coeffRef(idx) = p;
      }
    }
  };

  // Visit each leaf in `dt` to get the Assignment and leaf value
  // to populate the sparseTable.
  dt.visitWith(op);

  return sparseTable;
}

/* ************************************************************************ */
TableFactor::TableFactor(const DiscreteKeys& dkeys,
                         const DecisionTreeFactor& dtf)
    : TableFactor(dkeys, ComputeSparseTable(dkeys, dtf)) {}

/* ************************************************************************ */
TableFactor::TableFactor(const DecisionTreeFactor& dtf)
    : TableFactor(dtf.discreteKeys(), dtf) {}

/* ************************************************************************ */
TableFactor::TableFactor(const DiscreteConditional& c)
    : TableFactor(c.toTableFactor()) {}

/* ************************************************************************ */
SparseVector TableFactor::Convert(const DiscreteKeys& keys,
                                  const std::vector<double>& table) {
  size_t max_size = 1;
  for (auto&& [_, cardinality] : keys.cardinalities()) {
    max_size *= cardinality;
  }
  if (table.size() != max_size) {
    throw std::runtime_error(
        "The cardinalities of the keys don't match the number of values in the "
        "input.");
  }

  SparseVector sparse_table(table.size());
  // Count number of nonzero elements in table and reserve the space.
  // Take each value as a double: converting to an integer would truncate
  // probabilities below 1 to 0 and undercount.
  const uint64_t nnz = std::count_if(table.begin(), table.end(),
                                     [](double value) { return value != 0.0; });
  sparse_table.reserve(nnz);
  for (uint64_t i = 0; i < table.size(); i++) {
    if (table[i] != 0) sparse_table.insert(i) = table[i];
  }
  sparse_table.data().squeeze();
  return sparse_table;
}

/* ************************************************************************ */
SparseVector TableFactor::Convert(const DiscreteKeys& keys,
                                                 const std::string& table) {
  // Convert string to doubles.
  std::vector<double> ys;
  std::istringstream iss(table);
  std::copy(std::istream_iterator<double>(iss), std::istream_iterator<double>(),
            std::back_inserter(ys));
  return Convert(keys, ys);
}

/* ************************************************************************ */
bool TableFactor::equals(const DiscreteFactor& other, double tol) const {
  if (!dynamic_cast<const TableFactor*>(&other)) {
    return false;
  } else {
    const auto& f(static_cast<const TableFactor&>(other));
    return Base::equals(other, tol) &&
           sparse_table_.isApprox(f.sparse_table_, tol);
  }
}

/* ************************************************************************ */
double TableFactor::evaluate(const Assignment<Key>& values) const {
  // The table is laid out in key order, and each key's value is weighted by
  // its denominator, i.e., the product of the cardinalities of later keys.
  uint64_t idx = 0;
  for (auto&& [key, denominator] : denominators_) {
    auto it = values.find(key);
    if (it != values.end()) idx += it->second * denominator;
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
TableFactor TableFactor::operator*(const DiscreteConditional& c) const {
  return *this * c.toTableFactor();
}

/* ************************************************************************ */
DiscreteFactor::shared_ptr TableFactor::multiply(
    const DiscreteFactor::shared_ptr& f) const {
  DiscreteFactor::shared_ptr result;
  if (auto tf = std::dynamic_pointer_cast<TableFactor>(f)) {
    // If `f` is a TableFactor, we can simply call `operator*`.
    result = std::make_shared<TableFactor>(this->operator*(*tf));

  } else if (auto dtf = std::dynamic_pointer_cast<DecisionTreeFactor>(f)) {
    // Use the virtual conversion so derived decision tree factors can provide
    // their actual sparse representation.
    result =
        std::make_shared<TableFactor>(this->operator*(dtf->toTableFactor()));

  } else {
    // Let the other factor choose its most efficient table representation,
    // then multiply it without copying this potentially large sparse table.
    result = std::make_shared<TableFactor>(this->operator*(f->toTableFactor()));
  }
  return result;
}

/* ************************************************************************ */
DiscreteFactor::shared_ptr TableFactor::operator/(
    const DiscreteFactor::shared_ptr& f) const {
  if (auto tf = std::dynamic_pointer_cast<TableFactor>(f)) {
    return std::make_shared<TableFactor>(this->operator/(*tf));
  } else if (auto dtf = std::dynamic_pointer_cast<DecisionTreeFactor>(f)) {
    return std::make_shared<TableFactor>(
        this->operator/(TableFactor(f->discreteKeys(), *dtf)));
  } else {
    TableFactor divisor(f->toDecisionTreeFactor());
    return std::make_shared<TableFactor>(this->operator/(divisor));
  }
}

/* ************************************************************************ */
DecisionTreeFactor TableFactor::toDecisionTreeFactor() const {
  DiscreteKeys dkeys = discreteKeys();

  // If no keys, then return empty DecisionTreeFactor
  if (dkeys.size() == 0) {
    AlgebraicDecisionTree<Key> tree;
    // We can have an empty sparse_table_ or one with a single value.
    if (sparse_table_.size() != 0) {
      tree = AlgebraicDecisionTree<Key>(sparse_table_.coeff(0));
    }
    return DecisionTreeFactor(dkeys, tree);
  }

  // Build the tree from the nonzero entries only, so a sparse table over many
  // keys (e.g. a pruned hybrid mode posterior) costs time in its nonzeros, not
  // in the product of all cardinalities. Keys go from highest to lowest, the
  // order a DecisionTree keeps from root to leaves.
  DiscreteKeys orderedKeys = dkeys;
  std::sort(orderedKeys.begin(), orderedKeys.end(),
            [](const DiscreteKey& a, const DiscreteKey& b) {
              return a.first > b.first;
            });
  std::vector<std::pair<std::vector<size_t>, double>> entries;
  entries.reserve(sparse_table_.nonZeros());
  for (SparseIt it(sparse_table_); it; ++it) {
    std::vector<size_t> values;
    values.reserve(orderedKeys.size());
    for (const DiscreteKey& dkey : orderedKeys) {
      values.push_back(keyValueForIndex(dkey.first, it.index()));
    }
    entries.emplace_back(std::move(values), it.value());
  }
  std::sort(entries.begin(), entries.end());

  using Tree = DecisionTree<Key, double>;
  std::function<Tree(size_t, size_t, size_t)> build =
      [&](size_t level, size_t begin, size_t end) -> Tree {
    if (begin == end) return Tree(0.0);
    if (level == orderedKeys.size()) return Tree(entries[begin].second);
    std::vector<Tree> branches;
    branches.reserve(orderedKeys[level].second);
    size_t first = begin;
    for (size_t value = 0; value < orderedKeys[level].second; ++value) {
      size_t last = first;
      while (last < end && entries[last].first[level] == value) ++last;
      branches.push_back(build(level + 1, first, last));
      first = last;
    }
    return Tree(branches.begin(), branches.end(), orderedKeys[level].first);
  };

  // Build the decision tree
  const Tree built = build(0, 0, entries.size());
  // Merge identical branches to reduce size of the tree
  AlgebraicDecisionTree<Key> tree(Tree::Choice::Unique(built.root_));

  return DecisionTreeFactor(dkeys, tree);
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
  SparseVector child_sparse_table_(child_card);
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

  // Drop exact zeros, which would otherwise be stored and counted as values.
  child_sparse_table_.prune(0.0);
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
    cout << " | " << std::setw(10) << std::left << it.value() << " | "
         << it.index() << endl;
  }
  cout << "number of nnzs: " << sparse_table_.nonZeros() << endl;
}

/* ************************************************************************ */
DiscreteFactor::shared_ptr TableFactor::sum(size_t nrFrontals) const {
  return combine(nrFrontals, Ring::add);
}

/* ************************************************************************ */
DiscreteFactor::shared_ptr TableFactor::sum(const Ordering& keys) const {
  return combine(keys, Ring::add);
}

/* ************************************************************************ */
double TableFactor::max() const {
  // Assignments without a stored value are implicitly zero, so they count
  // toward the maximum too, e.g. an all-zero factor stores nothing.
  double max_value = sparse_table_.nonZeros() < sparse_table_.size()
                         ? 0.0
                         : std::numeric_limits<double>::lowest();
  for (SparseVector::InnerIterator it(sparse_table_); it; ++it) {
    max_value = std::max(max_value, it.value());
  }
  return max_value;
}

/* ************************************************************************ */
DiscreteValues TableFactor::argmax() const {
  uint64_t maxIndex = 0;
  double maxValue = 0.0;
  for (SparseIt it(sparse_table_); it; ++it) {
    if (it.value() > maxValue) {
      maxIndex = it.index();
      maxValue = it.value();
    }
  }
  return findAssignments(maxIndex);
}

/* ************************************************************************ */
DiscreteFactor::shared_ptr TableFactor::max(size_t nrFrontals) const {
  return combine(nrFrontals, Ring::max);
}

/* ************************************************************************ */
DiscreteFactor::shared_ptr TableFactor::max(const Ordering& keys) const {
  return combine(keys, Ring::max);
}

/* ************************************************************************ */
TableFactor TableFactor::apply(Unary op) const {
  // Initialize new factor.
  uint64_t cardi = 1;
  for (auto [key, c] : cardinalities_) cardi *= c;
  SparseVector sparse_table(cardi);
  sparse_table.reserve(sparse_table_.nonZeros());

  // Populate
  for (SparseIt it(sparse_table_); it; ++it) {
    sparse_table.coeffRef(it.index()) = op(it.value());
  }

  // Free unused memory and return.
  // Drop exact zeros, which would otherwise be stored and counted as values.
  sparse_table.prune(0.0);
  sparse_table.data().squeeze();
  return TableFactor(discreteKeys(), sparse_table);
}

/* ************************************************************************ */
TableFactor TableFactor::apply(UnaryAssignment op) const {
  // Initialize new factor.
  uint64_t cardi = 1;
  for (auto [key, c] : cardinalities_) cardi *= c;
  SparseVector sparse_table(cardi);
  sparse_table.reserve(sparse_table_.nonZeros());

  // Populate
  for (SparseIt it(sparse_table_); it; ++it) {
    DiscreteValues assignment = findAssignments(it.index());
    sparse_table.coeffRef(it.index()) = op(assignment, it.value());
  }

  // Free unused memory and return.
  // Drop exact zeros, which would otherwise be stored and counted as values.
  sparse_table.prune(0.0);
  sparse_table.data().squeeze();
  return TableFactor(discreteKeys(), sparse_table);
}

/* ************************************************************************ */
TableFactor TableFactor::applySubset(const TableFactor& factor, Binary op) const {
  SparseVector result(sparse_table_.size());
  if (factor.sparse_table_.nonZeros() == 0)
    return TableFactor(sorted_dkeys_, result);
  // A strict subset can broadcast each entry across several source entries.
  // Equal scopes can only intersect, so retain the tighter allocation bound.
  result.reserve(size() == factor.size()
                     ? std::min(sparse_table_.nonZeros(),
                                factor.sparse_table_.nonZeros())
                     : sparse_table_.nonZeros());

  const auto projection = makeSubsetProjection(
      factor.sorted_dkeys_, denominators_, factor.denominators_);

  // Inspect actual strides: the metadata-only constructor can change keys_
  // without changing the underlying storage order.
  bool sortedLayout = true;
  uint64_t stride = 1;
  for (auto key = sorted_dkeys_.rbegin(); key != sorted_dkeys_.rend(); ++key) {
    sortedLayout &= denominators_.at(key->first) == stride;
    stride *= key->second;
  }
  const auto& other = factor.sparse_table_;
  const bool dense = other.nonZeros() == other.size();
  const auto* indices = other.innerIndexPtr();
  for (SparseIt entry(sparse_table_); entry; ++entry) {
    uint64_t target = 0;
    for (const auto& term : projection)
      target += ((entry.index() / term.sourceStride) % term.cardinality) *
                term.targetStride;
    // Projection stays within other.size(); match Eigen's signed index type.
    const auto targetIndex = static_cast<SparseVector::StorageIndex>(target);
    Eigen::Index offset = targetIndex;
    if (!dense) {
      const auto* end = indices + other.nonZeros();
      const auto* found = std::lower_bound(indices, end, targetIndex);
      // Do not invoke a generic op on absent entries. Explicitly stored zeros
      // still participate, just as in the general sparse join.
      if (found == end || *found != targetIndex) continue;
      offset = found - indices;
    }
    const double value = op(entry.value(), other.valuePtr()[offset]);
    if (sortedLayout)
      result.insertBack(entry.index()) = value;
    else
      result.insert(uniqueRep(sorted_dkeys_, entry.index())) = value;
  }
  // Drop exact zeros, which would otherwise be stored and counted as values.
  result.prune(0.0);
  result.data().squeeze();
  return TableFactor(sorted_dkeys_, result);
}

/* ************************************************************************ */
TableFactor TableFactor::apply(const TableFactor& f, Binary op) const {
  // Only a default-constructed factor has an empty table, and it acts as the
  // identity. A keyless scalar always has a table of length 1, even when its
  // value is 0, so a computed zero still applies.
  if (keys_.empty() && sparse_table_.size() == 0)
    return f;
  else if (f.keys_.empty() && f.sparse_table_.size() == 0)
    return *this;
  // Multiplying by a subset or dividing by a separator needs only a lookup,
  // not a hash join with per-entry assignment maps. Check containment before
  // allocating the four scope vectors needed only by the general join. Compare
  // key/cardinality pairs, not just labels, to preserve the existing dispatch.
  if (f.size() <= size() &&
      std::includes(sorted_dkeys_.begin(), sorted_dkeys_.end(),
                    f.sorted_dkeys_.begin(), f.sorted_dkeys_.end())) {
    return applySubset(f, op);
  }
  // 1. Partition both scopes and compute their cardinalities in one pass.
  const ScopePartition scopes(sorted_dkeys_, f.sorted_dkeys_);
  // 2. Create hash table for input factor f
  unordered_map<uint64_t, AssignValList> factorMap =
      f.createMap(scopes.contractScope.keys, scopes.factorFreeScope.keys);
  // 3. Initialize the multiplied factor with a scope-aware upper bound on its
  // number of nonzeros.
  SparseVector multipliedSparseTable(
      scopes.unionScope.cardinality);
  multipliedSparseTable.reserve(
      scopes.reserveSize(sparse_table_.nonZeros(), f.sparse_table_.nonZeros()));
  // 4. Multiply.
  for (SparseIt it(sparse_table_); it; ++it) {
    uint64_t contractIndex = uniqueRep(scopes.contractScope.keys, it.index());
    const auto mapIt = factorMap.find(contractIndex);
    if (mapIt == factorMap.end()) continue;
    for (const auto& assignmentValue : mapIt->second) {
      uint64_t unionIndex =
          unionRep(scopes.unionScope.keys, assignmentValue.first, it.index());
      multipliedSparseTable.insert(unionIndex) =
          op(it.value(), assignmentValue.second);
    }
  }
  // 5. Free unused memory.
  // Drop exact zeros, which would otherwise be stored and counted as values.
  multipliedSparseTable.prune(0.0);
  multipliedSparseTable.data().squeeze();
  // 6. Create union keys and return.
  return TableFactor(scopes.unionScope.keys, multipliedSparseTable);
}

/* ************************************************************************ */
#ifdef GTSAM_ALLOW_DEPRECATED_SINCE_V43
DiscreteKeys TableFactor::contractDkeys(const TableFactor& factor) const {
  return ScopePartition(sorted_dkeys_, factor.sorted_dkeys_).contractScope.keys;
}

/* ************************************************************************ */
DiscreteKeys TableFactor::freeDkeys(const TableFactor& factor) const {
  return ScopePartition(sorted_dkeys_, factor.sorted_dkeys_).thisFreeScope.keys;
}

/* ************************************************************************ */
DiscreteKeys TableFactor::unionDkeys(const TableFactor& factor) const {
  return ScopePartition(sorted_dkeys_, factor.sorted_dkeys_).unionScope.keys;
}
#endif

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
  SparseVector combined_table(card);
  combined_table.reserve(sparse_table_.nonZeros());
  // Populate combined table.
  for (SparseIt it(sparse_table_); it; ++it) {
    uint64_t idx = uniqueRep(remain_dkeys, it.index());
    double new_val = op(combined_table.coeff(idx), it.value());
    combined_table.coeffRef(idx) = new_val;
  }
  // Free unused memory.
  // Drop exact zeros, which would otherwise be stored and counted as values.
  combined_table.prune(0.0);
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
  SparseVector combined_table(card);
  combined_table.reserve(sparse_table_.nonZeros());
  // Populate combined table.
  for (SparseIt it(sparse_table_); it; ++it) {
    uint64_t idx = uniqueRep(remain_dkeys, it.index());
    double new_val = op(combined_table.coeff(idx), it.value());
    combined_table.coeffRef(idx) = new_val;
  }
  // Free unused memory.
  // Drop exact zeros, which would otherwise be stored and counted as values.
  combined_table.prune(0.0);
  combined_table.data().squeeze();
  return std::make_shared<TableFactor>(remain_dkeys, combined_table);
}

/* ************************************************************************ */
size_t TableFactor::keyValueForIndex(Key target_key, uint64_t index) const {
  // http://phrogz.net/lazy-cartesian-product
  return (index / denominators_.at(target_key)) % cardinality(target_key);
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
DiscreteFactor::shared_ptr TableFactor::prune(size_t maxNrAssignments) const {
  const size_t N = maxNrAssignments;

  // Collect the stored values so we can find the N-th largest.
  vector<double> values;
  values.reserve(sparse_table_.nonZeros());
  for (SparseIt it(sparse_table_); it; ++it) {
    values.push_back(it.value());
  }

  // The number of values can be lower than N, so there is nothing to prune.
  if (values.size() <= N) return std::make_shared<TableFactor>(*this);

  // Keep every value at or above the N-th largest one, as
  // DecisionTreeFactor::prune does. Values tied with the threshold are all
  // kept, so more than N assignments can survive.
  SparseVector pruned(sparse_table_.size());
  if (N > 0) {
    std::nth_element(values.begin(), values.begin() + (N - 1), values.end(),
                     std::greater<double>());
    const double threshold = values[N - 1];
    for (SparseIt it(sparse_table_); it; ++it) {
      if (it.value() >= threshold) pruned.insert(it.index()) = it.value();
    }
  }

  return std::make_shared<TableFactor>(this->discreteKeys(), pruned);
}

/* ************************************************************************ */
TableFactor TableFactor::reorder(const DiscreteKeys& keys) const {
  // Equal sizes and cardinality maps rule out duplicate keys as well.
  if (keys.size() != keys_.size() || keys.cardinalities() != cardinalities_) {
    throw std::invalid_argument(
        "TableFactor::reorder: keys must be a permutation of the factor keys.");
  }

  // Move each nonzero entry to its index in the new layout, then insert them
  // in increasing order.
  std::vector<std::pair<int64_t, double>> entries;
  entries.reserve(sparse_table_.nonZeros());
  for (SparseIt it(sparse_table_); it; ++it) {
    entries.emplace_back(uniqueRep(keys, it.index()), it.value());
  }
  std::sort(entries.begin(), entries.end());
  SparseVector reordered(sparse_table_.size());
  reordered.reserve(entries.size());
  for (auto&& [index, value] : entries) reordered.insertBack(index) = value;
  return TableFactor(keys, reordered);
}

/* ************************************************************************ */
DiscreteFactor::shared_ptr TableFactor::restrict(
    const DiscreteValues& assignment) const {
  // Split our keys into those fixed by the assignment and those left free,
  // keeping the free keys in their original order.
  // Assigned keys that are not in this factor are ignored.
  DiscreteValues fixedValues;
  DiscreteKeys freeKeys;
  uint64_t freeCardinality = 1;
  for (Key key : keys_) {
    auto it = assignment.find(key);
    if (it != assignment.end()) {
      fixedValues[key] = it->second;
    } else {
      freeKeys.emplace_back(key, cardinality(key));
      freeCardinality *= cardinality(key);
    }
  }

  // Copy the nonzero entries that agree with the fixed values, re-indexed over
  // the free keys. Matching entries keep their relative order, so indices are
  // inserted in increasing order.
  SparseVector restrictedTable(freeCardinality);
  restrictedTable.reserve(sparse_table_.nonZeros());
  for (SparseIt it(sparse_table_); it; ++it) {
    const bool matches = std::all_of(
        fixedValues.begin(), fixedValues.end(), [&](const auto& keyValue) {
          return keyValueForIndex(keyValue.first, it.index()) ==
                 keyValue.second;
        });
    if (matches) {
      restrictedTable.insert(uniqueRep(freeKeys, it.index())) = it.value();
    }
  }
  restrictedTable.data().squeeze();
  return std::make_shared<TableFactor>(freeKeys, restrictedTable);
}

/* ************************************************************************ */
}  // namespace gtsam
