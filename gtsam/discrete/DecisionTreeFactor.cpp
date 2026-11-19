/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file DecisionTreeFactor.cpp
 * @brief discrete factor
 * @date Feb 14, 2011
 * @author Duy-Nguyen Ta
 * @author Frank Dellaert
 */

#include <gtsam/base/FastSet.h>
#include <gtsam/discrete/DecisionTreeFactor.h>
#include <gtsam/discrete/DiscreteConditional.h>
#include <gtsam/discrete/TableFactor.h>
#include <gtsam/hybrid/HybridValues.h>
#include <gtsam/inference/Ordering.h>

#include <functional>
#include <limits>
#include <queue>
#include <utility>
#include <vector>

using namespace std;

namespace gtsam {

  /* ************************************************************************ */
  DecisionTreeFactor::DecisionTreeFactor() {}

  /* ************************************************************************ */
  DecisionTreeFactor::DecisionTreeFactor(const DiscreteKeys& keys,
                                         const ADT& potentials)
      : DiscreteFactor(keys.indices(), keys.cardinalities()), ADT(potentials) {}

  /* ************************************************************************ */
  DecisionTreeFactor::DecisionTreeFactor(const DiscreteConditional& c)
      : DiscreteFactor(c.keys(), c.cardinalities()),
        AlgebraicDecisionTree<Key>(c.toDecisionTreeFactor()) {}

  /* ************************************************************************ */
  DecisionTreeFactor DecisionTreeFactor::operator*(
      const DiscreteConditional& c) const {
    return *this * c.toDecisionTreeFactor();
  }

  /* ************************************************************************ */
  bool DecisionTreeFactor::equals(const DiscreteFactor& other,
                                  double tol) const {
    if (!dynamic_cast<const DecisionTreeFactor*>(&other)) {
      return false;
    } else {
      const auto& f(static_cast<const DecisionTreeFactor&>(other));
      return Base::equals(other, tol) && ADT::equals(f, tol);
    }
  }

  /* ************************************************************************ */
  double DecisionTreeFactor::error(const DiscreteValues& values) const {
    return -std::log(evaluate(values));
  }
  
  /* ************************************************************************ */
  double DecisionTreeFactor::error(const HybridValues& values) const {
    return error(values.discrete());
  }

  /* ************************************************************************ */
  DiscreteFactor::shared_ptr DecisionTreeFactor::multiply(
      const DiscreteFactor::shared_ptr& f) const {
    DiscreteFactor::shared_ptr result;
    if (auto tf = std::dynamic_pointer_cast<TableFactor>(f)) {
      // If f is a TableFactor, use the virtual conversion so derived decision
      // tree factors can provide their actual sparse representation.
      result = std::make_shared<TableFactor>((*tf) * toTableFactor());

    } else if (auto dtf = std::dynamic_pointer_cast<DecisionTreeFactor>(f)) {
      // If `f` is a DecisionTreeFactor, simply call operator*.
      result = std::make_shared<DecisionTreeFactor>(this->operator*(*dtf));

    } else {
      // Simulate double dispatch in C++
      // Useful for other classes which inherit from DiscreteFactor and have
      // only `operator*(DecisionTreeFactor)` defined. Thus, other classes don't
      // need to be updated.
      result = std::make_shared<DecisionTreeFactor>(f->operator*(*this));
    }
    return result;
  }

  /* ************************************************************************ */
  TableFactor DecisionTreeFactor::toTableFactor() const {
    return TableFactor(*this);
  }

  /* ************************************************************************ */
  DiscreteFactor::shared_ptr DecisionTreeFactor::operator/(
      const DiscreteFactor::shared_ptr& f) const {
    if (auto tf = std::dynamic_pointer_cast<TableFactor>(f)) {
      // If `f` is a TableFactor, divide as tables, which is cheaper. Use the
      // virtual conversion so derived decision tree factors can provide their
      // actual sparse representation.
      return std::make_shared<TableFactor>(toTableFactor() / *tf);

    } else if (auto dtf = std::dynamic_pointer_cast<DecisionTreeFactor>(f)) {
      // If `f` is a DecisionTreeFactor, divide normally.
      return std::make_shared<DecisionTreeFactor>(this->operator/(*dtf));

    } else {
      // Else, convert `f` to a DecisionTreeFactor so we can divide
      return std::make_shared<DecisionTreeFactor>(
          this->operator/(f->toDecisionTreeFactor()));
    }
  }

  /* ************************************************************************ */
  double DecisionTreeFactor::safe_div(const double& a, const double& b) {
    // The use for safe_div is when we divide the product factor by the sum
    // factor. If the product or sum is zero, we accord zero probability to the
    // event.
    return (a == 0 || b == 0) ? 0 : (a / b);
  }

  /* ************************************************************************ */
  void DecisionTreeFactor::print(const string& s,
                                 const KeyFormatter& formatter) const {
    cout << s;
    cout << " f[";
    for (auto&& key : keys()) {
      cout << " (" << formatter(key) << "," << cardinality(key) << "),";
    }
    cout << " ]" << endl;
    ADT::print("", formatter);
  }

  /* ************************************************************************ */
  DecisionTreeFactor DecisionTreeFactor::apply(Unary op) const {
    // apply operand
    ADT result = ADT::apply(op);
    // Make a new factor
    return DecisionTreeFactor(discreteKeys(), result);
  }

  /* ************************************************************************ */
  DecisionTreeFactor DecisionTreeFactor::apply(UnaryAssignment op) const {
    // apply operand
    ADT result = ADT::apply(op);
    // Make a new factor
    return DecisionTreeFactor(discreteKeys(), result);
  }

  /* ************************************************************************ */
  DecisionTreeFactor DecisionTreeFactor::apply(const DecisionTreeFactor& f,
                                               Binary op) const {
    map<Key, size_t> cs;  // new cardinalities
    // make unique key-cardinality map
    for (Key j : keys()) cs[j] = cardinality(j);
    for (Key j : f.keys()) cs[j] = f.cardinality(j);
    // Convert map into keys
    DiscreteKeys keys;
    keys.reserve(cs.size());
    for (const auto& key : cs) {
      keys.emplace_back(key);
    }
    // apply operand
    ADT result = ADT::apply(f, op);
    // Make a new factor
    return DecisionTreeFactor(keys, result);
  }

  /* ************************************************************************ */
  DecisionTreeFactor::shared_ptr DecisionTreeFactor::combine(size_t nrFrontals,
                                                             Binary op) const {
    if (nrFrontals > size()) {
      throw invalid_argument(
          "DecisionTreeFactor::combine: invalid number of frontal "
          "keys " +
          std::to_string(nrFrontals) + ", nr.keys=" + std::to_string(size()));
    }

    // sum over nrFrontals keys
    size_t i;
    ADT result(*this);
    for (i = 0; i < nrFrontals; i++) {
      Key j = keys_[i];
      result = result.combine(j, cardinality(j), op);
    }

    // Create new factor, note we start with keys after nrFrontals:
    DiscreteKeys dkeys;
    for (; i < keys_.size(); i++) {
      Key j = keys_[i];
      dkeys.push_back(DiscreteKey(j, cardinality(j)));
    }
    return std::make_shared<DecisionTreeFactor>(dkeys, result);
  }

  /* ************************************************************************ */
  DecisionTreeFactor::shared_ptr DecisionTreeFactor::combine(
      const Ordering& frontalKeys, Binary op) const {
    if (frontalKeys.size() > size()) {
      throw invalid_argument(
          "DecisionTreeFactor::combine: invalid number of frontal "
          "keys " +
          std::to_string(frontalKeys.size()) + ", nr.keys=" +
          std::to_string(size()));
    }

    // sum over nrFrontals keys
    size_t i;
    ADT result(*this);
    for (i = 0; i < frontalKeys.size(); i++) {
      Key j = frontalKeys[i];
      result = result.combine(j, cardinality(j), op);
    }

    /*
    Create new factor, note we collect keys that are not in frontalKeys.
    
    Due to branch merging, the labels in `result` may be missing some keys.
    E.g. After branch merging, we may get a ADT like:
      Leaf [2] 1.0204082

    Hence, code below traverses the original keys and omits those in
    frontalKeys. We loop over cardinalities, which is O(n) even for a map, and
    then "contains" is a binary search on a small vector.
    */
    DiscreteKeys dkeys;
    for (auto&& [key, cardinality] : cardinalities_) {
      if (!frontalKeys.contains(key)) { 
        dkeys.push_back(DiscreteKey(key, cardinality));
      }
    }
    return std::make_shared<DecisionTreeFactor>(dkeys, result);
  }

  /* ************************************************************************ */
  std::vector<double> DecisionTreeFactor::probabilities() const {
    // Set of all keys
    KeySet allKeys(keys().begin(), keys().end());

    std::vector<double> probs;

    /* An operation that takes each leaf probability, and computes the
     * nrAssignments by checking the difference between the keys in the factor
     * and the keys in the assignment.
     * The nrAssignments is then used to append
     * the correct number of leaf probability values to the `probs` vector
     * defined above.
     */
    auto op = [&](const Assignment<Key>& a, double p) {
      // Get all the keys in the current assignment
      KeySet assignment_keys;
      for (auto&& [k, _] : a) {
        assignment_keys.insert(k);
      }

      // Find the keys missing in the assignment
      std::vector<Key> diff;
      std::set_difference(allKeys.begin(), allKeys.end(),
                          assignment_keys.begin(), assignment_keys.end(),
                          std::back_inserter(diff));

      // Compute the total number of assignments in the (pruned) subtree
      size_t nrAssignments = 1;
      for (auto&& k : diff) {
        nrAssignments *= cardinalities_.at(k);
      }
      // Add p `nrAssignments` times to the probs vector.
      probs.insert(probs.end(), nrAssignments, p);

      return p;
    };

    // Go through the tree
    this->visitWith(op);

    return probs;
  }

  /* ************************************************************************ */
  static std::string valueFormatter(const double& v) {
    std::stringstream ss;
    ss << std::setw(4) << std::setprecision(2) << std::fixed << v;
    return ss.str();
  }

  /** output to graphviz format, stream version */
  void DecisionTreeFactor::dot(std::ostream& os,
                               const KeyFormatter& keyFormatter,
                               bool showZero) const {
    ADT::dot(os, keyFormatter, valueFormatter, showZero);
  }

  /** output to graphviz format, open a file */
  void DecisionTreeFactor::dot(const std::string& name,
                              const KeyFormatter& keyFormatter,
                              bool showZero) const {
    ADT::dot(name, keyFormatter, valueFormatter, showZero);
  }

  /** output to graphviz format string */
  std::string DecisionTreeFactor::dot(const KeyFormatter& keyFormatter,
                                      bool showZero) const {
    return ADT::dot(keyFormatter, valueFormatter, showZero);
  }

  // Print out header.
  /* ************************************************************************ */
  string DecisionTreeFactor::markdown(const KeyFormatter& keyFormatter,
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
  string DecisionTreeFactor::html(const KeyFormatter& keyFormatter,
                                  const Names& names) const {
    stringstream ss;

    // Print out preamble.
    ss << "<div>\n<table class='DecisionTreeFactor'>\n  <thead>\n";

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
  DecisionTreeFactor::DecisionTreeFactor(const DiscreteKeys& keys,
                                         const vector<double>& table)
      : DiscreteFactor(keys.indices(), keys.cardinalities()),
        AlgebraicDecisionTree<Key>(keys, table) {}

  /* ************************************************************************ */
  DecisionTreeFactor::DecisionTreeFactor(const DiscreteKeys& keys,
                                         const string& table)
      : DiscreteFactor(keys.indices(), keys.cardinalities()),
        AlgebraicDecisionTree<Key>(keys, table) {}

  /** Min-heap of (value, nrAssignments) pairs holding the N largest values.
   * Each leaf adds one entry weighted by the assignments it covers, rather than
   * one copy per assignment, and the heap drops its smallest entries once the
   * rest cover N. It therefore holds at most min(N, #leaves) + 1 entries,
   * whether N is small or a leaf covers billions of assignments.
   */
  using WeightedValue = std::pair<double, size_t>;
  using WeightedMinHeap =
      std::priority_queue<WeightedValue, std::vector<WeightedValue>,
                          std::greater<WeightedValue>>;

  /* ************************************************************************ */
  double DecisionTreeFactor::computeThreshold(const size_t N) const {
    // With nothing to keep, no value can reach the threshold.
    if (N == 0) return std::numeric_limits<double>::infinity();

    WeightedMinHeap min_heap;
    // The min_heap's total count is tracked as its gap from N rather than as a
    // sum, which could overflow when N is close to SIZE_MAX.
    // While the heap covers fewer than N assignments, nrMissing is the
    // shortfall; afterwards nrSurplus is the excess, which stays below the
    // smallest entry's count since that entry could otherwise be dropped.
    size_t nrMissing = N, nrSurplus = 0;

    auto op = [&](const Assignment<Key>& a, double p) {
      // A full min_heap only takes values above its smallest one.
      if (nrMissing == 0 && p <= min_heap.top().first) return p;

      // A leaf covers every assignment of the keys missing from its path.
      // Iterate over the factor's keys rather than the tree's labels():
      // merging drops keys the values don't depend on from the tree,
      // but each leaf still stands for all of their assignments.
      // Counts beyond N don't change the threshold, so saturate at N before
      // multiplying to keep the count from overflowing.
      size_t nrAssignments = 1;
      for (auto&& [key, cardinality] : cardinalities_) {
        if (a.count(key) > 0) continue;
        nrAssignments = nrAssignments > N / cardinality
                            ? N
                            : nrAssignments * cardinality;
      }

      min_heap.emplace(p, nrAssignments);
      if (nrAssignments < nrMissing) {
        nrMissing -= nrAssignments;
        return p;
      }

      // The heap now covers N, with nrSurplus + nrPending to spare. Drop the
      // smallest entries while the rest still cover N. The new entry is never
      // dropped: each drop shrinks nrPending below its count.
      size_t nrPending = nrAssignments - nrMissing;
      nrMissing = 0;
      while (nrPending >= min_heap.top().second - nrSurplus) {
        nrPending -= min_heap.top().second - nrSurplus;
        nrSurplus = 0;
        min_heap.pop();
      }
      // Below the smallest entry's count by the loop condition, so no overflow.
      nrSurplus += nrPending;
      return p;
    };
    this->visitWith(op);

    // The factor has fewer than N assignments in total.
    if (nrMissing > 0) return 0.0;
    return min_heap.top().first;
  }

  /* ************************************************************************ */
  DiscreteFactor::shared_ptr DecisionTreeFactor::prune(
      size_t maxNrAssignments) const {
    // Keep every value at or above the N-th largest one. We don't stop after N
    // kept values: leaves are visited in tree order, not value order, so a cap
    // would drop larger values that come later. Values tied with the threshold
    // are all kept, so more than N assignments can survive.
    const double threshold = computeThreshold(maxNrAssignments);
    auto thresholdFunc = [threshold](const double& value) {
      return value < threshold ? 0.0 : value;
    };
    DecisionTree<Key, double> thresholded(*this, thresholdFunc);

    // Create pruned decision tree factor and return.
    return std::make_shared<DecisionTreeFactor>(this->discreteKeys(),
                                                thresholded);
  }

/* ************************************************************************ */
DiscreteFactor::shared_ptr DecisionTreeFactor::restrict(
    const DiscreteValues& assignment) const {
  ADT restricted_tree = ADT::restrict(assignment);
  // Get all the keys that are not restricted by the assignment, in order.
  // This ensures that the new restricted factor doesn't have keys
  // for which the information has been removed.
  DiscreteKeys restricted_keys;
  restricted_keys.reserve(keys_.size());
  for (Key key : keys_) {
    if (assignment.find(key) == assignment.end()) {
      restricted_keys.emplace_back(key, cardinality(key));
    }
  }
  // Create the restricted factor with the appropriate keys and tree.
  return std::make_shared<DecisionTreeFactor>(restricted_keys, restricted_tree);
}

  /* ************************************************************************ */
}  // namespace gtsam
