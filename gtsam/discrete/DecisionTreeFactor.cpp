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
#include <gtsam/hybrid/HybridValues.h>
#include <gtsam/discrete/DecisionTreeFactor.h>
#include <gtsam/discrete/DiscreteConditional.h>

#include <utility>

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
        AlgebraicDecisionTree<Key>(c) {}

  /* ************************************************************************ */
  bool DecisionTreeFactor::equals(const DiscreteFactor& other,
                                  double tol) const {
    if (!dynamic_cast<const DecisionTreeFactor*>(&other)) {
      return false;
    } else {
      const auto& f(static_cast<const DecisionTreeFactor&>(other));
      return ADT::equals(f, tol);
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
  AlgebraicDecisionTree<Key> DecisionTreeFactor::error() const {
    // Get all possible assignments
    DiscreteKeys dkeys = discreteKeys();
    // Reverse to make cartesian product output a more natural ordering.
    DiscreteKeys rdkeys(dkeys.rbegin(), dkeys.rend());
    const auto assignments = DiscreteValues::CartesianProduct(rdkeys);

    // Construct vector with error values
    std::vector<double> errors;
    for (const auto& assignment : assignments) {
      errors.push_back(error(assignment));
    }
    return AlgebraicDecisionTree<Key>(dkeys, errors);
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
  DecisionTreeFactor DecisionTreeFactor::apply(ADT::Unary op) const {
    // apply operand
    ADT result = ADT::apply(op);
    // Make a new factor
    return DecisionTreeFactor(discreteKeys(), result);
  }

  /* ************************************************************************ */
  DecisionTreeFactor DecisionTreeFactor::apply(ADT::UnaryAssignment op) const {
    // apply operand
    ADT result = ADT::apply(op);
    // Make a new factor
    return DecisionTreeFactor(discreteKeys(), result);
  }

  /* ************************************************************************ */
  DecisionTreeFactor DecisionTreeFactor::apply(const DecisionTreeFactor& f,
                                              ADT::Binary op) const {
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
  DecisionTreeFactor::shared_ptr DecisionTreeFactor::combine(
      size_t nrFrontals, ADT::Binary op) const {
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
      Key j = keys()[i];
      result = result.combine(j, cardinality(j), op);
    }

    // create new factor, note we start keys after nrFrontals
    DiscreteKeys dkeys;
    for (; i < keys().size(); i++) {
      Key j = keys()[i];
      dkeys.push_back(DiscreteKey(j, cardinality(j)));
    }
    return std::make_shared<DecisionTreeFactor>(dkeys, result);
  }

  /* ************************************************************************ */
  DecisionTreeFactor::shared_ptr DecisionTreeFactor::combine(
      const Ordering& frontalKeys, ADT::Binary op) const {
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

    // create new factor, note we collect keys that are not in frontalKeys
    /*
    Due to branch merging, the labels in `result` may be missing some keys
    E.g. After branch merging, we may get a ADT like:
      Leaf [2] 1.0204082

    This is missing the key values used for branching.
    */
    KeyVector difference, frontalKeys_(frontalKeys), keys_(keys());
    // Get the difference of the frontalKeys and the factor keys using set_difference
    std::sort(keys_.begin(), keys_.end());
    std::sort(frontalKeys_.begin(), frontalKeys_.end());
    std::set_difference(keys_.begin(), keys_.end(), frontalKeys_.begin(),
                        frontalKeys_.end(), back_inserter(difference));

    DiscreteKeys dkeys;
    for (Key key : difference) {
      dkeys.push_back(DiscreteKey(key, cardinality(key)));
    }
    return std::make_shared<DecisionTreeFactor>(dkeys, result);
  }

  /* ************************************************************************ */
  std::vector<std::pair<DiscreteValues, double>> DecisionTreeFactor::enumerate()
      const {
    // Get all possible assignments
    DiscreteKeys pairs = discreteKeys();
    // Reverse to make cartesian product output a more natural ordering.
    DiscreteKeys rpairs(pairs.rbegin(), pairs.rend());
    const auto assignments = DiscreteValues::CartesianProduct(rpairs);

    // Construct unordered_map with values
    std::vector<std::pair<DiscreteValues, double>> result;
    for (const auto& assignment : assignments) {
      result.emplace_back(assignment, operator()(assignment));
    }
    return result;
  }

  /* ************************************************************************ */
  std::vector<double> DecisionTreeFactor::probabilities() const {
    // Set of all keys
    std::set<Key> allKeys(keys().begin(), keys().end());

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
      std::set<Key> assignment_keys;
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
    this->apply(op);

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

  /* ************************************************************************ */
  DecisionTreeFactor DecisionTreeFactor::prune(size_t maxNrAssignments) const {
    const size_t N = maxNrAssignments;

    // Get the probabilities in the decision tree so we can threshold.
    std::vector<double> probabilities = this->probabilities();

    // The number of probabilities can be lower than max_leaves
    if (probabilities.size() <= N) {
      return *this;
    }

    std::sort(probabilities.begin(), probabilities.end(),
              std::greater<double>{});

    double threshold = probabilities[N - 1];

    // Now threshold the decision tree
    size_t total = 0;
    auto thresholdFunc = [threshold, &total, N](const double& value) {
      if (value < threshold || total >= N) {
        return 0.0;
      } else {
        total += 1;
        return value;
      }
    };
    DecisionTree<Key, double> thresholded(*this, thresholdFunc);

    // Create pruned decision tree factor and return.
    return DecisionTreeFactor(this->discreteKeys(), thresholded);
  }

  /* ************************************************************************ */
}  // namespace gtsam
