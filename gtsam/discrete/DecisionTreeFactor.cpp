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

#include <boost/make_shared.hpp>
#include <boost/format.hpp>
#include <utility>

using namespace std;

namespace gtsam {

  /* ************************************************************************ */
  DecisionTreeFactor::DecisionTreeFactor() {}

  /* ************************************************************************ */
  DecisionTreeFactor::DecisionTreeFactor(const DiscreteKeys& keys,
                                        const ADT& potentials)
      : DiscreteFactor(keys.indices()),
        ADT(potentials),
        cardinalities_(keys.cardinalities()) {}

  /* ************************************************************************ */
  DecisionTreeFactor::DecisionTreeFactor(const DiscreteConditional& c)
      : DiscreteFactor(c.keys()),
        AlgebraicDecisionTree<Key>(c),
        cardinalities_(c.cardinalities_) {}

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
    for (auto&& key : keys())
      cout << boost::format(" (%1%,%2%),") % formatter(key) % cardinality(key);
    cout << " ]" << endl;
    ADT::print("", formatter);
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
    for (const std::pair<const Key, size_t>& key : cs) keys.push_back(key);
    // apply operand
    ADT result = ADT::apply(f, op);
    // Make a new factor
    return DecisionTreeFactor(keys, result);
  }

  /* ************************************************************************ */
  DecisionTreeFactor::shared_ptr DecisionTreeFactor::combine(
      size_t nrFrontals, ADT::Binary op) const {
    if (nrFrontals > size())
      throw invalid_argument(
          (boost::format(
               "DecisionTreeFactor::combine: invalid number of frontal "
               "keys %d, nr.keys=%d") %
           nrFrontals % size())
              .str());

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
    return boost::make_shared<DecisionTreeFactor>(dkeys, result);
  }

  /* ************************************************************************ */
  DecisionTreeFactor::shared_ptr DecisionTreeFactor::combine(
      const Ordering& frontalKeys, ADT::Binary op) const {
    if (frontalKeys.size() > size())
      throw invalid_argument(
          (boost::format(
               "DecisionTreeFactor::combine: invalid number of frontal "
               "keys %d, nr.keys=%d") %
           frontalKeys.size() % size())
              .str());

    // sum over nrFrontals keys
    size_t i;
    ADT result(*this);
    for (i = 0; i < frontalKeys.size(); i++) {
      Key j = frontalKeys[i];
      result = result.combine(j, cardinality(j), op);
    }

    // create new factor, note we collect keys that are not in frontalKeys
    // TODO(frank): why do we need this??? result should contain correct keys!!!
    DiscreteKeys dkeys;
    for (i = 0; i < keys().size(); i++) {
      Key j = keys()[i];
      // TODO(frank): inefficient!
      if (std::find(frontalKeys.begin(), frontalKeys.end(), j) !=
          frontalKeys.end())
        continue;
      dkeys.push_back(DiscreteKey(j, cardinality(j)));
    }
    return boost::make_shared<DecisionTreeFactor>(dkeys, result);
  }

  /* ************************************************************************ */
  std::vector<std::pair<DiscreteValues, double>> DecisionTreeFactor::enumerate()
      const {
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
  DiscreteKeys DecisionTreeFactor::discreteKeys() const {
    DiscreteKeys result;
    for (auto&& key : keys()) {
      DiscreteKey dkey(key, cardinality(key));
      if (std::find(result.begin(), result.end(), dkey) == result.end()) {
        result.push_back(dkey);
      }
    }
    return result;
  }

  /* ************************************************************************ */
  static std::string valueFormatter(const double& v) {
    return (boost::format("%4.2g") % v).str();
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
      : DiscreteFactor(keys.indices()),
        AlgebraicDecisionTree<Key>(keys, table),
        cardinalities_(keys.cardinalities()) {}

  /* ************************************************************************ */
  DecisionTreeFactor::DecisionTreeFactor(const DiscreteKeys& keys,
                                        const string& table)
      : DiscreteFactor(keys.indices()),
        AlgebraicDecisionTree<Key>(keys, table),
        cardinalities_(keys.cardinalities()) {}

  /* ************************************************************************ */
  DecisionTreeFactor DecisionTreeFactor::prune(size_t maxNrAssignments) const {
    const size_t N = maxNrAssignments;

    // Get the probabilities in the decision tree so we can threshold.
    std::vector<double> probabilities;
    this->visitLeaf([&](const Leaf& leaf) {
      size_t nrAssignments = leaf.nrAssignments();
      double prob = leaf.constant();
      probabilities.insert(probabilities.end(), nrAssignments, prob);
    });

    // The number of probabilities can be lower than max_leaves
    if (probabilities.size() <= N) {
      return *this;
    }

    std::sort(probabilities.begin(), probabilities.end(),
              std::greater<double>{});

    double threshold = probabilities[N - 1];

    // Now threshold the decision tree
    auto thresholdFunc = [threshold](const double& value) {
      if (value < threshold) {
        return 0.0;
      } else {
        return value;
      }
    };
    DecisionTree<Key, double> thresholded(*this, thresholdFunc);

    // Create pruned decision tree factor and return.
    return DecisionTreeFactor(this->discreteKeys(), thresholded);
  }

  /* ************************************************************************ */
}  // namespace gtsam
