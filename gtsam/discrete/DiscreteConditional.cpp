/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 *  @file DiscreteConditional.cpp
 *  @date Feb 14, 2011
 *  @author Duy-Nguyen Ta
 *  @author Frank Dellaert
 */

#include <gtsam/base/Testable.h>
#include <gtsam/base/debug.h>
#include <gtsam/discrete/DiscreteConditional.h>
#include <gtsam/discrete/Signature.h>
#include <gtsam/hybrid/HybridValues.h>

#include <algorithm>
#include <random>
#include <set>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

using namespace std;
using std::pair;
using std::stringstream;
using std::vector;
namespace gtsam {

// Instantiate base class
template class GTSAM_EXPORT
    Conditional<DecisionTreeFactor, DiscreteConditional>;

/* ************************************************************************** */
DiscreteConditional::DiscreteConditional(const size_t nrFrontals,
                                         const DecisionTreeFactor& f)
    : BaseFactor(f / (*f.sum(nrFrontals))), BaseConditional(nrFrontals) {}

/* ************************************************************************** */
DiscreteConditional::DiscreteConditional(size_t nrFrontals,
                                         const DiscreteKeys& keys,
                                         const ADT& potentials)
    : BaseFactor(keys, potentials), BaseConditional(nrFrontals) {}

/* ************************************************************************** */
DiscreteConditional::DiscreteConditional(const DecisionTreeFactor& joint,
                                         const DecisionTreeFactor& marginal)
    : BaseFactor(joint / marginal),
      BaseConditional(joint.size() - marginal.size()) {}

/* ************************************************************************** */
DiscreteConditional::DiscreteConditional(const DecisionTreeFactor& joint,
                                         const DecisionTreeFactor& marginal,
                                         const Ordering& orderedKeys)
    : DiscreteConditional(joint, marginal) {
  keys_.clear();
  keys_.insert(keys_.end(), orderedKeys.begin(), orderedKeys.end());
}

/* ************************************************************************** */
DiscreteConditional::DiscreteConditional(const Signature& signature)
    : BaseFactor(signature.discreteKeys(), signature.cpt()),
      BaseConditional(1) {}

/* ************************************************************************** */
DiscreteConditional DiscreteConditional::operator*(
    const DiscreteConditional& other) const {
  // Take union of frontal keys
  std::set<Key> newFrontals;
  for (auto&& key : this->frontals()) newFrontals.insert(key);
  for (auto&& key : other.frontals()) newFrontals.insert(key);

  // Check if frontals overlapped
  if (nrFrontals() + other.nrFrontals() > newFrontals.size())
    throw std::invalid_argument(
        "DiscreteConditional::operator* called with overlapping frontal keys.");

  // Now, add cardinalities.
  DiscreteKeys discreteKeys;
  for (auto&& key : frontals())
    discreteKeys.emplace_back(key, cardinality(key));
  for (auto&& key : other.frontals())
    discreteKeys.emplace_back(key, other.cardinality(key));

  // Sort
  std::sort(discreteKeys.begin(), discreteKeys.end());

  // Add parents to set, to make them unique
  std::set<DiscreteKey> parents;
  for (auto&& key : this->parents())
    if (!newFrontals.count(key)) parents.emplace(key, cardinality(key));
  for (auto&& key : other.parents())
    if (!newFrontals.count(key)) parents.emplace(key, other.cardinality(key));

  // Finally, add parents to keys, in order
  for (auto&& dk : parents) discreteKeys.push_back(dk);

  ADT product = ADT::apply(other, ADT::Ring::mul);
  return DiscreteConditional(newFrontals.size(), discreteKeys, product);
}

/* ************************************************************************** */
DiscreteConditional DiscreteConditional::marginal(Key key) const {
  if (nrParents() > 0)
    throw std::invalid_argument(
        "DiscreteConditional::marginal: single argument version only valid for "
        "fully specified joint distributions (i.e., no parents).");

  // Calculate the keys as the frontal keys without the given key.
  DiscreteKeys discreteKeys{{key, cardinality(key)}};

  // Calculate sum
  ADT adt(*this);
  for (auto&& k : frontals())
    if (k != key) adt = adt.sum(k, cardinality(k));

  // Return new factor
  return DiscreteConditional(1, discreteKeys, adt);
}

/* ************************************************************************** */
void DiscreteConditional::print(const string& s,
                                const KeyFormatter& formatter) const {
  cout << s << " P( ";
  for (const_iterator it = beginFrontals(); it != endFrontals(); ++it) {
    cout << formatter(*it) << " ";
  }
  if (nrParents()) {
    cout << "| ";
    for (const_iterator it = beginParents(); it != endParents(); ++it) {
      cout << formatter(*it) << " ";
    }
  }
  cout << "):\n";
  ADT::print("", formatter);
  cout << endl;
}

/* ************************************************************************** */
bool DiscreteConditional::equals(const DiscreteFactor& other,
                                 double tol) const {
  if (!dynamic_cast<const DecisionTreeFactor*>(&other)) {
    return false;
  } else {
    const DecisionTreeFactor& f(static_cast<const DecisionTreeFactor&>(other));
    return DecisionTreeFactor::equals(f, tol);
  }
}

/* ************************************************************************** */
DiscreteConditional::ADT DiscreteConditional::choose(
    const DiscreteValues& given, bool forceComplete) const {
  // Get the big decision tree with all the levels, and then go down the
  // branches based on the value of the parent variables.
  DiscreteConditional::ADT adt(*this);
  size_t value;
  for (Key j : parents()) {
    try {
      value = given.at(j);
      adt = adt.choose(j, value);  // ADT keeps getting smaller.
    } catch (std::out_of_range&) {
      if (forceComplete) {
        given.print("parentsValues: ");
        throw runtime_error(
            "DiscreteConditional::choose: parent value missing");
      }
    }
  }
  return adt;
}

/* ************************************************************************** */
DiscreteConditional::shared_ptr DiscreteConditional::choose(
    const DiscreteValues& given) const {
  ADT adt = choose(given, false);  // P(F|S=given)

  // Collect all keys not in given.
  DiscreteKeys dKeys;
  for (Key j : frontals()) {
    dKeys.emplace_back(j, this->cardinality(j));
  }
  for (size_t i = nrFrontals(); i < size(); i++) {
    Key j = keys_[i];
    if (given.count(j) == 0) {
      dKeys.emplace_back(j, this->cardinality(j));
    }
  }
  return std::make_shared<DiscreteConditional>(nrFrontals(), dKeys, adt);
}

/* ************************************************************************** */
DecisionTreeFactor::shared_ptr DiscreteConditional::likelihood(
    const DiscreteValues& frontalValues) const {
  // Get the big decision tree with all the levels, and then go down the
  // branches based on the value of the frontal variables.
  ADT adt(*this);
  size_t value;
  for (Key j : frontals()) {
    try {
      value = frontalValues.at(j);
      adt = adt.choose(j, value);  // ADT keeps getting smaller.
    } catch (exception&) {
      frontalValues.print("frontalValues: ");
      throw runtime_error("DiscreteConditional::choose: frontal value missing");
    }
  }

  // Convert ADT to factor.
  DiscreteKeys discreteKeys;
  for (Key j : parents()) {
    discreteKeys.emplace_back(j, this->cardinality(j));
  }
  return std::make_shared<DecisionTreeFactor>(discreteKeys, adt);
}

/* ****************************************************************************/
DecisionTreeFactor::shared_ptr DiscreteConditional::likelihood(
    size_t frontal) const {
  if (nrFrontals() != 1)
    throw std::invalid_argument(
        "Single value likelihood can only be invoked on single-variable "
        "conditional");
  DiscreteValues values;
  values.emplace(keys_[0], frontal);
  return likelihood(values);
}

/* ************************************************************************** */
size_t DiscreteConditional::argmax(const DiscreteValues& parentsValues) const {
  ADT pFS = choose(parentsValues, true);  // P(F|S=parentsValues)

  // Initialize
  size_t maxValue = 0;
  double maxP = 0;
  DiscreteValues values = parentsValues;

  assert(nrFrontals() == 1);
  Key j = firstFrontalKey();
  for (size_t value = 0; value < cardinality(j); value++) {
    values[j] = value;
    double pValueS = (*this)(values);
    // Update MPE solution if better
    if (pValueS > maxP) {
      maxP = pValueS;
      maxValue = value;
    }
  }
  return maxValue;
}

/* ************************************************************************** */
void DiscreteConditional::sampleInPlace(DiscreteValues* values) const {
  assert(nrFrontals() == 1);
  Key j = (firstFrontalKey());
  size_t sampled = sample(*values);  // Sample variable given parents
  (*values)[j] = sampled;            // store result in partial solution
}

/* ************************************************************************** */
size_t DiscreteConditional::sample(const DiscreteValues& parentsValues) const {
  static mt19937 rng(2);  // random number generator

  // Get the correct conditional distribution
  ADT pFS = choose(parentsValues, true);  // P(F|S=parentsValues)

  // TODO(Duy): only works for one key now, seems horribly slow this way
  if (nrFrontals() != 1) {
    throw std::invalid_argument(
        "DiscreteConditional::sample can only be called on single variable "
        "conditionals");
  }
  Key key = firstFrontalKey();
  size_t nj = cardinality(key);
  vector<double> p(nj);
  DiscreteValues frontals;
  for (size_t value = 0; value < nj; value++) {
    frontals[key] = value;
    p[value] = pFS(frontals);  // P(F=value|S=parentsValues)
    if (p[value] == 1.0) {
      return value;  // shortcut exit
    }
  }
  std::discrete_distribution<size_t> distribution(p.begin(), p.end());
  return distribution(rng);
}

/* ************************************************************************** */
size_t DiscreteConditional::sample(size_t parent_value) const {
  if (nrParents() != 1)
    throw std::invalid_argument(
        "Single value sample() can only be invoked on single-parent "
        "conditional");
  DiscreteValues values;
  values.emplace(keys_.back(), parent_value);
  return sample(values);
}

/* ************************************************************************** */
size_t DiscreteConditional::sample() const {
  if (nrParents() != 0)
    throw std::invalid_argument(
        "sample() can only be invoked on no-parent prior");
  DiscreteValues values;
  return sample(values);
}

/* ************************************************************************* */
vector<DiscreteValues> DiscreteConditional::frontalAssignments() const {
  vector<pair<Key, size_t>> pairs;
  for (Key key : frontals()) pairs.emplace_back(key, cardinalities_.at(key));
  vector<pair<Key, size_t>> rpairs(pairs.rbegin(), pairs.rend());
  return DiscreteValues::CartesianProduct(rpairs);
}

/* ************************************************************************* */
vector<DiscreteValues> DiscreteConditional::allAssignments() const {
  vector<pair<Key, size_t>> pairs;
  for (Key key : parents()) pairs.emplace_back(key, cardinalities_.at(key));
  for (Key key : frontals()) pairs.emplace_back(key, cardinalities_.at(key));
  vector<pair<Key, size_t>> rpairs(pairs.rbegin(), pairs.rend());
  return DiscreteValues::CartesianProduct(rpairs);
}

/* ************************************************************************* */
// Print out signature.
static void streamSignature(const DiscreteConditional& conditional,
                            const KeyFormatter& keyFormatter,
                            stringstream* ss) {
  *ss << "P(";
  bool first = true;
  for (Key key : conditional.frontals()) {
    if (!first) *ss << ",";
    *ss << keyFormatter(key);
    first = false;
  }
  if (conditional.nrParents() > 0) {
    *ss << "|";
    bool first = true;
    for (Key parent : conditional.parents()) {
      if (!first) *ss << ",";
      *ss << keyFormatter(parent);
      first = false;
    }
  }
  *ss << "):";
}

/* ************************************************************************* */
std::string DiscreteConditional::markdown(const KeyFormatter& keyFormatter,
                                          const Names& names) const {
  stringstream ss;
  ss << " *";
  streamSignature(*this, keyFormatter, &ss);
  ss << "*\n" << std::endl;
  if (nrParents() == 0) {
    // We have no parents, call factor method.
    ss << DecisionTreeFactor::markdown(keyFormatter, names);
    return ss.str();
  }

  // Print out header.
  ss << "|";
  for (Key parent : parents()) {
    ss << "*" << keyFormatter(parent) << "*|";
  }

  auto frontalAssignments = this->frontalAssignments();
  for (const auto& a : frontalAssignments) {
    for (auto&& it = beginFrontals(); it != endFrontals(); ++it) {
      size_t index = a.at(*it);
      ss << DiscreteValues::Translate(names, *it, index);
    }
    ss << "|";
  }
  ss << "\n";

  // Print out separator with alignment hints.
  ss << "|";
  size_t n = frontalAssignments.size();
  for (size_t j = 0; j < nrParents() + n; j++) ss << ":-:|";
  ss << "\n";

  // Print out all rows.
  size_t count = 0;
  for (const auto& a : allAssignments()) {
    if (count == 0) {
      ss << "|";
      for (auto&& it = beginParents(); it != endParents(); ++it) {
        size_t index = a.at(*it);
        ss << DiscreteValues::Translate(names, *it, index) << "|";
      }
    }
    ss << operator()(a) << "|";
    count = (count + 1) % n;
    if (count == 0) ss << "\n";
  }
  return ss.str();
}

/* ************************************************************************ */
string DiscreteConditional::html(const KeyFormatter& keyFormatter,
                                 const Names& names) const {
  stringstream ss;
  ss << "<div>\n<p>  <i>";
  streamSignature(*this, keyFormatter, &ss);
  ss << "</i></p>\n";
  if (nrParents() == 0) {
    // We have no parents, call factor method.
    ss << DecisionTreeFactor::html(keyFormatter, names);
    return ss.str();
  }

  // Print out preamble.
  ss << "<table class='DiscreteConditional'>\n  <thead>\n";

  // Print out header row.
  ss << "    <tr>";
  for (Key parent : parents()) {
    ss << "<th><i>" << keyFormatter(parent) << "</i></th>";
  }
  auto frontalAssignments = this->frontalAssignments();
  for (const auto& a : frontalAssignments) {
    ss << "<th>";
    for (auto&& it = beginFrontals(); it != endFrontals(); ++it) {
      size_t index = a.at(*it);
      ss << DiscreteValues::Translate(names, *it, index);
    }
    ss << "</th>";
  }
  ss << "</tr>\n";

  // Finish header and start body.
  ss << "  </thead>\n  <tbody>\n";

  // Output all rows, one per assignment:
  size_t count = 0, n = frontalAssignments.size();
  for (const auto& a : allAssignments()) {
    if (count == 0) {
      ss << "    <tr>";
      for (auto&& it = beginParents(); it != endParents(); ++it) {
        size_t index = a.at(*it);
        ss << "<th>" << DiscreteValues::Translate(names, *it, index) << "</th>";
      }
    }
    ss << "<td>" << operator()(a) << "</td>";  // value
    count = (count + 1) % n;
    if (count == 0) ss << "</tr>\n";
  }

  // Finish up
  ss << "  </tbody>\n</table>\n</div>";
  return ss.str();
}

/* ************************************************************************* */
double DiscreteConditional::evaluate(const HybridValues& x) const {
  return this->evaluate(x.discrete());
}
/* ************************************************************************* */

}  // namespace gtsam
