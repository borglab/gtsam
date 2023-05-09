/*
 * Domain.cpp
 * @brief Domain restriction constraint
 * @date Feb 13, 2012
 * @author Frank Dellaert
 */

#include <gtsam/base/Testable.h>
#include <gtsam/discrete/DecisionTreeFactor.h>
#include <gtsam_unstable/discrete/Domain.h>

#include <boost/make_shared.hpp>

namespace gtsam {

using namespace std;

/* ************************************************************************* */
void Domain::print(const string& s, const KeyFormatter& formatter) const {
  //    cout << s << ": Domain on " << formatter(keys_[0]) << " (j=" <<
  //    formatter(keys_[0]) << ") with values";
  //    for (size_t v: values_) cout << " " << v;
  //    cout << endl;
  for (size_t v : values_) cout << v;
}

/* ************************************************************************* */
double Domain::operator()(const Values& values) const {
  return contains(values.at(keys_[0]));
}

/* ************************************************************************* */
DecisionTreeFactor Domain::toDecisionTreeFactor() const {
  DiscreteKeys keys;
  keys += DiscreteKey(keys_[0], cardinality_);
  vector<double> table;
  for (size_t i1 = 0; i1 < cardinality_; ++i1) table.push_back(contains(i1));
  DecisionTreeFactor converted(keys, table);
  return converted;
}

/* ************************************************************************* */
DecisionTreeFactor Domain::operator*(const DecisionTreeFactor& f) const {
  // TODO: can we do this more efficiently?
  return toDecisionTreeFactor() * f;
}

/* ************************************************************************* */
bool Domain::ensureArcConsistency(size_t j, vector<Domain>& domains) const {
  if (j != keys_[0]) throw invalid_argument("Domain check on wrong domain");
  Domain& D = domains[j];
  for (size_t value : values_)
    if (!D.contains(value)) throw runtime_error("Unsatisfiable");
  D = *this;
  return true;
}

/* ************************************************************************* */
bool Domain::checkAllDiff(const KeyVector keys, vector<Domain>& domains) {
  Key j = keys_[0];
  // for all values in this domain
  for (size_t value : values_) {
    // for all connected domains
    for (Key k : keys)
      // if any domain contains the value we cannot make this domain singleton
      if (k != j && domains[k].contains(value)) goto found;
    values_.clear();
    values_.insert(value);
    return true;  // we changed it
  found:;
  }
  return false;  // we did not change it
}

/* ************************************************************************* */
Constraint::shared_ptr Domain::partiallyApply(const Values& values) const {
  Values::const_iterator it = values.find(keys_[0]);
  if (it != values.end() && !contains(it->second))
    throw runtime_error("Domain::partiallyApply: unsatisfiable");
  return boost::make_shared<Domain>(*this);
}

/* ************************************************************************* */
Constraint::shared_ptr Domain::partiallyApply(
    const vector<Domain>& domains) const {
  const Domain& Dk = domains[keys_[0]];
  if (Dk.isSingleton() && !contains(*Dk.begin()))
    throw runtime_error("Domain::partiallyApply: unsatisfiable");
  return boost::make_shared<Domain>(Dk);
}

/* ************************************************************************* */
}  // namespace gtsam
