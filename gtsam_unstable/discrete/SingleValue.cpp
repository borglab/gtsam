/*
 * SingleValue.cpp
 * @brief domain constraint
 * @date Feb 13, 2012
 * @author Frank Dellaert
 */

#include <gtsam/base/Testable.h>
#include <gtsam/discrete/DecisionTreeFactor.h>
#include <gtsam_unstable/discrete/Domain.h>
#include <gtsam_unstable/discrete/SingleValue.h>

#include <boost/make_shared.hpp>

namespace gtsam {

using namespace std;

/* ************************************************************************* */
void SingleValue::print(const string& s, const KeyFormatter& formatter) const {
  cout << s << "SingleValue on "
       << "j=" << formatter(keys_[0]) << " with value " << value_ << endl;
}

/* ************************************************************************* */
double SingleValue::operator()(const DiscreteValues& values) const {
  return (double)(values.at(keys_[0]) == value_);
}

/* ************************************************************************* */
DecisionTreeFactor SingleValue::toDecisionTreeFactor() const {
  DiscreteKeys keys;
  keys += DiscreteKey(keys_[0], cardinality_);
  vector<double> table;
  for (size_t i1 = 0; i1 < cardinality_; i1++) table.push_back(i1 == value_);
  DecisionTreeFactor converted(keys, table);
  return converted;
}

/* ************************************************************************* */
DecisionTreeFactor SingleValue::operator*(const DecisionTreeFactor& f) const {
  // TODO: can we do this more efficiently?
  return toDecisionTreeFactor() * f;
}

/* ************************************************************************* */
bool SingleValue::ensureArcConsistency(Key j, Domains* domains) const {
  if (j != keys_[0])
    throw invalid_argument("SingleValue check on wrong domain");
  Domain& D = domains->at(j);
  if (D.isSingleton()) {
    if (D.firstValue() != value_) throw runtime_error("Unsatisfiable");
    return false;
  }
  D = Domain(discreteKey(), value_);
  return true;
}

/* ************************************************************************* */
Constraint::shared_ptr SingleValue::partiallyApply(const DiscreteValues& values) const {
  DiscreteValues::const_iterator it = values.find(keys_[0]);
  if (it != values.end() && it->second != value_)
    throw runtime_error("SingleValue::partiallyApply: unsatisfiable");
  return boost::make_shared<SingleValue>(keys_[0], cardinality_, value_);
}

/* ************************************************************************* */
Constraint::shared_ptr SingleValue::partiallyApply(
    const Domains& domains) const {
  const Domain& Dk = domains.at(keys_[0]);
  if (Dk.isSingleton() && !Dk.contains(value_))
    throw runtime_error("SingleValue::partiallyApply: unsatisfiable");
  return boost::make_shared<SingleValue>(discreteKey(), value_);
}

/* ************************************************************************* */
}  // namespace gtsam
