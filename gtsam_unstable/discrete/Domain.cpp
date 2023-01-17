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
#include <sstream>
namespace gtsam {

using namespace std;

/* ************************************************************************* */
void Domain::print(const string& s, const KeyFormatter& formatter) const {
  cout << s << ": Domain on " << formatter(key()) << " (j=" << formatter(key())
       << ") with values";
  for (size_t v : values_) cout << " " << v;
  cout << endl;
}

/* ************************************************************************* */
string Domain::base1Str() const {
  stringstream ss;
  for (size_t v : values_) ss << v + 1;
  return ss.str();
}

/* ************************************************************************* */
double Domain::operator()(const DiscreteValues& values) const {
  return contains(values.at(key()));
}

/* ************************************************************************* */
DecisionTreeFactor Domain::toDecisionTreeFactor() const {
  const DiscreteKeys keys{DiscreteKey(key(), cardinality_)};
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
bool Domain::ensureArcConsistency(Key j, Domains* domains) const {
  if (j != key()) throw invalid_argument("Domain check on wrong domain");
  Domain& D = domains->at(j);
  for (size_t value : values_)
    if (!D.contains(value)) throw runtime_error("Unsatisfiable");
  D = *this;
  return true;
}

/* ************************************************************************* */
std::optional<Domain> Domain::checkAllDiff(const KeyVector keys,
                                             const Domains& domains) const {
  Key j = key();
  // for all values in this domain
  for (const size_t value : values_) {
    // for all connected domains
    for (const Key k : keys)
      // if any domain contains the value we cannot make this domain singleton
      if (k != j && domains.at(k).contains(value)) goto found;
    // Otherwise: return a singleton:
    return Domain(this->discreteKey(), value);
  found:;
  }
  return {};  // we did not change it
}

/* ************************************************************************* */
Constraint::shared_ptr Domain::partiallyApply(const DiscreteValues& values) const {
  DiscreteValues::const_iterator it = values.find(key());
  if (it != values.end() && !contains(it->second))
    throw runtime_error("Domain::partiallyApply: unsatisfiable");
  return std::make_shared<Domain>(*this);
}

/* ************************************************************************* */
Constraint::shared_ptr Domain::partiallyApply(const Domains& domains) const {
  const Domain& Dk = domains.at(key());
  if (Dk.isSingleton() && !contains(*Dk.begin()))
    throw runtime_error("Domain::partiallyApply: unsatisfiable");
  return std::make_shared<Domain>(Dk);
}

/* ************************************************************************* */
}  // namespace gtsam
