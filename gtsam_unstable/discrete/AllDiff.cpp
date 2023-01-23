/*
 * AllDiff.cpp
 * @brief General "all-different" constraint
 * @date Feb 6, 2012
 * @author Frank Dellaert
 */

#include <gtsam/base/Testable.h>
#include <gtsam_unstable/discrete/AllDiff.h>
#include <gtsam_unstable/discrete/Domain.h>

#include <optional>

namespace gtsam {

/* ************************************************************************* */
AllDiff::AllDiff(const DiscreteKeys& dkeys) : Constraint(dkeys.indices()) {
  for (const DiscreteKey& dkey : dkeys) cardinalities_.insert(dkey);
}

/* ************************************************************************* */
void AllDiff::print(const std::string& s, const KeyFormatter& formatter) const {
  std::cout << s << "AllDiff on ";
  for (Key dkey : keys_) std::cout << formatter(dkey) << " ";
  std::cout << std::endl;
}

/* ************************************************************************* */
double AllDiff::operator()(const DiscreteValues& values) const {
  std::set<size_t> taken;  // record values taken by keys
  for (Key dkey : keys_) {
    size_t value = values.at(dkey);      // get the value for that key
    if (taken.count(value)) return 0.0;  // check if value alreday taken
    taken.insert(value);  // if not, record it as taken and keep checking
  }
  return 1.0;
}

/* ************************************************************************* */
DecisionTreeFactor AllDiff::toDecisionTreeFactor() const {
  // We will do this by converting the allDif into many BinaryAllDiff
  // constraints
  DecisionTreeFactor converted;
  size_t nrKeys = keys_.size();
  for (size_t i1 = 0; i1 < nrKeys; i1++)
    for (size_t i2 = i1 + 1; i2 < nrKeys; i2++) {
      BinaryAllDiff binary12(discreteKey(i1), discreteKey(i2));
      converted = converted * binary12.toDecisionTreeFactor();
    }
  return converted;
}

/* ************************************************************************* */
DecisionTreeFactor AllDiff::operator*(const DecisionTreeFactor& f) const {
  // TODO: can we do this more efficiently?
  return toDecisionTreeFactor() * f;
}

/* ************************************************************************* */
bool AllDiff::ensureArcConsistency(Key j, Domains* domains) const {
  Domain& Dj = domains->at(j);

  // Though strictly not part of allDiff, we check for
  // a value in domains->at(j) that does not occur in any other connected domain.
  // If found, we make this a singleton...
  // TODO: make a new constraint where this really is true
  std::optional<Domain> maybeChanged = Dj.checkAllDiff(keys_, *domains);
  if (maybeChanged) {
    Dj = *maybeChanged;
    return true;
  }

  // Check all other domains for singletons and erase corresponding values.
  // This is the same as arc-consistency on the equivalent binary constraints
  bool changed = false;
  for (Key k : keys_)
    if (k != j) {
      const Domain& Dk = domains->at(k);
      if (Dk.isSingleton()) {  // check if singleton
        size_t value = Dk.firstValue();
        if (Dj.contains(value)) {
          Dj.erase(value);  // erase value if true
          changed = true;
        }
      }
    }
  return changed;
}

/* ************************************************************************* */
Constraint::shared_ptr AllDiff::partiallyApply(const DiscreteValues& values) const {
  DiscreteKeys newKeys;
  // loop over keys and add them only if they do not appear in values
  for (Key k : keys_)
    if (values.find(k) == values.end()) {
      newKeys.push_back(DiscreteKey(k, cardinalities_.at(k)));
    }
  return std::make_shared<AllDiff>(newKeys);
}

/* ************************************************************************* */
Constraint::shared_ptr AllDiff::partiallyApply(
    const Domains& domains) const {
  DiscreteValues known;
  for (Key k : keys_) {
    const Domain& Dk = domains.at(k);
    if (Dk.isSingleton()) known[k] = Dk.firstValue();
  }
  return partiallyApply(known);
}

/* ************************************************************************* */
}  // namespace gtsam
