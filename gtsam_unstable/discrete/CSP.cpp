/*
 * CSP.cpp
 * @brief Constraint Satisfaction Problem class
 * @date Feb 6, 2012
 * @author Frank Dellaert
 */

#include <gtsam/base/Testable.h>
#include <gtsam/discrete/DiscreteBayesNet.h>
#include <gtsam_unstable/discrete/CSP.h>
#include <gtsam_unstable/discrete/Domain.h>

using namespace std;

namespace gtsam {

bool CSP::runArcConsistency(const VariableIndex& index,
                            Domains* domains) const {
  bool changed = false;

  // iterate over all variables in the index
  for (auto entry : index) {
    // Get the variable's key and associated factors:
    const Key key = entry.first;
    const FactorIndices& factors = entry.second;

    // If this domain is already a singleton, we do nothing.
    if (domains->at(key).isSingleton()) continue;

    // Otherwise, loop over all factors/constraints for variable with given key.
    for (size_t f : factors) {
      // If this factor is a constraint, call its ensureArcConsistency method:
      auto constraint = boost::dynamic_pointer_cast<Constraint>((*this)[f]);
      if (constraint) {
        changed = constraint->ensureArcConsistency(key, domains) || changed;
      }
    }
  }
  return changed;
}

// TODO(dellaert): This is AC1, which is inefficient as any change will cause
// the algorithm to revisit *all* variables again. Implement AC3.
Domains CSP::runArcConsistency(size_t cardinality, size_t maxIterations) const {
  // Create VariableIndex
  VariableIndex index(*this);

  // Initialize domains
  Domains domains;
  for (auto entry : index) {
    const Key key = entry.first;
    domains.emplace(key, DiscreteKey(key, cardinality));
  }

  // Iterate until convergence or not a single domain changed.
  for (size_t it = 0; it < maxIterations; it++) {
    bool changed = runArcConsistency(index, &domains);
    if (!changed) break;
  }
  return domains;
}

CSP CSP::partiallyApply(const Domains& domains) const {
  // Create new problem with all singleton variables removed
  // We do this by adding simplifying all factors using partial application.
  // TODO: create a new ordering as we go, to ensure a connected graph
  // KeyOrdering ordering;
  // vector<Index> dkeys;
  CSP new_csp;

  // Add tightened domains as new factors:
  for (auto key_domain : domains) {
    new_csp.emplace_shared<Domain>(key_domain.second);
  }

  // Reduce all existing factors:
  for (const DiscreteFactor::shared_ptr& f : factors_) {
    auto constraint = boost::dynamic_pointer_cast<Constraint>(f);
    if (!constraint)
      throw runtime_error("CSP:runArcConsistency: non-constraint factor");
    Constraint::shared_ptr reduced = constraint->partiallyApply(domains);
    if (reduced->size() > 1) {
      new_csp.push_back(reduced);
    }
  }
  return new_csp;
}
}  // namespace gtsam
