/*
 * CSP.cpp
 * @brief Constraint Satisfaction Problem class
 * @date Feb 6, 2012
 * @author Frank Dellaert
 */

#include <gtsam/base/Testable.h>
#include <gtsam_unstable/discrete/CSP.h>
#include <gtsam_unstable/discrete/Domain.h>

using namespace std;

namespace gtsam {

/// Find the best total assignment - can be expensive
CSP::sharedValues CSP::optimalAssignment() const {
  DiscreteBayesNet::shared_ptr chordal = this->eliminateSequential();
  sharedValues mpe = chordal->optimize();
  return mpe;
}

/// Find the best total assignment - can be expensive
CSP::sharedValues CSP::optimalAssignment(const Ordering& ordering) const {
  DiscreteBayesNet::shared_ptr chordal = this->eliminateSequential(ordering);
  sharedValues mpe = chordal->optimize();
  return mpe;
}

void CSP::runArcConsistency(size_t cardinality, size_t nrIterations,
                            bool print) const {
  // Create VariableIndex
  VariableIndex index(*this);
  // index.print();

  size_t n = index.size();

  // Initialize domains
  std::vector<Domain> domains;
  for (size_t j = 0; j < n; j++)
    domains.push_back(Domain(DiscreteKey(j, cardinality)));

  // Create array of flags indicating a domain changed or not
  std::vector<bool> changed(n);

  // iterate nrIterations over entire grid
  for (size_t it = 0; it < nrIterations; it++) {
    bool anyChange = false;
    // iterate over all cells
    for (size_t v = 0; v < n; v++) {
      // keep track of which domains changed
      changed[v] = false;
      // loop over all factors/constraints for variable v
      const FactorIndices& factors = index[v];
      for (size_t f : factors) {
        // if not already a singleton
        if (!domains[v].isSingleton()) {
          // get the constraint and call its ensureArcConsistency method
          Constraint::shared_ptr constraint =
              boost::dynamic_pointer_cast<Constraint>((*this)[f]);
          if (!constraint)
            throw runtime_error("CSP:runArcConsistency: non-constraint factor");
          changed[v] =
              constraint->ensureArcConsistency(v, domains) || changed[v];
        }
      }  // f
      if (changed[v]) anyChange = true;
    }  // v
    if (!anyChange) break;
    // TODO: Sudoku specific hack
    if (print) {
      if (cardinality == 9 && n == 81) {
        for (size_t i = 0, v = 0; i < (size_t)std::sqrt((double)n); i++) {
          for (size_t j = 0; j < (size_t)std::sqrt((double)n); j++, v++) {
            if (changed[v]) cout << "*";
            domains[v].print();
            cout << "\t";
          }  // i
          cout << endl;
        }  // j
      } else {
        for (size_t v = 0; v < n; v++) {
          if (changed[v]) cout << "*";
          domains[v].print();
          cout << "\t";
        }  // v
      }
      cout << endl;
    }  // print
  }    // it

#ifndef INPROGRESS
  // Now create new problem with all singleton variables removed
  // We do this by adding simplifying all factors using parial application
  // TODO: create a new ordering as we go, to ensure a connected graph
  // KeyOrdering ordering;
  // vector<Index> dkeys;
  for (const DiscreteFactor::shared_ptr& f : factors_) {
    Constraint::shared_ptr constraint =
        boost::dynamic_pointer_cast<Constraint>(f);
    if (!constraint)
      throw runtime_error("CSP:runArcConsistency: non-constraint factor");
    Constraint::shared_ptr reduced = constraint->partiallyApply(domains);
    if (print) reduced->print();
  }
#endif
}
}  // namespace gtsam
