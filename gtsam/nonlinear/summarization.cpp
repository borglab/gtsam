/**
 * @file summarization.cpp
 *
 * @date Jun 22, 2012
 * @author Alex Cunningham
 */

#include <gtsam/linear/GaussianSequentialSolver.h>
#include <gtsam/nonlinear/summarization.h>
#include <gtsam/nonlinear/LinearContainerFactor.h>

using namespace std;

namespace gtsam {

/* ************************************************************************* */
std::pair<GaussianFactorGraph,Ordering>
summarize(const NonlinearFactorGraph& graph, const Values& values,
    const KeySet& saved_keys, SummarizationMode mode) {
  const size_t nrEliminatedKeys = values.size() - saved_keys.size();
  //  // compute a new ordering with non-saved keys first
  //  Ordering ordering;
  //  KeySet eliminatedKeys;
  //  BOOST_FOREACH(const Key& key, values.keys()) {
  //    if (!saved_keys.count(key)) {
  //      eliminatedKeys.insert(key);
  //      ordering += key;
  //    }
  //  }
  //
  //  BOOST_FOREACH(const Key& key, saved_keys)
  //    ordering += key;

  // Compute a constrained ordering with variables grouped to end
  std::map<gtsam::Key, int> ordering_constraints;

  // group all saved variables together
  BOOST_FOREACH(const gtsam::Key& key, saved_keys)
    ordering_constraints.insert(make_pair(key, 1));

  Ordering ordering = *graph.orderingCOLAMDConstrained(values, ordering_constraints);

  // Linearize the system
  GaussianFactorGraph full_graph = *graph.linearize(values, ordering);
  GaussianFactorGraph summarized_system;

  std::vector<Index> indices;
  BOOST_FOREACH(const Key& k, saved_keys)
    indices.push_back(ordering[k]);

  //  PARTIAL_QR = 0,         /// Uses QR solver to eliminate, does not require fully constrained system
  //  PARTIAL_CHOLESKY = 1,   /// Uses Cholesky solver, does not require fully constrained system
  //  SEQUENTIAL_QR = 2,      /// Uses QR to compute full joint graph (needs fully constrained system)
  //  SEQUENTIAL_CHOLESKY = 3 /// Uses Cholesky to compute full joint graph (needs fully constrained system)

  switch (mode) {
  case PARTIAL_QR: {
    summarized_system.push_back(EliminateQR(full_graph, nrEliminatedKeys).second);
    break;
  }
  case PARTIAL_CHOLESKY: {
    summarized_system.push_back(EliminateCholesky(full_graph, nrEliminatedKeys).second);
    break;
  }
  case SEQUENTIAL_QR: {
    GaussianSequentialSolver solver(full_graph, true);
    summarized_system.push_back(*solver.jointFactorGraph(indices));
    break;
  }
  case SEQUENTIAL_CHOLESKY: {
    GaussianSequentialSolver solver(full_graph, false);
    summarized_system.push_back(*solver.jointFactorGraph(indices));
    break;
  }
  }
  return make_pair(summarized_system, ordering);
}

/* ************************************************************************* */
NonlinearFactorGraph summarizeAsNonlinearContainer(
    const NonlinearFactorGraph& graph, const Values& values,
    const KeySet& saved_keys, SummarizationMode mode) {
  GaussianFactorGraph summarized_graph;
  Ordering ordering;
  boost::tie(summarized_graph, ordering) = summarize(graph, values, saved_keys, mode);
  return LinearContainerFactor::convertLinearGraph(summarized_graph, ordering);
}

/* ************************************************************************* */
} // \namespace gtsam

