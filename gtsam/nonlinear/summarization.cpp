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

  // If we aren't eliminating anything, linearize and return
  if (!nrEliminatedKeys || saved_keys.empty()) {
    Ordering ordering = *values.orderingArbitrary();
    GaussianFactorGraph linear_graph = *graph.linearize(values, ordering);
    return make_pair(linear_graph, ordering);
  }

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

