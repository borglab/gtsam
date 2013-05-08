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
    const KeySet& saved_keys, bool useQR) {
  // compute a new ordering with non-saved keys first
  Ordering ordering;
  KeySet eliminatedKeys;
  BOOST_FOREACH(const Key& key, values.keys()) {
    if (!saved_keys.count(key)) {
      eliminatedKeys.insert(key);
      ordering += key;
    }
  }

  BOOST_FOREACH(const Key& key, saved_keys)
    ordering += key;

  // Linearize the system
  GaussianFactorGraph full_graph = *graph.linearize(values, ordering);
  GaussianFactorGraph summarized_system;
  if (useQR)
    summarized_system.push_back(EliminateQR(full_graph, eliminatedKeys.size()).second);
  else
    summarized_system.push_back(EliminateCholesky(full_graph, eliminatedKeys.size()).second);

  return make_pair(summarized_system, ordering);
}

/* ************************************************************************* */
NonlinearFactorGraph summarizeAsNonlinearContainer(
    const NonlinearFactorGraph& graph, const Values& values,
    const KeySet& saved_keys, bool useQR) {
  GaussianFactorGraph summarized_graph;
  Ordering ordering;
  boost::tie(summarized_graph, ordering) = summarize(graph, values, saved_keys, useQR);
  return LinearContainerFactor::convertLinearGraph(summarized_graph, ordering);
}

/* ************************************************************************* */
} // \namespace gtsam

