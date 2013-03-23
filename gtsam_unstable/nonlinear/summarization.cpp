/**
 * @file summarization.cpp
 *
 * @date Jun 22, 2012
 * @author Alex Cunningham
 */

#include <gtsam/linear/GaussianSequentialSolver.h>
#include <gtsam_unstable/nonlinear/summarization.h>
#include <gtsam_unstable/linear/bayesTreeOperations.h>

using namespace std;

namespace gtsam {

/* ************************************************************************* */
GaussianFactorGraph::shared_ptr summarizeGraphSequential(
    const GaussianFactorGraph& full_graph, const std::vector<Index>& indices, bool useQR) {
  GaussianSequentialSolver solver(full_graph, useQR);
  return solver.jointFactorGraph(indices);
}

/* ************************************************************************* */
GaussianFactorGraph::shared_ptr summarizeGraphSequential(
    const GaussianFactorGraph& full_graph, const Ordering& ordering, const KeySet& saved_keys, bool useQR) {
  std::vector<Index> indices;
  BOOST_FOREACH(const Key& k, saved_keys)
    indices.push_back(ordering[k]);
  return summarizeGraphSequential(full_graph, indices, useQR);
}

/* ************************************************************************* */
std::pair<GaussianFactorGraph,Ordering>
partialCholeskySummarization(const NonlinearFactorGraph& graph, const Values& values,
    const KeySet& overlap_keys) {
  // compute a new ordering with non-saved keys first
  Ordering ordering;
  KeySet eliminatedKeys;
  BOOST_FOREACH(const Key& key, values.keys()) {
    if (!overlap_keys.count(key)) {
      eliminatedKeys.insert(key);
      ordering += key;
    }
  }

  BOOST_FOREACH(const Key& key, overlap_keys)
    ordering += key;

  // reorder the system
  GaussianFactorGraph linearSystem = *graph.linearize(values, ordering);

  // Eliminate frontals
  GaussianFactorGraph summarized_system;
  summarized_system.push_back(EliminateCholesky(linearSystem, eliminatedKeys.size()).second);
  return make_pair(summarized_system, ordering);
}

/* ************************************************************************* */
} // \namespace gtsam

