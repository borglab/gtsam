/**
 * @file summarization.cpp
 *
 * @date Jun 22, 2012
 * @author Alex Cunningham
 */

#include <gtsam/linear/GaussianSequentialSolver.h>
#include <gtsam/nonlinear/summarization.h>
#include <gtsam/nonlinear/LinearContainerFactor.h>
//#include <gtsam_unstable/linear/bayesTreeOperations.h>

using namespace std;

namespace gtsam {

///* ************************************************************************* */
//GaussianFactorGraph::shared_ptr summarizeGraphSequential(
//    const GaussianFactorGraph& full_graph, const std::vector<Index>& indices, bool useQR) {
//  GaussianSequentialSolver solver(full_graph, useQR);
//  return solver.jointFactorGraph(indices);
//}
//
///* ************************************************************************* */
//GaussianFactorGraph::shared_ptr summarizeGraphSequential(
//    const GaussianFactorGraph& full_graph, const Ordering& ordering, const KeySet& saved_keys, bool useQR) {
//  std::vector<Index> indices;
//  BOOST_FOREACH(const Key& k, saved_keys)
//    indices.push_back(ordering[k]);
//  return summarizeGraphSequential(full_graph, indices, useQR);
//}

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

///* ************************************************************************* */
//std::pair<GaussianFactorGraph,Ordering>
//partialCholeskySummarization(const NonlinearFactorGraph& graph, const Values& values,
//    const KeySet& saved_keys) {
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
//
//  // reorder the system
//  GaussianFactorGraph linearSystem = *graph.linearize(values, ordering);
//
//  // Eliminate frontals
//  GaussianFactorGraph summarized_system;
//  summarized_system.push_back(EliminateCholesky(linearSystem, eliminatedKeys.size()).second);
//  return make_pair(summarized_system, ordering);
//}
//
//std::pair<GaussianFactorGraph,Ordering> GTSAM_EXPORT
//summarize(const NonlinearFactorGraph& graph, const Values& values,
//    const KeySet& saved_keys, bool useQR = true);


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

