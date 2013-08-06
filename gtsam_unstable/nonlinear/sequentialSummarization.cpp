/**
 * @file summarization.cpp
 *
 * @date Jun 22, 2012
 * @author Alex Cunningham
 */

#include <gtsam_unstable/nonlinear/sequentialSummarization.h>

#include <boost/foreach.hpp>

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
} // \namespace gtsam

