/**
 * @file bayesTreeOperations.cpp
 *
 * @date Jun 22, 2012
 * @author Alex Cunningham
 */

#include <gtsam_unstable/linear/bayesTreeOperations.h>

#include <boost/foreach.hpp>

using namespace std;

namespace gtsam {

/* ************************************************************************* */
std::set<Index> keysToIndices(const KeySet& keys, const Ordering& ordering) {
  std::set<Index> result;
  BOOST_FOREACH(const Key& key, keys)
    result.insert(ordering[key]);
  return result;
}

/* ************************************************************************* */
GaussianFactorGraph splitFactors(const GaussianFactorGraph& fullgraph) {
  GaussianFactorGraph result;
  BOOST_FOREACH(const GaussianFactor::shared_ptr& factor, fullgraph) {
      GaussianFactorGraph split = splitFactor(factor);
      result.push_back(split);
  }
  return result;
}

/* ************************************************************************* */
GaussianFactorGraph splitFactor(const GaussianFactor::shared_ptr& factor) {
  GaussianFactorGraph result;
  if (!factor) return result;

  // Needs to be a jacobian factor - just pass along hessians
  JacobianFactor::shared_ptr jf = boost::dynamic_pointer_cast<JacobianFactor>(factor);
  if (!jf) {
    result.push_back(factor);
    return result;
  }

  // Loop over variables and strip off factors using split conditionals
  // Assumes upper triangular structure
  JacobianFactor::const_iterator rowIt, colIt;
  const size_t totalRows = jf->rows();
  size_t rowsRemaining = totalRows;
  for (rowIt = jf->begin(); rowIt != jf->end(); ++rowIt) {
    // get dim of current variable
    size_t varDim = jf->getDim(rowIt);
    size_t startRow = totalRows - rowsRemaining;
    size_t nrRows = std::min(rowsRemaining, varDim);

    // Extract submatrices
    std::vector<std::pair<Index, Matrix> > matrices;
    for (colIt = rowIt; colIt != jf->end(); ++colIt) {
      Index idx = *colIt;
      const Matrix subA = jf->getA(colIt).middleRows(startRow, nrRows);
      matrices.push_back(make_pair(idx, subA));
    }

    Vector subB = jf->getb().segment(startRow, nrRows);
    Vector sigmas = jf->get_model()->sigmas().segment(startRow, nrRows);
    SharedDiagonal model;
    if (jf->get_model()->isConstrained())
      model = noiseModel::Constrained::MixedSigmas(sigmas);
    else
      model = noiseModel::Diagonal::Sigmas(sigmas);

    // extract matrices from each
    // assemble into new JacobianFactor
    result.add(matrices, subB, model);

    rowsRemaining -= nrRows;
  }
  return result;
}

/* ************************************************************************* */
GaussianFactorGraph removePriors(const GaussianFactorGraph& fullgraph) {
  GaussianFactorGraph result;
  BOOST_FOREACH(const GaussianFactor::shared_ptr& factor, fullgraph) {
    JacobianFactor::shared_ptr jf = boost::dynamic_pointer_cast<JacobianFactor>(factor);
    if (factor && (!jf || jf->size() > 1))
      result.push_back(factor);
  }
  return result;
}

/* ************************************************************************* */
void findCliques(const GaussianBayesTree::sharedClique& current_clique,
    std::set<GaussianConditional::shared_ptr>& result) {
  // Add the current clique
  result.insert(current_clique->conditional());

  // Add the parent if not root
  if (!current_clique->isRoot())
    findCliques(current_clique->parent(), result);
}

/* ************************************************************************* */
std::set<GaussianConditional::shared_ptr> findAffectedCliqueConditionals(
    const GaussianBayesTree& bayesTree, const std::set<Index>& savedIndices) {
  std::set<GaussianConditional::shared_ptr> affected_cliques;
  // FIXME: track previously found keys more efficiently
  BOOST_FOREACH(const Index& index, savedIndices) {
    GaussianBayesTree::sharedClique clique = bayesTree.nodes()[index];

    // add path back to root to affected set
    findCliques(clique, affected_cliques);
  }
  return affected_cliques;
}

/* ************************************************************************* */
std::deque<GaussianBayesTree::sharedClique>
findPathCliques(const GaussianBayesTree::sharedClique& initial) {
  std::deque<GaussianBayesTree::sharedClique> result, parents;
  if (initial->isRoot())
    return result;
  result.push_back(initial->parent());
  parents = findPathCliques(initial->parent());
  result.insert(result.end(), parents.begin(), parents.end());
  return result;
}

/* ************************************************************************* */
GaussianFactorGraph liquefy(const GaussianBayesTree::sharedClique& root, bool splitConditionals) {
  GaussianFactorGraph result;
  if (root && root->conditional()) {
    GaussianConditional::shared_ptr conditional = root->conditional();
    if (!splitConditionals)
      result.push_back(conditional->toFactor());
    else
      result.push_back(splitFactor(conditional->toFactor()));
  }
  BOOST_FOREACH(const GaussianBayesTree::sharedClique& child, root->children())
    result.push_back(liquefy(child, splitConditionals));
  return result;
}

/* ************************************************************************* */
GaussianFactorGraph liquefy(const GaussianBayesTree& bayesTree, bool splitConditionals) {
  return liquefy(bayesTree.root(), splitConditionals);
}

/* ************************************************************************* */
} // \namespace gtsam

