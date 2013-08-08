/**
 * @file summarization.cpp
 *
 * @date Jun 22, 2012
 * @author Alex Cunningham
 */

#include <gtsam_unstable/linear/conditioning.h>
#include <gtsam_unstable/linear/bayesTreeOperations.h>

#include <boost/assign/std/vector.hpp>

using namespace std;
using namespace boost::assign;

namespace gtsam {

/* ************************************************************************* */
GaussianConditional::shared_ptr conditionDensity(const GaussianConditional::shared_ptr& initConditional,
    const std::set<Key>& saved_indices, const VectorValues& solution) {
  const bool verbose = false;

  if (!initConditional)
    return initConditional;

  if (verbose) {
    cout << "backsubSummarize() Starting" << endl;
    initConditional->print("Full initial conditional");
  }

  // Check for presence of variables to remove
  std::set<Key> frontalsToRemove, parentsToRemove;
  BOOST_FOREACH(const Key& frontal, initConditional->frontals())
    if (!saved_indices.count(frontal))
      frontalsToRemove.insert(frontal);

  BOOST_FOREACH(const Key& parent, initConditional->parents())
    if (!saved_indices.count(parent))
      parentsToRemove.insert(parent);

  // If all variables in this conditional are to be saved, just return initial conditional
  if (frontalsToRemove.empty() && parentsToRemove.empty())
    return initConditional;

  // If none of the frontal variables are to be saved, return empty pointer
  if (frontalsToRemove.size() == initConditional->nrFrontals())
    return GaussianConditional::shared_ptr();

  // Collect dimensions of the new conditional
  if (verbose) cout << "  Collecting dimensions" << endl;
  size_t newTotalRows = 0, newTotalCols = 1; // Need spacing for RHS
  size_t newNrFrontals = 0;
  size_t oldOffset = 0;
  vector<size_t> newDims, oldDims;
  vector<size_t> oldColOffsets;
  vector<Key> newIndices;
  vector<size_t> newIdxToOldIdx; // Access to arrays, maps from new var to old var
  const vector<Key>& oldIndices = initConditional->keys();
  const size_t oldNrFrontals = initConditional->nrFrontals();
  GaussianConditional::const_iterator varIt = initConditional->beginFrontals();
  size_t oldIdx = 0;
  for (; varIt != initConditional->endFrontals(); ++varIt) {
    size_t varDim = initConditional->getDim(varIt);
    oldDims += varDim;
    if (!frontalsToRemove.count(*varIt)) {
      newTotalCols += varDim;
      newTotalRows += varDim;
      newDims += varDim;
      newIndices += *varIt;
      ++newNrFrontals;
      newIdxToOldIdx += oldIdx;
      oldColOffsets += oldOffset;
    }
    oldOffset += varDim;
    ++oldIdx;
  }
  varIt = initConditional->beginParents();
  for (; varIt != initConditional->endParents(); ++varIt) {
    size_t varDim = initConditional->getDim(varIt);
    oldDims += varDim;
    if (!parentsToRemove.count(*varIt)) {
      newTotalCols += varDim;
      newDims += varDim;
      newIndices += *varIt;
    }
  }
  newDims += 1; // For the RHS

  // Initialize new conditional
  Matrix full_matrix = Matrix::Zero(newTotalRows, newTotalCols);
  Vector sigmas = zero(newTotalRows);
  if (verbose) cout << "  Initializing new conditional\nfull_matrix:\n" << full_matrix << endl;

  // Fill in full matrix - iterate over rows for each sub-conditional
  const size_t oldRNrCols = initConditional->get_R().cols();
  size_t newColOffset = 0;
  for (size_t newfrontalIdx=0; newfrontalIdx<newNrFrontals; ++newfrontalIdx) {
    const size_t& dim = newDims.at(newfrontalIdx);
    if (verbose) cout << "     Filling in Matrix: newfrontalIdx " << newfrontalIdx
        << " frontalKey: " << newIndices[newfrontalIdx] << " dim: " << dim << endl;

    size_t oldColOffset = oldColOffsets.at(newfrontalIdx);

    // get R block, sliced by row
    Eigen::Block<GaussianConditional::constABlock> rblock =
        initConditional->get_R().block(oldColOffset, oldColOffset, dim, oldRNrCols-oldColOffset);
    if (verbose) cout << "   rblock\n" << rblock << endl;


    // set the R matrix for this var
    full_matrix.block(newColOffset, newColOffset, dim, dim) = rblock.leftCols(dim);
    if (verbose) cout << "   full_matrix: set R\n" << full_matrix << endl;

    // set RHS
    full_matrix.block(newColOffset, newTotalCols-1, dim, 1) = initConditional->get_d().segment(oldColOffset, dim);
    if (verbose) cout << "   full_matrix: set rhs\n" << full_matrix << endl;

    // set sigmas
    sigmas.segment(newColOffset, dim) = initConditional->get_model()->sigmas().segment(oldColOffset, dim);

    // add parents in R block, while updating rhs
    // Loop through old variable list
    size_t newParentStartCol = newColOffset + dim;
    size_t oldParentStartCol = dim; // Copying from Rblock - offset already accounted for
    for (size_t oldIdx = newIdxToOldIdx[newfrontalIdx]+1; oldIdx<oldNrFrontals; ++oldIdx) {
      Key parentKey = oldIndices[oldIdx];
      size_t parentDim = oldDims[oldIdx];
      if (verbose) cout << "   Adding parents from R: parentKey: " << parentKey << " parentDim: " << parentDim << endl;
      if (!frontalsToRemove.count(parentKey)) {
        if (verbose) {
          cout << "         Copying block (from): oldParentStartCol " << oldParentStartCol << endl;
          cout << "         Copying block   (to): newColOffset " << newColOffset << ", newParentStartCol " << newParentStartCol << endl;
        }
        full_matrix.block(newColOffset, newParentStartCol, dim, parentDim)
            = rblock.middleCols(oldParentStartCol, parentDim);
        newParentStartCol += parentDim;
        if (verbose) cout << "      full_matrix: add parent from R\n" << full_matrix << endl;
      } else {
        const Vector& parentSol = solution.at(parentKey);
        full_matrix.block(newColOffset, newTotalCols-1, dim, 1) -=
            rblock.middleCols(oldParentStartCol, parentDim) * parentSol;
        if (verbose) cout << "      full_matrix: update rhs from parent in R\n" << full_matrix << endl;
      }
      oldParentStartCol += parentDim;
    }

    // add parents (looping over original block structure), while updating rhs
    GaussianConditional::const_iterator oldParent = initConditional->beginParents();
    for (; oldParent != initConditional->endParents(); ++oldParent) {
      Key parentKey = *oldParent;
      size_t parentDim = initConditional->getDim(oldParent);
      if (verbose) cout << "   Adding parents from S: parentKey: " << parentKey << " parentDim: " << parentDim << endl;
      if (parentsToRemove.count(parentKey)) {
        // Solve out the variable
        const Vector& parentSol = solution.at(parentKey);
        assert((size_t)parentSol.size() == parentDim);
        full_matrix.block(newColOffset, newTotalCols-1, dim, 1) -=
          initConditional->get_S(oldParent).middleRows(oldColOffset, dim) * parentSol;
        if (verbose) cout << "      full_matrix: update rhs from parent in S\n" << full_matrix << endl;
      } else {
        // Copy the matrix block
        full_matrix.block(newColOffset, newParentStartCol, dim, parentDim) =
            initConditional->get_S(oldParent).middleRows(oldColOffset, dim);
        if (verbose) cout << "      full_matrix: add parent from S\n" << full_matrix << endl;
      }
    }

    // Increment the rows
    newColOffset += dim;
    oldColOffset += dim;
  }

  // Construct a new conditional
  if (verbose) cout << "backsubSummarize() Complete!" << endl;
//  GaussianConditional::rsd_type matrices(full_matrix, newDims.begin(), newDims.end());
  VerticalBlockMatrix matrices(newDims, full_matrix);
  return GaussianConditional::shared_ptr(new
      GaussianConditional(newIndices, newNrFrontals, matrices, noiseModel::Diagonal::Sigmas(sigmas)));
}

/* ************************************************************************* */
GaussianFactorGraph conditionDensity(const GaussianBayesTree& bayesTree,
    const std::set<Key>& saved_indices) {
  const bool verbose = false;

  VectorValues solution = bayesTree.optimize();

  // FIXME: set of conditionals does not manage possibility of solving out whole separators
  std::set<GaussianConditional::shared_ptr> affected_cliques = findAffectedCliqueConditionals(bayesTree, saved_indices);

  // Summarize conditionals separately
  GaussianFactorGraph summarized_graph;
  BOOST_FOREACH(const GaussianConditional::shared_ptr& conditional, affected_cliques) {
    if (verbose) conditional->print("Initial conditional");
    GaussianConditional::shared_ptr reducedConditional = conditionDensity(conditional, saved_indices, solution);
    if (reducedConditional) {
      if (verbose) reducedConditional->print("Final conditional");
      summarized_graph.push_back(reducedConditional);
    } else if (verbose) {
      cout << "Conditional removed after summarization!" << endl;
    }
  }
  return summarized_graph;
}

/* ************************************************************************* */

} // \namespace gtsam

