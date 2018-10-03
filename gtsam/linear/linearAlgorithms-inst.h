/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    linearAlgorithms-inst.h
 * @brief   Templated algorithms that are used in multiple places in linear
 * @author  Richard Roberts
 */

#pragma once

#include <gtsam/linear/GaussianConditional.h>
#include <gtsam/linear/VectorValues.h>
#include <gtsam/base/treeTraversal-inst.h>

#include <boost/optional.hpp>
#include <boost/shared_ptr.hpp>

namespace gtsam {
namespace internal {
namespace linearAlgorithms {

/* ************************************************************************* */
struct OptimizeData {
  boost::optional<OptimizeData&> parentData;
  FastMap<Key, VectorValues::const_iterator> cliqueResults;
};

/* ************************************************************************* */
/** Pre-order visitor for back-substitution in a Bayes tree.  The visitor
 * function operator()() optimizes the clique given the solution for the
 * parents, and returns the solution for the clique's frontal variables.  In
 * addition, it adds the solution to a global collected solution that will
 * finally be returned to the user.  The reason we pass the individual clique
 * solutions between nodes is to avoid log(n) lookups over all variables, they
 * instead then are only over a node's parent variables. */
template <class CLIQUE>
struct OptimizeClique {
  VectorValues collectedResult;

  OptimizeData operator()(const boost::shared_ptr<CLIQUE>& clique,
                          OptimizeData& parentData) {
    OptimizeData myData;
    myData.parentData = parentData;
    // Take any ancestor results we'll need
    for (Key parent : clique->conditional_->parents())
      myData.cliqueResults.emplace(parent,
                                   myData.parentData->cliqueResults.at(parent));

    // Solve and store in our results
    {
      const GaussianConditional& cg = *clique->conditional();
      // Solve matrix
      Vector xS;
      {
        // Count dimensions of vector
        DenseIndex dim = 0;
        FastVector<VectorValues::const_iterator> parentPointers;
        parentPointers.reserve(cg.nrParents());
        for (Key parent : cg.parents()) {
          parentPointers.push_back(myData.cliqueResults.at(parent));
          dim += parentPointers.back()->second.size();
        }

        // Fill parent vector
        xS.resize(dim);
        DenseIndex vectorPos = 0;
        for (const auto& it : parentPointers) {
          const Vector& parentVector = it->second;
          xS.block(vectorPos, 0, parentVector.size(), 1) =
              parentVector.block(0, 0, parentVector.size(), 1);
          vectorPos += parentVector.size();
        }
      }

      // NOTE(gareth): We can no longer write: xS = b - S * xS
      // This is because Eigen (as of 3.3) no longer evaluates S * xS into
      // a temporary, and the operation trashes valus in xS.
      // See: http://eigen.tuxfamily.org/index.php?title=3.3
      const Vector rhs = cg.getb() - cg.get_S() * xS;

      // TODO(gareth): Inline instantiation of Eigen::Solve and check flag
      const Vector solution =
          cg.get_R().triangularView<Eigen::Upper>().solve(rhs);

      // Check for indeterminant solution
      if (solution.hasNaN())
        throw IndeterminantLinearSystemException(cg.keys().front());

      // Insert solution into a VectorValues
      DenseIndex vectorPosition = 0;
      for (GaussianConditional::const_iterator it = cg.beginFrontals();
           it != cg.endFrontals(); ++it) {
        const auto dim = cg.getDim(it);
        VectorValues::const_iterator r =
            collectedResult.insert(*it, solution.segment(vectorPosition, dim));
        myData.cliqueResults.insert(make_pair(r->first, r));
        vectorPosition += dim;
      }
    }
    return myData;
  }
};

/* ************************************************************************* */
template <class BAYESTREE>
VectorValues optimizeBayesTree(const BAYESTREE& bayesTree) {
  gttic(linear_optimizeBayesTree);
  OptimizeData rootData;
  OptimizeClique<typename BAYESTREE::Clique> preVisitor;
  treeTraversal::no_op postVisitor;
  TbbOpenMPMixedScope
      threadLimiter;  // Limits OpenMP threads since we're mixing TBB and OpenMP
  treeTraversal::DepthFirstForestParallel(bayesTree, rootData, preVisitor,
                                          postVisitor);
  return preVisitor.collectedResult;
}
}  // namespace linearAlgorithms
}  // namespace internal
}  // namespace gtsam
