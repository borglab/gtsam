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

#include <gtsam/base/treeTraversal-inst.h>
#include <gtsam/linear/GaussianConditional.h>
#include <gtsam/linear/VectorValues.h>
#include <gtsam/linear/linearExceptions.h>

#include <memory>
#include <optional>

namespace gtsam
{
  namespace internal
  {
    namespace linearAlgorithms
    {
      /* ************************************************************************* */
      struct OptimizeData {
        OptimizeData* parentData = nullptr;
        FastMap<Key, VectorValues::const_iterator> cliqueResults;
      };

      /* ************************************************************************* */
      /** Pre-order visitor for back-substitution in a Bayes tree.  The visitor function operator()()
      *  optimizes the clique given the solution for the parents, and returns the solution for the
      *  clique's frontal variables.  In addition, it adds the solution to a global collected
      *  solution that will finally be returned to the user.  The reason we pass the individual
      *  clique solutions between nodes is to avoid log(n) lookups over all variables, they instead
      *  then are only over a node's parent variables. */
      template<class CLIQUE>
      struct OptimizeClique
      {
        VectorValues collectedResult;
        const VectorValues* rhs = nullptr;

        /** Optimize with stored d, or solve against a complete whitened rhs. */
        explicit OptimizeClique(const VectorValues* rhs = nullptr) : rhs(rhs) {}

        /** Solve this clique after its separator solution is available. */
        OptimizeData operator()(
          const std::shared_ptr<CLIQUE>& clique,
          OptimizeData& parentData)
        {
          OptimizeData myData;
          myData.parentData = &parentData;
          // Take any ancestor results we'll need
          for(Key parent: clique->conditional_->parents())
            myData.cliqueResults.emplace(parent, myData.parentData->cliqueResults.at(parent));

          // Solve and store in our results
          {
            const GaussianConditional& c = *clique->conditional();
            // Solve matrix
            Vector xS;
            {
              // Count dimensions of vector
              DenseIndex dim = 0;
              FastVector<VectorValues::const_iterator> parentPointers;
              parentPointers.reserve(clique->conditional()->nrParents());
              for(Key parent: clique->conditional()->parents()) {
                parentPointers.push_back(myData.cliqueResults.at(parent));
                dim += parentPointers.back()->second.size();
              }

              // Fill parent vector
              xS.resize(dim);
              DenseIndex vectorPos = 0;
              for(const VectorValues::const_iterator& parentPointer: parentPointers) {
                const Vector& parentVector = parentPointer->second;
                xS.block(vectorPos,0,parentVector.size(),1) = parentVector.block(0,0,parentVector.size(),1);
                vectorPos += parentVector.size();
              }
            }

            // Stored d is already in the raw conditional coordinates. An
            // arbitrary RHS uses whitened coordinates, so scale it before the
            // triangular solve: R x = D rhs - S xS, with D = diag(sigmas).
            Vector solution;
            if (rhs) {
              Vector frontalRhs(c.rows());
              rhs->fillVector(c.frontals(), frontalRhs);
              if (c.get_model())
                frontalRhs.array() *= c.get_model()->sigmasRef().array();
              internal::solveUpperConditional(c.R(), c.S(), frontalRhs, xS,
                                              &solution);
            } else {
              internal::solveUpperConditional(c.R(), c.S(), c.getb(), xS,
                                              &solution);
            }
            // RHS solves reject nonfinite values; optimize() rejects NaNs.
            if (rhs ? !solution.allFinite() : solution.hasNaN()) {
              throw IndeterminateSystemException(c.keys().front());
            }

            // Insert solution into a VectorValues
            DenseIndex vectorPosition = 0;
            for(GaussianConditional::const_iterator frontal = c.beginFrontals(); frontal != c.endFrontals(); ++frontal) {
              auto result = collectedResult.emplace(*frontal, solution.segment(vectorPosition, c.getDim(frontal)));
              if(!result.second)
                  throw std::runtime_error(
                      "Internal error while optimizing clique. Trying to insert key '" + DefaultKeyFormatter(*frontal)
                      + "' that exists.");

              VectorValues::const_iterator r = result.first;
              myData.cliqueResults.emplace(r->first, r);
              vectorPosition += c.getDim(frontal);
            }
          }
          return myData;
        }
      };

      /* ************************************************************************* */
      /** Back-substitute stored d, or a complete whitened rhs, through a tree. */
      template<class BAYESTREE>
      VectorValues optimizeBayesTree(const BAYESTREE& bayesTree,
                                    const VectorValues* rhs = nullptr)
      {
        gttic(linear_optimizeBayesTree);
        OptimizeData rootData;
        OptimizeClique<typename BAYESTREE::Clique> preVisitor(rhs);
        treeTraversal::no_op postVisitor;
        TbbOpenMPMixedScope threadLimiter; // Limits OpenMP threads since we're mixing TBB and OpenMP
        treeTraversal::DepthFirstForestParallel(bayesTree, rootData, preVisitor, postVisitor);
        return preVisitor.collectedResult;
      }
    }
  }
}
