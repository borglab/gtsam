/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    linearAlgorithms-inst.h
 * Shared exact Bayes-tree traversals for stored and changed right-hand sides.
 * @author  Richard Roberts
 * @author  Frank Dellaert
 */

#pragma once

#include <gtsam/base/treeTraversal-inst.h>
#include <gtsam/linear/GaussianConditional.h>
#include <gtsam/linear/VectorValues.h>
#include <gtsam/linear/linearExceptions.h>

#include <map>
#include <memory>
#include <optional>

namespace gtsam
{
  namespace internal
  {
    namespace linearAlgorithms
    {
      /* ************************************************************************* */
      /// Per-clique traversal state linking solved variables to their ancestors.
      struct OptimizeData {
        /// Parent visitor state, used to obtain already solved separator values.
        OptimizeData* parentData = nullptr;
        /// Available solution blocks indexed by their original variable keys.
        FastMap<Key, VectorValues::const_iterator> cliqueResults;
      };

      /* ************************************************************************* */
      /**
       * Pre-order visitor for exact Bayes-tree back-substitution.
       * Each clique receives its solved separator variables and contributes its
       * frontal solution to the collected result. Passing iterators between
       * visitors limits lookups to a clique's parents instead of the full graph.
       */
      template<class CLIQUE>
      struct OptimizeClique
      {
        /// Frontal solutions collected during the traversal.
        VectorValues collectedResult;
        /// Optional complete whitened RHS, validated before traversal.
        const VectorValues* rhs = nullptr;
        /// Whether an explicit RHS is an increment to the stored affine equations.
        bool addStoredRhs = false;

        /**
         * Use stored conditional right-hand sides when rhs is null. Otherwise,
         * solve against the supplied complete whitened blocks, adding stored d
         * when addStoredRhs is true. The caller retains ownership of rhs.
         */
        explicit OptimizeClique(const VectorValues* rhs = nullptr,
                                bool addStoredRhs = false)
            : rhs(rhs), addStoredRhs(addStoredRhs) {}

        /// Solve this clique after its separator solution is available.
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
            // Gather separator values in the conditional's parent order.
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
              // A forcing increment preserves the reference affine equations:
              // zero-sigma (hard) rows receive no RHS increment.
              if (addStoredRhs) frontalRhs += c.getb();
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

      /**
       * Solve the transpose conditional operator with a complete, validated RHS.
       * result supplies one block per frontal key and becomes the returned
       * solution. Each child owns its outgoing separator message. Conditional
       * sigma scaling occurs after that message is formed, including zero sigmas.
       * Nonfinite intermediate values throw IndeterminateSystemException.
       */
      template<class BAYESTREE>
      VectorValues transposeBayesTree(const BAYESTREE& bayesTree,
                                     VectorValues result)
      {
        // Allocate every clique's private RHS and message before parallel visits.
        using Clique = typename BAYESTREE::Clique;
        std::map<const Clique*, VectorValues> workspace;
        for (const auto& [key, clique] : bayesTree.nodes()) {
          const auto& conditional = *clique->conditional();
          if (key != conditional.firstFrontalKey()) continue;
          auto& local = workspace[clique.get()];
          for (Key frontal : conditional.frontals())
            local.emplace(frontal, result.at(frontal));
          for (auto parent = conditional.beginParents();
               parent != conditional.endParents(); ++parent)
            local.emplace(*parent, Vector::Zero(conditional.getDim(parent)));
        }

        auto solveClique = [&workspace, &result](
                               const std::shared_ptr<Clique>& clique) {
          auto& local = workspace.at(clique.get());
          // A child's separator can include higher ancestors as well as this
          // clique's frontals. Retain both kinds of contribution in the local map.
          for (const auto& child : clique->children) {
            const auto& message = workspace.at(child.get());
            for (Key separator : child->conditional()->parents())
              local.at(separator) += message.at(separator);
          }

          // Use only the cached conditional for this clique's transpose solve.
          const auto& conditional = *clique->conditional();
          conditional.solveTransposeInPlace(local);
          for (const auto& [key, value] : local) {
            if (!value.allFinite())
              throw IndeterminateSystemException(conditional.keys().front());
          }

          // Only this clique writes these frontals; ancestors read its separators.
          for (Key frontal : conditional.frontals())
            result.at(frontal).swap(local.at(frontal));
        };

        // Visit children first so their separator messages are complete.
        TbbOpenMPMixedScope threadLimiter;
        treeTraversal::PostOrderForestParallel(bayesTree, solveClique);

        return result;
      }

      /* ************************************************************************* */
      /**
       * Back-substitute through the tree using cached conditionals.
       * A null rhs uses stored d; otherwise rhs supplies a complete, validated
       * whitened block for each frontal key. With addStoredRhs, these blocks
       * increment the stored affine equations instead of replacing them.
       */
      template<class BAYESTREE>
      VectorValues optimizeBayesTree(const BAYESTREE& bayesTree,
                                    const VectorValues* rhs = nullptr,
                                    bool addStoredRhs = false)
      {
        gttic(linear_optimizeBayesTree);

        // Carry solved separator values through exact pre-order substitution.
        OptimizeData rootData;
        OptimizeClique<typename BAYESTREE::Clique> preVisitor(rhs, addStoredRhs);
        treeTraversal::no_op postVisitor;

        // Limit OpenMP threads while the tree traversal can also use TBB.
        TbbOpenMPMixedScope threadLimiter;
        treeTraversal::DepthFirstForestParallel(bayesTree, rootData, preVisitor, postVisitor);

        return preVisitor.collectedResult;
      }
    }
  }
}
