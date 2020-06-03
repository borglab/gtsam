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

#include <gtsam/linear/VectorValues.h>
#include <gtsam/linear/GaussianConditional.h>
#include <gtsam/base/treeTraversal-inst.h>

#include <boost/optional.hpp>
#include <boost/shared_ptr.hpp>

namespace gtsam
{
  namespace internal
  {
    namespace linearAlgorithms
    {
      /* ************************************************************************* */
      struct OptimizeData {
        boost::optional<OptimizeData&> parentData;
        FastMap<Key, VectorValues::const_iterator> cliqueResults;
        //VectorValues ancestorResults;
        //VectorValues results;
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

        OptimizeData operator()(
          const boost::shared_ptr<CLIQUE>& clique,
          OptimizeData& parentData)
        {
          OptimizeData myData;
          myData.parentData = parentData;
          // Take any ancestor results we'll need
          for(Key parent: clique->conditional_->parents())
            myData.cliqueResults.emplace(parent, myData.parentData->cliqueResults.at(parent));

          // Solve and store in our results
          {
            GaussianConditional& c = *clique->conditional();
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

            // NOTE(gareth): We can no longer write: xS = b - S * xS
            // This is because Eigen (as of 3.3) no longer evaluates S * xS into
            // a temporary, and the operation trashes valus in xS.
            // See: http://eigen.tuxfamily.org/index.php?title=3.3
            const Vector rhs = c.getb() - c.S() * xS;

            // TODO(gareth): Inline instantiation of Eigen::Solve and check flag
            const Vector solution = c.R().triangularView<Eigen::Upper>().solve(rhs);

            // Check for indeterminant solution
            if(solution.hasNaN()) throw IndeterminantLinearSystemException(c.keys().front());

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
      //OptimizeData OptimizePreVisitor(const GaussianBayesTreeClique::shared_ptr& clique, OptimizeData& parentData)
      //{
      //  // Create data - holds a pointer to our parent, a copy of parent solution, and our results
      //  OptimizeData myData;
      //  myData.parentData = parentData;
      //  // Take any ancestor results we'll need
      //  for(Key parent: clique->conditional_->parents())
      //    myData.ancestorResults.insert(parent, myData.parentData->ancestorResults[parent]);
      //  // Solve and store in our results
      //  myData.results.insert(clique->conditional()->solve(myData.ancestorResults));
      //  myData.ancestorResults.insert(myData.results);
      //  return myData;
      //}

      /* ************************************************************************* */
      //void OptimizePostVisitor(const GaussianBayesTreeClique::shared_ptr& clique, OptimizeData& myData)
      //{
      //  // Conglomerate our results to the parent
      //  myData.parentData->results.insert(myData.results);
      //}

      /* ************************************************************************* */
      template<class BAYESTREE>
      VectorValues optimizeBayesTree(const BAYESTREE& bayesTree)
      {
        gttic(linear_optimizeBayesTree);
        //internal::OptimizeData rootData; // Will hold final solution
        //treeTraversal::DepthFirstForest(*this, rootData, internal::OptimizePreVisitor, internal::OptimizePostVisitor);
        //return rootData.results;
        OptimizeData rootData;
        OptimizeClique<typename BAYESTREE::Clique> preVisitor;
        treeTraversal::no_op postVisitor;
        TbbOpenMPMixedScope threadLimiter; // Limits OpenMP threads since we're mixing TBB and OpenMP
        treeTraversal::DepthFirstForestParallel(bayesTree, rootData, preVisitor, postVisitor);
        return preVisitor.collectedResult;
      }
    }
  }
}
