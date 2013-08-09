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
          //BOOST_FOREACH(Key parent, clique->conditional_->parents())
          //  myData.ancestorResults.insert(parent, myData.parentData->ancestorResults[parent]);
          // Solve and store in our results
          VectorValues result = clique->conditional()->solve(collectedResult/*myData.ancestorResults*/);
          collectedResult.insert(result);
          //myData.ancestorResults.insert(result);
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
      //  BOOST_FOREACH(Key parent, clique->conditional_->parents())
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
        treeTraversal::DepthFirstForest(bayesTree, rootData, preVisitor);
        return preVisitor.collectedResult;
      }
    }
  }
}