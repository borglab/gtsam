/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    GaussianBayesTree.cpp
 * @brief   Gaussian Bayes Tree, the result of eliminating a GaussianJunctionTree
 * @brief   GaussianBayesTree
 * @author  Frank Dellaert
 * @author  Richard Roberts
 */

#include <gtsam/base/treeTraversal-inst.h>
#include <gtsam/inference/BayesTree-inst.h>
#include <gtsam/inference/BayesTreeCliqueBase-inst.h>
#include <gtsam/linear/GaussianBayesTree.h>
#include <gtsam/linear/GaussianBayesNet.h>
#include <gtsam/linear/VectorValues.h>

namespace gtsam {

  /* ************************************************************************* */
  namespace internal
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
    struct OptimizeClique
    {
      VectorValues collectedResult;

      OptimizeData operator()(
        const GaussianBayesTreeClique::shared_ptr& clique,
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
    double logDeterminant(const GaussianBayesTreeClique::shared_ptr& clique, double& parentSum)
    {
      parentSum += clique->conditional()->get_R().diagonal().unaryExpr(std::ptr_fun<double,double>(log)).sum();
      assert(false);
      return 0;
    }
  }

  /* ************************************************************************* */
  GaussianBayesTree::GaussianBayesTree(const GaussianBayesTree& other) :
    Base(other) {}

  /* ************************************************************************* */
  GaussianBayesTree& GaussianBayesTree::operator=(const GaussianBayesTree& other)
  {
    (void) Base::operator=(other);
    return *this;
  }

  /* ************************************************************************* */
  bool GaussianBayesTree::equals(const This& other, double tol) const
  {
    return Base::equals(other, tol);
  }

  /* ************************************************************************* */
  VectorValues GaussianBayesTree::optimize() const
  {
    gttic(GaussianBayesTree_optimize);
    //internal::OptimizeData rootData; // Will hold final solution
    //treeTraversal::DepthFirstForest(*this, rootData, internal::OptimizePreVisitor, internal::OptimizePostVisitor);
    //return rootData.results;
    internal::OptimizeData rootData;
    internal::OptimizeClique preVisitor;
    treeTraversal::DepthFirstForest(*this, rootData, preVisitor);
    return preVisitor.collectedResult;
  }

  /* ************************************************************************* */
  VectorValues GaussianBayesTree::optimizeGradientSearch() const
  {
    gttic(GaussianBayesTree_optimizeGradientSearch);
    return GaussianFactorGraph(*this).optimizeGradientSearch();
  }

  /* ************************************************************************* */
  VectorValues GaussianBayesTree::gradient(const VectorValues& x0) const {
    return GaussianFactorGraph(*this).gradient(x0);
  }

  /* ************************************************************************* */
  VectorValues GaussianBayesTree::gradientAtZero() const {
    return GaussianFactorGraph(*this).gradientAtZero();
  }

  /* ************************************************************************* */
  double GaussianBayesTree::error(const VectorValues& x) const {
    return GaussianFactorGraph(*this).error(x);
  }

  /* ************************************************************************* */
  double GaussianBayesTree::logDeterminant() const
  {
    if(this->roots_.empty()) {
      return 0.0;
    } else {
      double sum = 0.0;
      treeTraversal::DepthFirstForest(*this, sum, internal::logDeterminant);
      return sum;
    }
  }

  /* ************************************************************************* */
  double GaussianBayesTree::determinant() const
  {
    return exp(logDeterminant());
  }

  /* ************************************************************************* */
  Matrix GaussianBayesTree::marginalCovariance(Key key) const
  {
    return marginalFactor(key)->information().inverse();
  }


} // \namespace gtsam




