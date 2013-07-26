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
#include <gtsam/inference/BayesTreeUnordered-inst.h>
#include <gtsam/inference/BayesTreeCliqueBaseUnordered-inst.h>
#include <gtsam/linear/GaussianBayesTreeUnordered.h>
#include <gtsam/linear/GaussianBayesNetUnordered.h>
#include <gtsam/linear/VectorValuesUnordered.h>

namespace gtsam {

  /* ************************************************************************* */
  namespace internal
  {
    /* ************************************************************************* */
    struct OptimizeData {
      boost::optional<OptimizeData&> parentData;
      VectorValuesUnordered ancestorResults;
      //VectorValuesUnordered results;
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
      VectorValuesUnordered collectedResult;

      OptimizeData operator()(
        const GaussianBayesTreeCliqueUnordered::shared_ptr& clique,
        OptimizeData& parentData)
      {
        OptimizeData myData;
        myData.parentData = parentData;
        // Take any ancestor results we'll need
        //BOOST_FOREACH(Key parent, clique->conditional_->parents())
        //  myData.ancestorResults.insert(parent, myData.parentData->ancestorResults[parent]);
        // Solve and store in our results
        VectorValuesUnordered result = clique->conditional()->solve(collectedResult/*myData.ancestorResults*/);
        collectedResult.insert(result);
        //myData.ancestorResults.insert(result);
        return myData;
      }
    };

    /* ************************************************************************* */
    //OptimizeData OptimizePreVisitor(const GaussianBayesTreeCliqueUnordered::shared_ptr& clique, OptimizeData& parentData)
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
    //void OptimizePostVisitor(const GaussianBayesTreeCliqueUnordered::shared_ptr& clique, OptimizeData& myData)
    //{
    //  // Conglomerate our results to the parent
    //  myData.parentData->results.insert(myData.results);
    //}

    /* ************************************************************************* */
    double logDeterminant(const GaussianBayesTreeCliqueUnordered::shared_ptr& clique, double& parentSum)
    {
      parentSum += clique->conditional()->get_R().diagonal().unaryExpr(std::ptr_fun<double,double>(log)).sum();
      assert(false);
      return 0;
    }
  }

  /* ************************************************************************* */
  GaussianBayesTreeUnordered::GaussianBayesTreeUnordered(const GaussianBayesTreeUnordered& other) :
    Base(other) {}

  /* ************************************************************************* */
  GaussianBayesTreeUnordered& GaussianBayesTreeUnordered::operator=(const GaussianBayesTreeUnordered& other)
  {
    (void) Base::operator=(other);
    return *this;
  }

  /* ************************************************************************* */
  bool GaussianBayesTreeUnordered::equals(const This& other, double tol) const
  {
    return Base::equals(other, tol);
  }

  /* ************************************************************************* */
  VectorValuesUnordered GaussianBayesTreeUnordered::optimize() const
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

  ///* ************************************************************************* */
  //VectorValuesUnordered GaussianBayesTreeUnordered::optimizeGradientSearch() const
  //{
  //  gttic(Compute_Gradient);
  //  // Compute gradient (call gradientAtZero function, which is defined for various linear systems)
  //  VectorValuesUnordered grad = gradientAtZero();
  //  double gradientSqNorm = grad.dot(grad);
  //  gttoc(Compute_Gradient);

  //  gttic(Compute_Rg);
  //  // Compute R * g
  //  Errors Rg = GaussianFactorGraphUnordered(*this) * grad;
  //  gttoc(Compute_Rg);

  //  gttic(Compute_minimizing_step_size);
  //  // Compute minimizing step size
  //  double step = -gradientSqNorm / dot(Rg, Rg);
  //  gttoc(Compute_minimizing_step_size);

  //  gttic(Compute_point);
  //  // Compute steepest descent point
  //  scal(step, grad);
  //  gttoc(Compute_point);

  //  return grad;
  //}

  ///* ************************************************************************* */
  //VectorValuesUnordered GaussianBayesTreeUnordered::gradient(const VectorValuesUnordered& x0) const {
  //  return GaussianFactorGraphUnordered(*this).gradient(x0);
  //}

  ///* ************************************************************************* */
  //VectorValuesUnordered GaussianBayesTreeUnordered::gradientAtZero() const {
  //  return GaussianFactorGraphUnordered(*this).gradientAtZero();
  //}

  /* ************************************************************************* */
  double GaussianBayesTreeUnordered::logDeterminant() const
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
  double GaussianBayesTreeUnordered::determinant() const
  {
    return exp(logDeterminant());
  }

} // \namespace gtsam




