/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file   GaussianBayesNet.cpp
 * @brief  Chordal Bayes Net, the result of eliminating a factor graph
 * @author Frank Dellaert
 */

#include <gtsam/linear/GaussianBayesNetUnordered.h>
#include <gtsam/linear/GaussianFactorGraphUnordered.h>
#include <gtsam/base/timing.h>

#include <boost/foreach.hpp>

using namespace std;
using namespace gtsam;

#define FOREACH_PAIR( KEY, VAL, COL) BOOST_FOREACH (boost::tie(KEY,VAL),COL) 
#define REVERSE_FOREACH_PAIR( KEY, VAL, COL) BOOST_REVERSE_FOREACH (boost::tie(KEY,VAL),COL)

namespace gtsam {

  /* ************************************************************************* */
  bool GaussianBayesNetUnordered::equals(const This& bn, double tol = 1e-9) const
  {
    return Base::equals(bn, tol);
  }

  /* ************************************************************************* */
  VectorValuesUnordered GaussianBayesNetUnordered::optimize() const
  {
    VectorValuesUnordered soln;
    // (R*x)./sigmas = y by solving x=inv(R)*(y.*sigmas)
    /** solve each node in turn in topological sort order (parents first)*/
    BOOST_REVERSE_FOREACH(const sharedConditional& cg, *this) {
      // i^th part of R*x=y, x=inv(R)*y
      // (Rii*xi + R_i*x(i+1:))./si = yi <-> xi = inv(Rii)*(yi.*si - R_i*x(i+1:))
      soln.insert(cg->solve(soln));
    }
    return soln;
  }

  /* ************************************************************************* */
  VectorValuesUnordered GaussianBayesNetUnordered::backSubstitute(const VectorValuesUnordered& rhs) const
  {
    VectorValuesUnordered result;
    BOOST_REVERSE_FOREACH(const sharedConditional& cg, *this) {
      result.insert(cg->solveOtherRHS(result, rhs));
    }
    return result;
  }


  /* ************************************************************************* */
  // gy=inv(L)*gx by solving L*gy=gx.
  // gy=inv(R'*inv(Sigma))*gx
  // gz'*R'=gx', gy = gz.*sigmas
  VectorValuesUnordered GaussianBayesNetUnordered::backSubstituteTranspose(const VectorValuesUnordered& gx) const
  {
    // Initialize gy from gx
    // TODO: used to insert zeros if gx did not have an entry for a variable in bn
    VectorValuesUnordered gy = gx;

    // we loop from first-eliminated to last-eliminated
    // i^th part of L*gy=gx is done block-column by block-column of L
    BOOST_FOREACH(const sharedConditional& cg, *this)
      cg->solveTransposeInPlace(gy);

    return gy;
  }

  /* ************************************************************************* */
  VectorValuesUnordered GaussianBayesNetUnordered::optimizeGradientSearch() const
  {
    gttic(Compute_Gradient);
    // Compute gradient (call gradientAtZero function, which is defined for various linear systems)
    VectorValuesUnordered grad = gradientAtZero();
    double gradientSqNorm = grad.dot(grad);
    gttoc(Compute_Gradient);

    gttic(Compute_Rg);
    // Compute R * g
    Errors Rg = GaussianFactorGraphUnordered(*this) * grad;
    gttoc(Compute_Rg);

    gttic(Compute_minimizing_step_size);
    // Compute minimizing step size
    double step = -gradientSqNorm / dot(Rg, Rg);
    gttoc(Compute_minimizing_step_size);

    gttic(Compute_point);
    // Compute steepest descent point
    scal(step, grad);
    gttoc(Compute_point);

    return grad;
  }

  /* ************************************************************************* */  
  pair<Matrix,Vector> GaussianBayesNetUnordered::matrix() const
  {
    return GaussianFactorGraphUnordered(*this).jacobian();
  }

  /* ************************************************************************* */
  double determinant(const GaussianBayesNet& bayesNet) {
    double logDet = 0.0;

    BOOST_FOREACH(boost::shared_ptr<const GaussianConditional> cg, bayesNet){
      logDet += cg->get_R().diagonal().unaryExpr(ptr_fun<double,double>(log)).sum();
    }

    return exp(logDet);
  }

  /* ************************************************************************* */
  VectorValues gradient(const GaussianBayesNet& bayesNet, const VectorValues& x0) {
    return gradient(GaussianFactorGraph(bayesNet), x0);
  }

  /* ************************************************************************* */
  void gradientAtZero(const GaussianBayesNet& bayesNet, VectorValues& g) {
    gradientAtZero(GaussianFactorGraph(bayesNet), g);
  }

  /* ************************************************************************* */

} // namespace gtsam
