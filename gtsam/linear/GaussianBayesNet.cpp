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
 * @author Frank Dellaert, Varun Agrawal
 */

#include <gtsam/base/timing.h>
#include <gtsam/inference/FactorGraph-inst.h>
#include <gtsam/linear/GaussianBayesNet.h>
#include <gtsam/linear/GaussianFactorGraph.h>

#include <boost/range/adaptor/reversed.hpp>
#include <fstream>

using namespace std;
using namespace gtsam;

namespace gtsam {

  // Instantiate base class
  template class FactorGraph<GaussianConditional>;

  /* ************************************************************************* */
  bool GaussianBayesNet::equals(const This& bn, double tol) const
  {
    return Base::equals(bn, tol);
  }

  /* ************************************************************************* */
  VectorValues GaussianBayesNet::optimize() const
  {
    VectorValues soln; // no missing variables -> just create an empty vector
    return optimize(soln);
  }

  /* ************************************************************************* */
  VectorValues GaussianBayesNet::optimize(
      const VectorValues& solutionForMissing) const {
    VectorValues soln(solutionForMissing); // possibly empty
    // (R*x)./sigmas = y by solving x=inv(R)*(y.*sigmas)
    /** solve each node in turn in topological sort order (parents first)*/
    for (auto cg: boost::adaptors::reverse(*this)) {
      // i^th part of R*x=y, x=inv(R)*y
      // (Rii*xi + R_i*x(i+1:))./si = yi <-> xi = inv(Rii)*(yi.*si - R_i*x(i+1:))
      soln.insert(cg->solve(soln));
    }
    return soln;
  }

  /* ************************************************************************* */
  VectorValues GaussianBayesNet::optimizeGradientSearch() const
  {
    gttic(GaussianBayesTree_optimizeGradientSearch);
    return GaussianFactorGraph(*this).optimizeGradientSearch();
  }

  /* ************************************************************************* */
  VectorValues GaussianBayesNet::gradient(const VectorValues& x0) const {
    return GaussianFactorGraph(*this).gradient(x0);
  }

  /* ************************************************************************* */
  VectorValues GaussianBayesNet::gradientAtZero() const {
    return GaussianFactorGraph(*this).gradientAtZero();
  }

  /* ************************************************************************* */
  double GaussianBayesNet::error(const VectorValues& x) const {
    return GaussianFactorGraph(*this).error(x);
  }

  /* ************************************************************************* */
  VectorValues GaussianBayesNet::backSubstitute(const VectorValues& rhs) const
  {
    VectorValues result;
    // TODO this looks pretty sketchy. result is passed as the parents argument
    //  as it's filled up by solving the gaussian conditionals.
    for (auto cg: boost::adaptors::reverse(*this)) {
      result.insert(cg->solveOtherRHS(result, rhs));
    }
    return result;
  }


  /* ************************************************************************* */
  // gy=inv(L)*gx by solving L*gy=gx.
  // gy=inv(R'*inv(Sigma))*gx
  // gz'*R'=gx', gy = gz.*sigmas
  VectorValues GaussianBayesNet::backSubstituteTranspose(const VectorValues& gx) const
  {
    // Initialize gy from gx
    // TODO: used to insert zeros if gx did not have an entry for a variable in bn
    VectorValues gy = gx;

    // we loop from first-eliminated to last-eliminated
    // i^th part of L*gy=gx is done block-column by block-column of L
    for(const sharedConditional& cg: *this)
      cg->solveTransposeInPlace(gy);

    return gy;
  }

  ///* ************************************************************************* */
  //VectorValues GaussianBayesNet::optimizeGradientSearch() const
  //{
  //  gttic(Compute_Gradient);
  //  // Compute gradient (call gradientAtZero function, which is defined for various linear systems)
  //  VectorValues grad = gradientAtZero();
  //  double gradientSqNorm = grad.dot(grad);
  //  gttoc(Compute_Gradient);

  //  gttic(Compute_Rg);
  //  // Compute R * g
  //  Errors Rg = GaussianFactorGraph(*this) * grad;
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

  /* ************************************************************************* */
  Ordering GaussianBayesNet::ordering() const {
    GaussianFactorGraph factorGraph(*this);
    auto keys = factorGraph.keys();
    // add frontal keys in order
    Ordering ordering;
    for (const sharedConditional& cg : *this)
      if (cg) {
        for (Key key : cg->frontals()) {
          ordering.push_back(key);
          keys.erase(key);
        }
      }
    // add remaining keys in case Bayes net is incomplete
    for (Key key : keys) ordering.push_back(key);
    return ordering;
  }

  /* ************************************************************************* */
  pair<Matrix, Vector> GaussianBayesNet::matrix(const Ordering& ordering) const {
    // Convert to a GaussianFactorGraph and use its machinery
    GaussianFactorGraph factorGraph(*this);
    return factorGraph.jacobian(ordering);
  }

  /* ************************************************************************* */
  pair<Matrix, Vector> GaussianBayesNet::matrix() const {
    // recursively call with default ordering
    const auto defaultOrdering = this->ordering();
    return matrix(defaultOrdering);
  }

  ///* ************************************************************************* */
  //VectorValues GaussianBayesNet::gradient(const VectorValues& x0) const
  //{
  //  return GaussianFactorGraph(*this).gradient(x0);
  //}

  ///* ************************************************************************* */
  //VectorValues GaussianBayesNet::gradientAtZero() const
  //{
  //  return GaussianFactorGraph(*this).gradientAtZero();
  //}

  /* ************************************************************************* */
  double GaussianBayesNet::determinant() const
  {
    return exp(logDeterminant());
  }

  /* ************************************************************************* */
  double GaussianBayesNet::logDeterminant() const {
    double logDet = 0.0;
    for (const sharedConditional& cg : *this) {
      if (cg->get_model()) {
        Vector diag = cg->R().diagonal();
        cg->get_model()->whitenInPlace(diag);
        logDet += diag.unaryExpr([](double x) { return log(x); }).sum();
      } else {
        logDet +=
            cg->R().diagonal().unaryExpr([](double x) { return log(x); }).sum();
      }
    }
    return logDet;
  }

  /* ************************************************************************* */
  void GaussianBayesNet::saveGraph(const std::string& s,
                                   const KeyFormatter& keyFormatter) const {
    std::ofstream of(s.c_str());
    of << "digraph G{\n";

    for (auto conditional : boost::adaptors::reverse(*this)) {
      typename GaussianConditional::Frontals frontals = conditional->frontals();
      Key me = frontals.front();
      typename GaussianConditional::Parents parents = conditional->parents();
      for (Key p : parents)
        of << keyFormatter(p) << "->" << keyFormatter(me) << std::endl;
    }

    of << "}";
    of.close();
  }

  /* ************************************************************************* */

} // namespace gtsam
