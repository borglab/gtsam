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

#include <fstream>
#include <iterator>

using namespace std;
using namespace gtsam;

// In Wrappers we have no access to this so have a default ready
static std::mt19937_64 kRandomNumberGenerator(42);

namespace gtsam {

  // Instantiate base class
  template class FactorGraph<GaussianConditional>;

  /* ************************************************************************* */
  bool GaussianBayesNet::equals(const This& bn, double tol) const
  {
    return Base::equals(bn, tol);
  }

  /* ************************************************************************ */
  VectorValues GaussianBayesNet::optimize() const {
    VectorValues solution;  // no missing variables -> create an empty vector
    return optimize(solution);
  }

  VectorValues GaussianBayesNet::optimize(const VectorValues& given) const {
    VectorValues solution = given;
    // (R*x)./sigmas = y by solving x=inv(R)*(y.*sigmas)
    // solve each node in reverse topological sort order (parents first)
    for (auto it = std::make_reverse_iterator(end()); it != std::make_reverse_iterator(begin()); ++it) {
      // i^th part of R*x=y, x=inv(R)*y
      // (Rii*xi + R_i*x(i+1:))./si = yi =>
      // xi = inv(Rii)*(yi.*si - R_i*x(i+1:))
      solution.insert((*it)->solve(solution));
    }
    return solution;
  }

  /* ************************************************************************ */
  VectorValues GaussianBayesNet::sample(std::mt19937_64* rng) const {
    VectorValues result;  // no missing variables -> create an empty vector
    return sample(result, rng);
  }

  VectorValues GaussianBayesNet::sample(const VectorValues& given,
                                        std::mt19937_64* rng) const {
    VectorValues result(given);
    // sample each node in reverse topological sort order (parents first)
    for (auto it = std::make_reverse_iterator(end()); it != std::make_reverse_iterator(begin()); ++it) {
      const VectorValues sampled = (*it)->sample(result, rng);
      result.insert(sampled);
    }
    return result;
  }

  /* ************************************************************************ */
  VectorValues GaussianBayesNet::sample() const {
    return sample(&kRandomNumberGenerator);
  }

  VectorValues GaussianBayesNet::sample(const VectorValues& given) const {
    return sample(given, &kRandomNumberGenerator);
  }

  /* ************************************************************************ */
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
    double sum = 0.;
    for (const auto& gc : *this) {
      if (gc) sum += gc->error(x);
    }
    return sum;
  }

  /* ************************************************************************* */
  double GaussianBayesNet::logProbability(const VectorValues& x) const {
    double sum = 0.;
    for (const auto& gc : *this) {
      if (gc) sum += gc->logProbability(x);
    }
    return sum;
  }

  /* ************************************************************************* */
  double GaussianBayesNet::evaluate(const VectorValues& x) const {
    return exp(logProbability(x));
  }

  /* ************************************************************************* */
  VectorValues GaussianBayesNet::backSubstitute(const VectorValues& rhs) const
  {
    VectorValues result;
    // TODO this looks pretty sketchy. result is passed as the parents argument
    //  as it's filled up by solving the gaussian conditionals.
    for (auto it = std::make_reverse_iterator(end()); it != std::make_reverse_iterator(begin()); ++it) {
      result.insert((*it)->solveOtherRHS(result, rhs));
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
      logDet += cg->logDeterminant();
    }
    return logDet;
  }

  /* ************************************************************************* */
  double GaussianBayesNet::logNormalizationConstant() const {
    /*
    normalization constant = 1.0 / sqrt((2*pi)^n*det(Sigma))
    logConstant = -log(normalizationConstant)
      = 0.5 * n*log(2*pi) + 0.5 * log(det(Sigma))
    
    log(det(Sigma)) = -2.0 * logDeterminant()
    thus, logConstant = 0.5*n*log(2*pi) - logDeterminant()

    BayesNet logConstant = sum(0.5*n_i*log(2*pi) - logDeterminant_i())
    = sum(0.5*n_i*log(2*pi)) - sum(logDeterminant_i())
    = sum(0.5*n_i*log(2*pi)) - bn->logDeterminant()
    */
    double logNormConst = 0.0;
    for (const sharedConditional& cg : *this) {
      logNormConst += cg->logNormalizationConstant();
    }
    return logNormConst;
  }

  /* ************************************************************************* */

} // namespace gtsam
