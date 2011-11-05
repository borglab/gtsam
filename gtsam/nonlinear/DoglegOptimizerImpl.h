/**
 * @file    DoglegOptimizerImpl.h
 * @brief   Nonlinear factor graph optimizer using Powell's Dogleg algorithm (detail implementation)
 * @author  Richard Roberts
 */
#pragma once

#include <gtsam/linear/GaussianBayesNet.h>
#include <gtsam/linear/GaussianISAM.h> // To get optimize(BayesTree<GaussianConditional>)
#include <gtsam/nonlinear/NonlinearFactorGraph-inl.h>
#include <gtsam/nonlinear/Ordering.h>

namespace gtsam {

/** This class contains the implementation of the Dogleg algorithm.  It is used
 * by DoglegOptimizer and can be used to easily put together custom versions of
 * Dogleg.  Each function is well-documented and unit-tested.  The notation
 * here matches that in "trustregion.pdf" in gtsam_experimental/doc, see this
 * file for further explanation of the computations performed by this class.
 *
 * \tparam VALUES The LieValues or TupleValues type to hold the values to be
 * estimated.
 *
 * \tparam GAUSSIAN_SOLVER The linear solver to use at each iteration,
 * currently either GaussianSequentialSolver or GaussianMultifrontalSolver.
 * The latter is typically faster, especially for non-trivial problems.
 */
struct DoglegOptimizerImpl {

  struct IterationResult {
    double Delta;
    VectorValues dx_d;
    double f_error;
  };

  enum TrustRegionAdaptationMode {
    SEARCH_EACH_ITERATION,
    ONE_STEP_PER_ITERATION
  };

  /**
   * Compute the update point for one iteration of the Dogleg algorithm, given
   * an initial trust region radius \f$ \Delta \f$.  The trust region radius is
   * adapted based on the error of a NonlinearFactorGraph \f$ f(x) \f$, and
   * depending on the update mode \c mode.
   *
   * The update is computed using a quadratic approximation \f$ M(\delta x) \f$
   * of an original nonlinear error function (a NonlinearFactorGraph) \f$ f(x) \f$.
   * The quadratic approximation is represented as a GaussianBayesNet \bayesNet, which is
   * obtained by eliminating a GaussianFactorGraph resulting from linearizing
   * the nonlinear factor graph \f$ f(x) \f$.  Thus, \f$ M(\delta x) \f$ is
   * \f[
   * M(\delta x) = (R \delta x - d)^T (R \delta x - d)
   * \f]
   * where \f$ R \f$ and \f$ d \f$ together are a Bayes' net or Bayes' tree.
   * \f$ R \f$ is upper-triangular and \f$ d \f$ is a vector, in GTSAM represented
   * as a BayesNet<GaussianConditional> (GaussianBayesNet) or
   * BayesTree<GaussianConditional>, containing GaussianConditional s.
   *
   * @tparam M The type of the Bayes' net or tree, currently
   * either BayesNet<GaussianConditional> (or GaussianBayesNet) or BayesTree<GaussianConditional>.
   * @tparam F For normal usage this will be NonlinearFactorGraph<VALUES>.
   * @tparam VALUES The LieValues or TupleValues to pass to F::error() to evaluate
   * the error function.
   * @param initialDelta The initial trust region radius.
   * @param Rd The Bayes' net or tree as described above.
   * @param f The original nonlinear factor graph with which to evaluate the
   * accuracy of \f$ M(\delta x) \f$ to adjust \f$ \Delta \f$.
   * @param x0 The linearization point about which \bayesNet was created
   * @param ordering The variable ordering used to create \bayesNet
   * @param f_error The result of <tt>f.error(x0)</tt>.
   * @return A DoglegIterationResult containing the new \c Delta, the linear
   * update \c dx_d, and the resulting nonlinear error \c f_error.
   */
  template<class M, class F, class VALUES>
  static IterationResult Iterate(
      double Delta, TrustRegionAdaptationMode mode, const M& Rd,
      const F& f, const VALUES& x0, const Ordering& ordering, double f_error);

  /**
   * Compute the dogleg point given a trust region radius \f$ \Delta \f$.  The
   * dogleg point is the intersection between the dogleg path and the trust
   * region boundary, see doc/trustregion.pdf for more details.
   *
   * The update is computed using a quadratic approximation \f$ M(\delta x) \f$
   * of an original nonlinear error function (a NonlinearFactorGraph) \f$ f(x) \f$.
   * The quadratic approximation is represented as a GaussianBayesNet \bayesNet, which is
   * obtained by eliminating a GaussianFactorGraph resulting from linearizing
   * the nonlinear factor graph \f$ f(x) \f$.  Thus, \f$ M(\delta x) \f$ is
   * \f[
   * M(\delta x) = (R \delta x - d)^T (R \delta x - d)
   * \f]
   * where \f$ R \f$ and \f$ d \f$ together are a Bayes' net.  \f$ R \f$ is
   * upper-triangular and \f$ d \f$ is a vector, in GTSAM represented as a
   * GaussianBayesNet, containing GaussianConditional s.
   *
   * @param Delta The trust region radius
   * @param bayesNet The Bayes' net \f$ (R,d) \f$ as described above.
   * @return The dogleg point \f$ \delta x_d \f$
   */
  static VectorValues ComputeDoglegPoint(double Delta, const VectorValues& x_u, const VectorValues& x_n);

  /** Compute the minimizer \f$ \delta x_u \f$ of the line search along the gradient direction \f$ g \f$ of
   * the function
   * \f[
   * M(\delta x) = (R \delta x - d)^T (R \delta x - d)
   * \f]
   * where \f$ R \f$ is an upper-triangular matrix and \f$ d \f$ is a vector.
   * Together \f$ (R,d) \f$ are either a Bayes' net or a Bayes' tree.
   *
   * The same quadratic error function written as a Taylor expansion of the original
   * non-linear error function is
   * \f[
   * M(\delta x) = f(x_0) + g(x_0) + \frac{1}{2} \delta x^T G(x_0) \delta x,
   * \f]
   * @tparam M The type of the Bayes' net or tree, currently
   * either BayesNet<GaussianConditional> (or GaussianBayesNet) or BayesTree<GaussianConditional>.
   * @param Rd The Bayes' net or tree \f$ (R,d) \f$ as described above, currently
   * this must be of type BayesNet<GaussianConditional> (or GaussianBayesNet) or
   * BayesTree<GaussianConditional>.
   * @return The minimizer \f$ \delta x_u \f$ along the gradient descent direction.
   */
  template<class M>
  static VectorValues ComputeSteepestDescentPoint(const M& Rd);

  /** Compute the point on the line between the steepest descent point and the
   * Newton's method point intersecting the trust region boundary.
   * Mathematically, computes \f$ \tau \f$ such that \f$ 0<\tau<1 \f$ and
   * \f$ \| (1-\tau)\delta x_u + \tau\delta x_n \| = \Delta \f$, where \f$ \Delta \f$
   * is the trust region radius.
   * @param Delta Trust region radius
   * @param xu Steepest descent minimizer
   * @param xn Newton's method minimizer
   */
  static VectorValues ComputeBlend(double Delta, const VectorValues& x_u, const VectorValues& x_n);
};


/* ************************************************************************* */
template<class M, class F, class VALUES>
typename DoglegOptimizerImpl::IterationResult DoglegOptimizerImpl::Iterate(
    double Delta, TrustRegionAdaptationMode mode, const M& Rd,
    const F& f, const VALUES& x0, const Ordering& ordering, double f_error) {

  // Compute steepest descent and Newton's method points
  VectorValues dx_u = ComputeSteepestDescentPoint(Rd);
  VectorValues dx_n = optimize(Rd);
  const GaussianFactorGraph jfg(Rd);
  const double M_error = jfg.error(VectorValues::Zero(dx_u));

  // Result to return
  IterationResult result;

  bool stay = true;
  while(stay) {
    // Compute dog leg point
    result.dx_d = ComputeDoglegPoint(Delta, dx_u, dx_n);

    cout << "Delta = " << Delta << ", dx_d_norm = " << result.dx_d.vector().norm() << endl;

    // Compute expmapped solution
    const VALUES x_d(x0.expmap(result.dx_d, ordering));

    // Compute decrease in f
    result.f_error = f.error(x_d);

    // Compute decrease in M
    const double new_M_error = jfg.error(result.dx_d);

    cout << "f error: " << f_error << " -> " << result.f_error << endl;
    cout << "M error: " << M_error << " -> " << new_M_error << endl;

    // Compute gain ratio.  Here we take advantage of the invariant that the
    // Bayes' net error at zero is equal to the nonlinear error
    const double rho = fabs(M_error - new_M_error) < 1e-30 ?
        0.5 :
        (f_error - result.f_error) / (M_error - new_M_error);

    cout << "rho = " << rho << endl;

    if(rho >= 0.75) {
      // M agrees very well with f, so try to increase lambda
      const double dx_d_norm = result.dx_d.vector().norm();
      const double newDelta = std::max(Delta, 3.0 * dx_d_norm); // Compute new Delta


      if(mode == ONE_STEP_PER_ITERATION)
        stay = false;   // If not searching, just return with the new Delta
      else if(mode == SEARCH_EACH_ITERATION) {
        if(newDelta == Delta)
          stay = false; // Searching, but Newton's solution is within trust region so keep the same trust region
        else
          stay = true;  // Searching and increased Delta, so try again to increase Delta
      } else {
        assert(false); }

      Delta = newDelta; // Update Delta from new Delta

    } else if(0.75 > rho && rho >= 0.25) {
      // M agrees so-so with f, keep the same Delta
      stay = false;

    } else if(0.25 > rho && rho >= 0.0) {
      // M does not agree well with f, decrease Delta until it does
      Delta *= 0.5;
      if(mode == ONE_STEP_PER_ITERATION)
        stay = false;   // If not searching, just return with the new smaller delta
      else if(mode == SEARCH_EACH_ITERATION)
        stay = true;
      else {
        assert(false); }
    }

    else {
      // f actually increased, so keep decreasing Delta until f does not decrease
      assert(0.0 > rho);
      Delta *= 0.5;
      if(Delta > 1e-5)
        stay = true;
      else
        stay = false;
    }
  }

  // dx_d and f_error have already been filled in during the loop
  result.Delta = Delta;
  return result;
}

/* ************************************************************************* */
template<class M>
VectorValues DoglegOptimizerImpl::ComputeSteepestDescentPoint(const M& Rd) {

  // Compute gradient
  // Convert to JacobianFactor's to use existing gradient function
  FactorGraph<JacobianFactor> jfg(Rd);
  VectorValues grad = gradient(jfg, VectorValues::Zero(*allocateVectorValues(Rd)));
  double gradientSqNorm = grad.dot(grad);

  // Compute R * g
  Errors Rg = jfg * grad;

  // Compute minimizing step size
  double step = -gradientSqNorm / dot(Rg, Rg);

  // Compute steepest descent point
  scal(step, grad);
  return grad;
}

}
