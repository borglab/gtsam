/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    DoglegOptimizerImpl.h
 * @brief   Nonlinear factor graph optimizer using Powell's Dogleg algorithm (detail implementation)
 * @author  Richard Roberts
 */
#pragma once

#include <iomanip>

#include <gtsam/linear/GaussianBayesNet.h>
#include <gtsam/linear/GaussianISAM.h> // To get optimize(BayesTree<GaussianConditional>)
//#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Ordering.h>

namespace gtsam {

/** This class contains the implementation of the Dogleg algorithm.  It is used
 * by DoglegOptimizer and can be used to easily put together custom versions of
 * Dogleg.  Each function is well-documented and unit-tested.  The notation
 * here matches that in "trustregion.pdf" in gtsam_experimental/doc, see this
 * file for further explanation of the computations performed by this class.
 *
 * \tparam VALUES The Values or TupleValues type to hold the values to be
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

  /** Specifies how the trust region is adapted at each Dogleg iteration.  If
   * this is SEARCH_EACH_ITERATION, then the trust region radius will be
   * increased potentially multiple times during one iteration until increasing
   * it further no longer decreases the error.  If this is
   * ONE_STEP_PER_ITERATION, then the step in one iteration will not exceed the
   * current trust region radius, but the radius will be increased for the next
   * iteration if the error decrease is good.  The former will generally result
   * in slower iterations, but sometimes larger steps in early iterations.  The
   * latter generally results in faster iterations but it may take several
   * iterations before the trust region radius is increased to the optimal
   * value.  Generally ONE_STEP_PER_ITERATION should be used, corresponding to
   * most published descriptions of the algorithm.
   */
  enum TrustRegionAdaptationMode {
    SEARCH_EACH_ITERATION,
		SEARCH_REDUCE_ONLY,
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
   * The quadratic approximation is represented as a GaussianBayesNet \f$ \bayesNet \f$, which is
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
   * @tparam VALUES The Values or TupleValues to pass to F::error() to evaluate
   * the error function.
   * @param Delta The initial trust region radius.
   * @param mode See DoglegOptimizerImpl::TrustRegionAdaptationMode
   * @param Rd The Bayes' net or tree as described above.
   * @param f The original nonlinear factor graph with which to evaluate the
   * accuracy of \f$ M(\delta x) \f$ to adjust \f$ \Delta \f$.
   * @param x0 The linearization point about which \f$ \bayesNet \f$ was created
   * @param ordering The variable ordering used to create\f$ \bayesNet \f$
   * @param f_error The result of <tt>f.error(x0)</tt>.
   * @return A DoglegIterationResult containing the new \c Delta, the linear
   * update \c dx_d, and the resulting nonlinear error \c f_error.
   */
  template<class M, class F, class VALUES>
  static IterationResult Iterate(
      double Delta, TrustRegionAdaptationMode mode, const M& Rd,
      const F& f, const VALUES& x0, const Ordering& ordering, const double f_error, const bool verbose=false);

  /**
   * Compute the dogleg point given a trust region radius \f$ \Delta \f$.  The
   * dogleg point is the intersection between the dogleg path and the trust
   * region boundary, see doc/trustregion.pdf for more details.
   *
   * The update is computed using a quadratic approximation \f$ M(\delta x) \f$
   * of an original nonlinear error function (a NonlinearFactorGraph) \f$ f(x) \f$.
   * The quadratic approximation is represented as a GaussianBayesNet \f$ \bayesNet \f$, which is
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
   * @param dx_u The steepest descent point, i.e. the Cauchy point
   * @param dx_n The Gauss-Newton point
   * @return The dogleg point \f$ \delta x_d \f$
   */
  static VectorValues ComputeDoglegPoint(double Delta, const VectorValues& dx_u, const VectorValues& dx_n, const bool verbose=false);

  /** Compute the point on the line between the steepest descent point and the
   * Newton's method point intersecting the trust region boundary.
   * Mathematically, computes \f$ \tau \f$ such that \f$ 0<\tau<1 \f$ and
   * \f$ \| (1-\tau)\delta x_u + \tau\delta x_n \| = \Delta \f$, where \f$ \Delta \f$
   * is the trust region radius.
   * @param Delta Trust region radius
   * @param x_u Steepest descent minimizer
   * @param x_n Newton's method minimizer
   */
  static VectorValues ComputeBlend(double Delta, const VectorValues& x_u, const VectorValues& x_n, const bool verbose=false);
};


/* ************************************************************************* */
template<class M, class F, class VALUES>
typename DoglegOptimizerImpl::IterationResult DoglegOptimizerImpl::Iterate(
    double Delta, TrustRegionAdaptationMode mode, const M& Rd,
    const F& f, const VALUES& x0, const Ordering& ordering, const double f_error, const bool verbose) {

  // Compute steepest descent and Newton's method points
  tic(0, "optimizeGradientSearch");
  tic(0, "allocateVectorValues");
  VectorValues dx_u = *allocateVectorValues(Rd);
  toc(0, "allocateVectorValues");
  tic(1, "optimizeGradientSearchInPlace");
  optimizeGradientSearchInPlace(Rd, dx_u);
  toc(1, "optimizeGradientSearchInPlace");
  toc(0, "optimizeGradientSearch");
  tic(1, "optimizeInPlace");
  VectorValues dx_n(VectorValues::SameStructure(dx_u));
  optimizeInPlace(Rd, dx_n);
  toc(1, "optimizeInPlace");
  tic(2, "jfg error");
  const GaussianFactorGraph jfg(Rd);
  const double M_error = jfg.error(VectorValues::Zero(dx_u));
  toc(2, "jfg error");

  // Result to return
  IterationResult result;

  bool stay = true;
  enum { NONE, INCREASED_DELTA, DECREASED_DELTA } lastAction = NONE; // Used to prevent alternating between increasing and decreasing in one iteration
  while(stay) {
    tic(3, "Dog leg point");
    // Compute dog leg point
    result.dx_d = ComputeDoglegPoint(Delta, dx_u, dx_n, verbose);
    toc(3, "Dog leg point");

    if(verbose) std::cout << "Delta = " << Delta << ", dx_d_norm = " << result.dx_d.vector().norm() << std::endl;

    tic(4, "retract");
    // Compute expmapped solution
    const VALUES x_d(x0.retract(result.dx_d, ordering));
    toc(4, "retract");

    tic(5, "decrease in f");
    // Compute decrease in f
    result.f_error = f.error(x_d);
    toc(5, "decrease in f");

    tic(6, "decrease in M");
    // Compute decrease in M
    const double new_M_error = jfg.error(result.dx_d);
    toc(6, "decrease in M");

    if(verbose) std::cout << std::setprecision(15) << "f error: " << f_error << " -> " << result.f_error << std::endl;
    if(verbose) std::cout << std::setprecision(15) << "M error: " << M_error << " -> " << new_M_error << std::endl;

    tic(7, "adjust Delta");
    // Compute gain ratio.  Here we take advantage of the invariant that the
    // Bayes' net error at zero is equal to the nonlinear error
    const double rho = fabs(f_error - result.f_error) < 1e-15 || fabs(M_error - new_M_error) < 1e-15 ?
        0.5 :
        (f_error - result.f_error) / (M_error - new_M_error);

    if(verbose) std::cout << std::setprecision(15) << "rho = " << rho << std::endl;

    if(rho >= 0.75) {
      // M agrees very well with f, so try to increase lambda
      const double dx_d_norm = result.dx_d.vector().norm();
      const double newDelta = std::max(Delta, 3.0 * dx_d_norm); // Compute new Delta

      if(mode == ONE_STEP_PER_ITERATION || mode == SEARCH_REDUCE_ONLY)
        stay = false;   // If not searching, just return with the new Delta
      else if(mode == SEARCH_EACH_ITERATION) {
        if(newDelta == Delta || lastAction == DECREASED_DELTA)
          stay = false; // Searching, but Newton's solution is within trust region so keep the same trust region
        else {
          stay = true;  // Searching and increased Delta, so try again to increase Delta
          lastAction = INCREASED_DELTA;
        }
      } else {
        assert(false); }

      Delta = newDelta; // Update Delta from new Delta

    } else if(0.75 > rho && rho >= 0.25) {
      // M agrees so-so with f, keep the same Delta
      stay = false;

    } else if(0.25 > rho && rho >= 0.0) {
      // M does not agree well with f, decrease Delta until it does
      double newDelta;
			bool hitMinimumDelta;
      if(Delta > 1e-5) {
        newDelta = 0.5 * Delta;
				hitMinimumDelta = false;
			} else {
        newDelta = Delta;
				hitMinimumDelta = true;
			}
      if(mode == ONE_STEP_PER_ITERATION || /* mode == SEARCH_EACH_ITERATION && */ lastAction == INCREASED_DELTA || hitMinimumDelta)
        stay = false;   // If not searching, just return with the new smaller delta
      else if(mode == SEARCH_EACH_ITERATION || mode == SEARCH_REDUCE_ONLY) {
        stay = true;
        lastAction = DECREASED_DELTA;
      } else {
        assert(false); }

      Delta = newDelta; // Update Delta from new Delta

    } else {
			// f actually increased, so keep decreasing Delta until f does not decrease.
			// NOTE:  NaN and Inf solutions also will fall into this case, so that we
			// decrease Delta if the solution becomes undetermined.
      assert(0.0 > rho);
      if(Delta > 1e-5) {
        Delta *= 0.5;
        stay = true;
        lastAction = DECREASED_DELTA;
      } else {
        if(verbose) std::cout << "Warning:  Dog leg stopping because cannot decrease error with minimum Delta" << std::endl;
				result.dx_d.setZero(); // Set delta to zero - don't allow error to increase
				result.f_error = f_error;
        stay = false;
      }
    }
    toc(7, "adjust Delta");
  }

  // dx_d and f_error have already been filled in during the loop
  result.Delta = Delta;
  return result;
}

}
