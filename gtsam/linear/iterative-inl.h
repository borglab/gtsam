/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file iterative-inl.h
 * @brief Iterative methods, template implementation
 * @author Frank Dellaert
 * @date Dec 28, 2009
 */

#pragma once

#include <gtsam/linear/iterative.h>
#include <gtsam/linear/ConjugateGradientSolver.h>

namespace gtsam {

  /* ************************************************************************* */
  // state for CG method
  template<class S, class V, class E>
  struct CGState {

    typedef ConjugateGradientParameters Parameters;
    const Parameters &parameters_;

    int k;                     ///< iteration
    bool steepest;             ///< flag to indicate we are doing steepest descent
    V g, d;                    ///< gradient g and search direction d for CG
    double gamma, threshold;   ///< gamma (squared L2 norm of g) and convergence threshold
    E Ad;

    /* ************************************************************************* */
    // Constructor
    CGState(const S& Ab, const V& x, const Parameters &parameters, bool steep):
    parameters_(parameters),k(0),steepest(steep) {

      // Start with g0 = A'*(A*x0-b), d0 = - g0
      // i.e., first step is in direction of negative gradient
      g = Ab.gradient(x);
      d = g; // instead of negating gradient, alpha will be negated

      // init gamma and calculate threshold
      gamma = dot(g,g);
      threshold = std::max(parameters_.epsilon_abs(), parameters_.epsilon() * parameters_.epsilon() * gamma);

      // Allocate and calculate A*d for first iteration
      if (gamma > parameters_.epsilon_abs()) Ad = Ab * d;
    }

    /* ************************************************************************* */
    // print
    void print(const V& x) {
      std::cout << "iteration = " << k << std::endl;
      gtsam::print(x,"x");
      gtsam::print(g, "g");
      std::cout << "dotg = " << gamma << std::endl;
      gtsam::print(d, "d");
      gtsam::print(Ad, "Ad");
    }

    /* ************************************************************************* */
    // step the solution
    double takeOptimalStep(V& x) {
      // TODO: can we use gamma instead of dot(d,g) ????? Answer not trivial
      double alpha = -dot(d, g) / dot(Ad, Ad); // calculate optimal step-size
      x += alpha * d; // do step in new search direction, x += alpha*d
      return alpha;
    }

    /* ************************************************************************* */
    // take a step, return true if converged
    bool step(const S& Ab, V& x) {

      if ((++k) >= ((int)parameters_.maxIterations())) return true;

      //---------------------------------->
      double alpha = takeOptimalStep(x);

      // update gradient (or re-calculate at reset time)
      if (k % parameters_.reset() == 0) g = Ab.gradient(x);
      // axpy(alpha, Ab ^ Ad, g);  // g += alpha*(Ab^Ad)
      else Ab.transposeMultiplyAdd(alpha, Ad, g);

      // check for convergence
      double new_gamma = dot(g, g);

      if (parameters_.verbosity() != ConjugateGradientParameters::SILENT)
        std::cout << "iteration " << k << ": alpha = " << alpha
                  << ", dotg = " << new_gamma
                  << std::endl;

      if (new_gamma < threshold) return true;

      // calculate new search direction
      if (steepest) d = g;
      else {
        double beta = new_gamma / gamma;
        // d = g + d*beta;
        d *= beta;
        d += 1.0 * g;
      }

      gamma = new_gamma;

      // In-place recalculation Ad <- A*d to avoid re-allocating Ad
      Ab.multiplyInPlace(d, Ad);
      return false;
    }

  }; // CGState Class

  /* ************************************************************************* */
  // conjugate gradient method.
  // S: linear system, V: step vector, E: errors
  template<class S, class V, class E>
  V conjugateGradients(const S& Ab,  V x, const ConjugateGradientParameters &parameters, bool steepest) {

    CGState<S, V, E> state(Ab, x, parameters, steepest);

    if (parameters.verbosity() != ConjugateGradientParameters::SILENT)
      std::cout << "CG: epsilon = " << parameters.epsilon()
                << ", maxIterations = " << parameters.maxIterations()
                << ", ||g0||^2 = " << state.gamma
                << ", threshold = " << state.threshold
                << std::endl;

    if ( state.gamma < state.threshold ) {
      if (parameters.verbosity() != ConjugateGradientParameters::SILENT)
        std::cout << "||g0||^2 < threshold, exiting immediately !" << std::endl;

      return x;
    }

    // loop maxIterations times
    while (!state.step(Ab, x)) {}
    return x;
  }
/* ************************************************************************* */

} // namespace gtsam
