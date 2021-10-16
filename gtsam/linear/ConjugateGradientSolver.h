/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 *  @file   ConjugateGradientSolver.h
 *  @brief  Implementation of Conjugate Gradient solver for a linear system
 *  @author Yong-Dian Jian
 *  @author Sungtae An
 *  @date   Nov 6, 2014
 **/

#pragma once

#include <gtsam/linear/IterativeSolver.h>

namespace gtsam {

/**
 * parameters for the conjugate gradient method
 */
class GTSAM_EXPORT ConjugateGradientParameters : public IterativeOptimizationParameters {

public:
  typedef IterativeOptimizationParameters Base;
  typedef std::shared_ptr<ConjugateGradientParameters> shared_ptr;

  size_t minIterations_;  ///< minimum number of cg iterations
  size_t maxIterations_;  ///< maximum number of cg iterations
  size_t reset_;          ///< number of iterations before reset
  double epsilon_rel_;    ///< threshold for relative error decrease
  double epsilon_abs_;    ///< threshold for absolute error decrease

  /* Matrix Operation Kernel */
  enum BLASKernel {
    GTSAM = 0,        ///< Jacobian Factor Graph of GTSAM
  } blas_kernel_ ;

  ConjugateGradientParameters()
    : minIterations_(1), maxIterations_(500), reset_(501), epsilon_rel_(1e-3),
      epsilon_abs_(1e-3), blas_kernel_(GTSAM) {}

  ConjugateGradientParameters(size_t minIterations, size_t maxIterations, size_t reset,
    double epsilon_rel, double epsilon_abs, BLASKernel blas)
    : minIterations_(minIterations), maxIterations_(maxIterations), reset_(reset),
      epsilon_rel_(epsilon_rel), epsilon_abs_(epsilon_abs), blas_kernel_(blas) {}

  ConjugateGradientParameters(const ConjugateGradientParameters &p)
    : Base(p), minIterations_(p.minIterations_), maxIterations_(p.maxIterations_), reset_(p.reset_),
               epsilon_rel_(p.epsilon_rel_), epsilon_abs_(p.epsilon_abs_), blas_kernel_(GTSAM) {}

  /* general interface */
  inline size_t minIterations() const { return minIterations_; }
  inline size_t maxIterations() const { return maxIterations_; }
  inline size_t reset() const { return reset_; }
  inline double epsilon() const { return epsilon_rel_; }
  inline double epsilon_rel() const { return epsilon_rel_; }
  inline double epsilon_abs() const { return epsilon_abs_; }

  inline size_t getMinIterations() const { return minIterations_; }
  inline size_t getMaxIterations() const { return maxIterations_; }
  inline size_t getReset() const { return reset_; }
  inline double getEpsilon() const { return epsilon_rel_; }
  inline double getEpsilon_rel() const { return epsilon_rel_; }
  inline double getEpsilon_abs() const { return epsilon_abs_; }

  inline void setMinIterations(size_t value) { minIterations_ = value; }
  inline void setMaxIterations(size_t value) { maxIterations_ = value; }
  inline void setReset(size_t value) { reset_ = value; }
  inline void setEpsilon(double value) { epsilon_rel_ = value; }
  inline void setEpsilon_rel(double value) { epsilon_rel_ = value; }
  inline void setEpsilon_abs(double value) { epsilon_abs_ = value; }


  void print() const { Base::print(); }
  void print(std::ostream &os) const override;

  static std::string blasTranslator(const BLASKernel k) ;
  static BLASKernel blasTranslator(const std::string &s) ;
};

/*
 * A template for the linear preconditioned conjugate gradient method.
 * System class should support residual(v, g), multiply(v,Av), scal(alpha,v), dot(v,v), axpy(alpha,x,y)
 * leftPrecondition(v, L^{-1}v, rightPrecondition(v, L^{-T}v) where preconditioner M = L*L^T
 * Note that the residual is in the preconditioned domain. Refer to Section 9.2 of Saad's book.
 *
 ** REFERENCES:
 * [1] Y. Saad, "Preconditioned Iterations," in Iterative Methods for Sparse Linear Systems,
 * 2nd ed. SIAM, 2003, ch. 9, sec. 2, pp.276-281.
 */
template<class S, class V>
V preconditionedConjugateGradient(const S &system, const V &initial,
    const ConjugateGradientParameters &parameters) {

  V estimate, residual, direction, q1, q2;
  estimate = residual = direction = q1 = q2 = initial;

  system.residual(estimate, q1);                /* q1 = b-Ax */
  system.leftPrecondition(q1, residual);        /* r = L^{-1} (b-Ax) */
  system.rightPrecondition(residual, direction);/* p = L^{-T} r */

  double currentGamma = system.dot(residual, residual), prevGamma, alpha, beta;

  const size_t iMaxIterations = parameters.maxIterations(),
               iMinIterations = parameters.minIterations(),
               iReset = parameters.reset() ;
  const double threshold = std::max(parameters.epsilon_abs(),
                                    parameters.epsilon() * parameters.epsilon() * currentGamma);

  if (parameters.verbosity() >= ConjugateGradientParameters::COMPLEXITY )
    std::cout << "[PCG] epsilon = " << parameters.epsilon()
             << ", max = " << parameters.maxIterations()
             << ", reset = " << parameters.reset()
             << ", ||r0||^2 = " << currentGamma
             << ", threshold = " << threshold << std::endl;

  size_t k;
  for ( k = 1 ; k <= iMaxIterations && (currentGamma > threshold || k <= iMinIterations) ; k++ ) {

    if ( k % iReset == 0 ) {
      system.residual(estimate, q1);                      /* q1 = b-Ax */
      system.leftPrecondition(q1, residual);              /* r = L^{-1} (b-Ax) */
      system.rightPrecondition(residual, direction);      /* p = L^{-T} r */
      currentGamma = system.dot(residual, residual);
    }
    system.multiply(direction, q1);                       /* q1 = A p */
    alpha = currentGamma / system.dot(direction, q1);     /* alpha = gamma / (p' A p) */
    system.axpy(alpha, direction, estimate);              /* estimate += alpha * p */
    system.leftPrecondition(q1, q2);                      /* q2 = L^{-1} * q1 */
    system.axpy(-alpha, q2, residual);                    /* r -= alpha * q2 */
    prevGamma = currentGamma;
    currentGamma = system.dot(residual, residual);        /* gamma = |r|^2 */
    beta = currentGamma / prevGamma;
    system.rightPrecondition(residual, q1);               /* q1 = L^{-T} r */
    system.scal(beta, direction);
    system.axpy(1.0, q1, direction);                      /* p = q1 + beta * p */

    if (parameters.verbosity() >= ConjugateGradientParameters::ERROR )
       std::cout << "[PCG] k = " << k
                 << ", alpha = " << alpha
                 << ", beta = " << beta
                 << ", ||r||^2 = " << currentGamma
//                 << "\nx =\n" << estimate
//                 << "\nr =\n" << residual
                 << std::endl;
  }
  if (parameters.verbosity() >= ConjugateGradientParameters::COMPLEXITY )
     std::cout << "[PCG] iterations = " << k
               << ", ||r||^2 = " << currentGamma
               << std::endl;

  return estimate;
}


}
