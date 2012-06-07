/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file IterativeOptimizationParameters.h
 * @date Oct 22, 2010
 * @author Yong-Dian Jian
 */

#pragma once

#include <boost/shared_ptr.hpp>
#include <iostream>
#include <string>

namespace gtsam {

/**
 * parameters for the conjugate gradient method
 */
struct ConjugateGradientParameters {

  size_t minIterations_;  ///< minimum number of cg iterations
  size_t maxIterations_;  ///< maximum number of cg iterations
  size_t reset_;          ///< number of iterations before reset
  double epsilon_rel_;    ///< threshold for relative error decrease
  double epsilon_abs_;    ///< threshold for absolute error decrease

  /* Matrix Operation Kernel */
  enum BLASKernel {
    GTSAM = 0,        ///< Jacobian Factor Graph of GTSAM
    SBM,              ///< Sparse Block Matrix
    SBM_MT            ///< Sparse Block Matrix Multithreaded
  } blas_kernel_;

  size_t degree_;     ///< the maximum degree of the vertices to be eliminated before doing cg

  enum Verbosity { SILENT = 0, COMPLEXITY = 1, ERROR = 2} verbosity_ ;  /* Verbosity */

  ConjugateGradientParameters()
  : minIterations_(1), maxIterations_(500), reset_(501), epsilon_rel_(1e-3), epsilon_abs_(1e-3),
    blas_kernel_(GTSAM), degree_(0), verbosity_(SILENT) {}

  ConjugateGradientParameters(size_t minIterations, size_t maxIterations, size_t reset,
    double epsilon_rel, double epsilon_abs, BLASKernel blas = GTSAM, size_t degree = 0, Verbosity verbosity = SILENT)
    : minIterations_(minIterations), maxIterations_(maxIterations), reset_(reset),
      epsilon_rel_(epsilon_rel), epsilon_abs_(epsilon_abs), blas_kernel_(blas), degree_(degree), verbosity_(verbosity) {}

  /* general interface */
  inline size_t minIterations() const { return minIterations_; }
  inline size_t maxIterations() const { return maxIterations_; }
  inline size_t reset() const { return reset_; }
  inline double epsilon() const { return epsilon_rel_; }
  inline double epsilon_rel() const { return epsilon_rel_; }
  inline double epsilon_abs() const { return epsilon_abs_; }
  inline BLASKernel blas_kernel() const { return blas_kernel_; }
  inline size_t degree() const { return degree_; }
  inline Verbosity verbosity() const { return verbosity_; }

  void print() const {
    const std::string blasStr[3] = {"gtsam", "sbm", "sbm-mt"};
    std::cout << "ConjugateGradientParameters: "
              << "blas = " << blasStr[blas_kernel_]
              << ", minIter = " << minIterations_
              << ", maxIter = " << maxIterations_
              << ", resetIter = " << reset_
              << ", eps_rel = " << epsilon_rel_
              << ", eps_abs = " << epsilon_abs_
              << ", degree = " << degree_
              << ", verbosity = " << verbosity_
              << std::endl;
  }
};

/**
 * parameters for iterative linear solvers
 */
class IterativeOptimizationParameters {

public:

  typedef boost::shared_ptr<IterativeOptimizationParameters> shared_ptr;

  ConjugateGradientParameters cg_;                                      ///< Parameters for the Conjugate Gradient Method
  enum Kernel { PCG = 0, LSPCG = 1 } kernel_ ;                          ///< Iterative Method Kernel
  enum Verbosity { SILENT = 0, COMPLEXITY = 1, ERROR = 2} verbosity_ ;  ///< Verbosity

public:

  IterativeOptimizationParameters() : cg_(), kernel_(LSPCG), verbosity_(SILENT) {}

  IterativeOptimizationParameters(const IterativeOptimizationParameters &p)
    : cg_(p.cg_), kernel_(p.kernel_), verbosity_(p.verbosity_) {}

  IterativeOptimizationParameters(const ConjugateGradientParameters &c, Kernel kernel = LSPCG, Verbosity verbosity = SILENT)
    : cg_(c), kernel_(kernel), verbosity_(verbosity) {}

  virtual ~IterativeOptimizationParameters() {}

  /* general interface */
  inline Kernel kernel() const { return kernel_; }
  inline Verbosity verbosity() const { return verbosity_; }

  /* interface to cg parameters */
  inline const ConjugateGradientParameters& cg() const { return cg_; }
  inline size_t minIterations() const { return cg_.minIterations(); }
  inline size_t maxIterations() const { return cg_.maxIterations(); }
  inline size_t reset() const { return cg_.reset(); }
  inline double epsilon() const { return cg_.epsilon_rel(); }
  inline double epsilon_rel() const { return cg_.epsilon_rel(); }
  inline double epsilon_abs() const { return cg_.epsilon_abs(); }
  inline size_t degree() const { return cg_.degree(); }
  inline ConjugateGradientParameters::BLASKernel blas_kernel() const { return cg_.blas_kernel(); }

  virtual void print(const std::string &s="") const {
    const std::string kernelStr[2] = {"pcg", "lspcg"};
    std::cout << s << std::endl
              << "IterativeOptimizationParameters: "
              << "kernel = " << kernelStr[kernel_]
              << ", verbosity = " << verbosity_ << std::endl;
    cg_.print();
  }

};

}
