/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

#pragma once

#include <gtsam/linear/IterativeSolver.h>

namespace gtsam {

/**
 * parameters for the conjugate gradient method
 */

class ConjugateGradientParameters : public IterativeOptimizationParameters {
public:
  typedef IterativeOptimizationParameters Base;
  typedef boost::shared_ptr<ConjugateGradientParameters> shared_ptr;

  size_t minIterations_;  ///< minimum number of cg iterations
  size_t maxIterations_;  ///< maximum number of cg iterations
  size_t reset_;          ///< number of iterations before reset
  double epsilon_rel_;    ///< threshold for relative error decrease
  double epsilon_abs_;    ///< threshold for absolute error decrease

  ConjugateGradientParameters()
  : minIterations_(1), maxIterations_(500), reset_(501), epsilon_rel_(1e-3), epsilon_abs_(1e-3){}

  ConjugateGradientParameters(size_t minIterations, size_t maxIterations, size_t reset, double epsilon_rel, double epsilon_abs)
    : minIterations_(minIterations), maxIterations_(maxIterations), reset_(reset), epsilon_rel_(epsilon_rel), epsilon_abs_(epsilon_abs){}

  ConjugateGradientParameters(const ConjugateGradientParameters &p)
    : Base(p), minIterations_(p.minIterations_), maxIterations_(p.maxIterations_), reset_(p.reset_), epsilon_rel_(p.epsilon_rel_), epsilon_abs_(p.epsilon_abs_) {}

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

  virtual void print(const std::string &s="") const {
    Base::print();
    std::cout << "ConjugateGradientParameters: "
              << "minIter = " << minIterations_
              << ", maxIter = " << maxIterations_
              << ", resetIter = " << reset_
              << ", eps_rel = " << epsilon_rel_
              << ", eps_abs = " << epsilon_abs_
              << std::endl;
  }
};

}
