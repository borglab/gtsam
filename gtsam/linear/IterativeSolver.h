/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

#pragma once

#include <gtsam/linear/VectorValues.h>

namespace gtsam {

  /**
   * parameters for iterative linear solvers
   */
  class IterativeOptimizationParameters {

  public:

    typedef boost::shared_ptr<IterativeOptimizationParameters> shared_ptr;
    enum Kernel { CG = 0 } kernel_ ;                                          ///< Iterative Method Kernel
    enum Verbosity { SILENT, ERROR } verbosity_;

  public:

    IterativeOptimizationParameters(const IterativeOptimizationParameters &p)
      : kernel_(p.kernel_), verbosity_(p.verbosity_) {}

    IterativeOptimizationParameters(const Kernel kernel = CG, const Verbosity verbosity = SILENT)
      : kernel_(kernel), verbosity_(verbosity) {}

    virtual ~IterativeOptimizationParameters() {}

    /* general interface */
    inline Kernel kernel() const { return kernel_; }
    inline Verbosity verbosity() const { return verbosity_; }

    void print() const {
      const std::string kernelStr[1] = {"cg"};
      std::cout << "IterativeOptimizationParameters: "
                << "kernel = " << kernelStr[kernel_]
                << ", verbosity = " << verbosity_ << std::endl;
    }
  };

  class IterativeSolver {
  public:
    IterativeSolver(){}
    virtual ~IterativeSolver() {}
    virtual VectorValues::shared_ptr optimize () = 0;
  };

}
