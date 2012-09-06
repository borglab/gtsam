/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

#pragma once

#include <gtsam/linear/GaussianFactorGraph.h>
#include <gtsam/linear/VectorValues.h>
#include <string>

namespace gtsam {

  /**
   * parameters for iterative linear solvers
   */
  class IterativeOptimizationParameters {

  public:

    typedef boost::shared_ptr<IterativeOptimizationParameters> shared_ptr;
    enum Kernel { CG = 0 } kernel_ ;                                          ///< Iterative Method Kernel
    enum Verbosity { SILENT = 0, COMPLEXITY, ERROR } verbosity_;

  public:

    IterativeOptimizationParameters(const IterativeOptimizationParameters &p)
      : kernel_(p.kernel_), verbosity_(p.verbosity_) {}

    IterativeOptimizationParameters(const Kernel kernel = CG, const Verbosity verbosity = SILENT)
      : kernel_(kernel), verbosity_(verbosity) {}

    virtual ~IterativeOptimizationParameters() {}

    /* general interface */
    inline Kernel kernel() const { return kernel_; }
    inline Verbosity verbosity() const { return verbosity_; }

    /* matlab interface */
    std::string getKernel() const ;
    std::string getVerbosity() const;
    void setKernel(const std::string &s) ;
    void setVerbosity(const std::string &s) ;

    void print() const {
      std::cout << "IterativeOptimizationParameters: "
                << "kernel = " << kernelTranslator(kernel_)
                << ", verbosity = " << verbosityTranslator(verbosity_) << std::endl;
    }

    static Kernel kernelTranslator(const std::string &s);
    static Verbosity verbosityTranslator(const std::string &s);
    static std::string kernelTranslator(Kernel k);
    static std::string verbosityTranslator(Verbosity v);
  };

  class IterativeSolver {
  public:
    typedef boost::shared_ptr<IterativeSolver> shared_ptr;
    IterativeSolver() {}
    virtual ~IterativeSolver() {}

    /* interface to the nonlinear optimizer  */
    virtual VectorValues optimize () = 0;
    /* update interface to the nonlinear optimizer  */
    virtual void replaceFactors(const GaussianFactorGraph::shared_ptr &factorGraph, const double lambda) {}
  };

}
