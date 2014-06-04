/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

#pragma once

#include <gtsam/global_includes.h>

#include <string>
#include <iostream>

namespace gtsam {

  // Forward declarations
  class VectorValues;
  class GaussianFactorGraph;

  /**
   * parameters for iterative linear solvers
   */
  class GTSAM_EXPORT IterativeOptimizationParameters {

  public:

    typedef boost::shared_ptr<IterativeOptimizationParameters> shared_ptr;
    enum Verbosity { SILENT = 0, COMPLEXITY, ERROR } verbosity_;

  public:

    IterativeOptimizationParameters(const IterativeOptimizationParameters &p)
      : verbosity_(p.verbosity_) {}

    IterativeOptimizationParameters(const Verbosity verbosity = SILENT)
      : verbosity_(verbosity) {}

    virtual ~IterativeOptimizationParameters() {}

    /* general interface */
    inline Verbosity verbosity() const { return verbosity_; }

    /* matlab interface */
    std::string getKernel() const ;
    std::string getVerbosity() const;
    void setKernel(const std::string &s) ;
    void setVerbosity(const std::string &s) ;

    virtual void print() const ;

    static Verbosity verbosityTranslator(const std::string &s);
    static std::string verbosityTranslator(Verbosity v);
  };

  class GTSAM_EXPORT IterativeSolver {
  public:
    typedef boost::shared_ptr<IterativeSolver> shared_ptr;
    IterativeSolver() {}
    virtual ~IterativeSolver() {}

    /* interface to the nonlinear optimizer  */
    virtual VectorValues optimize () = 0;

    /* interface to the nonlinear optimizer  */
    virtual VectorValues optimize (const VectorValues &initial) = 0;

    /* update interface to the nonlinear optimizer  */
    virtual void replaceFactors(const boost::shared_ptr<GaussianFactorGraph> &factorGraph, const double lambda) {}
  };

}
