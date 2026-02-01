/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    DoglegOptimizer.h
 * @brief
 * @author  Richard Roberts
 * @date   Feb 26, 2012
 */

#pragma once

#include <gtsam/nonlinear/NonlinearOptimizer.h>

namespace gtsam {

class DoglegOptimizer;

/** Parameters for Levenberg-Marquardt optimization.  Note that this parameters
 * class inherits from NonlinearOptimizerParams, which specifies the parameters
 * common to all nonlinear optimization algorithms.  This class also contains
 * all of those parameters.
 */
class GTSAM_EXPORT DoglegParams : public NonlinearOptimizerParams {
public:
  /** See DoglegParams::dlVerbosity */
  enum VerbosityDL {
    SILENT,
    VERBOSE
  };

  double deltaInitial; ///< The initial trust region radius (default: 10.0)
  VerbosityDL verbosityDL; ///< The verbosity level for Dogleg (default: SILENT), see also NonlinearOptimizerParams::verbosity

  DoglegParams() :
    deltaInitial(1.0), verbosityDL(SILENT) {}

  ~DoglegParams() override {}

  void print(const std::string& str = "") const override {
    NonlinearOptimizerParams::print(str);
    std::cout << "               deltaInitial: " << deltaInitial << "\n";
    std::cout.flush();
  }

  double getDeltaInitial() const { return deltaInitial; }
  std::string getVerbosityDL() const { return verbosityDLTranslator(verbosityDL); }

  void setDeltaInitial(double deltaInitial) { this->deltaInitial = deltaInitial; }
  void setVerbosityDL(const std::string& verbosityDL) { this->verbosityDL = verbosityDLTranslator(verbosityDL); }

private:
  VerbosityDL verbosityDLTranslator(const std::string& verbosityDL) const;
  std::string verbosityDLTranslator(VerbosityDL verbosityDL) const;
};

/**
 * This class performs Dogleg nonlinear optimization
 */
class GTSAM_EXPORT DoglegOptimizer : public NonlinearOptimizer {

protected:
  DoglegParams params_;

public:
  typedef std::shared_ptr<DoglegOptimizer> shared_ptr;

  /// @name Standard interface
  /// @{

  /** Standard constructor, requires a nonlinear factor graph, initial
   * variable assignments, and optimization parameters.  For convenience this
   * version takes plain objects instead of shared pointers, but internally
   * copies the objects.
   * @param graph The nonlinear factor graph to optimize
   * @param initialValues The initial variable assignments
   * @param params The optimization parameters
   */
  DoglegOptimizer(const NonlinearFactorGraph& graph, const Values& initialValues,
      const DoglegParams& params = DoglegParams());

  /** Standard constructor, requires a nonlinear factor graph, initial
   * variable assignments, and optimization parameters.  For convenience this
   * version takes plain objects instead of shared pointers, but internally
   * copies the objects.
   * @param graph The nonlinear factor graph to optimize
   * @param initialValues The initial variable assignments
   */
  DoglegOptimizer(const NonlinearFactorGraph& graph, const Values& initialValues,
                  const Ordering& ordering);

  /// @}

  /// @name Advanced interface
  /// @{

  /** Virtual destructor */
  ~DoglegOptimizer() override {}

  /** 
   * Perform a single iteration, returning GaussianFactorGraph corresponding to 
   * the linearized factor graph.
   */
  GaussianFactorGraph::shared_ptr iterate() override;

  /** Read-only access the parameters */
  const DoglegParams& params() const { return params_; }

  /** Access the current trust region radius delta */
  double getDelta() const;

  /// @}

protected:
  /** Access the parameters (base class version) */
  const NonlinearOptimizerParams& _params() const override { return params_; }

  /** Internal function for computing a COLAMD ordering if no ordering is specified */
  DoglegParams ensureHasOrdering(DoglegParams params, const NonlinearFactorGraph& graph) const;
};

}
