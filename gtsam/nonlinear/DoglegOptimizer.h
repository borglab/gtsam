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
 * @date 	Feb 26, 2012
 */

#pragma once

#include <gtsam/nonlinear/SuccessiveLinearizationOptimizer.h>

namespace gtsam {

class DoglegOptimizer;

/** Parameters for Levenberg-Marquardt optimization.  Note that this parameters
 * class inherits from NonlinearOptimizerParams, which specifies the parameters
 * common to all nonlinear optimization algorithms.  This class also contains
 * all of those parameters.
 */
class DoglegParams : public SuccessiveLinearizationParams {
public:
  /** See DoglegParams::dlVerbosity */
  enum VerbosityDL {
    SILENT,
    VERBOSE
  };

  double deltaInitial; ///< The initial trust region radius (default: 1.0)
  VerbosityDL verbosityDL; ///< The verbosity level for Dogleg (default: SILENT), see also NonlinearOptimizerParams::verbosity

  DoglegParams() :
    deltaInitial(1.0), verbosityDL(SILENT) {}

  virtual ~DoglegParams() {}

  virtual void print(const std::string& str = "") const {
    SuccessiveLinearizationParams::print(str);
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
 * State for DoglegOptimizer
 */
class DoglegState : public NonlinearOptimizerState {
public:
  double Delta;

  DoglegState() {}

  virtual ~DoglegState() {}

protected:
  DoglegState(const NonlinearFactorGraph& graph, const Values& values, const DoglegParams& params, unsigned int iterations = 0) :
    NonlinearOptimizerState(graph, values, iterations), Delta(params.deltaInitial) {}

  friend class DoglegOptimizer;
};

/**
 * This class performs Dogleg nonlinear optimization
 */
class DoglegOptimizer : public NonlinearOptimizer {

protected:
	DoglegParams params_;
	DoglegState state_;

public:
  typedef boost::shared_ptr<DoglegOptimizer> shared_ptr;

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
      const DoglegParams& params = DoglegParams()) :
        NonlinearOptimizer(graph), params_(ensureHasOrdering(params, graph, initialValues)), state_(graph, initialValues, params_) {}

  /** Standard constructor, requires a nonlinear factor graph, initial
   * variable assignments, and optimization parameters.  For convenience this
   * version takes plain objects instead of shared pointers, but internally
   * copies the objects.
   * @param graph The nonlinear factor graph to optimize
   * @param initialValues The initial variable assignments
   */
  DoglegOptimizer(const NonlinearFactorGraph& graph, const Values& initialValues, const Ordering& ordering) :
        NonlinearOptimizer(graph) {
    params_.ordering = ordering;
    state_ = DoglegState(graph, initialValues, params_); }

  /// @}

  /// @name Advanced interface
  /// @{

  /** Virtual destructor */
  virtual ~DoglegOptimizer() {}

  /** Perform a single iteration, returning a new NonlinearOptimizer class
   * containing the updated variable assignments, which may be retrieved with
   * values().
   */
  virtual void iterate();

  /** Access the parameters */
  const DoglegParams& params() const { return params_; }

  /** Access the last state */
  const DoglegState& state() const { return state_; }

  /** Access the current trust region radius Delta */
  double getDelta() const { return state_.Delta; }

  /// @}

protected:
  /** Access the parameters (base class version) */
  virtual const NonlinearOptimizerParams& _params() const { return params_; }

  /** Access the state (base class version) */
  virtual const NonlinearOptimizerState& _state() const { return state_; }

  /** Internal function for computing a COLAMD ordering if no ordering is specified */
  DoglegParams ensureHasOrdering(DoglegParams params, const NonlinearFactorGraph& graph, const Values& values) const {
    if(!params.ordering)
      params.ordering = *graph.orderingCOLAMD(values);
    return params;
  }
};

}
