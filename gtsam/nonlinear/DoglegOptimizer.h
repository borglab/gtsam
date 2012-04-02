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
 * @created Feb 26, 2012
 */

#pragma once

#include <gtsam/nonlinear/DirectOptimizer.h>

namespace gtsam {

/** Parameters for Levenberg-Marquardt optimization.  Note that this parameters
 * class inherits from NonlinearOptimizerParams, which specifies the parameters
 * common to all nonlinear optimization algorithms.  This class also contains
 * all of those parameters.
 */
class DoglegParams : public DirectOptimizerParams {
public:
  /** See DoglegParams::dlVerbosity */
  enum DLVerbosity {
    SILENT,
    VERBOSE
  };

  double deltaInitial; ///< The initial trust region radius (default: 1.0)
  DLVerbosity dlVerbosity; ///< The verbosity level for Dogleg (default: SILENT), see also NonlinearOptimizerParams::verbosity

  DoglegParams() :
    deltaInitial(1.0), dlVerbosity(SILENT) {}

  virtual ~DoglegParams() {}

  virtual void print(const std::string& str = "") const {
    DirectOptimizerParams::print(str);
    std::cout << "               deltaInitial: " << deltaInitial << "\n";
    std::cout.flush();
  }
};

/**
 * State for DoglegOptimizer
 */
class DoglegState : public NonlinearOptimizerState {
public:

  double Delta;

};

/**
 * This class performs Dogleg nonlinear optimization
 */
class DoglegOptimizer : public DirectOptimizer {

public:

  typedef boost::shared_ptr<DoglegParams> SharedParams;
  typedef boost::shared_ptr<DoglegState> SharedState;
  typedef boost::shared_ptr<DoglegOptimizer> shared_ptr;

  /// @name Standard interface
  /// @{

  /** Standard constructor, requires a nonlinear factor graph, initial
   * variable assignments, and optimization parameters.  For convenience this
   * version takes plain objects instead of shared pointers, but internally
   * copies the objects.
   * @param graph The nonlinear factor graph to optimize
   * @param values The initial variable assignments
   * @param params The optimization parameters
   */
  DoglegOptimizer(const NonlinearFactorGraph& graph,
      const DoglegParams& params = DoglegParams(),
      const Ordering& ordering = Ordering()) :
        DirectOptimizer(SharedGraph(new NonlinearFactorGraph(graph))),
        params_(new DoglegParams(params)) {}

  /** Standard constructor, requires a nonlinear factor graph, initial
   * variable assignments, and optimization parameters.  For convenience this
   * version takes plain objects instead of shared pointers, but internally
   * copies the objects.
   * @param graph The nonlinear factor graph to optimize
   * @param values The initial variable assignments
   * @param params The optimization parameters
   */
  DoglegOptimizer(const NonlinearFactorGraph& graph,
      const Ordering& ordering) :
        DirectOptimizer(SharedGraph(new NonlinearFactorGraph(graph))),
        params_(new DoglegParams()) {}

  /** Standard constructor, requires a nonlinear factor graph, initial
   * variable assignments, and optimization parameters.
   * @param graph The nonlinear factor graph to optimize
   * @param values The initial variable assignments
   * @param params The optimization parameters
   */
  DoglegOptimizer(const SharedGraph& graph,
      const DoglegParams& params = DoglegParams(),
      const SharedOrdering& ordering = SharedOrdering()) :
        DirectOptimizer(graph),
        params_(new DoglegParams(params)) {}

  /** Access the parameters */
  virtual const NonlinearOptimizer::SharedParams& params() const { return params_; }

  /** Access the parameters */
  const DoglegOptimizer::SharedParams& params() const { return params_; }

  /// @}

  /// @name Advanced interface
  /// @{

  /** Virtual destructor */
  virtual ~DoglegOptimizer() {}

  /** Perform a single iteration, returning a new NonlinearOptimizer class
   * containing the updated variable assignments, which may be retrieved with
   * values().
   */
  virtual NonlinearOptimizer::SharedState iterate(const SharedState& current) const;

  /** Create a copy of the NonlinearOptimizer */
  virtual NonlinearOptimizer::shared_ptr clone() const {
    return boost::make_shared<DoglegOptimizer>(*this); }

  /// @}

protected:

  const SharedParams params_;

  virtual void setParams(const NonlinearOptimizer::SharedParams& newParams) { params_ = newParams; }
};

}
