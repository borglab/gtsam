/**
 * @file    DoglegOptimizer.h
 * @brief   Nonlinear factor graph optimizer using Powell's Dogleg algorithm
 * @author  Richard Roberts
 */

#pragma once

#include <gtsam/nonlinear/NonlinearFactorGraph.h>

namespace gtsam {

/**
 * A class to perform nonlinear optimization using Powell's dogleg algorithm.
 * This class is functional, meaning every method is \c const, and returns a new
 * copy of the class.
 *
 * \tparam VALUES The LieValues or TupleValues type to hold the values to be
 * estimated.
 *
 * \tparam GAUSSIAN_SOLVER The linear solver to use at each iteration,
 * currently either GaussianSequentialSolver or GaussianMultifrontalSolver.
 * The latter is typically faster, especially for non-trivial problems.
 */
template<class VALUES, class GAUSSIAN_SOLVER>
class DoglegOptimizer {

protected:

  typedef DoglegOptimizer<VALUES, GAUSSIAN_SOLVER> This; ///< Typedef to this class

  const sharedGraph graph_;
  const sharedValues values_;
  const double error_;

public:

  typedef VALUES ValuesType; ///< Typedef of the VALUES template parameter
  typedef GAUSSIAN_SOLVER SolverType; ///< Typedef of the GAUSSIAN_SOLVER template parameter
  typedef NonlinearFactorGraph<VALUES> GraphType; ///< A nonlinear factor graph templated on ValuesType

  typedef boost::shared_ptr<const GraphType> sharedGraph; ///< A shared_ptr to GraphType
  typedef boost::shared_ptr<const ValuesType> sharedValues; ///< A shared_ptr to ValuesType


  /**
   * Construct a DoglegOptimizer from the factor graph to optimize and the
   * initial estimate of the variable values, using the default variable
   * ordering method, currently COLAMD.
   * @param graph  The factor graph to optimize
   * @param initialization  An initial estimate of the variable values
   */
  DoglegOptimizer(sharedGraph graph, sharedValues initialization);

  /**
   * Construct a DoglegOptimizer from the factor graph to optimize and the
   * initial estimate of the variable values, using the default variable
   * ordering method, currently COLAMD. (this non-shared-pointer version
   * incurs a performance hit for copying, see DoglegOptimizer(sharedGraph, sharedValues)).
   * @param graph  The factor graph to optimize
   * @param initialization  An initial estimate of the variable values
   */
  DoglegOptimizer(const GraphType& graph, const ValuesType& initialization);

};

}
