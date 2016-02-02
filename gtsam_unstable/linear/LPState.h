/**
 * @file     LPState.h
 * @brief    This struct holds the state of QPSolver at each iteration    
 * @author   Ivan Dario Jimenez
 * @date     1/24/16
 */

#include <gtsam/linear/VectorValues.h>
#include "InequalityFactorGraph.h"

namespace gtsam {

/*
 * This struct contains the state information for a single iteration of an
 * active set method iteration.
 */
struct LPState {
  // A itermediate value for the value of the final solution.
  VectorValues values;
  // Constains the set of duals computed during the iteration that retuned this
  // state.
  VectorValues duals;
  // An inequality Factor Graph that contains only the active constriants.
  InequalityFactorGraph workingSet;
  // True if the algorithm has converged to a solution
  bool converged;
  // counter for the number of iteration. Incremented at the end of each iter.
  size_t iterations;

  /// default constructor
  LPState() :
      values(), duals(), workingSet(), converged(false), iterations(0) {
  }

  /// constructor with initial values
  LPState(const VectorValues& initialValues, const VectorValues& initialDuals,
      const InequalityFactorGraph& initialWorkingSet, bool _converged,
      size_t _iterations) :
      values(initialValues), duals(initialDuals), workingSet(initialWorkingSet), converged(
          _converged), iterations(_iterations) {
  }
};

}
