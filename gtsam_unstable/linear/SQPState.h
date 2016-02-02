/**
 * @file     SQPState.h
 * @brief    This class is used to store the current state of
 *           an SQPSolver iteration.
 * @author   Ivan Dario Jimenez
 * @date     2/2/16
 */

#pragma once
#include <gtsam/base/Matrix.h>

namespace gtsam {
class SQPState {
  Vector values;
  Vector duals;
  Vector workingSet;

  bool converged;
  size_t iterations;

  SQPState() :
    values(), duals(), workingSet(),converged(false), iterations(0) {}

  SQPState(const Vector& initialValues, const Vector& initialDuals,
           const Vector& initialWorkingSet, const bool _converged,
           const size_t iterations) :
    values(initialValues), duals(initialDuals), workingSet(initialWorkingSet),
    converged(_converged), iterations(iterations) {}
};
}
