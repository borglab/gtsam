/**
 * @file     SQPSolver.h
 * @brief    This class is used to solve a Non-linear Problem
 *           (NP) and finds a local solution using SQP. See
 *           Nocedal Algorithm 18.3 for details.
 * @author   Ivan Dario Jimenez
 * @date     2/2/16
 */

#pragma once

#include <gtsam_unstable/linear/NP.h>
#include <gtsam_unstable/linear/ActiveSetSolver.h>
#include <gtsam_unstable/linear/SQPState.h>

class SQPSolver : public ActiveSetSolver {
  const NP& np_; //!< the nonlinear programming problem
public:
  SQPSolver(const NP& np) : np_(np){}

  NPState iterate(const NPState& state) const;
  /*
   * This function will optimize the Non-linear problem given a set of initial values
   */
  pair<Vector, Vector> optimize(const Vector& initialValues) const;

  /*
   * This function will attempt to optimize the Non-linear problem without the need
   * of initial values.
   */
  pair<Vector, Vector> optimize();
};
