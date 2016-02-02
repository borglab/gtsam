/**
 * @file     NP.h
 * @brief    Use this virtual class to represent a non-linear
 *           problem. An implementation of this should provide
 *           approximation and differentiation services to
 *           allow SQP to work on the given problem.
 * @author   Ivan Dario Jimenez
 * @date     2/2/16
 */

#pragma once

#include <gtsam_unstable/linear/QP.h>

namespace gtsam {

class NP {
public:
  /*
   * This function takes the a point and returns a quadratic 
   * problem approximation to the non-linear Problem.
   */
  virtual QP approximate(const Vector& x) const = 0;

  /*
   * Differentiates the objective function of the non-linear 
   * problem at point x.
   */
  virtual Vector objectiveDerivative(const Vector& x) const = 0;

  /*
   * Differentiates the constraints at point x.
   */
  virtual Vector constraintsDerivative(const Vector& x) const = 0;
};

}
