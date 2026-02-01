/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file SimpleRotation.cpp
 * @brief This is a super-simple example of optimizing a single rotation according to a single prior
 * @date Jul 1, 2010
 * @author Frank Dellaert
 * @author Alex Cunningham
 */

  /**
   * This example will perform a relatively trivial optimization on
   * a single variable with a single factor.
   */

// In this example, a 2D rotation will be used as the variable of interest
#include <gtsam/geometry/Rot2.h>

// Each variable in the system (poses) must be identified with a unique key.
// We can either use simple integer keys (1, 2, 3, ...) or symbols (X1, X2, L1).
// Here we will use symbols
#include <gtsam/inference/Symbol.h>

// In GTSAM, measurement functions are represented as 'factors'. Several common factors
// have been provided with the library for solving robotics/SLAM/Bundle Adjustment problems.
// We will apply a simple prior on the rotation. We do so via the `addPrior` convenience
// method in NonlinearFactorGraph.

// When the factors are created, we will add them to a Factor Graph. As the factors we are using
// are nonlinear factors, we will need a Nonlinear Factor Graph.
#include <gtsam/nonlinear/NonlinearFactorGraph.h>

// The nonlinear solvers within GTSAM are iterative solvers, meaning they linearize the
// nonlinear functions around an initial linearization point, then solve the linear system
// to update the linearization point. This happens repeatedly until the solver converges
// to a consistent set of variable values. This requires us to specify an initial guess
// for each variable, held in a Values container.
#include <gtsam/nonlinear/Values.h>

// Finally, once all of the factors have been added to our factor graph, we will want to
// solve/optimize to graph to find the best (Maximum A Posteriori) set of variable values.
// GTSAM includes several nonlinear optimizers to perform this step. Here we will use the
// standard Levenberg-Marquardt solver
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>


using namespace std;
using namespace gtsam;

const double degree = M_PI / 180;

int main() {
  /**
   *    Step 1: Create a factor to express a unary constraint
   * The "prior" in this case is the measurement from a sensor,
   * with a model of the noise on the measurement.
   *
   * The "Key" created here is a label used to associate parts of the
   * state (stored in "RotValues") with particular factors.  They require
   * an index to allow for lookup, and should be unique.
   *
   * In general, creating a factor requires:
   *  - A key or set of keys labeling the variables that are acted upon
   *  - A measurement value
   *  - A measurement model with the correct dimensionality for the factor
   */
  Rot2 prior = Rot2::fromAngle(30 * degree);
  prior.print("goal angle");
  auto model = noiseModel::Isotropic::Sigma(1, 1 * degree);
  Symbol key('x', 1);

  /**
   *    Step 2: Create a graph container and add the factor to it
   * Before optimizing, all factors need to be added to a Graph container,
   * which provides the necessary top-level functionality for defining a
   * system of constraints.
   *
   * In this case, there is only one factor, but in a practical scenario,
   * many more factors would be added.
   */
  NonlinearFactorGraph graph;
  graph.addPrior(key, prior, model);
  graph.print("full graph");

  /**
   *    Step 3: Create an initial estimate
   * An initial estimate of the solution for the system is necessary to
   * start optimization.  This system state is the "RotValues" structure,
   * which is similar in structure to a STL map, in that it maps
   * keys (the label created in step 1) to specific values.
   *
   * The initial estimate provided to optimization will be used as
   * a linearization point for optimization, so it is important that
   * all of the variables in the graph have a corresponding value in
   * this structure.
   *
   * The interface to all RotValues types is the same, it only depends
   * on the type of key used to find the appropriate value map if there
   * are multiple types of variables.
   */
  Values initial;
  initial.insert(key, Rot2::fromAngle(20 * degree));
  initial.print("initial estimate");

  /**
   *    Step 4: Optimize
   * After formulating the problem with a graph of constraints
   * and an initial estimate, executing optimization is as simple
   * as calling a general optimization function with the graph and
   * initial estimate.  This will yield a new RotValues structure
   * with the final state of the optimization.
   */
  Values result = LevenbergMarquardtOptimizer(graph, initial).optimize();
  result.print("final result");

  return 0;
}
