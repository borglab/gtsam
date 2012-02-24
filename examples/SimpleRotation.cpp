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

#include <cmath>
#include <iostream>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/geometry/Rot2.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/nonlinear/Symbol.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/NonlinearOptimization.h>

/*
 * TODO: make factors independent of RotValues
 * TODO: make toplevel documentation
 * TODO: Clean up nonlinear optimization API
 */

using namespace std;
using namespace gtsam;

const double degree = M_PI / 180;

int main() {

	/**
	 * This example will perform a relatively trivial optimization on
	 * a single variable with a single factor.
	 */

	/**
	 *    Step 1: create a factor on to express a unary constraint
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
	SharedDiagonal model = noiseModel::Isotropic::Sigma(1, 1 * degree);
	Symbol key('x',1);
	PriorFactor<Rot2> factor(key, prior, model);

	/**
	 *    Step 2: create a graph container and add the factor to it
	 * Before optimizing, all factors need to be added to a Graph container,
	 * which provides the necessary top-level functionality for defining a
	 * system of constraints.
	 *
	 * In this case, there is only one factor, but in a practical scenario,
	 * many more factors would be added.
	 */
	NonlinearFactorGraph graph;
	graph.add(factor);
	graph.print("full graph");

	/**
	 *    Step 3: create an initial estimate
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
	 *    Step 4: optimize
	 * After formulating the problem with a graph of constraints
	 * and an initial estimate, executing optimization is as simple
	 * as calling a general optimization function with the graph and
	 * initial estimate.  This will yield a new RotValues structure
	 * with the final state of the optimization.
	 */
	Values result = optimize(graph, initial);
	result.print("final result");

	return 0;
}
