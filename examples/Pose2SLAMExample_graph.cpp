/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file Pose2SLAMExample_graph->cpp
 * @brief Read graph from file and perform GraphSLAM
 * @date June 3, 2012
 * @author Frank Dellaert
 */

#include <gtsam/slam/dataset.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/geometry/Pose2.h>
#include <boost/tuple/tuple.hpp>
#include <cmath>

using namespace std;
using namespace gtsam;

int main(int argc, char** argv) {

	// Read File and create graph and initial estimate
	// we are in build/examples, data is in examples/Data
	NonlinearFactorGraph::shared_ptr graph ;
	Values::shared_ptr initial;
	SharedDiagonal model = noiseModel::Diagonal::Sigmas(Vector_(3, 0.05, 0.05, 5.0*M_PI/180.0));
	boost::tie(graph,initial) = load2D("../../examples/Data/w100-odom.graph",model);
	initial->print("Initial estimate:\n");

	// Add a Gaussian prior on first poses
	Pose2 priorMean(0.0, 0.0, 0.0); // prior at origin
	SharedDiagonal priorNoise = noiseModel::Diagonal::Sigmas(Vector_(3, 0.01, 0.01, 0.01));
	graph->add(PriorFactor<Pose2>(0, priorMean, priorNoise));

	// Single Step Optimization using Levenberg-Marquardt
	Values result = LevenbergMarquardtOptimizer(*graph, *initial).optimize();
	result.print("\nFinal result:\n");

	// Plot the covariance of the last pose
	Marginals marginals(*graph, result);
	cout.precision(2);
  cout << "\nP3:\n" << marginals.marginalCovariance(99) << endl;

return 0;
}
