/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/*
 * Solve a simple 3 by 3 grid of Pose2 SLAM problem by using easy SPCG interface
 */


#include <boost/shared_ptr.hpp>

#include <gtsam/inference/graph-inl.h>
#include <gtsam/inference/FactorGraph-inl.h>
#include <gtsam/nonlinear/NonlinearOptimization-inl.h>
#include <gtsam/slam/pose2SLAM.h>

using namespace std;
using namespace gtsam;
using namespace pose2SLAM;

Graph graph;
Values initial;
Values result;



/* ************************************************************************* */
int main(void) {

	/* generate synthetic data */
	const SharedGaussian sigma(noiseModel::Unit::Create(0.1));
	Key x1(1), x2(2), x3(3), x4(4), x5(5), x6(6), x7(7), x8(8), x9(9);

	// create a 3 by 3 grid
	// x3 x6 x9
	// x2 x5 x8
	// x1 x4 x7
	graph.addConstraint(x1,x2,Pose2(0,2,0),sigma) ;
	graph.addConstraint(x2,x3,Pose2(0,2,0),sigma) ;
	graph.addConstraint(x4,x5,Pose2(0,2,0),sigma) ;
	graph.addConstraint(x5,x6,Pose2(0,2,0),sigma) ;
	graph.addConstraint(x7,x8,Pose2(0,2,0),sigma) ;
	graph.addConstraint(x8,x9,Pose2(0,2,0),sigma) ;
	graph.addConstraint(x1,x4,Pose2(2,0,0),sigma) ;
	graph.addConstraint(x4,x7,Pose2(2,0,0),sigma) ;
	graph.addConstraint(x2,x5,Pose2(2,0,0),sigma) ;
	graph.addConstraint(x5,x8,Pose2(2,0,0),sigma) ;
	graph.addConstraint(x3,x6,Pose2(2,0,0),sigma) ;
	graph.addConstraint(x6,x9,Pose2(2,0,0),sigma) ;
	graph.addPrior(x1, Pose2(0,0,0), sigma) ;

	initial.insert(x1, Pose2(  0,  0,   0));
	initial.insert(x2, Pose2(  0, 2.1, 0.01));
	initial.insert(x3, Pose2(  0, 3.9,-0.01));
	initial.insert(x4, Pose2(2.1,-0.1,    0));
	initial.insert(x5, Pose2(1.9, 2.1, 0.02));
	initial.insert(x6, Pose2(2.0, 3.9,-0.02));
	initial.insert(x7, Pose2(4.0, 0.1, 0.03 ));
	initial.insert(x8, Pose2(3.9, 2.1, 0.01));
	initial.insert(x9, Pose2(4.1, 3.9,-0.01));
	/* done */


	graph.print("full graph") ;
	initial.print("initial estimate");
	result = optimizeSPCG(graph, initial);
	result.print("final result") ;
	return 0 ;
}

