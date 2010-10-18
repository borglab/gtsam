/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/*
 * Pose2SLAMwSPCG.cpp
 *
 *  Created on: Oct 18, 2010
 *      Author: Yong-Dian Jian
 *
 *   Demonstrate how to use SPCG solver to solve Pose2 SLAM problem
 */

#include <boost/shared_ptr.hpp>

#include <gtsam/inference/graph-inl.h>
#include <gtsam/inference/FactorGraph-inl.h>
#include <gtsam/linear/SubgraphSolver-inl.h>
#include <gtsam/nonlinear/NonlinearOptimizer-inl.h>
#include <gtsam/slam/pose2SLAM.h>

using namespace std;
using namespace gtsam;
using namespace pose2SLAM;

typedef boost::shared_ptr<Graph> sharedGraph;
typedef boost::shared_ptr<Values> sharedValue;


typedef NonlinearOptimizer<Graph, Values, SubgraphPreconditioner, SubgraphSolver<Graph,Values> > SPCGOptimizer;

sharedGraph G;
Values initial;
Values result;

void generateData() ;

/* ************************************************************************* */
int main(void) {

	const SharedGaussian sigma(noiseModel::Unit::Create(0.1));

	// generate measurement and initial configuration
	generateData() ;

	cout << "Initialize .... " << endl;
	SPCGOptimizer::shared_solver solver(new SPCGOptimizer::solver(*G, initial)) ;
	sharedValue SV(new Values(initial)) ;
	SPCGOptimizer optimizer(G, SV, solver) ;

	cout << "before optimization, sum of error is " << optimizer.error() << endl;

	cout << "Optimize .... " << endl;
	NonlinearOptimizationParameters parameter;
	SPCGOptimizer optimizer2 = optimizer.levenbergMarquardt(parameter);

	cout << "after optimization, sum of error is " << optimizer2.error() << endl;

	result = *optimizer2.config() ;
	result.print("result") ;

	return 0 ;
}

void generateData() {

	// noise model
	const SharedGaussian sigma(noiseModel::Unit::Create(0.1));

	Key x1(1), x2(2), x3(3), x4(4), x5(5), x6(6), x7(7), x8(8), x9(9);

	// create a 3 by 3 grid
	// x3 x6 x9
	// x2 x5 x8
	// x1 x4 x7
	G = sharedGraph(new Graph) ;
	G->addConstraint(x1,x2,Pose2(0,2,0),sigma) ;
	G->addConstraint(x2,x3,Pose2(0,2,0),sigma) ;
	G->addConstraint(x4,x5,Pose2(0,2,0),sigma) ;
	G->addConstraint(x5,x6,Pose2(0,2,0),sigma) ;
	G->addConstraint(x7,x8,Pose2(0,2,0),sigma) ;
	G->addConstraint(x8,x9,Pose2(0,2,0),sigma) ;
	G->addConstraint(x1,x4,Pose2(2,0,0),sigma) ;
	G->addConstraint(x4,x7,Pose2(2,0,0),sigma) ;
	G->addConstraint(x2,x5,Pose2(2,0,0),sigma) ;
	G->addConstraint(x5,x8,Pose2(2,0,0),sigma) ;
	G->addConstraint(x3,x6,Pose2(2,0,0),sigma) ;
	G->addConstraint(x6,x9,Pose2(2,0,0),sigma) ;
	G->addPrior(x1, Pose2(0,0,0), sigma) ;

	initial.insert(x1, Pose2(  0,  0,   0));
	initial.insert(x2, Pose2(  0, 2.1, 0.01));
	initial.insert(x3, Pose2(  0, 3.9,-0.01));
	initial.insert(x4, Pose2(2.1,-0.1,    0));
	initial.insert(x5, Pose2(1.9, 2.1, 0.02));
	initial.insert(x6, Pose2(2.0, 3.9,-0.02));
	initial.insert(x7, Pose2(4.0, 0.1, 0.03 ));
	initial.insert(x8, Pose2(3.9, 2.1, 0.01));
	initial.insert(x9, Pose2(4.1, 3.9,-0.01));
}
