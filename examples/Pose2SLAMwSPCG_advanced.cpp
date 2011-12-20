/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/*
 * @file Pose2SLAMwSPCG_advanced.cpp
 * @brief Solve a simple 3 by 3 grid of Pose2 SLAM problem by using advanced SPCG interface
 * @author Yong Dian
 * Created October 21, 2010
 */

#include <boost/shared_ptr.hpp>

#include <gtsam/slam/pose2SLAM.h>
#include <gtsam/linear/SubgraphSolver.h>
#include <gtsam/nonlinear/NonlinearOptimizer.h>

using namespace std;
using namespace gtsam;
using namespace pose2SLAM;

typedef boost::shared_ptr<Graph> sharedGraph ;
typedef boost::shared_ptr<pose2SLAM::Values> sharedValue ;
//typedef NonlinearOptimizer<Graph, Values, SubgraphPreconditioner, SubgraphSolver<Graph,Values> > SPCGOptimizer;


typedef SubgraphSolver<Graph, GaussianFactorGraph, pose2SLAM::Values> Solver;
typedef boost::shared_ptr<Solver> sharedSolver ;
typedef NonlinearOptimizer<Graph, pose2SLAM::Values, GaussianFactorGraph, Solver> SPCGOptimizer;

sharedGraph graph;
sharedValue initial;
pose2SLAM::Values result;

/* ************************************************************************* */
int main(void) {

	/* generate synthetic data */
	const SharedNoiseModel sigma(noiseModel::Unit::Create(0.1));
	Key x1(1), x2(2), x3(3), x4(4), x5(5), x6(6), x7(7), x8(8), x9(9);

	graph = boost::make_shared<Graph>() ;
	initial = boost::make_shared<pose2SLAM::Values>() ;

	// create a 3 by 3 grid
	// x3 x6 x9
	// x2 x5 x8
	// x1 x4 x7
	graph->addConstraint(x1,x2,Pose2(0,2,0),sigma) ;
	graph->addConstraint(x2,x3,Pose2(0,2,0),sigma) ;
	graph->addConstraint(x4,x5,Pose2(0,2,0),sigma) ;
	graph->addConstraint(x5,x6,Pose2(0,2,0),sigma) ;
	graph->addConstraint(x7,x8,Pose2(0,2,0),sigma) ;
	graph->addConstraint(x8,x9,Pose2(0,2,0),sigma) ;
	graph->addConstraint(x1,x4,Pose2(2,0,0),sigma) ;
	graph->addConstraint(x4,x7,Pose2(2,0,0),sigma) ;
	graph->addConstraint(x2,x5,Pose2(2,0,0),sigma) ;
	graph->addConstraint(x5,x8,Pose2(2,0,0),sigma) ;
	graph->addConstraint(x3,x6,Pose2(2,0,0),sigma) ;
	graph->addConstraint(x6,x9,Pose2(2,0,0),sigma) ;
	graph->addPrior(x1, Pose2(0,0,0), sigma) ;

	initial->insert(x1, Pose2(  0,  0,   0));
	initial->insert(x2, Pose2(  0, 2.1, 0.01));
	initial->insert(x3, Pose2(  0, 3.9,-0.01));
	initial->insert(x4, Pose2(2.1,-0.1,    0));
	initial->insert(x5, Pose2(1.9, 2.1, 0.02));
	initial->insert(x6, Pose2(2.0, 3.9,-0.02));
	initial->insert(x7, Pose2(4.0, 0.1, 0.03 ));
	initial->insert(x8, Pose2(3.9, 2.1, 0.01));
	initial->insert(x9, Pose2(4.1, 3.9,-0.01));
	/* done with generating data */


	graph->print("full graph") ;
	initial->print("initial estimate") ;

	sharedSolver solver(new Solver(*graph, *initial)) ;
	SPCGOptimizer optimizer(graph, initial, solver->ordering(), solver) ;

	cout << "before optimization, sum of error is " << optimizer.error() << endl;
	SPCGOptimizer optimizer2 = optimizer.levenbergMarquardt();
	cout << "after optimization, sum of error is " << optimizer2.error() << endl;

	result = *optimizer2.values() ;
	result.print("final result") ;

	return 0 ;
}

