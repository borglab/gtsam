/*
 * @file testNonlinearConstraint.cpp
 * @brief demos of constrained optimization using existing gtsam components
 * @author Alex Cunningham
 */

#include <iostream>
#include <cmath>
#include <boost/assign/std/list.hpp> // for operator +=
#include <boost/assign/std/map.hpp> // for insert
#include <boost/foreach.hpp>
#include <boost/shared_ptr.hpp>

#include <CppUnitLite/TestHarness.h>

#define GTSAM_MAGIC_KEY

#include <Point2.h>
#include <Pose3.h>
#include <GaussianFactorGraph.h>
#include <NonlinearOptimizer.h>
#include <simulated2D.h>
#include <Ordering.h>
#include <visualSLAM.h>

// templated implementations
#include <NonlinearFactorGraph-inl.h>
#include <NonlinearOptimizer-inl.h>
#include <TupleConfig-inl.h>

using namespace std;
using namespace gtsam;
using namespace boost;
using namespace boost::assign;

//// Models to use
//SharedDiagonal probModel1 = sharedSigma(1,1.0);
//SharedDiagonal probModel2 = sharedSigma(2,1.0);
//SharedDiagonal constraintModel1 = noiseModel::Constrained::All(1);
//
//// trick from some reading group
//#define FOREACH_PAIR( KEY, VAL, COL) BOOST_FOREACH (boost::tie(KEY,VAL),COL)
//
///* ********************************************************************* */
//// full components
//typedef simulated2D::Config Config2D;
//typedef NonlinearFactorGraph<Config2D> Graph2D;
//typedef NonlinearEquality<Config2D, simulated2D::PoseKey, Point2> NLE;
//typedef boost::shared_ptr<simulated2D::GenericMeasurement<Config2D> > shared;
//typedef NonlinearOptimizer<Graph2D, Config2D> Optimizer;

/* ********************************************************************* */
// This is an obstacle avoidance demo, where there is a trajectory of
// three points, where there is a circular obstacle in the middle.  There
// is a binary inequality constraint connecting the obstacle to the
// states, which enforces a minimum distance.
/* ********************************************************************* */

//typedef NonlinearConstraint2<Config2D, PoseKey, Point2, PointKey, Point2> AvoidConstraint;
//typedef shared_ptr<AvoidConstraint> shared_a;
//typedef NonlinearEquality<Config2D, simulated2D::PoseKey, Point2> PoseNLConstraint;
//typedef shared_ptr<PoseNLConstraint> shared_pc;
//typedef NonlinearEquality<Config2D, simulated2D::PointKey, Point2> ObstacleConstraint;
//typedef shared_ptr<ObstacleConstraint> shared_oc;
//
//
//namespace constrained_avoid1 {
//// avoidance radius
//double radius = 1.0;
//
//// binary avoidance constraint
///** g(x) = ||x2-obs||^2 - radius^2 > 0 */
//Vector g_func(const Config2D& config, const PoseKey& x, const PointKey& obs) {
//	double dist2 = config[x].dist(config[obs]);
//	double thresh = radius*radius;
//	return Vector_(1, dist2-thresh);
//}
//
///** jacobian at pose */
//Matrix jac_g1(const Config2D& config, const PoseKey& x, const PointKey& obs) {
//	Point2 p = config[x]-config[obs];
//	return Matrix_(1,2, 2.0*p.x(), 2.0*p.y());
//}
//
///** jacobian at obstacle */
//Matrix jac_g2(const Config2D& config, const PoseKey& x, const PointKey& obs) {
//	Point2 p = config[x]-config[obs];
//	return Matrix_(1,2, -2.0*p.x(), -2.0*p.y());
//}
//}
//
//pair<NLGraph, Config2D> obstacleAvoidGraph() {
//	// Keys
//	PoseKey x1(1), x2(2), x3(3);
//	PointKey l1(1);
//	LagrangeKey L20(20);
//
//	// Constrained Points
//	Point2 pt_x1,
//		   pt_x3(10.0, 0.0),
//		   pt_l1(5.0, -0.5);
//
//	shared_pc e1(new PoseNLConstraint(x1, pt_x1));
//	shared_pc e2(new PoseNLConstraint(x3, pt_x3));
//	shared_oc e3(new ObstacleConstraint(l1, pt_l1));
//
//	// measurement from x1 to x2
//	Point2 x1x2(5.0, 0.0);
//	shared f1(new simulated2D::Odometry(x1x2, sigma, 1,2));
//
//	// measurement from x2 to x3
//	Point2 x2x3(5.0, 0.0);
//	shared f2(new simulated2D::Odometry(x2x3, sigma, 2,3));
//
//	// create a binary inequality constraint that forces the middle point away from
//	//  the obstacle
//	shared_a c1(new AvoidConstraint(boost::bind(constrained_avoid1::g_func, _1, x2, l1),
//							x2, boost::bind(constrained_avoid1::jac_g1, _1, x2, l1),
//						    l1,boost::bind(constrained_avoid1::jac_g2, _1, x2, l1),
//						    1, L20, false));
//
//	// construct the graph
//	NLGraph graph;
//	graph.push_back(e1);
//	graph.push_back(e2);
//	graph.push_back(e3);
//	graph.push_back(c1);
//	graph.push_back(f1);
//	graph.push_back(f2);
//
//	// make a config of the fixed values, for convenience
//	Config2D config;
//	config.insert(x1, pt_x1);
//	config.insert(x3, pt_x3);
//	config.insert(l1, pt_l1);
//
//	return make_pair(graph, config);
//}
//
///* ********************************************************************* */
//TEST ( SQPOptimizer, inequality_avoid ) {
//	// create the graph
//	NLGraph graph; Config2D feasible;
//	boost::tie(graph, feasible) = obstacleAvoidGraph();
//
//	// create the rest of the config
//	shared_ptr<Config2D> init(new Config2D(feasible));
//	PoseKey x2(2);
//	init->insert(x2, Point2(5.0, 100.0));
//
//	// create an ordering
//	Ordering ord;
//	ord += "x1", "x2", "x3", "l1";
//
//	// create an optimizer
//	Optimizer optimizer(graph, ord, init);
//
//	// perform an iteration of optimization
//	// NOTE: the constraint will be inactive in the first iteration,
//	// so it will violate the constraint after one iteration
//	Optimizer afterOneIteration = optimizer.iterate(Optimizer::SILENT);
//
//	Config2D exp1(feasible);
//	exp1.insert(x2, Point2(5.0, 0.0));
//	CHECK(assert_equal(exp1, *(afterOneIteration.config())));
//
//	// the second iteration will activate the constraint and force the
//	// config to a viable configuration.
//	Optimizer after2ndIteration = afterOneIteration.iterate(Optimizer::SILENT);
//
//	Config2D exp2(feasible);
//	exp2.insert(x2, Point2(5.0, 0.5));
//	CHECK(assert_equal(exp2, *(after2ndIteration.config())));
//}
//
///* ********************************************************************* */
//TEST ( SQPOptimizer, inequality_avoid_iterative ) {
//	// create the graph
//	NLGraph graph; Config2D feasible;
//	boost::tie(graph, feasible) = obstacleAvoidGraph();
//
//	// create the rest of the config
//	shared_ptr<Config2D> init(new Config2D(feasible));
//	PoseKey x2(2);
//	init->insert(x2, Point2(5.0, 100.0));
//
//	// create an ordering
//	Ordering ord;
//	ord += "x1", "x2", "x3", "l1";
//
//	// create an optimizer
//	Optimizer optimizer(graph, ord, init);
//
//	double relThresh = 1e-5; // minimum change in error between iterations
//	double absThresh = 1e-5; // minimum error necessary to converge
//	double constraintThresh = 1e-9; // minimum constraint error to be feasible
//	Optimizer final = optimizer.iterateSolve(relThresh, absThresh, constraintThresh);
//
//	// verify
//	Config2D exp2(feasible);
//	exp2.insert(x2, Point2(5.0, 0.5));
//	CHECK(assert_equal(exp2, *(final.config())));
//}

/* ************************************************************************* */
int main() { TestResult tr; return TestRegistry::runAllTests(tr); }
/* ************************************************************************* */
