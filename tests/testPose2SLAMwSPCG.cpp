/**
 * @file testPose2SLAMwSPCG
 * @author Alex Cunningham
 */

#include <CppUnitLite/TestHarness.h>

#include <gtsam/slam/pose2SLAM.h>
#include <gtsam/nonlinear/NonlinearOptimization.h>

using namespace std;
using namespace gtsam;
using namespace pose2SLAM;

const double tol = 1e-5;

#if ENABLE_SPCG
/* ************************************************************************* */
TEST(testPose2SLAMwSPCG, example1) {

	/* generate synthetic data */
	const SharedNoiseModel sigma(noiseModel::Unit::Create(0.1));
	Key x1(1), x2(2), x3(3), x4(4), x5(5), x6(6), x7(7), x8(8), x9(9);

	// create a 3 by 3 grid
	// x3 x6 x9
	// x2 x5 x8
	// x1 x4 x7
	Graph graph;
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

	Values initial;
	initial.insert(x1, Pose2(  0,  0,   0));
	initial.insert(x2, Pose2(  0, 2.1, 0.01));
	initial.insert(x3, Pose2(  0, 3.9,-0.01));
	initial.insert(x4, Pose2(2.1,-0.1,    0));
	initial.insert(x5, Pose2(1.9, 2.1, 0.02));
	initial.insert(x6, Pose2(2.0, 3.9,-0.02));
	initial.insert(x7, Pose2(4.0, 0.1, 0.03 ));
	initial.insert(x8, Pose2(3.9, 2.1, 0.01));
	initial.insert(x9, Pose2(4.1, 3.9,-0.01));

	Values expected;
	expected.insert(x1, Pose2(0.0, 0.0, 0.0));
	expected.insert(x2, Pose2(0.0, 2.0, 0.0));
	expected.insert(x3, Pose2(0.0, 4.0, 0.0));
	expected.insert(x4, Pose2(2.0, 0.0, 0.0));
	expected.insert(x5, Pose2(2.0, 2.0, 0.0));
	expected.insert(x6, Pose2(2.0, 4.0, 0.0));
	expected.insert(x7, Pose2(4.0, 0.0, 0.0 ));
	expected.insert(x8, Pose2(4.0, 2.0, 0.0));
	expected.insert(x9, Pose2(4.0, 4.0, 0.0));

	Values actual = optimizeSPCG(graph, initial);

	EXPECT(assert_equal(expected, actual, tol));
}
#endif

/* ************************************************************************* */
int main() { TestResult tr; return TestRegistry::runAllTests(tr); }
/* ************************************************************************* */
