/**
 * @file    testGaussianISAM2.cpp
 * @brief   Unit tests for GaussianISAM2
 * @author  Michael Kaess
 */

#include <boost/foreach.hpp>
#include <boost/assign/std/list.hpp> // for operator +=
using namespace boost::assign;

#include <gtsam/CppUnitLite/TestHarness.h>

#define GTSAM_MAGIC_KEY

#include <gtsam/nonlinear/Ordering.h>
#include <gtsam/linear/GaussianBayesNet.h>
#include <gtsam/slam/GaussianISAM2.h>
#include <gtsam/slam/smallExample.h>
#include <gtsam/slam/planarSLAM.h>

using namespace std;
using namespace gtsam;
using namespace example;

const double tol = 1e-4;

///* ************************************************************************* */
//TEST( ISAM2, solving )
//{
//	Graph nlfg = createNonlinearFactorGraph();
//	Values noisy = createNoisyValues();
//	Ordering ordering;
//	ordering += symbol('x', 1);
//	ordering += symbol('x', 2);
//	ordering += symbol('l', 1);
//	// FIXME: commented out due due to compile error in ISAM - this should be fixed
////	GaussianISAM2 btree(nlfg, ordering, noisy);
////	VectorValues actualDelta = optimize2(btree);
////	VectorValues delta = createCorrectDelta();
////	CHECK(assert_equal(delta, actualDelta, 0.01));
////	Values actualSolution = noisy.expmap(actualDelta);
////	Values solution = createValues();
////	CHECK(assert_equal(solution, actualSolution, tol));
//}
//
///* ************************************************************************* */
//TEST( ISAM2, ISAM2_smoother )
//{
//	// Create smoother with 7 nodes
//	Graph smoother;
//	Values poses;
//	boost::tie(smoother, poses) = createNonlinearSmoother(7);
//
//	// run ISAM2 for every factor
//	GaussianISAM2 actual;
//	BOOST_FOREACH(boost::shared_ptr<NonlinearFactor<Values> > factor, smoother) {
//		Graph factorGraph;
//		factorGraph.push_back(factor);
//		actual.update(factorGraph, poses);
//	}
//
//	// Create expected Bayes Tree by solving smoother with "natural" ordering
//	Ordering ordering;
//	for (int t = 1; t <= 7; t++) ordering += symbol('x', t);
//	GaussianISAM2 expected(smoother, ordering, poses);
//
//	// Check whether BayesTree is correct
//	CHECK(assert_equal(expected, actual));
//
//	// obtain solution
//	VectorValues e; // expected solution
//	Vector v = Vector_(2, 0., 0.);
//	// FIXME: commented out due due to compile error in ISAM - this should be fixed
////	for (int i=1; i<=7; i++)
////		e.insert(symbol('x', i), v);
////	VectorValues optimized = optimize2(actual); // actual solution
////	CHECK(assert_equal(e, optimized));
//}
//
///* ************************************************************************* */
//TEST( ISAM2, ISAM2_smoother2 )
//{
//	// Create smoother with 7 nodes
//	Graph smoother;
//	Values poses;
//	boost::tie(smoother, poses) = createNonlinearSmoother(7);
//
//	// Create initial tree from first 4 timestamps in reverse order !
//	Ordering ord; ord += "x4","x3","x2","x1";
//	Graph factors1;
//	for (int i=0;i<7;i++) factors1.push_back(smoother[i]);
//	GaussianISAM2 actual(factors1, ord, poses);
//
//	// run ISAM2 with remaining factors
//	Graph factors2;
//	for (int i=7;i<13;i++) factors2.push_back(smoother[i]);
//	actual.update(factors2, poses);
//
//	// Create expected Bayes Tree by solving smoother with "natural" ordering
//	Ordering ordering;
//	for (int t = 1; t <= 7; t++) ordering += symbol('x', t);
//	GaussianISAM2 expected(smoother, ordering, poses);
//
//	CHECK(assert_equal(expected, actual));
//}

/* ************************************************************************* */
TEST(ISAM2, slamlike_solution)
{
  typedef planarSLAM::PoseKey PoseKey;
  typedef planarSLAM::PointKey PointKey;

  double wildfire = -1.0;
  planarSLAM::Values init;
  planarSLAM::Values fullinit;
  GaussianISAM2_P isam;
  planarSLAM::Graph newfactors;
  planarSLAM::Graph fullgraph;
  SharedDiagonal odoNoise = sharedSigmas(Vector_(3, 0.1, 0.1, M_PI/100.0));
  SharedDiagonal brNoise = sharedSigmas(Vector_(2, M_PI/100.0, 0.1));

  size_t i = 0;

  newfactors = planarSLAM::Graph();
  init.clear();
  newfactors.addPrior(0, Pose2(0.0, 0.0, 0.0), odoNoise);
  init.insert(PoseKey(0), Pose2(0.01, 0.01, 0.01));
  fullinit.insert(PoseKey(0), Pose2(0.01, 0.01, 0.01));
  isam.update(newfactors, init, wildfire, 0.0, false);
  fullgraph.push_back(newfactors);

  for( ; i<5; ++i) {
    newfactors = planarSLAM::Graph();
    init.clear();
    newfactors.addOdometry(i, i+1, Pose2(1.0, 0.0, 0.0), odoNoise);
    init.insert(PoseKey(i+1), Pose2(double(i+1)+0.1, -0.1, 0.01));
    fullinit.insert(PoseKey(i+1), Pose2(double(i+1)+0.1, -0.1, 0.01));
    isam.update(newfactors, init, wildfire, 0.0, false);
    fullgraph.push_back(newfactors);
  }

  newfactors = planarSLAM::Graph();
  init.clear();
  newfactors.addOdometry(i, i+1, Pose2(1.0, 0.0, 0.0), odoNoise);
  newfactors.addBearingRange(i, 0, Rot2::fromAngle(M_PI/4.0), 5.0, brNoise);
  newfactors.addBearingRange(i, 1, Rot2::fromAngle(-M_PI/4.0), 5.0, brNoise);
  init.insert(PoseKey(i+1), Pose2(1.01, 0.01, 0.01));
  init.insert(PointKey(0), Point2(5.0/sqrt(2.0), 5.0/sqrt(2.0)));
  init.insert(PointKey(1), Point2(5.0/sqrt(2.0), -5.0/sqrt(2.0)));
  fullinit.insert(PoseKey(i+1), Pose2(1.01, 0.01, 0.01));
  fullinit.insert(PointKey(0), Point2(5.0/sqrt(2.0), 5.0/sqrt(2.0)));
  fullinit.insert(PointKey(1), Point2(5.0/sqrt(2.0), -5.0/sqrt(2.0)));
  isam.update(newfactors, init, wildfire, 0.0, false);
  fullgraph.push_back(newfactors);
  ++ i;

  for( ; i<5; ++i) {
    newfactors = planarSLAM::Graph();
    init.clear();
    newfactors.addOdometry(i, i+1, Pose2(1.0, 0.0, 0.0), odoNoise);
    init.insert(PoseKey(i+1), Pose2(double(i+1)+0.1, -0.1, 0.01));
    fullinit.insert(PoseKey(i+1), Pose2(double(i+1)+0.1, -0.1, 0.01));
    isam.update(newfactors, init, wildfire, 0.0, false);
    fullgraph.push_back(newfactors);
  }

  newfactors = planarSLAM::Graph();
  init.clear();
  newfactors.addOdometry(i, i+1, Pose2(1.0, 0.0, 0.0), odoNoise);
  newfactors.addBearingRange(i, 0, Rot2::fromAngle(M_PI/4.0 + M_PI/16.0), 4.5, brNoise);
  newfactors.addBearingRange(i, 1, Rot2::fromAngle(-M_PI/4.0 + M_PI/16.0), 4.5, brNoise);
  init.insert(PoseKey(i+1), Pose2(6.9, 0.1, 0.01));
  fullinit.insert(PoseKey(i+1), Pose2(6.9, 0.1, 0.01));
  isam.update(newfactors, init, wildfire, 0.0, false);
  fullgraph.push_back(newfactors);
  ++ i;

//  newfactors = planarSLAM::Graph();
//  init.clear();
//  isam.update(newfactors, init, wildfire, 0.0, true);
//  isam.update(newfactors, init, wildfire, 0.0, true);
//  isam.update(newfactors, init, wildfire, 0.0, true);
//  isam.update(newfactors, init, wildfire, 0.0, true);
//  isam.update(newfactors, init, wildfire, 0.0, true);

  // Compare solutions
  planarSLAM::Values actual = isam.calculateEstimate();
  Ordering ordering = isam.getOrdering(); // *fullgraph.orderingCOLAMD(fullinit).first;
  GaussianFactorGraph linearized = *fullgraph.linearize(fullinit, ordering);
//  linearized.print("Expected linearized: ");
  GaussianBayesNet gbn = *Inference::Eliminate(linearized);
//  gbn.print("Expected bayesnet: ");
  VectorValues delta = optimize(gbn);
  planarSLAM::Values expected = fullinit.expmap(delta, ordering);
//  planarSLAM::Values expected = *NonlinearOptimizer<planarSLAM::Graph, planarSLAM::Values>::optimizeLM(fullgraph, fullinit);

  CHECK(assert_equal(expected, actual));
}

/* ************************************************************************* */
int main() { TestResult tr; return TestRegistry::runAllTests(tr);}
/* ************************************************************************* */
