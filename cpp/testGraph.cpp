/*
 * testGraph.cpp
 *
 *   Created on: Jan 12, 2010
 *       Author: nikai
 *  Description: unit test for graph-inl.h
 */

#include <iostream>
#include <boost/shared_ptr.hpp>
#include <boost/assign/std/list.hpp>

#include <CppUnitLite/TestHarness.h>

#include "Pose2Graph.h"
#include "LieConfig-inl.h"
#include "graph-inl.h"

using namespace std;
using namespace boost;
using namespace gtsam;

/* ************************************************************************* */
TEST( Graph, composePoses )
{
	Pose2Graph graph;
	Matrix cov = eye(3);
	graph.push_back(boost::shared_ptr<Pose2Factor>(new Pose2Factor(1,2, Pose2(2.0, 0.0, 0.0), cov)));
	graph.push_back(boost::shared_ptr<Pose2Factor>(new Pose2Factor(2,3, Pose2(3.0, 0.0, 0.0), cov)));

	PredecessorMap<Pose2Config::Key> tree;
	tree.insert(make_pair(1,2));
	tree.insert(make_pair(2,2));
	tree.insert(make_pair(3,2));

	Pose2 rootPose(3.0, 0.0, 0.0);

	boost::shared_ptr<Pose2Config> actual = composePoses<Pose2Graph, Pose2Factor,
			Pose2, Pose2Config> (graph, tree, rootPose);

	Pose2Config expected;
	expected.insert(1, Pose2(1.0, 0.0, 0.0));
	expected.insert(2, Pose2(3.0, 0.0, 0.0));
	expected.insert(3, Pose2(6.0, 0.0, 0.0));

	LONGS_EQUAL(3, actual->size());
	CHECK(assert_equal(expected, *actual));
}

/* ************************************************************************* */
int main() {
	TestResult tr;
	return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
