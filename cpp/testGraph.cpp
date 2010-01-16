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

#include "pose2SLAM.h"
#include "LieConfig-inl.h"
#include "graph-inl.h"

using namespace std;
using namespace boost;
using namespace gtsam;

/* ************************************************************************* */
TEST( Graph, predecessorMap2Graph )
{
	typedef SGraph<string>::Vertex SVertex;
	SGraph<string> graph;
	SVertex root;
	map<string, SVertex> key2vertex;

	PredecessorMap<string> p_map;
	p_map.insert("x1", "x2");
	p_map.insert("x2", "x2");
	p_map.insert("x3", "x2");
	tie(graph, root, key2vertex) = predecessorMap2Graph<SGraph<string>, SVertex, string>(p_map);

	LONGS_EQUAL(3, boost::num_vertices(graph));
	CHECK(root == key2vertex["x2"]);
}

/* ************************************************************************* */
TEST( Graph, composePoses )
{
	Pose2Graph graph;
	Matrix cov = eye(3);
	graph.addConstraint(1,2, Pose2(2.0, 0.0, 0.0), cov);
	graph.addConstraint(2,3, Pose2(3.0, 0.0, 0.0), cov);

	PredecessorMap<Pose2Config::Key> tree;
	tree.insert(1,2);
	tree.insert(2,2);
	tree.insert(3,2);

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
