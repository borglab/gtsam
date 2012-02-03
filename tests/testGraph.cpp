/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file testGraph.cpp
 * @date Jan 12, 2010
 * @author nikai
 * @brief unit test for graph-inl.h
 */

#include <iostream>
#include <boost/shared_ptr.hpp>
#include <boost/assign/std/list.hpp> // for operator +=
using namespace boost::assign;

#include <CppUnitLite/TestHarness.h>

// TODO: DANGEROUS, create shared pointers
#define GTSAM_MAGIC_GAUSSIAN 3

#include <gtsam/slam/pose2SLAM.h>
#include <gtsam/inference/graph-inl.h>

using namespace std;
using namespace gtsam;

/* ************************************************************************* */
// x1 -> x2
//		-> x3 -> x4
//    -> x5
TEST ( Ordering, predecessorMap2Keys ) {
	typedef TypedSymbol<Pose2,'x'> PoseKey;
	PredecessorMap<PoseKey> p_map;
	p_map.insert(1,1);
	p_map.insert(2,1);
	p_map.insert(3,1);
	p_map.insert(4,3);
	p_map.insert(5,1);

	list<PoseKey> expected;
	expected += 4,5,3,2,1;//PoseKey(4), PoseKey(5), PoseKey(3), PoseKey(2), PoseKey(1);

	list<PoseKey> actual = predecessorMap2Keys<PoseKey>(p_map);
	LONGS_EQUAL(expected.size(), actual.size());

	list<PoseKey>::const_iterator it1 = expected.begin();
	list<PoseKey>::const_iterator it2 = actual.begin();
	for(; it1!=expected.end(); it1++, it2++)
		CHECK(*it1 == *it2)
}

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
	pose2SLAM::Graph graph;
	Matrix cov = eye(3);
	Pose2 p1(1.0, 2.0, 0.3), p2(4.0, 5.0, 0.6), p3(7.0, 8.0, 0.9), p4(2.0, 2.0, 2.9);
	Pose2 p12=p1.between(p2), p23=p2.between(p3), p43=p4.between(p3);
	graph.addOdometry(1,2, p12, cov);
	graph.addOdometry(2,3, p23, cov);
	graph.addOdometry(4,3, p43, cov);

	PredecessorMap<pose2SLAM::PoseKey> tree;
	tree.insert(pose2SLAM::PoseKey(1),2);
	tree.insert(pose2SLAM::PoseKey(2),2);
	tree.insert(pose2SLAM::PoseKey(3),2);
	tree.insert(pose2SLAM::PoseKey(4),3);

	Pose2 rootPose = p2;

	boost::shared_ptr<Values> actual = composePoses<pose2SLAM::Graph, pose2SLAM::Odometry,
			Pose2, pose2SLAM::PoseKey> (graph, tree, rootPose);

	Values expected;
	expected.insert(pose2SLAM::PoseKey(1), p1);
	expected.insert(pose2SLAM::PoseKey(2), p2);
	expected.insert(pose2SLAM::PoseKey(3), p3);
	expected.insert(pose2SLAM::PoseKey(4), p4);

	LONGS_EQUAL(4, actual->size());
	CHECK(assert_equal(expected, *actual));
}

/* ************************************************************************* */
int main() {
	TestResult tr;
	return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
