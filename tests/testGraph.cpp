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

Key kx(size_t i) { return Symbol('x',i); }

/* ************************************************************************* */
// x1 -> x2
//		-> x3 -> x4
//    -> x5
TEST ( Ordering, predecessorMap2Keys ) {
	PredecessorMap<Key> p_map;
	p_map.insert(kx(1),kx(1));
	p_map.insert(kx(2),kx(1));
	p_map.insert(kx(3),kx(1));
	p_map.insert(kx(4),kx(3));
	p_map.insert(kx(5),kx(1));

	list<Key> expected;
	expected += kx(4),kx(5),kx(3),kx(2),kx(1);//PoseKey(4), PoseKey(5), PoseKey(3), PoseKey(2), PoseKey(1);

	list<Key> actual = predecessorMap2Keys<Key>(p_map);
	LONGS_EQUAL(expected.size(), actual.size());

	list<Key>::const_iterator it1 = expected.begin();
	list<Key>::const_iterator it2 = actual.begin();
	for(; it1!=expected.end(); it1++, it2++)
		CHECK(*it1 == *it2)
}

/* ************************************************************************* */
TEST( Graph, predecessorMap2Graph )
{
	typedef SGraph<string>::Vertex SVertex;
	SGraph<Key> graph;
	SVertex root;
	map<Key, SVertex> key2vertex;

	PredecessorMap<Key> p_map;
	p_map.insert(kx(1), kx(2));
	p_map.insert(kx(2), kx(2));
	p_map.insert(kx(3), kx(2));
	boost::tie(graph, root, key2vertex) = predecessorMap2Graph<SGraph<Key>, SVertex, Key>(p_map);

	LONGS_EQUAL(3, boost::num_vertices(graph));
	CHECK(root == key2vertex[kx(2)]);
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

	PredecessorMap<Key> tree;
	tree.insert(pose2SLAM::PoseKey(1),pose2SLAM::PoseKey(2));
	tree.insert(pose2SLAM::PoseKey(2),pose2SLAM::PoseKey(2));
	tree.insert(pose2SLAM::PoseKey(3),pose2SLAM::PoseKey(2));
	tree.insert(pose2SLAM::PoseKey(4),pose2SLAM::PoseKey(3));

	Pose2 rootPose = p2;

	boost::shared_ptr<Values> actual = composePoses<pose2SLAM::Graph, pose2SLAM::Odometry,
			Pose2, Key> (graph, tree, rootPose);

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
