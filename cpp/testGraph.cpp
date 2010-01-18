/*
 * testGraph.cpp
 *
 *   Created on: Jan 12, 2010
 *       Author: nikai
 *  Description: unit test for graph-inl.h
 */

#include <iostream>
#include <boost/shared_ptr.hpp>
#include <boost/assign/std/list.hpp> // for operator +=
using namespace boost::assign;

#include <CppUnitLite/TestHarness.h>

// TODO: DANGEROUS, create shared pointers
#define GTSAM_MAGIC_GAUSSIAN 3

#include "pose2SLAM.h"
#include "TupleConfig-inl.h"
#include "graph-inl.h"
#include "FactorGraph-inl.h"

using namespace std;
using namespace gtsam;

/* ************************************************************************* */
// x1 -> x2
//		-> x3 -> x4
//    -> x5
TEST ( Ordering, predecessorMap2Keys ) {
	typedef TypedSymbol<Pose2,'x'> Key;
	PredecessorMap<Key> p_map;
	p_map.insert(1,1);
	p_map.insert(2,1);
	p_map.insert(3,1);
	p_map.insert(4,3);
	p_map.insert(5,1);

	list<Key> expected;
	expected += 4,5,3,2,1;//Key(4), Key(5), Key(3), Key(2), Key(1);

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
