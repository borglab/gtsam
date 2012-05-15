/*
 * testTypedDiscreteFactorGraph.cpp
 * @brief test readable factor graphs
 * @author Duy-Nguyen Ta
 * @author Frank Dellaert
 * @date Feb 14, 2011
 */

#include <gtsam_unstable/discrete/TypedDiscreteFactorGraph.h>
#include <CppUnitLite/TestHarness.h>
#include <boost/foreach.hpp>

using namespace std;
using namespace gtsam;

/* ******************************************************************************** */
// initialize some common test values
DiscreteKey v0("v0"), v1("v1"), v2("v2", 3);
TypedDiscreteFactor::Values values;

void init() {
	values[v0] = 0;
	values[v1] = 0;
	values[v2] = 1;
}

string path("../../../gtsam/discrete/tests/data/");

/* ************************************************************************* */
TEST( TypedDiscreteFactorGraph, parseUAI)
{
	// This reads in a small factor graph on discrete variables v0, v1, and v2
	TypedDiscreteFactorGraph graph(path + "UAI/sampleMARKOV.uai");

	//	GTSAM_PRINT(graph);

	LONGS_EQUAL(3,graph.nrFactors())

	double expectedP001 = 0.436 * 0.128 * 0.333;
	double actualP001 = graph(values);
	DOUBLES_EQUAL(expectedP001,actualP001,1e-9)
}

/* ************************************************************************* */
TEST( TypedDiscreteFactorGraph, parseUAI1)
{
	// This reads in a big graph from UAI 2008
	TypedDiscreteFactorGraph graph(path + "UAI/uai08_test1.uai");
	//	GTSAM_PRINT(graph);
	LONGS_EQUAL(54,graph.nrFactors())
}

/* ************************************************************************* */
TEST( TypedDiscreteFactorGraph, parseUAI2)
{
	// This reads in a big graph from UAI 2008
	TypedDiscreteFactorGraph graph(path + "UAI/uai08_test2.uai");
	//	GTSAM_PRINT(graph);
	LONGS_EQUAL(21,graph.nrFactors())
}

/* ************************************************************************* */
TEST( TypedDiscreteFactorGraph, parseUAI3)
{
	// This reads in a big graph from UAI 2008
	TypedDiscreteFactorGraph graph(path + "UAI/uai08_test3.uai");
	//	GTSAM_PRINT(graph);
	LONGS_EQUAL(13,graph.nrFactors())
}

///* ******************************************************************************** */
//// global test data
//
//string graphFilename(path + "UAI/uai08_test2.uai");
//string evidFilename (path + "UAI/uai08_test2.uai.evid");
//
///**
// *  [Cluster] Ordering splitted from libDAI
// *  {x9}, {x10}, {x11}, {x12}, {x13}, {x14}, {x15}, {x16}, {x17}, {x18}, {x19}, {x20},
// *  {x0, x1, x8}, {x2, x6, x7}, {x4, x6, x8},
// *  {x1, x5, x7}, {x1, x3, x7, x8}, {x3, x6, x7, x8})
// *
// */
//size_t ordering[21]   = {9,10,11,12,13,14,15,16,17,18,19,20,0,2,4,5,1,3,6,7,8};
//vector<Index> vOrdering;
//
//// Container for all data read from files.
//TypedDiscreteFactorGraph container;
//
//// The factor graph generated from the data, after assigning the elimination ordering
//// for each variable
//DiscreteFactorGraph::shared_ptr graph;
//
///* ******************************************************************************** */
//// Initialize all test data
//void initTestData()
//{
//  container.readFromFile_UAI(graphFilename);
//  container.readEvidence_UAI(evidFilename);
//  for (size_t i = 0; i<21; i++) vOrdering.push_back(ordering[i]);
//  container.setOrdering(vOrdering);
//  graph = container.generateFactorGraph();
//}
//
//
///* ******************************************************************************** */
//// Test reading .fg file from libDAI
//TEST( TypedDiscreteFactorGraph, readFG)
//{
//  TypedDiscreteFactorGraph graph;
//  graph.readFromFile_FG(path + "FG/alarm.fg");
////  graph.print();
//}
//
///* ******************************************************************************** */
//TEST( TypedDiscreteFactorGraph, testSequentialSolver)
//{
////  tic(0, "Sequential Solver");
//
//  boost::shared_ptr<PotentialTable::MapAssignment> actualMPE
//        = DiscreteSequentialSolver(*graph).optimize();
//  BOOST_FOREACH(const PotentialTable::MapAssignment::value_type asg, *actualMPE)
//    cout << vOrdering[asg.first] << ": " << asg.second << endl;
//
////  toc(0, "Sequential Solver");
//  tictoc_print();
//
//}
//
///* ******************************************************************************** */
//void backSubstitute(const BayesTree<DiscreteConditional>::sharedClique currentClique,
//    PotentialTable::MapAssignment& assignments) {
//
//  // solve the bayes net in the current node
//  DiscreteBayesNet::const_reverse_iterator it = currentClique->rbegin();
//  for (; it!=currentClique->rend(); ++it) {
//    size_t val = (*it)->solve(assignments); // Solve for that variable
//    assignments[(*it)->key()] = val;   // store result in partial solution
//  }
//
//  // solve the bayes nets in the child nodes
//  BOOST_FOREACH(const BayesTree<DiscreteConditional>::sharedClique& child,
//      currentClique->children()) {
//    backSubstitute(child, assignments);
//  }
//}
//
//
//void optimizeMultiFrontal(DiscreteFactorGraph::shared_ptr graph) {
//
//  VariableIndex::shared_ptr structure(new VariableIndex(*graph));
//  JunctionTree<DiscreteFactorGraph>::shared_ptr junctionTree(new JunctionTree<DiscreteFactorGraph>(*graph, *structure));
//  BayesTree<DiscreteConditional>::sharedClique rootClique = junctionTree->eliminate();
//
////  toc(1, "GJT eliminate");
//
////  tictoc_print();
//
////  // Allocate solution vector
////  tic(2, "allocate VectorValues");
////  vector<size_t> dims(rootClique->back()->key() + 1, 0);
////  countDims(rootClique, dims);
////  VectorValues result(dims);
////  toc(2, "allocate VectorValues");
////
////  // back-substitution
////  tic(3, "back-substitute");
////  btreeBackSubstitute(rootClique, result);
////  toc(3, "back-substitute");
////  return result;
//}
//
//
///* ******************************************************************************** */
//
//TEST( TypedDiscreteFactorGraph, multifrontalSolver)
//{
//  tic(0, "Multifrontal Solver");
//  optimizeMultiFrontal(graph);
//
//  toc(0, "Multifrontal Solver");
//  tictoc_print();
//}


/* ************************************************************************* */
int main() { /*initTestData(); */
	init();
	TestResult tr;
	return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */

