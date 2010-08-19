/**
 * @file    testClusterTree.cpp
 * @brief   Unit tests for Bayes Tree
 * @author  Kai Ni
 * @author  Frank Dellaert
 */

#include <boost/assign/std/list.hpp> // for operator +=
using namespace boost::assign;

#include <gtsam/CppUnitLite/TestHarness.h>

#include <gtsam/inference/SymbolicFactorGraph.h>
#include <gtsam/inference/ClusterTree-inl.h>

using namespace gtsam;

// explicit instantiation and typedef
template class ClusterTree<SymbolicFactorGraph>;
typedef ClusterTree<SymbolicFactorGraph> SymbolicClusterTree;

/* ************************************************************************* */
int main() {
	TestResult tr;
	return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
