/*
 * @file testSQPOptimizer.cpp
 * @brief tests the optimization algorithm for nonlinear graphs with nonlinear constraints
 * @author Alex Cunningham
 */

#include <CppUnitLite/TestHarness.h>
#include "NonlinearFactorGraph.h"
#include "NonlinearConstraint.h"
#include "VectorConfig.h"
#include "Ordering.h"
#include "SQPOptimizer.h"

// implementations
#include "NonlinearConstraint-inl.h"
#include "SQPOptimizer-inl.h"

using namespace std;
using namespace gtsam;
	
// typedefs
typedef NonlinearFactorGraph<VectorConfig> NLGraph;
typedef boost::shared_ptr<VectorConfig> shared_config;

TEST ( SQPOptimizer, basic ) {
	// create a basic optimizer
	NLGraph graph;
	Ordering ordering;
	shared_config config(new VectorConfig);

	SQPOptimizer<NLGraph, VectorConfig> optimizer(graph, ordering, config);

	// verify components
	CHECK(assert_equal(graph, *(optimizer.graph())));
	CHECK(assert_equal(ordering, *(optimizer.ordering())));
	CHECK(assert_equal(*config, *(optimizer.config())));
}

/* ************************************************************************* */
int main() { TestResult tr; return TestRegistry::runAllTests(tr); }
/* ************************************************************************* */
