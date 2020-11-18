/**
 *  @file  testMFAS.cpp
 *  @brief Unit tests for the MFAS class
 *  @author Akshay Krishnan
 *  @date July 2020
 */

#include <gtsam/sfm/MFAS.h>

#include <CppUnitLite/TestHarness.h>

using namespace std;
using namespace gtsam;

/**
 * We (partially) use the example from the paper on 1dsfm
 * (https://research.cs.cornell.edu/1dsfm/docs/1DSfM_ECCV14.pdf, Fig 1, Page 5)
 * for the unit tests here. The only change is that we leave out node 4 and use
 * only nodes 0-3. This makes the test easier to understand and also
 * avoids an ambiguity in the ground truth ordering that arises due to
 * insufficient edges in the geaph when using the 4th node.
 */

// edges in the graph - last edge from node 3 to 0 is an outlier
vector<MFAS::KeyPair> edges = {make_pair(3, 2), make_pair(0, 1), make_pair(3, 1),
                         make_pair(1, 2), make_pair(0, 2), make_pair(3, 0)};
// nodes in the graph
KeyVector nodes = {Key(0), Key(1), Key(2), Key(3)};
// weights from projecting in direction-1 (bad direction, outlier accepted)
vector<double> weights1 = {2, 1.5, 0.5, 0.25, 1, 0.75};
// weights from projecting in direction-2 (good direction, outlier rejected)
vector<double> weights2 = {0.5, 0.75, -0.25, 0.75, 1, 0.5};

// helper function to obtain map from keypairs to weights from the 
// vector representations
map<MFAS::KeyPair, double> getEdgeWeights(const vector<MFAS::KeyPair> &edges,
                                         const vector<double> &weights) {
  map<MFAS::KeyPair, double> edgeWeights;
  for (size_t i = 0; i < edges.size(); i++) {
    edgeWeights[edges[i]] = weights[i];
  }
  return edgeWeights;
}

// test the ordering and the outlierWeights function using weights2 - outlier
// edge is rejected when projected in a direction that gives weights2
TEST(MFAS, OrderingWeights2) {
  MFAS mfas_obj(getEdgeWeights(edges, weights2));

  KeyVector ordered_nodes = mfas_obj.computeOrdering();

  // ground truth (expected) ordering in this example
  KeyVector gt_ordered_nodes = {0, 1, 3, 2};

  // check if the expected ordering is obtained
  for (size_t i = 0; i < ordered_nodes.size(); i++) {
    EXPECT_LONGS_EQUAL(gt_ordered_nodes[i], ordered_nodes[i]);
  }

  map<MFAS::KeyPair, double> outlier_weights = mfas_obj.computeOutlierWeights();

  // since edge between 3 and 0 is inconsistent with the ordering, it must have
  // positive outlier weight, other outlier weights must be zero
  for (auto &edge : edges) {
    if (edge == make_pair(Key(3), Key(0)) ||
        edge == make_pair(Key(0), Key(3))) {
      EXPECT_DOUBLES_EQUAL(outlier_weights[edge], 0.5, 1e-6);
    } else {
      EXPECT_DOUBLES_EQUAL(outlier_weights[edge], 0, 1e-6);
    }
  }
}

// test the ordering function and the outlierWeights method using
// weights1 (outlier edge is accepted when projected in a direction that
// produces weights1)
TEST(MFAS, OrderingWeights1) {
  MFAS mfas_obj(getEdgeWeights(edges, weights1));

  KeyVector ordered_nodes = mfas_obj.computeOrdering();

  // "ground truth" expected ordering in this example
  KeyVector gt_ordered_nodes = {3, 0, 1, 2};

  // check if the expected ordering is obtained
  for (size_t i = 0; i < ordered_nodes.size(); i++) {
    EXPECT_LONGS_EQUAL(gt_ordered_nodes[i], ordered_nodes[i]);
  }

  map<MFAS::KeyPair, double> outlier_weights = mfas_obj.computeOutlierWeights();

  // since edge between 3 and 0 is inconsistent with the ordering, it must have
  // positive outlier weight, other outlier weights must be zero
  for (auto &edge : edges) {
    EXPECT_DOUBLES_EQUAL(outlier_weights[edge], 0, 1e-6);
  }
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
