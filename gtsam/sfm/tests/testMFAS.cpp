#include <CppUnitLite/TestHarness.h>

#include <iostream>

#include "gtsam/sfm/mfas.h"

using namespace std;
using namespace gtsam;

/* We (partially) use the example from the paper on 1dsfm
 * (https://research.cs.cornell.edu/1dsfm/docs/1DSfM_ECCV14.pdf, Fig 1, Page 5)
 * for the unit tests here. The only change is that we leave out node 4 and use
 * only nodes 0-3. This not only makes the test easier to understand but also
 * avoids an ambiguity in the ground truth ordering that arises due to
 * insufficient edges in the geaph. */

// edges in the graph - last edge from node 3 to 0 is an outlier
vector<KeyPair> graph = {make_pair(3, 2), make_pair(0, 1), make_pair(3, 1),
                         make_pair(1, 2), make_pair(0, 2), make_pair(3, 0)};
// nodes in the graph
KeyVector nodes = {Key(0), Key(1), Key(2), Key(3)};
// weights from projecting in direction-1 (bad direction, outlier accepted)
vector<double> weights1 = {2, 1.5, 0.5, 0.25, 1, 0.75};
// weights from projecting in direction-2 (good direction, outlier rejected)
vector<double> weights2 = {0.5, 0.75, -0.25, 0.75, 1, 0.5};

// Testing the flipNegEdges function for weights1
TEST(MFAS, FlipNegEdges) {
  vector<KeyPair> graph_copy = graph;
  vector<double> weights1_positive = weights1;
  mfas::flipNegEdges(graph_copy, weights1_positive);

  // resulting graph and edges must be of same size
  EXPECT_LONGS_EQUAL(graph_copy.size(), graph.size());
  EXPECT_LONGS_EQUAL(weights1_positive.size(), weights1.size());

  for (unsigned int i = 0; i < weights1.size(); i++) {
    if (weights1[i] < 0) {
      // if original weight was negative, edges must be flipped and new weight
      // must be positive
      EXPECT_DOUBLES_EQUAL(weights1_positive[i], -weights1[i], 1e-6);
      EXPECT(graph_copy[i].first == graph[i].second &&
             graph_copy[i].second == graph[i].first);
    } else {
      // unchanged if original weight was positive
      EXPECT_DOUBLES_EQUAL(weights1_positive[i], weights1[i], 1e-6);
      EXPECT(graph_copy[i].first == graph[i].first &&
             graph_copy[i].second == graph[i].second);
    }
  }
}

// test the ordering and the outlierWeights function using weights2 - outlier
// edge is rejected when projected in a direction that gives weights2
TEST(MFAS, OrderingWeights2) {
  vector<KeyPair> graph_copy = graph;
  vector<double> weights2_positive = weights2;
  mfas::flipNegEdges(graph_copy, weights2_positive);
  FastMap<Key, int> ordered_positions;
  // compute ordering from positive edge weights
  mfas::mfasRatio(graph_copy, weights2_positive, nodes, ordered_positions);

  // expected ordering in this example
  FastMap<Key, int> gt_ordered_positions;
  gt_ordered_positions[0] = 0;
  gt_ordered_positions[1] = 1;
  gt_ordered_positions[3] = 2;
  gt_ordered_positions[2] = 3;

  // check if the expected ordering is obtained
  for (auto node : nodes) {
    EXPECT_LONGS_EQUAL(gt_ordered_positions[node], ordered_positions[node]);
  }

  // testing the outlierWeights method
  FastMap<KeyPair, double> outlier_weights;
  mfas::outlierWeights(graph_copy, weights2_positive, gt_ordered_positions,
                       outlier_weights);
  // since edge between 3 and 0 is inconsistent with the ordering, it must have
  // positive outlier weight, other outlier weights must be zero
  for (auto &edge : graph_copy) {
    if (edge == make_pair(Key(3), Key(0)) ||
        edge == make_pair(Key(0), Key(3))) {
      EXPECT_DOUBLES_EQUAL(outlier_weights[edge], 0.5, 1e-6);
    } else {
      EXPECT_DOUBLES_EQUAL(outlier_weights[edge], 0, 1e-6);
    }
  }
}

// test the ordering function and the outlierWeights method using
// weights2 (outlier edge is accepted when projected in a direction that
// produces weights2)
TEST(MFAS, OrderingWeights1) {
  vector<KeyPair> graph_copy = graph;
  vector<double> weights1_positive = weights1;
  mfas::flipNegEdges(graph_copy, weights1_positive);
  FastMap<Key, int> ordered_positions;
  // compute ordering from positive edge weights
  mfas::mfasRatio(graph_copy, weights1_positive, nodes, ordered_positions);

  // expected "ground truth" ordering in this example
  FastMap<Key, int> gt_ordered_positions;
  gt_ordered_positions[3] = 0;
  gt_ordered_positions[0] = 1;
  gt_ordered_positions[1] = 2;
  gt_ordered_positions[2] = 3;

  // check if the expected ordering is obtained
  for (auto node : nodes) {
    EXPECT_LONGS_EQUAL(gt_ordered_positions[node], ordered_positions[node]);
  }

  // since all edges (including the outlier) are consistent with this ordering,
  // all outlier_weights must be zero
  FastMap<KeyPair, double> outlier_weights;
  mfas::outlierWeights(graph, weights1_positive, gt_ordered_positions,
                       outlier_weights);
  for (auto &edge : graph) {
    EXPECT_DOUBLES_EQUAL(outlier_weights[edge], 0, 1e-6);
  }
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
