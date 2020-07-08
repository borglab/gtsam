#include "gtsam/sfm/mfas.h"
#include <CppUnitLite/TestHarness.h>

using namespace std;
using namespace gtsam;

// example from the paper
Key k0(0), k1(1), k2(2), k3(3), k4(4);
KeyPair e3_2(k3, k2), e0_1(k0, k1), e4_2(k4, k2),
    e3_1(k3, k1), e4_0(k4, k0), e1_2(k1, k2),
    e0_2(k0, k2), out_e3_0(k3, k0);

vector<KeyPair> graph = {make_pair(3, 2), make_pair(0, 1), make_pair(4, 2),
                         make_pair(3, 1), make_pair(4, 0),
                         make_pair(1, 2), make_pair(0, 2), make_pair(3, 0)};
KeyVector nodes = {0, 1, 2, 3, 4};
vector<double> weights1 = {2, 1.5, -2, -0.5, -0.5, 0.25, 1, 0.75};


TEST(MFAS, FlipNegEdges) {
  vector<KeyPair> graph_copy = graph;
  vector<double> weights1_copy = weights1;
  mfas::flipNegEdges(graph_copy, weights1_copy);

  EXPECT_LONGS_EQUAL(graph_copy.size(), graph.size());
  EXPECT_LONGS_EQUAL(weights1_copy.size(), weights1.size());

  for (int i = 0; i < weights1.size(); i++) {
    if (weights1[i] < 0) {
      EXPECT_DOUBLES_EQUAL(weights1_copy[i], -weights1[i], 1e-6);
      EXPECT(graph_copy[i].first == graph[i].second &&
          graph_copy[i].second == graph[i].first);
    } else {
      EXPECT_DOUBLES_EQUAL(weights1_copy[i], weights1[i], 1e-6);
      EXPECT(graph_copy[i].first == graph[i].first &&
          graph_copy[i].second == graph[i].second);
    }
  }
}

// TEST(MFAS, Ordering) {

// }

TEST(MFAS, OrderingWithoutRemoval) {
  vector<KeyPair> graph_copy = graph;
  vector<double> weights1_copy = weights1;
  mfas::flipNegEdges(graph_copy, weights1_copy);
  FastMap<Key, int> ordered_positions;
  mfas::mfasRatio(graph_copy, weights1_copy, nodes, ordered_positions);

  FastMap<Key, int> gt_ordered_positions;
  gt_ordered_positions[4] = 0;
  gt_ordered_positions[3] = 1;
  gt_ordered_positions[0] = 2;
  gt_ordered_positions[1] = 3;
  gt_ordered_positions[2] = 4;

  for(auto it = ordered_positions.begin(); it != ordered_positions.end(); ++it)
  {
    EXPECT_LONGS_EQUAL(gt_ordered_positions[it->first], it->second);
  }
}

TEST(MFAS, BrokenWeights) {
  vector<KeyPair> graph_copy = graph;
  vector<double> weights1_copy = weights1;
  mfas::flipNegEdges(graph_copy, weights1_copy);

  FastMap<Key, int> gt_ordered_positions;
  gt_ordered_positions[4] = 0;
  gt_ordered_positions[3] = 1;
  gt_ordered_positions[0] = 2;
  gt_ordered_positions[1] = 3;
  gt_ordered_positions[2] = 4;

  FastMap<Key, double> broken_weights;
  mfas::brokenWeights(graph, weights1_copy, gt_ordered_positions,
                      broken_weights);
  for (auto it = broken_weights.begin(); it != broken_weights.end(); it++) {
    EXPECT_LONGS_EQUAL(it->second, 0);
  }
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
