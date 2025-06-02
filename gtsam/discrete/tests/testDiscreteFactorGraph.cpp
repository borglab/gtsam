/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/*
 *  @file testDiscreteFactorGraph.cpp
 *  @date Feb 14, 2011
 *  @author Duy-Nguyen Ta
 */

#include <CppUnitLite/TestHarness.h>
#include <gtsam/base/TestableAssertions.h>
#include <gtsam/discrete/DiscreteBayesTree.h>
#include <gtsam/discrete/DiscreteEliminationTree.h>
#include <gtsam/discrete/DiscreteFactor.h>
#include <gtsam/discrete/DiscreteFactorGraph.h>
#include <gtsam/inference/BayesNet.h>
#include <gtsam/inference/Symbol.h>

using namespace std;
using namespace gtsam;

using symbol_shorthand::L;
using symbol_shorthand::R;
using symbol_shorthand::S;
using symbol_shorthand::T;

/* ************************************************************************* */
TEST_UNSAFE(DiscreteFactorGraph, debugScheduler) {
  DiscreteKey PC(0, 4), ME(1, 4), AI(2, 4), A(3, 3);

  DiscreteFactorGraph graph;
  graph.add(AI, "1 0 0 1");
  graph.add(AI, "1 1 1 0");
  graph.add(A & AI, "1 1 1 0   1 1 1 1   0 1 1 1");
  graph.add(ME, "0 1 0 0");
  graph.add(ME, "1 1 1 0");
  graph.add(A & ME, "1 1 1 0   1 1 1 1   0 1 1 1");
  graph.add(PC, "1 0 1 0");
  graph.add(PC, "1 1 1 0");
  graph.add(A & PC, "1 1 1 0   1 1 1 1   0 1 1 1");
  graph.add(ME & AI, "0 1 1 1   1 0 1 1   1 1 0 1  1 1 1 0");
  graph.add(PC & ME, "0 1 1 1   1 0 1 1   1 1 0 1  1 1 1 0");
  graph.add(PC & AI, "0 1 1 1   1 0 1 1   1 1 0 1  1 1 1 0");

  // Check MPE.
  auto actualMPE = graph.optimize();
  EXPECT(assert_equal({{0, 2}, {1, 1}, {2, 0}, {3, 0}}, actualMPE));
}

/* ************************************************************************* */
/// TEST_DISABLED the () operator of DiscreteFactorGraph
TEST_UNSAFE(DiscreteFactorGraph, DiscreteFactorGraphEvaluationTest) {
  // Three keys P1 and P2
  DiscreteKey P1(0, 2), P2(1, 2), P3(2, 3);

  // Create the DiscreteFactorGraph
  DiscreteFactorGraph graph;
  graph.add(P1, "0.9 0.3");
  graph.add(P2, "0.9 0.6");
  graph.add(P1 & P2, "4 1 10 4");

  // Instantiate DiscreteValues
  DiscreteValues values;
  values[0] = 1;
  values[1] = 1;

  // Check if graph evaluation works ( 0.3*0.6*4 )
  EXPECT_DOUBLES_EQUAL(.72, graph(values), 1e-9);

  // Creating a new TEST_DISABLED with third node and adding unary and ternary
  // factors on it
  graph.add(P3, "0.9 0.2 0.5");
  graph.add(P1 & P2 & P3, "1 2 3 4 5 6 7 8 9 10 11 12");

  // Below values lead to selecting the 8th index in the ternary factor table
  values[0] = 1;
  values[1] = 0;
  values[2] = 1;

  // Check if graph evaluation works (0.3*0.9*1*0.2*8)
  EXPECT_DOUBLES_EQUAL(4.32, graph(values), 1e-9);

  // Below values lead to selecting the 3rd index in the ternary factor table
  values[0] = 0;
  values[1] = 1;
  values[2] = 0;

  // Check if graph evaluation works (0.9*0.6*1*0.9*4)
  EXPECT_DOUBLES_EQUAL(1.944, graph(values), 1e-9);

  // Check if graph product works
  DecisionTreeFactor product = graph.product()->toDecisionTreeFactor();
  EXPECT_DOUBLES_EQUAL(1.944, product(values), 1e-9);
}

/* ************************************************************************* */
TEST_DISABLED(DiscreteFactorGraph, TEST_DISABLED) {
  // Declare keys and ordering
  DiscreteKey C(0, 2), B(1, 2), A(2, 2);

  // A simple factor graph (A)-fAC-(C)-fBC-(B)
  // with smoothness priors
  DiscreteFactorGraph graph;
  graph.add(A & C, "3 1 1 3");
  graph.add(C & B, "3 1 1 3");

  // TEST_DISABLED EliminateDiscrete
  const Ordering frontalKeys{0};
  const auto [conditional, newFactorPtr] =
      EliminateDiscrete(graph, frontalKeys);

  DecisionTreeFactor newFactor =
      *std::dynamic_pointer_cast<DecisionTreeFactor>(newFactorPtr);

  // Normalize newFactor by max for comparison with expected
  auto denominator = newFactor.max(newFactor.size())->toDecisionTreeFactor();

  newFactor = newFactor / denominator;

  // Check Conditional
  CHECK(conditional);
  Signature signature((C | B, A) = "9/1 1/1 1/1 1/9");
  DiscreteConditional expectedConditional(signature);
  EXPECT(assert_equal(expectedConditional, *conditional));

  // Check Factor
  CHECK(&newFactor);
  DecisionTreeFactor expectedFactor(B & A, "10 6 6 10");
  // Normalize by max.
  denominator =
      expectedFactor.max(expectedFactor.size())->toDecisionTreeFactor();
  // Ensure denominator is correct.
  expectedFactor = expectedFactor / denominator;
  EXPECT(assert_equal(expectedFactor, newFactor));

  // TEST_DISABLED using elimination tree
  const Ordering ordering{0, 1, 2};
  DiscreteEliminationTree etree(graph, ordering);
  const auto [actual, remainingGraph] = etree.eliminate(&EliminateDiscrete);

  // Check Bayes net
  DiscreteBayesNet expectedBayesNet;
  expectedBayesNet.add(signature);
  expectedBayesNet.add(B | A = "5/3 3/5");
  expectedBayesNet.add(A % "1/1");
  EXPECT(assert_equal(expectedBayesNet, *actual));

  // TEST_DISABLED eliminateSequential
  DiscreteBayesNet::shared_ptr actual2 = graph.eliminateSequential(ordering);
  EXPECT(assert_equal(expectedBayesNet, *actual2));

  // TEST_DISABLED mpe
  DiscreteValues mpe{{0, 0}, {1, 0}, {2, 0}};
  auto actualMPE = graph.optimize();
  EXPECT(assert_equal(mpe, actualMPE));
  EXPECT_DOUBLES_EQUAL(9, graph(mpe), 1e-5);  // regression

  // TEST_DISABLED sumProduct alias with all orderings:
  auto mpeProbability = expectedBayesNet(mpe);
  EXPECT_DOUBLES_EQUAL(0.28125, mpeProbability, 1e-5);  // regression

  // Using custom ordering
  DiscreteBayesNet bayesNet = graph.sumProduct(ordering);
  EXPECT_DOUBLES_EQUAL(mpeProbability, bayesNet(mpe), 1e-5);

  for (Ordering::OrderingType orderingType :
       {Ordering::COLAMD, Ordering::METIS, Ordering::NATURAL,
        Ordering::CUSTOM}) {
    auto bayesNet = graph.sumProduct(orderingType);
    EXPECT_DOUBLES_EQUAL(mpeProbability, bayesNet(mpe), 1e-5);
  }
}

/* ************************************************************************* */
TEST_UNSAFE(DiscreteFactorGraph, testMaxProduct) {
  // Declare a bunch of keys
  DiscreteKey C(0, 2), A(1, 2), B(2, 2);

  // Create Factor graph
  DiscreteFactorGraph graph;
  graph.add(C & A, "0.2 0.8 0.3 0.7");
  graph.add(C & B, "0.1 0.9 0.4 0.6");

  // Created expected MPE
  DiscreteValues mpe{{0, 0}, {1, 1}, {2, 1}};

  // Do max-product with different orderings
  for (Ordering::OrderingType orderingType :
       {Ordering::COLAMD, Ordering::METIS, Ordering::NATURAL,
        Ordering::CUSTOM}) {
    DiscreteLookupDAG dag = graph.maxProduct(orderingType);
    auto actualMPE = dag.argmax();
    EXPECT(assert_equal(mpe, actualMPE));
    auto actualMPE2 = graph.optimize();  // all in one
    EXPECT(assert_equal(mpe, actualMPE2));
  }
}

/* ************************************************************************* */
TEST_DISABLED(DiscreteFactorGraph, marginalIsNotMPE) {
  // Declare 2 keys
  DiscreteKey A(0, 2), B(1, 2);

  // Create Bayes net such that marginal on A is bigger for 0 than 1, but the
  // MPE does not have A=0.
  DiscreteBayesNet bayesNet;
  bayesNet.add(B | A = "1/1 1/2");
  bayesNet.add(A % "10/9");

  // The expected MPE is A=1, B=1
  DiscreteValues mpe{{0, 1}, {1, 1}};

  // Which we verify using max-product:
  DiscreteFactorGraph graph(bayesNet);
  auto actualMPE = graph.optimize();
  EXPECT(assert_equal(mpe, actualMPE));
  EXPECT_DOUBLES_EQUAL(0.315789, graph(mpe), 1e-5);  // regression
}

/* ************************************************************************* */
TEST_DISABLED(DiscreteFactorGraph, testMPE_Darwiche09book_p244) {
  // The factor graph in Darwiche09book, page 244
  DiscreteKey A(4, 2), C(3, 2), S(2, 2), T1(0, 2), T2(1, 2);

  // Create Factor graph
  DiscreteFactorGraph graph;
  graph.add(S, "0.55 0.45");
  graph.add(S & C, "0.05 0.95 0.01 0.99");
  graph.add(C & T1, "0.80 0.20 0.20 0.80");
  graph.add(S & C & T2, "0.80 0.20 0.20 0.80 0.95 0.05 0.05 0.95");
  graph.add(T1 & T2 & A, "1 0 0 1 0 1 1 0");
  graph.add(A, "1 0");  // evidence, A = yes (first choice in Darwiche)

  DiscreteValues mpe{{0, 1}, {1, 1}, {2, 1}, {3, 1}, {4, 0}};
  EXPECT_DOUBLES_EQUAL(0.33858, graph(mpe), 1e-5);  // regression
  // You can check visually by printing product:
  // graph.product().print("Darwiche-product");

  // Check MPE.
  auto actualMPE = graph.optimize();
  EXPECT(assert_equal(mpe, actualMPE));

  // Check Bayes Net
  const Ordering ordering{0, 1, 2, 3, 4};
  auto chordal = graph.eliminateSequential(ordering);
  EXPECT_LONGS_EQUAL(5, chordal->size());

  // Let us create the Bayes tree here, just for fun, because we don't use it
  DiscreteBayesTree::shared_ptr bayesTree =
      graph.eliminateMultifrontal(ordering);
  //  bayesTree->print("Bayes Tree");
  EXPECT_LONGS_EQUAL(2, bayesTree->size());
}

/* ************************************************************************* */
TEST_DISABLED(DiscreteFactorGraph, Dot) {
  // Create Factor graph
  DiscreteFactorGraph graph;
  DiscreteKey C(0, 2), A(1, 2), B(2, 2);
  graph.add(C & A, "0.2 0.8 0.3 0.7");
  graph.add(C & B, "0.1 0.9 0.4 0.6");

  string actual = graph.dot();
  string expected =
      "graph {\n"
      "  size=\"5,5\";\n"
      "\n"
      "  var0[label=\"0\"];\n"
      "  var1[label=\"1\"];\n"
      "  var2[label=\"2\"];\n"
      "\n"
      "  factor0[label=\"\", shape=point];\n"
      "  var0--factor0;\n"
      "  var1--factor0;\n"
      "  factor1[label=\"\", shape=point];\n"
      "  var0--factor1;\n"
      "  var2--factor1;\n"
      "}\n";
  EXPECT(actual == expected);
}

/* ************************************************************************* */
TEST_DISABLED(DiscreteFactorGraph, DotWithNames) {
  // Create Factor graph
  DiscreteFactorGraph graph;
  DiscreteKey C(0, 2), A(1, 2), B(2, 2);
  graph.add(C & A, "0.2 0.8 0.3 0.7");
  graph.add(C & B, "0.1 0.9 0.4 0.6");

  vector<string> names{"C", "A", "B"};
  auto formatter = [names](Key key) { return names[key]; };
  string actual = graph.dot(formatter);
  string expected =
      "graph {\n"
      "  size=\"5,5\";\n"
      "\n"
      "  var0[label=\"C\"];\n"
      "  var1[label=\"A\"];\n"
      "  var2[label=\"B\"];\n"
      "\n"
      "  factor0[label=\"\", shape=point];\n"
      "  var0--factor0;\n"
      "  var1--factor0;\n"
      "  factor1[label=\"\", shape=point];\n"
      "  var0--factor1;\n"
      "  var2--factor1;\n"
      "}\n";
  EXPECT(actual == expected);
}

/* ************************************************************************* */
// Check markdown representation looks as expected.
TEST_DISABLED(DiscreteFactorGraph, markdown) {
  // Create Factor graph
  DiscreteFactorGraph graph;
  DiscreteKey C(0, 2), A(1, 2), B(2, 2);
  graph.add(C & A, "0.2 0.8 0.3 0.7");
  graph.add(C & B, "0.1 0.9 0.4 0.6");

  string expected =
      "`DiscreteFactorGraph` of size 2\n"
      "\n"
      "factor 0:\n"
      "|C|A|value|\n"
      "|:-:|:-:|:-:|\n"
      "|0|0|0.2|\n"
      "|0|1|0.8|\n"
      "|1|0|0.3|\n"
      "|1|1|0.7|\n"
      "\n"
      "factor 1:\n"
      "|C|B|value|\n"
      "|:-:|:-:|:-:|\n"
      "|0|0|0.1|\n"
      "|0|1|0.9|\n"
      "|1|0|0.4|\n"
      "|1|1|0.6|\n\n";
  vector<string> names{"C", "A", "B"};
  auto formatter = [names](Key key) { return names[key]; };
  string actual = graph.markdown(formatter);
  EXPECT(actual == expected);

  // Make sure values are correctly displayed.
  DiscreteValues values;
  values[0] = 1;
  values[1] = 0;
  EXPECT_DOUBLES_EQUAL(0.3, graph[0]->operator()(values), 1e-9);
}

TEST(DiscreteFactorGraph, EliminationBug) {
  // L is front left, R is front right
  // S is hind left, T is hind right
  DiscreteKey FL52(L(52), 2), FL53(L(53), 2), FL129(L(129), 2),
      FL130(L(130), 2), FL139(L(139), 2), FL140(L(140), 2), FL200(L(200), 2),
      FL201(L(201), 2), FR200(R(200), 2), FR201(R(201), 2), RL195(S(195), 2),
      RL196(S(196), 2), RL200(S(200), 2), RL201(S(201), 2), RR200(T(200), 2),
      RR201(T(201), 2);

  Eigen::SparseVector<double> p;
  p.resize(256);
  p.insert(0) = 0.25;
  p.insert(2) = 0.25;
  p.insert(4) = 0.25;
  p.insert(6) = 0.25;
  p.insert(8) = 0.25;
  p.insert(10) = 0.25;
  p.insert(16) = 0.25;
  p.insert(18) = 0.25;
  p.insert(20) = 0.25;
  p.insert(22) = 0.25;
  p.insert(24) = 0.25;
  p.insert(26) = 0.25;
  p.insert(32) = 0.25;
  p.insert(34) = 0.25;
  p.insert(36) = 0.25;
  p.insert(38) = 0.25;
  p.insert(40) = 0.25;
  p.insert(42) = 0.25;
  p.insert(64) = 1.00;
  p.insert(66) = 1.00;
  p.insert(68) = 1.00;
  p.insert(70) = 1.00;
  p.insert(72) = 1.00;
  p.insert(74) = 1.00;
  p.insert(80) = 1.00;
  p.insert(82) = 1.00;
  p.insert(84) = 1.00;
  p.insert(86) = 1.00;
  p.insert(88) = 1.00;
  p.insert(90) = 1.00;
  p.insert(96) = 1.00;
  p.insert(98) = 1.00;
  p.insert(100) = 1.00;
  p.insert(102) = 1.00;
  p.insert(104) = 1.00;
  p.insert(106) = 1.00;
  p.insert(128) = 0.75;
  p.insert(130) = 0.75;
  p.insert(132) = 0.75;
  p.insert(134) = 0.75;
  p.insert(136) = 0.75;
  p.insert(138) = 0.75;
  p.insert(144) = 0.75;
  p.insert(146) = 0.75;
  p.insert(148) = 0.75;
  p.insert(150) = 0.75;
  p.insert(152) = 0.75;
  p.insert(154) = 0.75;
  p.insert(160) = 0.75;
  p.insert(162) = 0.75;
  p.insert(164) = 0.75;
  p.insert(166) = 0.75;
  p.insert(168) = 0.75;
  p.insert(170) = 0.75;

  TableFactor f1(
      DiscreteKeys{FL52, FL53, FL129, FL130, FL139, FL140, RL195, RL196}, p);
  // f1.print();

  Eigen::SparseVector<double> p2;
  p2.resize(65536);
  p2.insert(796) = 0.811558;
  p2.insert(797) = 0.811558;
  p2.insert(798) = 0.811558;
  p2.insert(860) = 0.811558;
  p2.insert(861) = 0.811558;
  p2.insert(862) = 0.811558;
  p2.insert(924) = 0.811558;
  p2.insert(925) = 0.811558;
  p2.insert(926) = 0.811558;
  p2.insert(1820) = 0.811558;
  p2.insert(1821) = 0.811558;
  p2.insert(1822) = 0.811558;
  p2.insert(1884) = 0.811558;
  p2.insert(1885) = 0.811558;
  p2.insert(1886) = 0.811558;
  p2.insert(1948) = 0.811558;
  p2.insert(1949) = 0.811558;
  p2.insert(1950) = 0.811558;
  p2.insert(2844) = 0.811558;
  p2.insert(2845) = 0.811558;
  p2.insert(2846) = 0.811558;
  p2.insert(2908) = 0.811558;
  p2.insert(2909) = 0.811558;
  p2.insert(2910) = 0.811558;
  p2.insert(2972) = 0.811558;
  p2.insert(2973) = 0.811558;
  p2.insert(2974) = 0.811558;
  p2.insert(3868) = 0.95767;
  p2.insert(3869) = 0.95767;
  p2.insert(3870) = 0.95767;
  p2.insert(3932) = 0.95767;
  p2.insert(3933) = 0.95767;
  p2.insert(3934) = 0.95767;
  p2.insert(3996) = 0.95767;
  p2.insert(3997) = 0.95767;
  p2.insert(3998) = 0.95767;
  p2.insert(4892) = 0.811558;
  p2.insert(4893) = 0.811558;
  p2.insert(4894) = 0.811558;
  p2.insert(4956) = 0.811558;
  p2.insert(4957) = 0.811558;
  p2.insert(4958) = 0.811558;
  p2.insert(5020) = 0.811558;
  p2.insert(5021) = 0.811558;
  p2.insert(5022) = 0.811558;
  p2.insert(5916) = 0.811558;
  p2.insert(5917) = 0.811558;
  p2.insert(5918) = 0.811558;
  p2.insert(5980) = 0.811558;
  p2.insert(5981) = 0.811558;
  p2.insert(5982) = 0.811558;
  p2.insert(6044) = 0.811558;
  p2.insert(6045) = 0.811558;
  p2.insert(6046) = 0.811558;
  p2.insert(6940) = 0.811558;
  p2.insert(6941) = 0.811558;
  p2.insert(6942) = 0.811558;
  p2.insert(7004) = 0.811558;
  p2.insert(7005) = 0.811558;
  p2.insert(7006) = 0.811558;
  p2.insert(7068) = 0.811558;
  p2.insert(7069) = 0.811558;
  p2.insert(7070) = 0.811558;
  p2.insert(7964) = 0.95767;
  p2.insert(7965) = 0.95767;
  p2.insert(7966) = 0.95767;
  p2.insert(8028) = 0.95767;
  p2.insert(8029) = 0.95767;
  p2.insert(8030) = 0.95767;
  p2.insert(8092) = 0.95767;
  p2.insert(8093) = 0.95767;
  p2.insert(8094) = 0.95767;
  p2.insert(8988) = 0.811558;
  p2.insert(8989) = 0.811558;
  p2.insert(8990) = 0.811558;
  p2.insert(9052) = 0.811558;
  p2.insert(9053) = 0.811558;
  p2.insert(9054) = 0.811558;
  p2.insert(9116) = 0.811558;
  p2.insert(9117) = 0.811558;
  p2.insert(9118) = 0.811558;
  p2.insert(10012) = 0.811558;
  p2.insert(10013) = 0.811558;
  p2.insert(10014) = 0.811558;
  p2.insert(10076) = 0.811558;
  p2.insert(10077) = 0.811558;
  p2.insert(10078) = 0.811558;
  p2.insert(10140) = 0.811558;
  p2.insert(10141) = 0.811558;
  p2.insert(10142) = 0.811558;
  p2.insert(11036) = 0.811558;
  p2.insert(11037) = 0.811558;
  p2.insert(11038) = 0.811558;
  p2.insert(11100) = 0.811558;
  p2.insert(11101) = 0.811558;
  p2.insert(11102) = 0.811558;
  p2.insert(11164) = 0.811558;
  p2.insert(11165) = 0.811558;
  p2.insert(11166) = 0.811558;
  p2.insert(12060) = 0.95767;
  p2.insert(12061) = 0.95767;
  p2.insert(12062) = 0.95767;
  p2.insert(12124) = 0.95767;
  p2.insert(12125) = 0.95767;
  p2.insert(12126) = 0.95767;
  p2.insert(12188) = 0.95767;
  p2.insert(12189) = 0.95767;
  p2.insert(12190) = 0.95767;
  p2.insert(13084) = 0.853836;
  p2.insert(13085) = 0.853836;
  p2.insert(13086) = 0.853836;
  p2.insert(13148) = 0.853836;
  p2.insert(13149) = 0.853836;
  p2.insert(13150) = 0.853836;
  p2.insert(13212) = 0.853836;
  p2.insert(13213) = 0.853836;
  p2.insert(13214) = 0.853836;
  p2.insert(14108) = 0.853836;
  p2.insert(14109) = 0.853836;
  p2.insert(14110) = 0.853836;
  p2.insert(14172) = 0.853836;
  p2.insert(14173) = 0.853836;
  p2.insert(14174) = 0.853836;
  p2.insert(14236) = 0.853836;
  p2.insert(14237) = 0.853836;
  p2.insert(14238) = 0.853836;
  p2.insert(15132) = 0.853836;
  p2.insert(15133) = 0.853836;
  p2.insert(15134) = 0.853836;
  p2.insert(15196) = 0.853836;
  p2.insert(15197) = 0.853836;
  p2.insert(15198) = 0.853836;
  p2.insert(15260) = 0.853836;
  p2.insert(15261) = 0.853836;
  p2.insert(15262) = 0.853836;
  p2.insert(16156) = 1;
  p2.insert(16157) = 1;
  p2.insert(16158) = 1;
  p2.insert(16220) = 1;
  p2.insert(16221) = 1;
  p2.insert(16222) = 1;
  p2.insert(16284) = 1;
  p2.insert(16285) = 1;
  p2.insert(16286) = 1;
  p2.insert(17180) = 0.811558;
  p2.insert(17181) = 0.811558;
  p2.insert(17182) = 0.811558;
  p2.insert(17244) = 0.811558;
  p2.insert(17245) = 0.811558;
  p2.insert(17246) = 0.811558;
  p2.insert(17308) = 0.811558;
  p2.insert(17309) = 0.811558;
  p2.insert(17310) = 0.811558;
  p2.insert(18204) = 0.811558;
  p2.insert(18205) = 0.811558;
  p2.insert(18206) = 0.811558;
  p2.insert(18268) = 0.811558;
  p2.insert(18269) = 0.811558;
  p2.insert(18270) = 0.811558;
  p2.insert(18332) = 0.811558;
  p2.insert(18333) = 0.811558;
  p2.insert(18334) = 0.811558;
  p2.insert(19228) = 0.811558;
  p2.insert(19229) = 0.811558;
  p2.insert(19230) = 0.811558;
  p2.insert(19292) = 0.811558;
  p2.insert(19293) = 0.811558;
  p2.insert(19294) = 0.811558;
  p2.insert(19356) = 0.811558;
  p2.insert(19357) = 0.811558;
  p2.insert(19358) = 0.811558;
  p2.insert(20252) = 0.95767;
  p2.insert(20253) = 0.95767;
  p2.insert(20254) = 0.95767;
  p2.insert(20316) = 0.95767;
  p2.insert(20317) = 0.95767;
  p2.insert(20318) = 0.95767;
  p2.insert(20380) = 0.95767;
  p2.insert(20381) = 0.95767;
  p2.insert(20382) = 0.95767;
  p2.insert(21276) = 0.811558;
  p2.insert(21277) = 0.811558;
  p2.insert(21278) = 0.811558;
  p2.insert(21340) = 0.811558;
  p2.insert(21341) = 0.811558;
  p2.insert(21342) = 0.811558;
  p2.insert(21404) = 0.811558;
  p2.insert(21405) = 0.811558;
  p2.insert(21406) = 0.811558;
  p2.insert(22300) = 0.811558;
  p2.insert(22301) = 0.811558;
  p2.insert(22302) = 0.811558;
  p2.insert(22364) = 0.811558;
  p2.insert(22365) = 0.811558;
  p2.insert(22366) = 0.811558;
  p2.insert(22428) = 0.811558;
  p2.insert(22429) = 0.811558;
  p2.insert(22430) = 0.811558;
  p2.insert(23324) = 0.811558;
  p2.insert(23325) = 0.811558;
  p2.insert(23326) = 0.811558;
  p2.insert(23388) = 0.811558;
  p2.insert(23389) = 0.811558;
  p2.insert(23390) = 0.811558;
  p2.insert(23452) = 0.811558;
  p2.insert(23453) = 0.811558;
  p2.insert(23454) = 0.811558;
  p2.insert(24348) = 0.95767;
  p2.insert(24349) = 0.95767;
  p2.insert(24350) = 0.95767;
  p2.insert(24412) = 0.95767;
  p2.insert(24413) = 0.95767;
  p2.insert(24414) = 0.95767;
  p2.insert(24476) = 0.95767;
  p2.insert(24477) = 0.95767;
  p2.insert(24478) = 0.95767;
  p2.insert(25372) = 0.811558;
  p2.insert(25373) = 0.811558;
  p2.insert(25374) = 0.811558;
  p2.insert(25436) = 0.811558;
  p2.insert(25437) = 0.811558;
  p2.insert(25438) = 0.811558;
  p2.insert(25500) = 0.811558;
  p2.insert(25501) = 0.811558;
  p2.insert(25502) = 0.811558;
  p2.insert(26396) = 0.811558;
  p2.insert(26397) = 0.811558;
  p2.insert(26398) = 0.811558;
  p2.insert(26460) = 0.811558;
  p2.insert(26461) = 0.811558;
  p2.insert(26462) = 0.811558;
  p2.insert(26524) = 0.811558;
  p2.insert(26525) = 0.811558;
  p2.insert(26526) = 0.811558;
  p2.insert(27420) = 0.811558;
  p2.insert(27421) = 0.811558;
  p2.insert(27422) = 0.811558;
  p2.insert(27484) = 0.811558;
  p2.insert(27485) = 0.811558;
  p2.insert(27486) = 0.811558;
  p2.insert(27548) = 0.811558;
  p2.insert(27549) = 0.811558;
  p2.insert(27550) = 0.811558;
  p2.insert(28444) = 0.95767;
  p2.insert(28445) = 0.95767;
  p2.insert(28446) = 0.95767;
  p2.insert(28508) = 0.95767;
  p2.insert(28509) = 0.95767;
  p2.insert(28510) = 0.95767;
  p2.insert(28572) = 0.95767;
  p2.insert(28573) = 0.95767;
  p2.insert(28574) = 0.95767;
  p2.insert(29468) = 0.853836;
  p2.insert(29469) = 0.853836;
  p2.insert(29470) = 0.853836;
  p2.insert(29532) = 0.853836;
  p2.insert(29533) = 0.853836;
  p2.insert(29534) = 0.853836;
  p2.insert(29596) = 0.853836;
  p2.insert(29597) = 0.853836;
  p2.insert(29598) = 0.853836;
  p2.insert(30492) = 0.853836;
  p2.insert(30493) = 0.853836;
  p2.insert(30494) = 0.853836;
  p2.insert(30556) = 0.853836;
  p2.insert(30557) = 0.853836;
  p2.insert(30558) = 0.853836;
  p2.insert(30620) = 0.853836;
  p2.insert(30621) = 0.853836;
  p2.insert(30622) = 0.853836;
  p2.insert(31516) = 0.853836;
  p2.insert(31517) = 0.853836;
  p2.insert(31518) = 0.853836;
  p2.insert(31580) = 0.853836;
  p2.insert(31581) = 0.853836;
  p2.insert(31582) = 0.853836;
  p2.insert(31644) = 0.853836;
  p2.insert(31645) = 0.853836;
  p2.insert(31646) = 0.853836;
  p2.insert(32540) = 1;
  p2.insert(32541) = 1;
  p2.insert(32542) = 1;
  p2.insert(32604) = 1;
  p2.insert(32605) = 1;
  p2.insert(32606) = 1;
  p2.insert(32668) = 1;
  p2.insert(32669) = 1;
  p2.insert(32670) = 1;
  p2.insert(33564) = 0.811558;
  p2.insert(33565) = 0.811558;
  p2.insert(33566) = 0.811558;
  p2.insert(33628) = 0.811558;
  p2.insert(33629) = 0.811558;
  p2.insert(33630) = 0.811558;
  p2.insert(33692) = 0.811558;
  p2.insert(33693) = 0.811558;
  p2.insert(33694) = 0.811558;
  p2.insert(34588) = 0.811558;
  p2.insert(34589) = 0.811558;
  p2.insert(34590) = 0.811558;
  p2.insert(34652) = 0.811558;
  p2.insert(34653) = 0.811558;
  p2.insert(34654) = 0.811558;
  p2.insert(34716) = 0.811558;
  p2.insert(34717) = 0.811558;
  p2.insert(34718) = 0.811558;
  p2.insert(35612) = 0.811558;
  p2.insert(35613) = 0.811558;
  p2.insert(35614) = 0.811558;
  p2.insert(35676) = 0.811558;
  p2.insert(35677) = 0.811558;
  p2.insert(35678) = 0.811558;
  p2.insert(35740) = 0.811558;
  p2.insert(35741) = 0.811558;
  p2.insert(35742) = 0.811558;
  p2.insert(36636) = 0.95767;
  p2.insert(36637) = 0.95767;
  p2.insert(36638) = 0.95767;
  p2.insert(36700) = 0.95767;
  p2.insert(36701) = 0.95767;
  p2.insert(36702) = 0.95767;
  p2.insert(36764) = 0.95767;
  p2.insert(36765) = 0.95767;
  p2.insert(36766) = 0.95767;
  p2.insert(37660) = 0.811558;
  p2.insert(37661) = 0.811558;
  p2.insert(37662) = 0.811558;
  p2.insert(37724) = 0.811558;
  p2.insert(37725) = 0.811558;
  p2.insert(37726) = 0.811558;
  p2.insert(37788) = 0.811558;
  p2.insert(37789) = 0.811558;
  p2.insert(37790) = 0.811558;
  p2.insert(38684) = 0.811558;
  p2.insert(38685) = 0.811558;
  p2.insert(38686) = 0.811558;
  p2.insert(38748) = 0.811558;
  p2.insert(38749) = 0.811558;
  p2.insert(38750) = 0.811558;
  p2.insert(38812) = 0.811558;
  p2.insert(38813) = 0.811558;
  p2.insert(38814) = 0.811558;
  p2.insert(39708) = 0.811558;
  p2.insert(39709) = 0.811558;
  p2.insert(39710) = 0.811558;
  p2.insert(39772) = 0.811558;
  p2.insert(39773) = 0.811558;
  p2.insert(39774) = 0.811558;
  p2.insert(39836) = 0.811558;
  p2.insert(39837) = 0.811558;
  p2.insert(39838) = 0.811558;
  p2.insert(40732) = 0.95767;
  p2.insert(40733) = 0.95767;
  p2.insert(40734) = 0.95767;
  p2.insert(40796) = 0.95767;
  p2.insert(40797) = 0.95767;
  p2.insert(40798) = 0.95767;
  p2.insert(40860) = 0.95767;
  p2.insert(40861) = 0.95767;
  p2.insert(40862) = 0.95767;
  p2.insert(41756) = 0.811558;
  p2.insert(41757) = 0.811558;
  p2.insert(41758) = 0.811558;
  p2.insert(41820) = 0.811558;
  p2.insert(41821) = 0.811558;
  p2.insert(41822) = 0.811558;
  p2.insert(41884) = 0.811558;
  p2.insert(41885) = 0.811558;
  p2.insert(41886) = 0.811558;
  p2.insert(42780) = 0.811558;
  p2.insert(42781) = 0.811558;
  p2.insert(42782) = 0.811558;
  p2.insert(42844) = 0.811558;
  p2.insert(42845) = 0.811558;
  p2.insert(42846) = 0.811558;
  p2.insert(42908) = 0.811558;
  p2.insert(42909) = 0.811558;
  p2.insert(42910) = 0.811558;
  p2.insert(43804) = 0.811558;
  p2.insert(43805) = 0.811558;
  p2.insert(43806) = 0.811558;
  p2.insert(43868) = 0.811558;
  p2.insert(43869) = 0.811558;
  p2.insert(43870) = 0.811558;
  p2.insert(43932) = 0.811558;
  p2.insert(43933) = 0.811558;
  p2.insert(43934) = 0.811558;
  p2.insert(44828) = 0.95767;
  p2.insert(44829) = 0.95767;
  p2.insert(44830) = 0.95767;
  p2.insert(44892) = 0.95767;
  p2.insert(44893) = 0.95767;
  p2.insert(44894) = 0.95767;
  p2.insert(44956) = 0.95767;
  p2.insert(44957) = 0.95767;
  p2.insert(44958) = 0.95767;
  p2.insert(45852) = 0.853836;
  p2.insert(45853) = 0.853836;
  p2.insert(45854) = 0.853836;
  p2.insert(45916) = 0.853836;
  p2.insert(45917) = 0.853836;
  p2.insert(45918) = 0.853836;
  p2.insert(45980) = 0.853836;
  p2.insert(45981) = 0.853836;
  p2.insert(45982) = 0.853836;
  p2.insert(46876) = 0.853836;
  p2.insert(46877) = 0.853836;
  p2.insert(46878) = 0.853836;
  p2.insert(46940) = 0.853836;
  p2.insert(46941) = 0.853836;
  p2.insert(46942) = 0.853836;
  p2.insert(47004) = 0.853836;
  p2.insert(47005) = 0.853836;
  p2.insert(47006) = 0.853836;
  p2.insert(47900) = 0.853836;
  p2.insert(47901) = 0.853836;
  p2.insert(47902) = 0.853836;
  p2.insert(47964) = 0.853836;
  p2.insert(47965) = 0.853836;
  p2.insert(47966) = 0.853836;
  p2.insert(48028) = 0.853836;
  p2.insert(48029) = 0.853836;
  p2.insert(48030) = 0.853836;
  p2.insert(48924) = 1;
  p2.insert(48925) = 1;
  p2.insert(48926) = 1;
  p2.insert(48988) = 1;
  p2.insert(48989) = 1;
  p2.insert(48990) = 1;
  p2.insert(49052) = 1;
  p2.insert(49053) = 1;
  p2.insert(49054) = 1;
  p2.insert(49948) = 0.813079;
  p2.insert(49949) = 0.813079;
  p2.insert(49950) = 0.813079;
  p2.insert(50012) = 0.813079;
  p2.insert(50013) = 0.813079;
  p2.insert(50014) = 0.813079;
  p2.insert(50076) = 0.813079;
  p2.insert(50077) = 0.813079;
  p2.insert(50078) = 0.813079;
  p2.insert(50972) = 0.813079;
  p2.insert(50973) = 0.813079;
  p2.insert(50974) = 0.813079;
  p2.insert(51036) = 0.813079;
  p2.insert(51037) = 0.813079;
  p2.insert(51038) = 0.813079;
  p2.insert(51100) = 0.813079;
  p2.insert(51101) = 0.813079;
  p2.insert(51102) = 0.813079;
  p2.insert(51996) = 0.813079;
  p2.insert(51997) = 0.813079;
  p2.insert(51998) = 0.813079;
  p2.insert(52060) = 0.813079;
  p2.insert(52061) = 0.813079;
  p2.insert(52062) = 0.813079;
  p2.insert(52124) = 0.813079;
  p2.insert(52125) = 0.813079;
  p2.insert(52126) = 0.813079;
  p2.insert(53020) = 0.958315;
  p2.insert(53021) = 0.958315;
  p2.insert(53022) = 0.958315;
  p2.insert(53084) = 0.958315;
  p2.insert(53085) = 0.958315;
  p2.insert(53086) = 0.958315;
  p2.insert(53148) = 0.958315;
  p2.insert(53149) = 0.958315;
  p2.insert(53150) = 0.958315;
  p2.insert(54044) = 0.813079;
  p2.insert(54045) = 0.813079;
  p2.insert(54046) = 0.813079;
  p2.insert(54108) = 0.813079;
  p2.insert(54109) = 0.813079;
  p2.insert(54110) = 0.813079;
  p2.insert(54172) = 0.813079;
  p2.insert(54173) = 0.813079;
  p2.insert(54174) = 0.813079;
  p2.insert(55068) = 0.813079;
  p2.insert(55069) = 0.813079;
  p2.insert(55070) = 0.813079;
  p2.insert(55132) = 0.813079;
  p2.insert(55133) = 0.813079;
  p2.insert(55134) = 0.813079;
  p2.insert(55196) = 0.813079;
  p2.insert(55197) = 0.813079;
  p2.insert(55198) = 0.813079;
  p2.insert(56092) = 0.813079;
  p2.insert(56093) = 0.813079;
  p2.insert(56094) = 0.813079;
  p2.insert(56156) = 0.813079;
  p2.insert(56157) = 0.813079;
  p2.insert(56158) = 0.813079;
  p2.insert(56220) = 0.813079;
  p2.insert(56221) = 0.813079;
  p2.insert(56222) = 0.813079;
  p2.insert(57116) = 0.958315;
  p2.insert(57117) = 0.958315;
  p2.insert(57118) = 0.958315;
  p2.insert(57180) = 0.958315;
  p2.insert(57181) = 0.958315;
  p2.insert(57182) = 0.958315;
  p2.insert(57244) = 0.958315;
  p2.insert(57245) = 0.958315;
  p2.insert(57246) = 0.958315;
  p2.insert(58140) = 0.813079;
  p2.insert(58141) = 0.813079;
  p2.insert(58142) = 0.813079;
  p2.insert(58204) = 0.813079;
  p2.insert(58205) = 0.813079;
  p2.insert(58206) = 0.813079;
  p2.insert(58268) = 0.813079;
  p2.insert(58269) = 0.813079;
  p2.insert(58270) = 0.813079;
  p2.insert(59164) = 0.813079;
  p2.insert(59165) = 0.813079;
  p2.insert(59166) = 0.813079;
  p2.insert(59228) = 0.813079;
  p2.insert(59229) = 0.813079;
  p2.insert(59230) = 0.813079;
  p2.insert(59292) = 0.813079;
  p2.insert(59293) = 0.813079;
  p2.insert(59294) = 0.813079;
  p2.insert(60188) = 0.813079;
  p2.insert(60189) = 0.813079;
  p2.insert(60190) = 0.813079;
  p2.insert(60252) = 0.813079;
  p2.insert(60253) = 0.813079;
  p2.insert(60254) = 0.813079;
  p2.insert(60316) = 0.813079;
  p2.insert(60317) = 0.813079;
  p2.insert(60318) = 0.813079;
  p2.insert(61212) = 0.958315;
  p2.insert(61213) = 0.958315;
  p2.insert(61214) = 0.958315;
  p2.insert(61276) = 0.958315;
  p2.insert(61277) = 0.958315;
  p2.insert(61278) = 0.958315;
  p2.insert(61340) = 0.958315;
  p2.insert(61341) = 0.958315;
  p2.insert(61342) = 0.958315;
  p2.insert(62236) = 0.855446;
  p2.insert(62237) = 0.855446;
  p2.insert(62238) = 0.855446;
  p2.insert(62300) = 0.855446;
  p2.insert(62301) = 0.855446;
  p2.insert(62302) = 0.855446;
  p2.insert(62364) = 0.855446;
  p2.insert(62365) = 0.855446;
  p2.insert(62366) = 0.855446;
  p2.insert(63260) = 0.855446;
  p2.insert(63261) = 0.855446;
  p2.insert(63262) = 0.855446;
  p2.insert(63324) = 0.855446;
  p2.insert(63325) = 0.855446;
  p2.insert(63326) = 0.855446;
  p2.insert(63388) = 0.855446;
  p2.insert(63389) = 0.855446;
  p2.insert(63390) = 0.855446;
  p2.insert(64284) = 0.855446;
  p2.insert(64285) = 0.855446;
  p2.insert(64286) = 0.855446;
  p2.insert(64348) = 0.855446;
  p2.insert(64349) = 0.855446;
  p2.insert(64350) = 0.855446;
  p2.insert(64412) = 0.855446;
  p2.insert(64413) = 0.855446;
  p2.insert(64414) = 0.855446;
  p2.insert(65308) = 0.999849;
  p2.insert(65309) = 0.999849;
  p2.insert(65310) = 0.999849;
  p2.insert(65372) = 0.999849;
  p2.insert(65373) = 0.999849;
  p2.insert(65374) = 0.999849;
  p2.insert(65436) = 0.999849;
  p2.insert(65437) = 0.999849;
  p2.insert(65438) = 0.999849;

  TableFactor f2(
      DiscreteKeys{FL52, FL53, FL129, FL130, FL139, FL140, FL200, FL201, FR200,
                   FR201, RL195, RL196, RL200, RL201, RR200, RR201},
      p2);
  // f2.print();

  DiscreteFactorGraph dfg;
  dfg.push_back(f1);
  dfg.push_back(f2);
  // dfg.print();

  auto [conditional, tau] =
      dfg.eliminatePartialSequential(Ordering{FL52.first}, EliminateDiscrete);
  // FL52
  conditional->print();
  tau->print();
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
