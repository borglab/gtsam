/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/*
 * @file    testDiscreteConditional.cpp
 * @brief   unit tests for DiscreteConditional
 * @author  Duy-Nguyen Ta
 * @author  Frank dellaert
 * @date    Feb 14, 2011
 */

#include <CppUnitLite/TestHarness.h>
#include <gtsam/base/serializationTestHelpers.h>
#include <gtsam/discrete/DecisionTreeFactor.h>
#include <gtsam/discrete/DiscreteConditional.h>
#include <gtsam/discrete/TableFactor.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/discrete/DiscreteFactorGraph.h>
#include <gtsam/discrete/DiscreteBayesNet.h>


using namespace std;
using namespace gtsam;

/* ************************************************************************* */
TEST(DiscreteConditional, constructors) {
  DiscreteKey X(0, 2), Y(2, 3), Z(1, 2);  // watch ordering !

  DiscreteConditional actual(X | Y = "1/1 2/3 1/4");
  EXPECT_LONGS_EQUAL(0, *(actual.beginFrontals()));
  EXPECT_LONGS_EQUAL(2, *(actual.beginParents()));
  EXPECT(actual.endParents() == actual.end());
  EXPECT(actual.endFrontals() == actual.beginParents());

  DecisionTreeFactor f1(X & Y, "0.5 0.4 0.2 0.5 0.6 0.8");
  DiscreteConditional expected1(1, f1);
  EXPECT(assert_equal(expected1, actual, 1e-9));

  DecisionTreeFactor f2(
      X & Y & Z, "0.2 0.5 0.3 0.6 0.4 0.7 0.25 0.55 0.35 0.65 0.45 0.75");
  DiscreteConditional actual2(1, f2);
  DecisionTreeFactor expected2 = f2 / f2.sum(1)->toDecisionTreeFactor();
  EXPECT(actual2.keys() == f2.keys());
  EXPECT(assert_equal<DecisionTreeFactor::ADT>(expected2,
                                               actual2.toDecisionTreeFactor()));

  std::vector<double> probs{0.2, 0.5, 0.3, 0.6, 0.4, 0.7, 0.25, 0.55, 0.35, 0.65, 0.45, 0.75};
  DiscreteConditional actual3(X, {Y, Z}, probs);
  DecisionTreeFactor expected3 = f2;
  EXPECT(assert_equal(expected3, static_cast<DecisionTreeFactor>(actual3)));
}

/* ************************************************************************* */
TEST(DiscreteConditional, constructors_alt_interface) {
  DiscreteKey X(0, 2), Y(2, 3), Z(1, 2);  // watch ordering !

  const Signature::Row r1{1, 1}, r2{2, 3}, r3{1, 4};
  const Signature::Table table{r1, r2, r3};
  DiscreteConditional actual1(X, {Y}, table);

  DecisionTreeFactor f1(X & Y, "0.5 0.4 0.2 0.5 0.6 0.8");
  DiscreteConditional expected1(1, f1);
  EXPECT(assert_equal(expected1, actual1, 1e-9));

  DecisionTreeFactor f2(
      X & Y & Z, "0.2 0.5 0.3 0.6 0.4 0.7 0.25 0.55 0.35 0.65 0.45 0.75");
  DiscreteConditional actual2(1, f2);
  DecisionTreeFactor expected2 = f2 / f2.sum(1)->toDecisionTreeFactor();
  EXPECT(actual2.keys() == f2.keys());
  EXPECT(assert_equal<DecisionTreeFactor::ADT>(expected2,
                                               actual2.toDecisionTreeFactor()));
}

/* ************************************************************************* */
TEST(DiscreteConditional, constructors2) {
  DiscreteKey C(0, 2), B(1, 2);
  Signature signature((C | B) = "4/1 3/1");
  DiscreteConditional actual(signature);

  DecisionTreeFactor expected(C & B, "0.8 0.75 0.2 0.25");
  EXPECT(assert_equal(expected, static_cast<DecisionTreeFactor>(actual)));
}

/* ************************************************************************* */
TEST(DiscreteConditional, constructors3) {
  DiscreteKey C(0, 2), B(1, 2), A(2, 2);
  Signature signature((C | B, A) = "4/1 1/1 1/1 1/4");
  DiscreteConditional actual(signature);

  DecisionTreeFactor expected(C & B & A, "0.8 0.5 0.5 0.2 0.2 0.5 0.5 0.8");
  EXPECT(assert_equal(expected, static_cast<DecisionTreeFactor>(actual)));
}

/* ****************************************************************************/
// Test evaluate for a discrete Prior P(Asia).
TEST(DiscreteConditional, PriorProbability) {
  constexpr Key asiaKey = 0;
  const DiscreteKey Asia(asiaKey, 2);
  DiscreteConditional dc(Asia, "4/6");
  DiscreteValues values{{asiaKey, 0}};
  EXPECT_DOUBLES_EQUAL(0.4, dc.evaluate(values), 1e-9);
  EXPECT(DiscreteConditional::CheckInvariants(dc, values));
}

/* ************************************************************************* */
// Check that error, logProbability, evaluate all work as expected.
TEST(DiscreteConditional, probability) {
  DiscreteKey C(2, 2), D(4, 2), E(3, 2);
  DiscreteConditional C_given_DE((C | D, E) = "4/1 1/1 1/1 1/4");

  DiscreteValues given {{C.first, 1}, {D.first, 0}, {E.first, 0}};
  EXPECT_DOUBLES_EQUAL(0.2, C_given_DE.evaluate(given), 1e-9);
  EXPECT_DOUBLES_EQUAL(0.2, C_given_DE(given), 1e-9);
  EXPECT_DOUBLES_EQUAL(log(0.2), C_given_DE.logProbability(given), 1e-9);
  EXPECT_DOUBLES_EQUAL(-log(0.2), C_given_DE.error(given), 1e-9);
  EXPECT(DiscreteConditional::CheckInvariants(C_given_DE, given));
}

/* ************************************************************************* */
// Check calculation of joint P(A,B)
TEST(DiscreteConditional, Multiply) {
  DiscreteKey A(1, 2), B(0, 2);
  DiscreteConditional conditional(A | B = "1/2 2/1");
  DiscreteConditional prior(B % "1/2");

  // The expected factor, with keys in the sorted order of the product
  DecisionTreeFactor f(B & A, "1 2 4 2");
  DiscreteConditional expected(2, f);

  // P(A,B) = P(A|B) * P(B) = P(B) * P(A|B)
  for (auto&& actual : {prior * conditional, conditional * prior}) {
    EXPECT_LONGS_EQUAL(2, actual.nrFrontals());
    KeyVector frontals(actual.beginFrontals(), actual.endFrontals());
    EXPECT((frontals == KeyVector{0, 1}));
    for (auto&& it : actual.enumerate()) {
      const DiscreteValues& v = it.first;
      EXPECT_DOUBLES_EQUAL(actual(v), conditional(v) * prior(v), 1e-9);
    }
    // And for good measure:
    EXPECT(assert_equal(expected, actual));
  }
}

/* ************************************************************************* */
// Check calculation of conditional joint P(A,B|C)
TEST(DiscreteConditional, Multiply2) {
  DiscreteKey A(0, 2), B(1, 2), C(2, 2);
  DiscreteConditional A_given_B(A | B = "1/3 3/1");
  DiscreteConditional B_given_C(B | C = "1/3 3/1");

  // P(A,B|C) = P(A|B)P(B|C) = P(B|C)P(A|B)
  for (auto&& actual : {A_given_B * B_given_C, B_given_C * A_given_B}) {
    EXPECT_LONGS_EQUAL(2, actual.nrFrontals());
    EXPECT_LONGS_EQUAL(1, actual.nrParents());
    KeyVector frontals(actual.beginFrontals(), actual.endFrontals());
    EXPECT((frontals == KeyVector{0, 1}));
    for (auto&& it : actual.enumerate()) {
      const DiscreteValues& v = it.first;
      EXPECT_DOUBLES_EQUAL(actual(v), A_given_B(v) * B_given_C(v), 1e-9);
    }
  }
}

/* ************************************************************************* */
// Check calculation of conditional joint P(A,B|C), double check keys
TEST(DiscreteConditional, Multiply3) {
  DiscreteKey A(1, 2), B(2, 2), C(0, 2);  // different keys!!!
  DiscreteConditional A_given_B(A | B = "1/3 3/1");
  DiscreteConditional B_given_C(B | C = "1/3 3/1");

  // P(A,B|C) = P(A|B)P(B|C) = P(B|C)P(A|B)
  for (auto&& actual : {A_given_B * B_given_C, B_given_C * A_given_B}) {
    EXPECT_LONGS_EQUAL(2, actual.nrFrontals());
    EXPECT_LONGS_EQUAL(1, actual.nrParents());
    KeyVector frontals(actual.beginFrontals(), actual.endFrontals());
    EXPECT((frontals == KeyVector{1, 2}));
    for (auto&& it : actual.enumerate()) {
      const DiscreteValues& v = it.first;
      EXPECT_DOUBLES_EQUAL(actual(v), A_given_B(v) * B_given_C(v), 1e-9);
    }
  }
}

/* ************************************************************************* */
// Check calculation of conditional joint P(A,B,C|D,E) = P(A,B|D) P(C|D,E)
TEST(DiscreteConditional, Multiply4) {
  DiscreteKey A(0, 2), B(1, 2), C(2, 2), D(4, 2), E(3, 2);
  DiscreteConditional A_given_B(A | B = "1/3 3/1");
  DiscreteConditional B_given_D(B | D = "1/3 3/1");
  DiscreteConditional AB_given_D = A_given_B * B_given_D;
  DiscreteConditional C_given_DE((C | D, E) = "4/1 1/1 1/1 1/4");

  // P(A,B,C|D,E) = P(A,B|D) P(C|D,E) = P(C|D,E) P(A,B|D)
  for (auto&& actual : {AB_given_D * C_given_DE, C_given_DE * AB_given_D}) {
    EXPECT_LONGS_EQUAL(3, actual.nrFrontals());
    EXPECT_LONGS_EQUAL(2, actual.nrParents());
    KeyVector frontals(actual.beginFrontals(), actual.endFrontals());
    EXPECT((frontals == KeyVector{0, 1, 2}));
    KeyVector parents(actual.beginParents(), actual.endParents());
    EXPECT((parents == KeyVector{3, 4}));
    for (auto&& it : actual.enumerate()) {
      const DiscreteValues& v = it.first;
      EXPECT_DOUBLES_EQUAL(actual(v), AB_given_D(v) * C_given_DE(v), 1e-9);
    }
  }
}

/* ************************************************************************* */
// Check calculation of marginals for joint P(A,B)
TEST(DiscreteConditional, marginals) {
  DiscreteKey A(1, 2), B(0, 2);
  DiscreteConditional conditional(A | B = "1/2 2/1");
  DiscreteConditional prior(B % "1/2");
  DiscreteConditional pAB = prior * conditional;

  // P(A=0) = P(A=0|B=0)P(B=0) + P(A=0|B=1)P(B=1) = 1*1 + 2*2 = 5
  // P(A=1) = P(A=1|B=0)P(B=0) + P(A=1|B=1)P(B=1) = 2*1 + 1*2 = 4
  DiscreteConditional actualA = pAB.marginal(A.first);
  DiscreteConditional pA(A % "5/4");
  EXPECT(assert_equal(pA, actualA));
  EXPECT(actualA.frontals() == KeyVector{1});
  EXPECT_LONGS_EQUAL(0, actualA.nrParents());

  DiscreteConditional actualB = pAB.marginal(B.first);
  EXPECT(assert_equal(prior, actualB));
  EXPECT(actualB.frontals() == KeyVector{0});
  EXPECT_LONGS_EQUAL(0, actualB.nrParents());
}

/* ************************************************************************* */
// Check calculation of marginals in case branches are pruned
TEST(DiscreteConditional, marginals2) {
  DiscreteKey A(0, 2), B(1, 2);  // changing keys need to make pruning happen!
  DiscreteConditional conditional(A | B = "2/2 3/1");
  DiscreteConditional prior(B % "1/2");
  DiscreteConditional pAB = prior * conditional;
  // P(A=0) = P(A=0|B=0)P(B=0) + P(A=0|B=1)P(B=1) = 2*1 + 3*2 = 8
  // P(A=1) = P(A=1|B=0)P(B=0) + P(A=1|B=1)P(B=1) = 2*1 + 1*2 = 4
  DiscreteConditional actualA = pAB.marginal(A.first);
  DiscreteConditional pA(A % "8/4");
  EXPECT(assert_equal(pA, actualA));

  DiscreteConditional actualB = pAB.marginal(B.first);
  EXPECT(assert_equal(prior, actualB));
}

/* ************************************************************************* */
TEST(DiscreteConditional, likelihood) {
  DiscreteKey X(0, 2), Y(1, 3);
  DiscreteConditional conditional(X | Y = "2/8 4/6 5/5");

  auto actual0 = conditional.likelihood(0);
  DecisionTreeFactor expected0(Y, "0.2 0.4 0.5");
  EXPECT(assert_equal(expected0, *actual0, 1e-9));

  auto actual1 = conditional.likelihood(1);
  DecisionTreeFactor expected1(Y, "0.8 0.6 0.5");
  EXPECT(assert_equal(expected1, *actual1, 1e-9));
}

/* ************************************************************************* */
// Check choose on P(C|D,E)
TEST(DiscreteConditional, choose) {
  DiscreteKey C(2, 2), D(4, 2), E(3, 2);
  DiscreteConditional C_given_DE((C | D, E) = "4/1 1/1 1/1 1/4");

  // Case 1: no given values: no-op
  DiscreteValues given;
  auto actual1 = C_given_DE.choose(given);
  EXPECT(assert_equal(C_given_DE, *actual1, 1e-9));

  // Case 2: 1 given value
  given[D.first] = 1;
  auto actual2 = C_given_DE.choose(given);
  EXPECT_LONGS_EQUAL(1, actual2->nrFrontals());
  EXPECT_LONGS_EQUAL(1, actual2->nrParents());
  DiscreteConditional expected2(C | E = "1/1 1/4");
  EXPECT(assert_equal(expected2, *actual2, 1e-9));

  // Case 2: 2 given values
  given[E.first] = 0;
  auto actual3 = C_given_DE.choose(given);
  EXPECT_LONGS_EQUAL(1, actual3->nrFrontals());
  EXPECT_LONGS_EQUAL(0, actual3->nrParents());
  DiscreteConditional expected3(C % "1/1");
  EXPECT(assert_equal(expected3, *actual3, 1e-9));
}

/* ************************************************************************* */
namespace key_order_fixture {
// Keys deliberately not in increasing order: X comes first but has key 1.
const DiscreteKey X(1, 2), Y(0, 3), Z(2, 2);

// The stored factor takes the conditional's key order, whatever its input
// order, and conditionals in different representations compare equal.
TEST(DiscreteConditional, StoredFactorKeyOrder) {
  const std::string spec = "1 4 2 5 3 6";  // over (Y, X)
  const DiscreteConditional fromTable(
      1, X & Y, std::make_shared<TableFactor>(Y & X, spec));
  EXPECT(fromTable.factor()->keys() == fromTable.keys());
  EXPECT(std::dynamic_pointer_cast<TableFactor>(fromTable.factor()));

  const DecisionTreeFactor tree(Y & X, spec);
  const DiscreteConditional fromTree(
      1, X & Y, std::make_shared<DecisionTreeFactor>(tree));
  EXPECT(fromTree.factor()->keys() == fromTree.keys());
  for (auto&& [assignment, value] : tree.enumerate()) {
    EXPECT_DOUBLES_EQUAL(value, fromTable(assignment), 1e-9);
  }
  EXPECT(assert_equal(fromTree, fromTable));
}

// Construction rejects potentials whose cardinalities or keys do not match,
// even when the key ids are already in order.
TEST(DiscreteConditional, MismatchedPotentials) {
  const DiscreteKey threeStateX(X.first, 3);
  const auto table = std::make_shared<TableFactor>(X, "0.5 0.5");
  CHECK_EXCEPTION(DiscreteConditional(1, DiscreteKeys{threeStateX}, table),
                  std::invalid_argument);
  const auto tree = std::make_shared<DecisionTreeFactor>(X, "0.5 0.5");
  CHECK_EXCEPTION(DiscreteConditional(1, DiscreteKeys{threeStateX}, tree),
                  std::invalid_argument);
  CHECK_EXCEPTION(DiscreteConditional(1, X & Y, table), std::invalid_argument);
}

// Constructing from a factor takes its first key as the frontal and normalizes
// over it, even when that key is not the smallest.
TEST(DiscreteConditional, FromFactorKeyOrder) {
  const DecisionTreeFactor joint(X & Y, std::vector<double>{1, 2, 3, 4, 5, 6});
  const DiscreteConditional conditional(1, joint);
  EXPECT_LONGS_EQUAL(X.first, conditional.firstFrontalKey());
  for (size_t y = 0; y < Y.second; y++) {
    double total = 0.0;
    for (size_t x = 0; x < X.second; x++) {
      total += conditional({{X.first, x}, {Y.first, y}});
    }
    EXPECT_DOUBLES_EQUAL(1.0, total, 1e-9);
  }
}

// Products, choose, and removeDiscreteModes keep the stored factor in the
// conditional's key order, and match the decision tree results.
TEST(DiscreteConditional, DerivedKeyOrder) {
  const std::vector<double> zGivenX{0.25, 0.5, 0.75, 0.5};  // over (Z, X)
  const DiscreteConditional tableZ(
      1, Z & X, std::make_shared<TableFactor>(Z & X, zGivenX));
  const DiscreteConditional treeZ(1, Z & X,
                                  DecisionTreeFactor::ADT(Z & X, zGivenX));
  const DiscreteConditional xGivenY(X, {Y}, "1/1 1/2 2/1");

  const DiscreteConditional product = tableZ * xGivenY;
  EXPECT(product.factor()->keys() == product.keys());
  EXPECT(assert_equal(treeZ * xGivenY, product));

  const auto chosen = product.choose({{Y.first, 1}});
  EXPECT(chosen->factor()->keys() == chosen->keys());

  DiscreteConditional removed = product;
  removed.removeDiscreteModes({{X.first, 0}});
  EXPECT(removed.factor()->keys() == removed.keys());
  EXPECT(std::dynamic_pointer_cast<TableFactor>(removed.factor()));
  DiscreteConditional treeRemoved = treeZ * xGivenY;
  treeRemoved.removeDiscreteModes({{X.first, 0}});
  EXPECT(assert_equal(treeRemoved, removed));
}

}  // namespace key_order_fixture
/* ************************************************************************* */
// Pruning returns a renormalized conditional with the original key order.
TEST(DiscreteConditional, Prune) {
  // Keys are deliberately not in sorted order.
  DiscreteKey A(5, 2), B(3, 2);
  DiscreteConditional conditional(A, {B}, "1/4 2/3");  // P(A|B)

  // Keeping the top two values, 0.8 and 0.6, leaves A=1 for each B.
  auto pruned =
      std::dynamic_pointer_cast<DiscreteConditional>(conditional.prune(2));
  CHECK(pruned);
  EXPECT(conditional.keys() == pruned->keys());
  EXPECT_LONGS_EQUAL(1, pruned->nrFrontals());
  DiscreteConditional expected(A, {B}, "0/1 0/1");
  EXPECT(assert_equal(expected, *pruned, 1e-9));
}

/* ************************************************************************* */
// Check argmax on P(C|D) and P(D), plus tie-breaking for P(B)
TEST(DiscreteConditional, Argmax) {
  DiscreteKey C(2, 2), D(4, 2);
  DiscreteConditional B_prior(D, "1/1");
  DiscreteConditional D_prior(D, "1/3");
  DiscreteConditional C_given_D((C | D) = "1/4 1/1");

  // Case 1: Tie breaking
  size_t actual1 = B_prior.argmax();
  // In the case of ties, the first value is chosen.
  EXPECT_LONGS_EQUAL(0, actual1);
  // Case 2: No parents
  size_t actual2 = D_prior.argmax();
  // Selects 1 since it has 0.75 probability
  EXPECT_LONGS_EQUAL(1, actual2);

  // Case 3: Given parent values
  DiscreteValues given;
  given[D.first] = 1;
  size_t actual3 = C_given_D.argmax(given);
  // Should be 0 since D=1 gives 0.5/0.5
  EXPECT_LONGS_EQUAL(0, actual3);
  
  given[D.first] = 0;
  size_t actual4 = C_given_D.argmax(given);
  EXPECT_LONGS_EQUAL(1, actual4);
}

/* ************************************************************************* */
// Multiplication preserves normalization after pruning a table-backed prior.
TEST(DiscreteConditional, PrunedTableConditionalProduct) {
  const DiscreteKey A(0, 2), C(2, 2);
  const DiscreteConditional prior(1, TableFactor(A, "0.4 0.6"));
  const auto pruned =
      std::dynamic_pointer_cast<DiscreteConditional>(prior.prune(1));
  CHECK(pruned);
  EXPECT(std::dynamic_pointer_cast<TableFactor>(pruned->factor()));

  const DiscreteConditional conditional(C, {A}, "1/1 1/1");  // P(C|A)
  const DiscreteConditional joint = conditional * *pruned;

  EXPECT_DOUBLES_EQUAL(0.0, joint(DiscreteValues{{A.first, 0}, {C.first, 0}}),
                       1e-12);
  EXPECT_DOUBLES_EQUAL(0.0, joint(DiscreteValues{{A.first, 0}, {C.first, 1}}),
                       1e-12);
  EXPECT_DOUBLES_EQUAL(0.5, joint(DiscreteValues{{A.first, 1}, {C.first, 0}}),
                       1e-12);
  EXPECT_DOUBLES_EQUAL(0.5, joint(DiscreteValues{{A.first, 1}, {C.first, 1}}),
                       1e-12);

  double total = 0.0;
  for (const auto& values : joint.allAssignments()) total += joint(values);
  EXPECT_DOUBLES_EQUAL(1.0, total, 1e-12);
}

/* ************************************************************************* */
// Mode removal updates table storage and metadata without renormalizing.
TEST(DiscreteConditional, RemoveDiscreteModesUpdatesTable) {
  const DiscreteKey A(0, 2), C(2, 2);
  DiscreteConditional conditional(2, TableFactor(A & C, "1 2 3 4"));
  conditional.removeDiscreteModes(DiscreteValues{{A.first, 1}});

  EXPECT_LONGS_EQUAL(1, conditional.size());
  EXPECT_LONGS_EQUAL(1, conditional.nrFrontals());
  EXPECT_LONGS_EQUAL(0, conditional.nrParents());
  EXPECT_LONGS_EQUAL(C.first, conditional.firstFrontalKey());
  EXPECT(std::dynamic_pointer_cast<TableFactor>(conditional.factor()));
  EXPECT(conditional.factor()->keys() == conditional.keys());

  // Keep the A=1 slice of the normalized joint, with its original mass.
  EXPECT_DOUBLES_EQUAL(0.3, conditional(DiscreteValues{{C.first, 0}}), 1e-12);
  EXPECT_DOUBLES_EQUAL(0.4, conditional(DiscreteValues{{C.first, 1}}), 1e-12);

  const DecisionTreeFactor expected(C, "0.3 0.4");
  const DecisionTreeFactor converted = conditional.toDecisionTreeFactor();
  EXPECT_LONGS_EQUAL(1, converted.size());
  EXPECT(assert_equal(expected, converted));
}

/* ************************************************************************* */
// Check markdown representation looks as expected, no parents.
TEST(DiscreteConditional, markdown_prior) {
  DiscreteKey A(Symbol('x', 1), 3);
  DiscreteConditional conditional(A % "1/2/2");
  string expected =
      " *P(x1):*\n\n"
      "|x1|value|\n"
      "|:-:|:-:|\n"
      "|0|0.2|\n"
      "|1|0.4|\n"
      "|2|0.4|\n";
  string actual = conditional.markdown();
  EXPECT(actual == expected);
}

/* ************************************************************************* */
// Check markdown representation looks as expected, no parents + names.
TEST(DiscreteConditional, markdown_prior_names) {
  Symbol x1('x', 1);
  DiscreteKey A(x1, 3);
  DiscreteConditional conditional(A % "1/2/2");
  string expected =
      " *P(x1):*\n\n"
      "|x1|value|\n"
      "|:-:|:-:|\n"
      "|A0|0.2|\n"
      "|A1|0.4|\n"
      "|A2|0.4|\n";
  DecisionTreeFactor::Names names{{x1, {"A0", "A1", "A2"}}};
  string actual = conditional.markdown(DefaultKeyFormatter, names);
  EXPECT(actual == expected);
}

/* ************************************************************************* */
// Check markdown representation looks as expected, multivalued.
TEST(DiscreteConditional, markdown_multivalued) {
  DiscreteKey A(Symbol('a', 1), 3), B(Symbol('b', 1), 5);
  DiscreteConditional conditional(
      A | B = "2/88/10 2/20/78 33/33/34 33/33/34 95/2/3");
  string expected =
      " *P(a1|b1):*\n\n"
      "|*b1*|0|1|2|\n"
      "|:-:|:-:|:-:|:-:|\n"
      "|0|0.02|0.88|0.1|\n"
      "|1|0.02|0.2|0.78|\n"
      "|2|0.33|0.33|0.34|\n"
      "|3|0.33|0.33|0.34|\n"
      "|4|0.95|0.02|0.03|\n";
  string actual = conditional.markdown();
  EXPECT(actual == expected);
}

/* ************************************************************************* */
// Check markdown representation looks as expected, two parents + names.
TEST(DiscreteConditional, markdown) {
  DiscreteKey A(2, 2), B(1, 2), C(0, 3);
  DiscreteConditional conditional(A, {B, C}, "0/1 1/3  1/1 3/1  0/1 1/0");
  string expected =
      " *P(A|B,C):*\n\n"
      "|*B*|*C*|T|F|\n"
      "|:-:|:-:|:-:|:-:|\n"
      "|-|Zero|0|1|\n"
      "|-|One|0.25|0.75|\n"
      "|-|Two|0.5|0.5|\n"
      "|+|Zero|0.75|0.25|\n"
      "|+|One|0|1|\n"
      "|+|Two|1|0|\n";
  vector<string> keyNames{"C", "B", "A"};
  auto formatter = [keyNames](Key key) { return keyNames[key]; };
  DecisionTreeFactor::Names names{
      {0, {"Zero", "One", "Two"}}, {1, {"-", "+"}}, {2, {"T", "F"}}};
  string actual = conditional.markdown(formatter, names);
  EXPECT(actual == expected);
}

/* ************************************************************************* */
// Check html representation looks as expected, two parents + names.
TEST(DiscreteConditional, html) {
  DiscreteKey A(2, 2), B(1, 2), C(0, 3);
  DiscreteConditional conditional(A, {B, C}, "0/1 1/3  1/1 3/1  0/1 1/0");
  string expected =
      "<div>\n"
      "<p>  <i>P(A|B,C):</i></p>\n"
      "<table class='DiscreteConditional'>\n"
      "  <thead>\n"
      "    <tr><th><i>B</i></th><th><i>C</i></th><th>T</th><th>F</th></tr>\n"
      "  </thead>\n"
      "  <tbody>\n"
      "    <tr><th>-</th><th>Zero</th><td>0</td><td>1</td></tr>\n"
      "    <tr><th>-</th><th>One</th><td>0.25</td><td>0.75</td></tr>\n"
      "    <tr><th>-</th><th>Two</th><td>0.5</td><td>0.5</td></tr>\n"
      "    <tr><th>+</th><th>Zero</th><td>0.75</td><td>0.25</td></tr>\n"
      "    <tr><th>+</th><th>One</th><td>0</td><td>1</td></tr>\n"
      "    <tr><th>+</th><th>Two</th><td>1</td><td>0</td></tr>\n"
      "  </tbody>\n"
      "</table>\n"
      "</div>";
  vector<string> keyNames{"C", "B", "A"};
  auto formatter = [keyNames](Key key) { return keyNames[key]; };
  DecisionTreeFactor::Names names{
      {0, {"Zero", "One", "Two"}}, {1, {"-", "+"}}, {2, {"T", "F"}}};
  string actual = conditional.html(formatter, names);
  EXPECT(actual == expected);
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
