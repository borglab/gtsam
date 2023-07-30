/*
 * testCSP.cpp
 * @brief develop code for CSP solver
 * @date Feb 5, 2012
 * @author Frank Dellaert
 */

#include <gtsam_unstable/discrete/CSP.h>
#include <gtsam_unstable/discrete/Domain.h>

#include <CppUnitLite/TestHarness.h>

#include <fstream>
#include <iostream>

using namespace std;
using namespace gtsam;

/* ************************************************************************* */
TEST(CSP, SingleValue) {
  // Create keys for Idaho, Arizona, and Utah, allowing two colors for each:
  size_t nrColors = 3;
  DiscreteKey ID(0, nrColors), AZ(1, nrColors), UT(2, nrColors);

  // Check that a single value is equal to a decision stump with only one "1":
  SingleValue singleValue(AZ, 2);
  DecisionTreeFactor f1(AZ, "0 0 1");
  EXPECT(assert_equal(f1, singleValue.toDecisionTreeFactor()));

  // Create domains
  Domains domains;
  domains.emplace(0, Domain(ID));
  domains.emplace(1, Domain(AZ));
  domains.emplace(2, Domain(UT));

  // Ensure arc-consistency: just wipes out values in AZ domain:
  EXPECT(singleValue.ensureArcConsistency(1, &domains));
  LONGS_EQUAL(3, domains.at(0).nrValues());
  LONGS_EQUAL(1, domains.at(1).nrValues());
  LONGS_EQUAL(3, domains.at(2).nrValues());
}

/* ************************************************************************* */
TEST(CSP, BinaryAllDif) {
  // Create keys for Idaho, Arizona, and Utah, allowing 2 colors for each:
  size_t nrColors = 2;
  DiscreteKey ID(0, nrColors), AZ(1, nrColors), UT(2, nrColors);

  // Check construction and conversion
  BinaryAllDiff c1(ID, UT);
  DecisionTreeFactor f1(ID & UT, "0 1 1 0");
  EXPECT(assert_equal(f1, c1.toDecisionTreeFactor()));

  // Check construction and conversion
  BinaryAllDiff c2(UT, AZ);
  DecisionTreeFactor f2(UT & AZ, "0 1 1 0");
  EXPECT(assert_equal(f2, c2.toDecisionTreeFactor()));

  // Check multiplication of factors with constraint:
  DecisionTreeFactor f3 = f1 * f2;
  EXPECT(assert_equal(f3, c1 * f2));
  EXPECT(assert_equal(f3, c2 * f1));
}

/* ************************************************************************* */
TEST(CSP, AllDiff) {
  // Create keys for Idaho, Arizona, and Utah, allowing two colors for each:
  size_t nrColors = 3;
  DiscreteKey ID(0, nrColors), AZ(1, nrColors), UT(2, nrColors);

  // Check construction and conversion
  vector<DiscreteKey> dkeys{ID, UT, AZ};
  AllDiff alldiff(dkeys);
  DecisionTreeFactor actual = alldiff.toDecisionTreeFactor();
  // GTSAM_PRINT(actual);
  actual.dot("actual");
  DecisionTreeFactor f2(
      ID & AZ & UT,
      "0 0 0  0 0 1  0 1 0   0 0 1  0 0 0  1 0 0   0 1 0  1 0 0  0 0 0");
  EXPECT(assert_equal(f2, actual));

  // Create domains.
  Domains domains;
  domains.emplace(0, Domain(ID));
  domains.emplace(1, Domain(AZ));
  domains.emplace(2, Domain(UT));

  // First constrict AZ domain:
  SingleValue singleValue(AZ, 2);
  EXPECT(singleValue.ensureArcConsistency(1, &domains));

  // Arc-consistency
  EXPECT(alldiff.ensureArcConsistency(0, &domains));
  EXPECT(!alldiff.ensureArcConsistency(1, &domains));
  EXPECT(alldiff.ensureArcConsistency(2, &domains));
  LONGS_EQUAL(2, domains.at(0).nrValues());
  LONGS_EQUAL(1, domains.at(1).nrValues());
  LONGS_EQUAL(2, domains.at(2).nrValues());
}

/* ************************************************************************* */
TEST(CSP, allInOne) {
  // Create keys for Idaho, Arizona, and Utah, allowing 3 colors for each:
  size_t nrColors = 2;
  DiscreteKey ID(0, nrColors), AZ(1, nrColors), UT(2, nrColors);

  // Create the CSP
  CSP csp;
  csp.addAllDiff(ID, UT);
  csp.addAllDiff(UT, AZ);

  // Check an invalid combination, with ID==UT==AZ all same color
  DiscreteValues invalid;
  invalid[ID.first] = 0;
  invalid[UT.first] = 0;
  invalid[AZ.first] = 0;
  EXPECT_DOUBLES_EQUAL(0, csp(invalid), 1e-9);

  // Check a valid combination
  DiscreteValues valid;
  valid[ID.first] = 0;
  valid[UT.first] = 1;
  valid[AZ.first] = 0;
  EXPECT_DOUBLES_EQUAL(1, csp(valid), 1e-9);

  // Just for fun, create the product and check it
  DecisionTreeFactor product = csp.product();
  // product.dot("product");
  DecisionTreeFactor expectedProduct(ID & AZ & UT, "0 1 0 0 0 0 1 0");
  EXPECT(assert_equal(expectedProduct, product));

  // Solve
  auto mpe = csp.optimize();
  DiscreteValues expected {{ID.first, 1}, {UT.first, 0}, {AZ.first, 1}};
  EXPECT(assert_equal(expected, mpe));
  EXPECT_DOUBLES_EQUAL(1, csp(mpe), 1e-9);
}

/* ************************************************************************* */
TEST(CSP, WesternUS) {
  // Create keys for all states in Western US, with 4 color possibilities.
  size_t nrColors = 4;
  DiscreteKey WA(0, nrColors), OR(3, nrColors), CA(1, nrColors),
      NV(2, nrColors), ID(8, nrColors), UT(9, nrColors), AZ(10, nrColors),
      MT(4, nrColors), WY(5, nrColors), CO(7, nrColors), NM(6, nrColors);

  // Create the CSP
  CSP csp;
  csp.addAllDiff(WA, ID);
  csp.addAllDiff(WA, OR);
  csp.addAllDiff(OR, ID);
  csp.addAllDiff(OR, CA);
  csp.addAllDiff(OR, NV);
  csp.addAllDiff(CA, NV);
  csp.addAllDiff(CA, AZ);
  csp.addAllDiff(ID, MT);
  csp.addAllDiff(ID, WY);
  csp.addAllDiff(ID, UT);
  csp.addAllDiff(ID, NV);
  csp.addAllDiff(NV, UT);
  csp.addAllDiff(NV, AZ);
  csp.addAllDiff(UT, WY);
  csp.addAllDiff(UT, CO);
  csp.addAllDiff(UT, NM);
  csp.addAllDiff(UT, AZ);
  csp.addAllDiff(AZ, CO);
  csp.addAllDiff(AZ, NM);
  csp.addAllDiff(MT, WY);
  csp.addAllDiff(WY, CO);
  csp.addAllDiff(CO, NM);

  DiscreteValues mpe{{0, 2}, {1, 3}, {2, 2}, {3, 1}, {4, 1}, {5, 3},
                     {6, 3}, {7, 2}, {8, 0}, {9, 1}, {10, 0}};

  // Create ordering according to example in ND-CSP.lyx
  const Ordering ordering{0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10};

  // Solve using that ordering:
  auto actualMPE = csp.optimize(ordering);

  EXPECT(assert_equal(mpe, actualMPE));
  EXPECT_DOUBLES_EQUAL(1, csp(mpe), 1e-9);

  // Write out the dual graph for hmetis
#ifdef DUAL
  VariableIndexOrdered index(csp);
  index.print("index");
  ofstream os("/Users/dellaert/src/hmetis-1.5-osx-i686/US-West-dual.txt");
  index.outputMetisFormat(os);
#endif
}

/* ************************************************************************* */
TEST(CSP, ArcConsistency) {
  // Create keys for Idaho, Arizona, and Utah, allowing three colors for each:
  size_t nrColors = 3;
  DiscreteKey ID(0, nrColors), AZ(1, nrColors), UT(2, nrColors);

  // Create the CSP using just one all-diff constraint, plus constrain Arizona.
  CSP csp;
  vector<DiscreteKey> dkeys{ID, UT, AZ};
  csp.addAllDiff(dkeys);
  csp.addSingleValue(AZ, 2);
  // GTSAM_PRINT(csp);

  // Check an invalid combination, with ID==UT==AZ all same color
  DiscreteValues invalid;
  invalid[ID.first] = 0;
  invalid[UT.first] = 1;
  invalid[AZ.first] = 0;
  EXPECT_DOUBLES_EQUAL(0, csp(invalid), 1e-9);

  // Check a valid combination
  DiscreteValues valid;
  valid[ID.first] = 0;
  valid[UT.first] = 1;
  valid[AZ.first] = 2;
  EXPECT_DOUBLES_EQUAL(1, csp(valid), 1e-9);

  // Solve
  auto mpe = csp.optimize();
  DiscreteValues expected {{ID.first, 1}, {UT.first, 0}, {AZ.first, 2}};
  EXPECT(assert_equal(expected, mpe));
  EXPECT_DOUBLES_EQUAL(1, csp(mpe), 1e-9);

  // ensure arc-consistency, i.e., narrow domains...
  Domains domains;
  domains.emplace(0, Domain(ID));
  domains.emplace(1, Domain(AZ));
  domains.emplace(2, Domain(UT));
  
  SingleValue singleValue(AZ, 2);
  AllDiff alldiff(dkeys);
  EXPECT(singleValue.ensureArcConsistency(1, &domains));
  EXPECT(alldiff.ensureArcConsistency(0, &domains));
  EXPECT(!alldiff.ensureArcConsistency(1, &domains));
  EXPECT(alldiff.ensureArcConsistency(2, &domains));
  LONGS_EQUAL(2, domains.at(0).nrValues());
  LONGS_EQUAL(1, domains.at(1).nrValues());
  LONGS_EQUAL(2, domains.at(2).nrValues());

  // Parial application, version 1
  DiscreteValues known;
  known[AZ.first] = 2;
  DiscreteFactor::shared_ptr reduced1 = alldiff.partiallyApply(known);
  DecisionTreeFactor f3(ID & UT, "0 1 1  1 0 1  1 1 0");
  EXPECT(assert_equal(f3, reduced1->toDecisionTreeFactor()));
  DiscreteFactor::shared_ptr reduced2 = singleValue.partiallyApply(known);
  DecisionTreeFactor f4(AZ, "0 0 1");
  EXPECT(assert_equal(f4, reduced2->toDecisionTreeFactor()));

  // Parial application, version 2
  DiscreteFactor::shared_ptr reduced3 = alldiff.partiallyApply(domains);
  EXPECT(assert_equal(f3, reduced3->toDecisionTreeFactor()));
  DiscreteFactor::shared_ptr reduced4 = singleValue.partiallyApply(domains);
  EXPECT(assert_equal(f4, reduced4->toDecisionTreeFactor()));

  // full arc-consistency test
  csp.runArcConsistency(nrColors);
  // GTSAM_PRINT(csp);
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
