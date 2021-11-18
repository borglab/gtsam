/*
 * testCSP.cpp
 * @brief develop code for CSP solver
 * @date Feb 5, 2012
 * @author Frank Dellaert
 */

#include <gtsam_unstable/discrete/CSP.h>
#include <gtsam_unstable/discrete/Domain.h>

#include <boost/assign/std/map.hpp>
using boost::assign::insert;
#include <CppUnitLite/TestHarness.h>

#include <fstream>
#include <iostream>

using namespace std;
using namespace gtsam;

/* ************************************************************************* */
TEST_UNSAFE(BinaryAllDif, allInOne) {
  // Create keys and ordering
  size_t nrColors = 2;
  //  DiscreteKey ID("Idaho", nrColors), UT("Utah", nrColors), AZ("Arizona",
  //  nrColors);
  DiscreteKey ID(0, nrColors), UT(2, nrColors), AZ(1, nrColors);

  // Check construction and conversion
  BinaryAllDiff c1(ID, UT);
  DecisionTreeFactor f1(ID & UT, "0 1 1 0");
  EXPECT(assert_equal(f1, c1.toDecisionTreeFactor()));

  // Check construction and conversion
  BinaryAllDiff c2(UT, AZ);
  DecisionTreeFactor f2(UT & AZ, "0 1 1 0");
  EXPECT(assert_equal(f2, c2.toDecisionTreeFactor()));

  DecisionTreeFactor f3 = f1 * f2;
  EXPECT(assert_equal(f3, c1 * f2));
  EXPECT(assert_equal(f3, c2 * f1));
}

/* ************************************************************************* */
TEST_UNSAFE(CSP, allInOne) {
  // Create keys and ordering
  size_t nrColors = 2;
  DiscreteKey ID(0, nrColors), UT(2, nrColors), AZ(1, nrColors);

  // Create the CSP
  CSP csp;
  csp.addAllDiff(ID, UT);
  csp.addAllDiff(UT, AZ);

  // Check an invalid combination, with ID==UT==AZ all same color
  DiscreteFactor::Values invalid;
  invalid[ID.first] = 0;
  invalid[UT.first] = 0;
  invalid[AZ.first] = 0;
  EXPECT_DOUBLES_EQUAL(0, csp(invalid), 1e-9);

  // Check a valid combination
  DiscreteFactor::Values valid;
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
  CSP::sharedValues mpe = csp.optimalAssignment();
  CSP::Values expected;
  insert(expected)(ID.first, 1)(UT.first, 0)(AZ.first, 1);
  EXPECT(assert_equal(expected, *mpe));
  EXPECT_DOUBLES_EQUAL(1, csp(*mpe), 1e-9);
}

/* ************************************************************************* */
TEST_UNSAFE(CSP, WesternUS) {
  // Create keys
  size_t nrColors = 4;
  DiscreteKey
      // Create ordering according to example in ND-CSP.lyx
      WA(0, nrColors),
      OR(3, nrColors), CA(1, nrColors), NV(2, nrColors), ID(8, nrColors),
      UT(9, nrColors), AZ(10, nrColors), MT(4, nrColors), WY(5, nrColors),
      CO(7, nrColors), NM(6, nrColors);

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

  // Solve
  Ordering ordering;
  ordering += Key(0), Key(1), Key(2), Key(3), Key(4), Key(5), Key(6), Key(7),
      Key(8), Key(9), Key(10);
  CSP::sharedValues mpe = csp.optimalAssignment(ordering);
  // GTSAM_PRINT(*mpe);
  CSP::Values expected;
  insert(expected)(WA.first, 1)(CA.first, 1)(NV.first, 3)(OR.first, 0)(
      MT.first, 1)(WY.first, 0)(NM.first, 3)(CO.first, 2)(ID.first, 2)(
      UT.first, 1)(AZ.first, 0);

  // TODO: Fix me! mpe result seems to be right. (See the printing)
  // It has the same prob as the expected solution.
  // Is mpe another solution, or the expected solution is unique???
  EXPECT(assert_equal(expected, *mpe));
  EXPECT_DOUBLES_EQUAL(1, csp(*mpe), 1e-9);

  // Write out the dual graph for hmetis
#ifdef DUAL
  VariableIndexOrdered index(csp);
  index.print("index");
  ofstream os("/Users/dellaert/src/hmetis-1.5-osx-i686/US-West-dual.txt");
  index.outputMetisFormat(os);
#endif
}

/* ************************************************************************* */
TEST_UNSAFE(CSP, AllDiff) {
  // Create keys and ordering
  size_t nrColors = 3;
  DiscreteKey ID(0, nrColors), UT(2, nrColors), AZ(1, nrColors);

  // Create the CSP
  CSP csp;
  vector<DiscreteKey> dkeys;
  dkeys += ID, UT, AZ;
  csp.addAllDiff(dkeys);
  csp.addSingleValue(AZ, 2);
  //  GTSAM_PRINT(csp);

  // Check construction and conversion
  SingleValue s(AZ, 2);
  DecisionTreeFactor f1(AZ, "0 0 1");
  EXPECT(assert_equal(f1, s.toDecisionTreeFactor()));

  // Check construction and conversion
  AllDiff alldiff(dkeys);
  DecisionTreeFactor actual = alldiff.toDecisionTreeFactor();
  //  GTSAM_PRINT(actual);
  //  actual.dot("actual");
  DecisionTreeFactor f2(
      ID & AZ & UT,
      "0 0 0  0 0 1  0 1 0   0 0 1  0 0 0  1 0 0   0 1 0  1 0 0  0 0 0");
  EXPECT(assert_equal(f2, actual));

  // Check an invalid combination, with ID==UT==AZ all same color
  DiscreteFactor::Values invalid;
  invalid[ID.first] = 0;
  invalid[UT.first] = 1;
  invalid[AZ.first] = 0;
  EXPECT_DOUBLES_EQUAL(0, csp(invalid), 1e-9);

  // Check a valid combination
  DiscreteFactor::Values valid;
  valid[ID.first] = 0;
  valid[UT.first] = 1;
  valid[AZ.first] = 2;
  EXPECT_DOUBLES_EQUAL(1, csp(valid), 1e-9);

  // Solve
  CSP::sharedValues mpe = csp.optimalAssignment();
  CSP::Values expected;
  insert(expected)(ID.first, 1)(UT.first, 0)(AZ.first, 2);
  EXPECT(assert_equal(expected, *mpe));
  EXPECT_DOUBLES_EQUAL(1, csp(*mpe), 1e-9);

  // Arc-consistency
  vector<Domain> domains;
  domains += Domain(ID), Domain(AZ), Domain(UT);
  SingleValue singleValue(AZ, 2);
  EXPECT(singleValue.ensureArcConsistency(1, domains));
  EXPECT(alldiff.ensureArcConsistency(0, domains));
  EXPECT(!alldiff.ensureArcConsistency(1, domains));
  EXPECT(alldiff.ensureArcConsistency(2, domains));
  LONGS_EQUAL(2, domains[0].nrValues());
  LONGS_EQUAL(1, domains[1].nrValues());
  LONGS_EQUAL(2, domains[2].nrValues());

  // Parial application, version 1
  DiscreteFactor::Values known;
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
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
