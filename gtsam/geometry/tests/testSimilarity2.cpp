/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file   testSimilarity2.cpp
 * @brief  Unit tests for Similarity2 class
 * @author Varun Agrawal
 */

#include <CppUnitLite/TestHarness.h>
#include <gtsam/base/Testable.h>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/base/testLie.h>
#include <gtsam/geometry/Similarity2.h>

#include <functional>

using namespace std::placeholders;
using namespace gtsam;
using namespace std;

GTSAM_CONCEPT_TESTABLE_INST(Similarity2)

static const Point2 P(0.2, 0.7);
static const Rot2 R = Rot2::fromAngle(0.3);
static const double s = 4;

//******************************************************************************
TEST(Similarity2, Concepts) {
  GTSAM_CONCEPT_ASSERT(IsGroup<Similarity2>);
  GTSAM_CONCEPT_ASSERT(IsManifold<Similarity2>);
  GTSAM_CONCEPT_ASSERT(IsMatrixLieGroup<Similarity2>);
}

//******************************************************************************
TEST(Similarity2, Constructors) {
  Similarity2 sim2_Construct1;
  Similarity2 sim2_Construct2(s);
  Similarity2 sim2_Construct3(R, P, s);
  Similarity2 sim2_Construct4(R.matrix(), P, s);
}

//******************************************************************************
TEST(Similarity2, Getters) {
  Similarity2 sim2_default;
  EXPECT(assert_equal(Rot2(), sim2_default.rotation()));
  EXPECT(assert_equal(Point2(0, 0), sim2_default.translation()));
  EXPECT_DOUBLES_EQUAL(1.0, sim2_default.scale(), 1e-9);
}

/* ************************************************************************* */
TEST(Similarity2, HatAndVee) {
  // Create a few test vectors
  Vector4 v1(1, 2, 3, 4);
  Vector4 v2(0.1, -0.5, 1.0, -1.0);
  Vector4 v3(0.0, 0.0, 0.0, 0.0);

  // Test that Vee(Hat(v)) == v for various inputs
  EXPECT(assert_equal(v1, Similarity2::Vee(Similarity2::Hat(v1))));
  EXPECT(assert_equal(v2, Similarity2::Vee(Similarity2::Hat(v2))));
  EXPECT(assert_equal(v3, Similarity2::Vee(Similarity2::Hat(v3))));

  // Check the structure of the Lie Algebra element
  Matrix3 expected;
  expected << 0, -3, 1,
              3, 0, 2,
              0, 0, -4;

  EXPECT(assert_equal(expected, Similarity2::Hat(v1)));
}

/* ************************************************************************* */
// Checks correct exponential map (Expmap) with brute force matrix exponential
TEST(Similarity2, BruteForceExpmap) {
  const Vector4 xi(0.1, 0.2, 0.3, 0.4);
  EXPECT(assert_equal(Similarity2::Expmap(xi), expm<Similarity2>(xi), 1e-4));
}

//******************************************************************************
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
//******************************************************************************
