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

#include <gtsam/geometry/Similarity2.h>

#include <gtsam/base/Testable.h>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/base/testLie.h>
#include <gtsam/base/Vector.h>

#include <CppUnitLite/TestHarness.h>

#include <functional>
#include <vector>

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
TEST(Similarity2, Compose) {
  // Test group operation: compose two Similarity2 elements.
  Rot2 R1 = Rot2::fromDegrees(30);
  Point2 t1(1, 2);
  double s1 = 2.0;
  Similarity2 S1(R1, t1, s1);

  Rot2 R2 = Rot2::fromDegrees(45);
  Point2 t2(-1, 1);
  double s2 = 4.0;
  Similarity2 S2(R2, t2, s2);

  Similarity2 S3 = S1.compose(S2);

  // Compose manually
  Rot2 expected_R = R1 * R2;
  double expected_s = s1 * s2;
  Point2 expected_t = t1 / s2 + R1.matrix() * t2;
  Similarity2 expected_S3(expected_R, expected_t, expected_s);

  EXPECT(assert_equal(expected_S3, S3));
  EXPECT(assert_equal(expected_S3, S1 * S2));
  EXPECT(assert_equal<Matrix3>(S3.matrix(), S1.matrix() * S2.matrix()));
}

//******************************************************************************
TEST(Similarity2, Inverse) {
  // Test group operation: inverse of a Similarity2 element.
  Rot2 R = Rot2::fromDegrees(60);
  Point2 t(3, -2);
  double s = 4.0;
  Similarity2 S(R, t, s);
  Similarity2 S_inv = S.inverse();

  // Check that S * S_inv is identity
  Similarity2 I_sim = S.compose(S_inv);
  Similarity2 expected_I;
  EXPECT(assert_equal(expected_I, I_sim));
}

//******************************************************************************
TEST(Similarity2, Identity) {
  // Test that the identity Similarity2 acts as expected.
  Similarity2 S_id;
  Rot2 R = Rot2::fromDegrees(10);
  Point2 t(5, 7);
  double s = 2.5;
  Similarity2 S(R, t, s);

  // Compose with identity
  EXPECT(assert_equal(S, S.compose(S_id)));
  EXPECT(assert_equal(S, S_id.compose(S)));
}

//******************************************************************************
TEST(Similarity2, InverseMatrix) {
  Rot2 R = Rot2::fromDegrees(60);
  Point2 t(3, -2);
  double s = 4.0;
  Similarity2 S(R, t, s);

  Matrix3 S_inv_mat = S.inverse().matrix();
  Matrix3 S_mat_inv = S.matrix().inverse();

  EXPECT(assert_equal(S_inv_mat, S_mat_inv));
}

//******************************************************************************
TEST(Similarity2, TransformFrom_Point2) {
  // Setup
  Rot2 R = Rot2::fromAngle(M_PI / 4); // 45 degrees
  Point2 t(1.0, 2.0);
  double s = 3.0;
  Similarity2 sim(R, t, s);

  Point2 p(2.0, 0.0);

  // Expected: s * (R * p + t)
  Point2 expected = s * (R * p + t);

  Point2 actual = sim.transformFrom(p);

  EXPECT(assert_equal(expected, actual, 1e-9));
}

//******************************************************************************
TEST(Similarity2, TransformFrom_Pose2) {
  // Setup
  Rot2 R_sim = Rot2::fromAngle(M_PI / 6); // 30 degrees
  Point2 t_sim(1.0, -1.0);
  double s_sim = 2.0;
  Similarity2 sim(R_sim, t_sim, s_sim);

  Rot2 R_pose = Rot2::fromAngle(-M_PI / 4); // -45 degrees
  Point2 t_pose(3.0, 4.0);
  Pose2 pose(R_pose, t_pose);

  Rot2 expected_R = R_sim * R_pose;
  Point2 expected_t = s_sim * (R_sim * t_pose + t_sim);
  Pose2 expected(expected_R, expected_t);

  Pose2 actual = sim.transformFrom(pose);

  EXPECT(assert_equal(expected, actual, 1e-9));
}

//******************************************************************************
TEST(Similarity2, Vec) {
  const double theta = 0.3;
  const Rot2 R_test = Rot2::fromAngle(theta);
  const Point2 t_test(0.2, 0.7);
  const double s_test = 4.0;
  const Similarity2 sim(R_test, t_test, s_test);

  // 1. Test the Value
  Vector9 expected_vec;
  const double c = cos(theta), si = sin(theta);
  expected_vec << c, si, 0,               // First column
    -si, c, 0,                            // Second column
    t_test.x(), t_test.y(), 1.0 / s_test; // Third column
  Vector9 actual_vec = sim.vec();
  EXPECT(assert_equal(expected_vec, actual_vec, 1e-9));

  // 2. Test the Jacobian
  Matrix94 H_actual;
  sim.vec(H_actual);
  auto vec_fun = [](const Similarity2& sim_arg) -> Vector9 {
    return sim_arg.vec();
    };
  Matrix H_numerical = numericalDerivative11<Vector9, Similarity2, 4>(vec_fun, sim);
  EXPECT(assert_equal(H_numerical, H_actual, 1e-7));
}

//******************************************************************************
TEST(Similarity2, AdjointMap) {
  // Create a non-trivial Similarity2 object
  const Rot2 R = Rot2::fromAngle(-0.7);
  const Point2 t(1.5, -2.3);
  const double s = 0.5;
  const Similarity2 sim(R, t, s);

  // Call the specialized AdjointMap
  Matrix4 specialized_Adj = sim.AdjointMap();

  // Call the generic AdjointMap from the base class
  Matrix4 generic_Adj = static_cast<const MatrixLieGroup<Similarity2, 4, 3>*>(&sim)->AdjointMap();

  // Assert that they are equal
  EXPECT(assert_equal(specialized_Adj, generic_Adj, 1e-9));
}

//******************************************************************************
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
//******************************************************************************
