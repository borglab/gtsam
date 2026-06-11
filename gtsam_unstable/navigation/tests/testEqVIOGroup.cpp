/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file testEqVIOGroup.cpp
 * @brief Unit tests for EqVIOGroup.
 * @author Rohan Bansal
 */

#include <CppUnitLite/TestHarness.h>
#include <gtsam/base/TestableAssertions.h>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam_unstable/navigation/EqVIOCommon.h>

#include <cmath>
#include <vector>

using namespace gtsam;
using namespace gtsam::eqvio;

namespace {

template <int N, typename G>
void testLieGroupDerivativesN(TestResult& result_, const std::string& name_,
                              const G& t1, const G& t2) {
  Matrix H1, H2;
  using T = traits<G>;
  using OJ = OptionalJacobian<T::dimension, T::dimension>;

  OJ none;
  EXPECT(assert_equal<G>(t1.inverse(), T::Inverse(t1, H1)));
  EXPECT(assert_equal(numericalDerivative21<G, G, OJ, N>(T::Inverse, t1, none),
                      H1));

  EXPECT(assert_equal<G>(t2.inverse(), T::Inverse(t2, H1)));
  EXPECT(assert_equal(numericalDerivative21<G, G, OJ, N>(T::Inverse, t2, none),
                      H1));

  EXPECT(assert_equal<G>(t1 * t2, T::Compose(t1, t2, H1, H2)));
  EXPECT(assert_equal(
      numericalDerivative41<G, G, G, OJ, OJ, N>(T::Compose, t1, t2, none, none),
      H1));
  EXPECT(assert_equal(
      numericalDerivative42<G, G, G, OJ, OJ, N>(T::Compose, t1, t2, none, none),
      H2));

  EXPECT(assert_equal<G>(t1.inverse() * t2, T::Between(t1, t2, H1, H2)));
  EXPECT(assert_equal(
      numericalDerivative41<G, G, G, OJ, OJ, N>(T::Between, t1, t2, none, none),
      H1));
  EXPECT(assert_equal(
      numericalDerivative42<G, G, G, OJ, OJ, N>(T::Between, t1, t2, none, none),
      H2));
}

template <int N, typename G>
void testChartDerivativesN(TestResult& result_, const std::string& name_,
                           const G& t1, const G& t2) {
  Matrix H1, H2;
  using T = traits<G>;
  using V = typename T::TangentVector;
  using OJ = OptionalJacobian<T::dimension, T::dimension>;

  OJ none;
  const V w12 = T::Local(t1, t2);
  EXPECT(assert_equal<G>(t2, T::Retract(t1, w12, H1, H2)));
  EXPECT(assert_equal(numericalDerivative41<G, G, V, OJ, OJ, N>(
                          T::Retract, t1, w12, none, none),
                      H1));
  EXPECT(assert_equal(numericalDerivative42<G, G, V, OJ, OJ, N>(
                          T::Retract, t1, w12, none, none),
                      H2));

  EXPECT(assert_equal(w12, T::Local(t1, t2, H1, H2)));
  EXPECT(assert_equal(
      numericalDerivative41<V, G, G, OJ, OJ, N>(T::Local, t1, t2, none, none),
      H1));
  EXPECT(assert_equal(
      numericalDerivative42<V, G, G, OJ, OJ, N>(T::Local, t1, t2, none, none),
      H2));
}

using SE23 = Se23;
using LandmarkGroup = gtsam::eqvio::LandmarkGroup;

const Rot3 kR1 = Rot3::RzRyRx(0.1, -0.2, 0.3);
const Rot3 kR2 = Rot3::RzRyRx(-0.3, 0.2, -0.1);
const Rot3 kR3 = Rot3::RzRyRx(0.2, 0.1, -0.25);
const Rot3 kR4 = Rot3::RzRyRx(-0.15, -0.1, 0.05);

const SE23::Matrix3K kX1 =
    (SE23::Matrix3K() << 1.0, -0.4, 0.3, 0.9, -0.8, 0.2).finished();
const SE23::Matrix3K kX2 =
    (SE23::Matrix3K() << -0.5, 0.7, 0.1, -0.2, 0.4, 1.1).finished();

const SE23 kA1(kR1, kX1);
const SE23 kA2(kR2, kX2);

const Bias kBeta1(Vector3(-0.4, 0.2, 0.05), Vector3(0.1, -0.2, 0.3));
const Bias kBeta2(Vector3(0.2, -0.15, 0.07), Vector3(-0.05, 0.3, -0.1));

const Pose3 kB1(kR3, Point3(0.2, -0.5, 1.1));
const Pose3 kB2(kR4, Point3(-0.6, 0.7, 0.3));

const SOT3 kQ1(SO3::Expmap((Vector3() << 0.08, -0.04, 0.05).finished()),
               std::log(1.2));
const SOT3 kQ2(SO3::Expmap((Vector3() << -0.03, 0.06, -0.02).finished()),
               std::log(0.95));
const SOT3 kQ3(SO3::Expmap((Vector3() << 0.04, 0.07, -0.08).finished()),
               std::log(1.1));
const SOT3 kQ4(SO3::Expmap((Vector3() << -0.06, -0.02, 0.09).finished()),
               std::log(1.05));
const SOT3 kQ5(SO3::Expmap((Vector3() << 0.01, 0.02, 0.03).finished()),
               std::log(0.98));
const SOT3 kQ6(SO3::Expmap((Vector3() << -0.02, 0.03, -0.01).finished()),
               std::log(1.03));

LandmarkGroup MakeQ0() { return LandmarkGroup(0); }
LandmarkGroup MakeQ1A() { return LandmarkGroup({kQ1}); }
LandmarkGroup MakeQ1B() { return LandmarkGroup({kQ2}); }
LandmarkGroup MakeQ3A() { return LandmarkGroup({kQ1, kQ2, kQ3}); }
LandmarkGroup MakeQ3B() { return LandmarkGroup({kQ4, kQ5, kQ6}); }

VioGroup MakeG0() { return makeVioGroup(kA1, kBeta1, kB1, MakeQ0()); }
VioGroup MakeG0b() { return makeVioGroup(kA2, kBeta2, kB2, MakeQ0()); }
VioGroup MakeG1() { return makeVioGroup(kA1, kBeta1, kB1, MakeQ1A()); }
VioGroup MakeG1b() { return makeVioGroup(kA2, kBeta2, kB2, MakeQ1B()); }
VioGroup MakeG3() { return makeVioGroup(kA1, kBeta1, kB1, MakeQ3A()); }
VioGroup MakeG3b() { return makeVioGroup(kA2, kBeta2, kB2, MakeQ3B()); }

Vector Xi0() {
  return (Vector(21) << 0.05, -0.04, 0.03, 0.2, -0.1, 0.15, -0.05, 0.07, -0.09,
          0.1, -0.08, 0.06, -0.04, 0.02, 0.03, 0.01, -0.02, 0.04, -0.03, 0.05,
          -0.01)
      .finished();
}

Vector Xi1() {
  Vector xi(25);
  xi << Xi0(), 0.03, -0.02, 0.01, 0.04;
  return xi;
}

Vector Xi3() {
  Vector xi(33);
  xi << Xi0(), 0.03, -0.02, 0.01, 0.04, -0.01, 0.05, -0.03, 0.02, 0.02, 0.01,
      -0.04, 0.03;
  return xi;
}

}  // namespace

// Verifies VIOGroup satisfies the required concept checks.
TEST(VIOGroup, Concept) {
  GTSAM_CONCEPT_ASSERT(IsGroup<VioGroup>);
  GTSAM_CONCEPT_ASSERT(IsManifold<VioGroup>);
  GTSAM_CONCEPT_ASSERT(IsLieGroup<VioGroup>);
  GTSAM_CONCEPT_ASSERT(IsTestable<VioGroup>);
}

// Verifies identity sizing and factor accessors.
TEST(VIOGroup, ConstructorsAndAccessors) {
  const VioGroup empty = makeVioGroupIdentity();
  EXPECT_LONGS_EQUAL(0, N_landmarkCount(empty));
  EXPECT_LONGS_EQUAL(21, Dim_groupTangent(empty));

  const VioGroup identity3 = makeVioGroupIdentity(3);
  EXPECT_LONGS_EQUAL(3, N_landmarkCount(identity3));
  EXPECT_LONGS_EQUAL(33, Dim_groupTangent(identity3));

  const VioGroup g = MakeG3();
  EXPECT(assert_equal(kA1, A_sensorKinematics(g)));
  EXPECT(assert_equal(kBeta1.vector(), Beta_biasOffset(g).vector()));
  EXPECT(assert_equal(kB1, B_cameraExtrinsics(g)));
  EXPECT(assert_equal(MakeQ3A(), Q_landmarkTransforms(g)));
}

// Verifies compose, between, and inverse behavior.
TEST(VIOGroup, GroupOperations) {
  const VioGroup g1 = MakeG3();
  const VioGroup g2 = MakeG3b();
  const VioGroup composed = g1.compose(g2);

  EXPECT(assert_equal(A_sensorKinematics(g1).compose(A_sensorKinematics(g2)),
                      A_sensorKinematics(composed)));
  EXPECT(assert_equal((Beta_biasOffset(g1) + Beta_biasOffset(g2)).vector(),
                      Beta_biasOffset(composed).vector()));
  EXPECT(assert_equal(B_cameraExtrinsics(g1).compose(B_cameraExtrinsics(g2)),
                      B_cameraExtrinsics(composed)));
  EXPECT(
      assert_equal(Q_landmarkTransforms(g1).compose(Q_landmarkTransforms(g2)),
                   Q_landmarkTransforms(composed)));

  const VioGroup between = g1.between(g2);
  EXPECT(assert_equal(g1.inverse() * g2, between));
  EXPECT(assert_equal(makeVioGroupIdentity(3), g1.compose(g1.inverse())));
}

// Verifies Expmap/Logmap round-trip and adjoint consistency.
TEST(VIOGroup, ExpmapLogmapAndAdjoint) {
  const Vector xi0 = Xi0();
  const Vector xi1 = Xi1();
  const Vector xi3 = Xi3();

  const VioGroup g0 = VioGroup::Expmap(xi0);
  const VioGroup g1 = VioGroup::Expmap(xi1);
  const VioGroup g3 = VioGroup::Expmap(xi3);

  EXPECT(assert_equal(xi0, VioGroup::Logmap(g0), 1e-9));
  EXPECT(assert_equal(xi1, VioGroup::Logmap(g1), 1e-9));
  EXPECT(assert_equal(xi3, VioGroup::Logmap(g3), 1e-9));

  VioGroup core(SensorCore(A_sensorKinematics(g3), Beta_biasOffset(g3)),
                LandmarkCore(B_cameraExtrinsics(g3), Q_landmarkTransforms(g3)));
  EXPECT(assert_equal(core.AdjointMap(), g3.AdjointMap(), 1e-9));
}

// Verifies Lie and chart derivatives for n=0 landmarks.
TEST(VIOGroup, DerivativesN0) {
#if defined(GTSAM_ROT3_EXPMAP) || defined(GTSAM_USE_QUATERNIONS)
  const VioGroup id = makeVioGroupIdentity();
  const VioGroup g = MakeG0();
  const VioGroup h = MakeG0b();

  testLieGroupDerivativesN<21>(result_, name_, id, g);
  testLieGroupDerivativesN<21>(result_, name_, g, h);
  testChartDerivativesN<21>(result_, name_, id, g);
  testChartDerivativesN<21>(result_, name_, g, h);
#else
  EXPECT(true);
#endif
}

// Verifies Lie and chart derivatives for n=1 landmark.
TEST(VIOGroup, DerivativesN1) {
#if defined(GTSAM_ROT3_EXPMAP) || defined(GTSAM_USE_QUATERNIONS)
  const VioGroup id = makeVioGroupIdentity(1);
  const VioGroup g = MakeG1();
  const VioGroup h = MakeG1b();

  testLieGroupDerivativesN<25>(result_, name_, id, g);
  testLieGroupDerivativesN<25>(result_, name_, g, h);
  testChartDerivativesN<25>(result_, name_, id, g);
  testChartDerivativesN<25>(result_, name_, g, h);
#else
  EXPECT(true);
#endif
}

// Verifies Lie and chart derivatives for n=3 landmarks.
TEST(VIOGroup, DerivativesN3) {
#if defined(GTSAM_ROT3_EXPMAP) || defined(GTSAM_USE_QUATERNIONS)
  const VioGroup id = makeVioGroupIdentity(3);
  const VioGroup g = MakeG3();
  const VioGroup h = MakeG3b();

  testLieGroupDerivativesN<33>(result_, name_, id, g);
  testLieGroupDerivativesN<33>(result_, name_, g, h);
  testChartDerivativesN<33>(result_, name_, id, g);
  testChartDerivativesN<33>(result_, name_, g, h);
#else
  EXPECT(true);
#endif
}

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
