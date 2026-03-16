/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file   testEqVIOState.cpp
 * @brief  Unit tests for VIOState manifold
 * @author Rohan Bansal
 */

#include <CppUnitLite/TestHarness.h>
#include <gtsam/base/TestableAssertions.h>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam_unstable/navigation/EqVIOState.h>

#include <stdexcept>
#include <vector>

using namespace gtsam;
using namespace gtsam::eqvio;

namespace {

template <int N>
void testChartDerivativesN(TestResult& result_, const std::string& name_,
                           const VIOState& t1, const VIOState& t2) {
  Matrix H1, H2;
  using T = traits<VIOState>;
  using V = typename T::TangentVector;
  using OJ = OptionalJacobian<T::dimension, T::dimension>;

  OJ none;
  const V w12 = T::Local(t1, t2);
  EXPECT(assert_equal<VIOState>(t2, T::Retract(t1, w12, H1, H2)));
  EXPECT(assert_equal(
      numericalDerivative41<VIOState, VIOState, V, OJ, OJ, N>(T::Retract, t1,
                                                               w12, none, none),
      H1, 1e-5));
  EXPECT(assert_equal(
      numericalDerivative42<VIOState, VIOState, V, OJ, OJ, N>(T::Retract, t1,
                                                               w12, none, none),
      H2, 1e-5));

  EXPECT(assert_equal(w12, T::Local(t1, t2, H1, H2), 1e-9));
  EXPECT(assert_equal(
      numericalDerivative41<V, VIOState, VIOState, OJ, OJ, N>(T::Local, t1, t2,
                                                               none, none),
      H1, 1e-5));
  EXPECT(assert_equal(
      numericalDerivative42<V, VIOState, VIOState, OJ, OJ, N>(T::Local, t1, t2,
                                                               none, none),
      H2, 1e-5));
}

VIOSensorState MakeSensor1() {
  VIOSensorState s;
  s.inputBias = VIOBias(Vector3(0.03, -0.01, 0.02), Vector3(0.1, -0.2, 0.05));
  s.pose = Pose3(Rot3::RzRyRx(0.2, -0.1, 0.15), Point3(0.4, -0.2, 1.0));
  s.velocity = Vector3(0.5, -0.3, 0.2);
  s.cameraOffset =
      Pose3(Rot3::RzRyRx(-0.08, 0.04, -0.03), Point3(0.1, 0.0, 0.05));
  return s;
}

VIOSensorState MakeSensor2() {
  VIOSensorState s;
  s.inputBias =
      VIOBias(Vector3(0.02, 0.05, -0.06), Vector3(-0.04, 0.07, -0.03));
  s.pose = Pose3(Rot3::RzRyRx(-0.1, 0.2, -0.25), Point3(-0.3, 0.5, 0.8));
  s.velocity = Vector3(-0.2, 0.4, -0.1);
  s.cameraOffset =
      Pose3(Rot3::RzRyRx(0.06, -0.03, 0.02), Point3(-0.05, 0.02, 0.09));
  return s;
}

std::vector<Landmark> Lms0() { return {}; }

std::vector<Landmark> Lms1A() { return {{Point3(1.0, -0.5, 4.0), 11}}; }

std::vector<Landmark> Lms1B() { return {{Point3(0.8, -0.3, 3.7), 11}}; }

std::vector<Landmark> Lms3A() {
  return {{Point3(1.0, -0.5, 4.0), 11},
          {Point3(-0.6, 0.4, 3.2), 22},
          {Point3(0.2, 0.7, 5.1), 33}};
}

std::vector<Landmark> Lms3B() {
  return {{Point3(0.9, -0.45, 3.8), 11},
          {Point3(-0.5, 0.35, 3.4), 22},
          {Point3(0.1, 0.65, 4.9), 33}};
}

VIOState MakeState0A() { return VIOState(MakeSensor1(), Lms0()); }
VIOState MakeState0B() { return VIOState(MakeSensor2(), Lms0()); }
VIOState MakeState1A() { return VIOState(MakeSensor1(), Lms1A()); }
VIOState MakeState1B() { return VIOState(MakeSensor2(), Lms1B()); }
VIOState MakeState3A() { return VIOState(MakeSensor1(), Lms3A()); }
VIOState MakeState3B() { return VIOState(MakeSensor2(), Lms3B()); }

}  // namespace

//******************************************************************************
// Verifies VIOState satisfies manifold and testable concept checks.
TEST(VIOState, Concept) {
  GTSAM_CONCEPT_ASSERT(IsManifold<VIOState>);
  GTSAM_CONCEPT_ASSERT(IsTestable<VIOState>);
}

//******************************************************************************
// Verifies dynamic dimensions and landmark id accessors.
TEST(VIOState, DimensionsAndAccessors) {
  const VIOState s0 = MakeState0A();
  const VIOState s1 = MakeState1A();
  const VIOState s3 = MakeState3A();

  EXPECT_LONGS_EQUAL(0, s0.n());
  EXPECT_LONGS_EQUAL(21, s0.dim());

  EXPECT_LONGS_EQUAL(1, s1.n());
  EXPECT_LONGS_EQUAL(24, s1.dim());
  EXPECT((s1.ids() == std::vector<int>{11}));

  EXPECT_LONGS_EQUAL(3, s3.n());
  EXPECT_LONGS_EQUAL(30, s3.dim());
  EXPECT((s3.ids() == std::vector<int>{11, 22, 33}));
}

//******************************************************************************
// Verifies localCoordinates/retract round-trip consistency.
TEST(VIOState, RetractLocalRoundTrip) {
#if defined(GTSAM_ROT3_EXPMAP) || defined(GTSAM_USE_QUATERNIONS)
  const VIOState s3a = MakeState3A();
  const VIOState s3b = MakeState3B();

  const Vector v = s3a.localCoordinates(s3b);
  const VIOState recovered = s3a.retract(v);
  EXPECT(assert_equal(s3b, recovered, 1e-9));
#else
  EXPECT(true);
#endif
}

//******************************************************************************
// Verifies chart Jacobians for n=0 landmarks.
TEST(VIOState, DerivativesN0) {
#if defined(GTSAM_ROT3_EXPMAP) || defined(GTSAM_USE_QUATERNIONS)
  testChartDerivativesN<21>(result_, name_, MakeState0A(), MakeState0B());
#else
  EXPECT(true);
#endif
}

//******************************************************************************
// Verifies chart Jacobians for n=1 landmark.
TEST(VIOState, DerivativesN1) {
#if defined(GTSAM_ROT3_EXPMAP) || defined(GTSAM_USE_QUATERNIONS)
  testChartDerivativesN<24>(result_, name_, MakeState1A(), MakeState1B());
#else
  EXPECT(true);
#endif
}

//******************************************************************************
// Verifies chart Jacobians for n=3 landmarks.
TEST(VIOState, DerivativesN3) {
#if defined(GTSAM_ROT3_EXPMAP) || defined(GTSAM_USE_QUATERNIONS)
  testChartDerivativesN<30>(result_, name_, MakeState3A(), MakeState3B());
#else
  EXPECT(true);
#endif
}

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
