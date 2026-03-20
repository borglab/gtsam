/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file   testVIOEqFMatricesInvDepth.cpp
 * @brief  Unit tests for inverse-depth VIO EqF matrix suite
 */

#include <CppUnitLite/TestHarness.h>
#include <gtsam/base/TestableAssertions.h>
#include <gtsam_unstable/navigation/EqVIOSymmetry.h>

#include <memory>
#include <stdexcept>
#include <utility>
#include <vector>

using namespace gtsam;
using namespace gtsam::eqvio;

namespace {

Se23 MakeA(const Rot3& R, const Point3& t, const Vector3& w) {
  Se23::Matrix3K x;
  x.col(0) = t;
  x.col(1) = w;
  return Se23(R, x);
}

SensorState SensorFixture() {
  SensorState s;
  s.inputBias = Bias(
      (Vector3() << 0.01, -0.02, 0.03).finished(),
      (Vector3() << 0.04, -0.01, 0.02).finished());
  s.pose = Pose3(Rot3::RzRyRx(0.1, -0.05, 0.2), Point3(0.3, -0.4, 1.2));
  s.velocity = Vector3(0.2, -0.1, 0.05);
  s.cameraOffset =
      Pose3(Rot3::RzRyRx(-0.02, 0.03, -0.01), Point3(0.1, 0.02, 0.08));
  return s;
}

State State0() { return State(SensorFixture(), {}); }
State State1() { return State(SensorFixture(), {{Point3(0.8, -0.2, 4.5), 11}}); }
State State3() {
  return State(SensorFixture(),
                  {{Point3(0.8, -0.2, 4.5), 11},
                   {Point3(-0.6, 0.3, 3.8), 22},
                   {Point3(0.1, 0.7, 5.2), 33}});
}

VioGroup Group0() { return makeVioGroupIdentity(0); }
VioGroup Group1() {
  const SOT3 q1(SO3::Expmap((Vector3() << 0.02, -0.01, 0.03).finished()),
                std::log(1.1));
  return makeVioGroup(
      MakeA(Rot3::RzRyRx(0.03, -0.02, 0.01), Point3(0.05, -0.01, 0.02),
            Vector3(0.01, -0.02, 0.03)),
      Bias((Vector3() << 0.01, 0.0, -0.01).finished(),
              (Vector3() << 0.02, -0.01, 0.03).finished()),
      Pose3(Rot3::RzRyRx(-0.01, 0.02, -0.03), Point3(0.02, 0.01, -0.01)),
      LandmarkGroup({q1}));
}
VioGroup Group3() {
  const SOT3 q1(SO3::Expmap((Vector3() << 0.02, -0.01, 0.03).finished()),
                std::log(1.1));
  const SOT3 q2(SO3::Expmap((Vector3() << -0.01, 0.03, -0.02).finished()),
                std::log(0.95));
  const SOT3 q3(SO3::Expmap((Vector3() << 0.01, 0.02, 0.01).finished()),
                std::log(1.05));
  return makeVioGroup(
      MakeA(Rot3::RzRyRx(0.03, -0.02, 0.01), Point3(0.05, -0.01, 0.02),
            Vector3(0.01, -0.02, 0.03)),
      Bias((Vector3() << 0.01, 0.0, -0.01).finished(),
              (Vector3() << 0.02, -0.01, 0.03).finished()),
      Pose3(Rot3::RzRyRx(-0.01, 0.02, -0.03), Point3(0.02, 0.01, -0.01)),
      LandmarkGroup({q1, q2, q3}));
}

IMUInput ImuFixture() {
  IMUInput u;
  u.gyr = Vector3(0.02, -0.01, 0.03);
  u.acc = Vector3(0.1, -0.05, 9.7);
  u.gyrBiasVel = Vector3(0.01, 0.0, -0.01);
  u.accBiasVel = Vector3(-0.02, 0.01, 0.0);
  return u;
}

bool IsFinite(const Matrix& M) { return M.array().isFinite().all(); }

}  // namespace

//******************************************************************************
TEST(VIOEqFMatricesInvDepth, Selector) {
  const EqFCoordinateSuite* suite = &EqFCoordinateSuite_invdepth;
  EXPECT(suite != nullptr);
}

//******************************************************************************
TEST(VIOEqFMatricesInvDepth, ShapesAndFinite) {
  const auto camera =
      std::make_shared<CameraModel>(Pose3::Identity(), Cal3_S2(1, 1, 0, 0, 0));
  const EqFCoordinateSuite& suite = EqFCoordinateSuite_invdepth;

  for (const auto& pair :
       std::vector<std::pair<State, VioGroup>>{{State0(), Group0()},
                                                   {State1(), Group1()},
                                                   {State3(), Group3()}}) {
    const State& xi0 = pair.first;
    const VioGroup& X = pair.second;
    const IMUInput imu = ImuFixture();
    const VisionMeasurement y =
        measureSystemState(stateGroupAction(X, xi0), camera);

    const Matrix A = suite.stateMatrixA(X, xi0, imu);
    const Matrix B = suite.inputMatrixB(X, xi0);
    const Matrix C = suite.outputMatrixC(xi0, X, y, camera, true);

    EXPECT_LONGS_EQUAL(xi0.dim(), A.rows());
    EXPECT_LONGS_EQUAL(xi0.dim(), A.cols());
    EXPECT_LONGS_EQUAL(xi0.dim(), B.rows());
    EXPECT_LONGS_EQUAL(IMUInput::CompDim, B.cols());
    EXPECT_LONGS_EQUAL(2 * static_cast<long>(y.size()), C.rows());
    EXPECT_LONGS_EQUAL(xi0.dim(), C.cols());

    EXPECT(IsFinite(A));
    EXPECT(IsFinite(B));
    EXPECT(IsFinite(C));
  }
}

//******************************************************************************
TEST(VIOEqFMatricesInvDepth, StateChartRoundTrip) {
  const EqFCoordinateSuite& suite = EqFCoordinateSuite_invdepth;

  for (const State& xi0 : std::vector<State>{State1(), State3()}) {
    const Vector eps = Vector::LinSpaced(xi0.dim(), -1e-3, 1e-3);
    const State xi = suite.stateChartInv(eps, xi0);
    const Vector epsRecovered = suite.stateChart(xi, xi0);
    EXPECT(assert_equal(eps, epsRecovered, 1e-8));
  }
}

//******************************************************************************
TEST(VIOEqFMatricesInvDepth, SmallStepDiscreteConsistency) {
  const EqFCoordinateSuite& suite = EqFCoordinateSuite_invdepth;

  const State xi0 = State3();
  const VioGroup X = Group3();
  const IMUInput imu = ImuFixture();

  const double dt = 1e-6;
  const Matrix A = suite.stateMatrixA(X, xi0, imu);
  const Matrix Phi = suite.stateMatrixADiscrete(X, xi0, imu, dt);
  const Matrix PhiApprox = Matrix::Identity(xi0.dim(), xi0.dim()) + dt * A;

  EXPECT(assert_equal(PhiApprox, Phi, 1e-4));
}

//******************************************************************************
TEST(VIOEqFMatricesInvDepth, InnovationLiftConsistency) {
  const EqFCoordinateSuite& suite = EqFCoordinateSuite_invdepth;

  for (const State& xi0 : std::vector<State>{State1(), State3()}) {
    const Vector gamma = Vector::LinSpaced(xi0.dim(), -0.1, 0.1);
    const double eps = 1e-3;
    const Vector lift = suite.liftInnovation(eps * gamma, xi0);
    const VioGroup dCont = VioGroup::Expmap(lift);
    const VioGroup dDisc = suite.liftInnovationDiscrete(eps * gamma, xi0);

    const Vector err = dCont.localCoordinates(dDisc);
    EXPECT(err.norm() < 5e-3);
  }
}

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
