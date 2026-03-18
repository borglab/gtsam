/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file   testVIOEqFMatricesEqvioParity.cpp
 * @brief  Parity tests mirroring eqvio's original EqF matrix tests
 */

#include <CppUnitLite/TestHarness.h>
#include <gtsam/base/TestableAssertions.h>
#include <gtsam_unstable/navigation/VIOEqFMatrices.h>
#include <gtsam_unstable/navigation/EqVIOSymmetry.h>

#include <cmath>
#include <limits>
#include <memory>
#include <stdexcept>
#include <vector>

using namespace gtsam;
using namespace gtsam::eqvio;

namespace {

constexpr int kTestReps = 10;

class SimplePinholeCamera final : public VIOCameraModel {
 public:
  explicit SimplePinholeCamera(double fx = 450.0, double fy = 450.0,
                               double cx = 400.0, double cy = 240.0)
      : fx_(fx), fy_(fy), cx_(cx), cy_(cy) {}

  Point2 projectPoint(const Point3& p) const override {
    if (std::abs(p.z()) < 1e-12) {
      throw std::invalid_argument("SimplePinholeCamera: z is near zero");
    }
    return Point2(fx_ * p.x() / p.z() + cx_, fy_ * p.y() / p.z() + cy_);
  }

  Vector3 undistortPoint(const Point2& y) const override {
    return Vector3((y.x() - cx_) / fx_, (y.y() - cy_) / fy_, 1.0);
  }

  Matrix23 projectionJacobian(const Vector3& y) const override {
    if (std::abs(y.z()) < 1e-12) {
      throw std::invalid_argument("SimplePinholeCamera: z is near zero");
    }
    Matrix23 J;
    const double z2 = y.z() * y.z();
    J << fx_ / y.z(), 0.0, -fx_ * y.x() / z2, 0.0, fy_ / y.z(),
        -fy_ * y.y() / z2;
    return J;
  }

 private:
  double fx_, fy_, cx_, cy_;
};

VIOSE23 MakeA(const Rot3& R, const Point3& t, const Vector3& w) {
  VIOSE23::Matrix3K x;
  x.col(0) = t;
  x.col(1) = w;
  return VIOSE23(R, x);
}

double SOT3Scale(const SOT3& Q) { return std::exp(Q.second); }

Vector3 SOT3ApplyInverse(const SOT3& Q, const Vector3& p) {
  return (1.0 / SOT3Scale(Q)) * (Q.first.matrix().transpose() * p);
}

VIOState ReasonableStateElement(const std::vector<int>& ids) {
  VIOSensorState sensor;
  sensor.inputBias =
      VIOBias(0.1 * Vector3::Random(), 0.1 * Vector3::Random());
  sensor.pose = Pose3::Expmap(0.1 * Vector6::Random());
  sensor.velocity = Vector3::Random();
  sensor.cameraOffset = Pose3::Expmap(0.1 * Vector6::Random());

  std::vector<Landmark> lms(ids.size());
  for (size_t i = 0; i < ids.size(); ++i) {
    Point3 p = 10.0 * Vector3::Random();
    p.z() += 20.0;
    lms[i] = Landmark{p, ids[i]};
  }
  return VIOState(sensor, lms);
}

VIOGroup ReasonableGroupElement(const std::vector<int>& ids) {
  const Pose3 Apose = Pose3::Expmap(0.1 * Vector6::Random());
  const Vector3 w = 0.1 * Vector3::Random();
  const Pose3 B = Pose3::Expmap(0.1 * Vector6::Random());
  const VIOBias beta(0.1 * Vector3::Random(), 0.1 * Vector3::Random());

  std::vector<SOT3> Q(ids.size());
  for (size_t i = 0; i < ids.size(); ++i) {
    const SO3 R = SO3::Expmap(0.02 * Vector3::Random());
    const double a = 2.0 * static_cast<double>(rand()) / RAND_MAX + 1.0;
    Q[i] = SOT3(R, std::log(a));
  }

  return makeVIOGroup(MakeA(Apose.rotation(), Apose.translation(), w), beta, B,
                      VIOLandmarkGroup(Q));
}

IMUInput RandomVelocityElement() {
  IMUInput vel;
  vel.gyr = Vector3::Random();
  vel.acc = Vector3::Random();
  vel.gyrBiasVel = Vector3::Random();
  vel.accBiasVel = Vector3::Random();
  vel.stamp = 0.0;
  return vel;
}

Matrix NumericalDifferential(const std::function<Vector(const Vector&)>& f,
                             const Vector& x0, double h) {
  const int n = static_cast<int>(x0.size());
  const Vector y0 = f(x0);
  const int m = static_cast<int>(y0.size());
  Matrix J = Matrix::Zero(m, n);
  for (int j = 0; j < n; ++j) {
    Vector dx = Vector::Zero(n);
    dx(j) = h;
    J.col(j) = (f(x0 + dx) - f(x0 - dx)) / (2.0 * h);
  }
  return J;
}

Rot3 RotationFromTwoVectors(const Vector3& from, const Vector3& to) {
  const double eps = 1e-12;
  const Vector3 a = from.normalized();
  const Vector3 b = to.normalized();
  const double c = std::clamp(a.dot(b), -1.0, 1.0);

  if (c > 1.0 - eps) return Rot3::Identity();

  if (c < -1.0 + eps) {
    Vector3 axis = a.unitOrthogonal();
    axis.normalize();
    return Rot3::Expmap(std::acos(-1.0) * axis);
  }

  Vector3 axis = a.cross(b);
  const double s = axis.norm();
  axis /= s;
  return Rot3::Expmap(std::atan2(s, c) * axis);
}

Matrix23 E3ProjectSphereDiff(const Vector3& eta) {
  const Matrix23 I23 = Matrix23::Identity();
  const Vector3 e3 = Vector3::UnitZ();
  Matrix23 diff =
      I23 * (Matrix3::Identity() * (1.0 - eta.z()) + (eta - e3) * e3.transpose());
  diff /= std::pow(1.0 - e3.dot(eta), 2.0);
  return diff;
}

Matrix23 SphereChartStereoDiff0(const Vector3& pole) {
  const Rot3 sphereRot = RotationFromTwoVectors(-pole, Vector3::UnitZ());
  const Vector3 etaRotated = sphereRot.matrix() * pole;
  return E3ProjectSphereDiff(etaRotated) * sphereRot.matrix();
}

Matrix CoordinateDifferentialInvDepthEuclid(const VIOState& xi0) {
  Matrix M = Matrix::Identity(xi0.dim(), xi0.dim());
  for (size_t i = 0; i < xi0.n(); ++i) {
    const Point3 q0 = xi0.cameraLandmarks[i].p;
    const double rho0 = 1.0 / q0.norm();
    const Vector3 y0 = q0 * rho0;

    Matrix3 Mi;
    Mi.block<2, 3>(0, 0) = rho0 * SphereChartStereoDiff0(y0) *
                           (Matrix3::Identity() - y0 * y0.transpose());
    Mi.block<1, 3>(2, 0) = -rho0 * rho0 * y0.transpose();
    M.block<3, 3>(VIOSensorState::CompDim + 3 * static_cast<int>(i),
                  VIOSensorState::CompDim + 3 * static_cast<int>(i)) = Mi;
  }
  return M;
}

bool IsMatrixClose(const Matrix& A, const Matrix& B, double h = -1.0) {
  if (A.rows() != B.rows() || A.cols() != B.cols()) return false;
  if (!A.array().isFinite().all() || !B.array().isFinite().all()) return false;
  if (h < 0.0) h = std::cbrt(std::numeric_limits<double>::epsilon());
  for (int i = 0; i < A.rows(); ++i) {
    for (int j = 0; j < A.cols(); ++j) {
      const double tol = std::max(h, h * 1e1 * std::abs(A(i, j)));
      if (std::abs(A(i, j) - B(i, j)) > tol) return false;
    }
  }
  return true;
}

bool IsDifferentialClose(const std::function<Vector(const Vector&)>& f,
                         const Vector& x0, const Matrix& Df, double h) {
  const Matrix numericalDf = NumericalDifferential(f, x0, h);
  return IsMatrixClose(Df, numericalDf, h);
}

}  // namespace

//******************************************************************************
TEST(VIOEqFMatricesParity, EuclidInvDepthCompatibility) {
  srand(0);
  const std::vector<int> ids = {0, 1, 2, 3, 4};
  const auto camera = std::make_shared<SimplePinholeCamera>();

  const EqFCoordinateSuite* euclid = getCoordinates(CoordinateChoice::Euclidean);
  const EqFCoordinateSuite* invdepth =
      getCoordinates(CoordinateChoice::InvDepth);
  EXPECT(euclid != nullptr);
  EXPECT(invdepth != nullptr);
  if (!euclid || !invdepth) return;

  for (int rep = 0; rep < kTestReps; ++rep) {
    const VIOState xi0 = ReasonableStateElement(ids);
    const VIOGroup X = ReasonableGroupElement(ids);
    const IMUInput vel = RandomVelocityElement();

    const Matrix M = CoordinateDifferentialInvDepthEuclid(xi0);

    const Matrix A_e = euclid->stateMatrixA(X, xi0, vel);
    const Matrix A_i = invdepth->stateMatrixA(X, xi0, vel);
    EXPECT((A_i - M * A_e * M.inverse()).norm() < 1e-3);

    const Matrix B_e = euclid->inputMatrixB(X, xi0);
    const Matrix B_i = invdepth->inputMatrixB(X, xi0);
    EXPECT((B_i - M * B_e).norm() < 1e-3);

    const VIOState xiHat = stateGroupAction(X, xi0);
    const VisionMeasurement yHat = measureSystemState(xiHat, camera);
    const Matrix C_e = euclid->outputMatrixC(xi0, X, yHat, camera);
    const Matrix C_i = invdepth->outputMatrixC(xi0, X, yHat, camera);
    EXPECT(C_e.array().isFinite().all());
    EXPECT(C_i.array().isFinite().all());
  }
}

//******************************************************************************
TEST(VIOEqFMatricesParity, StateMatrixA) {
  srand(0);
  const std::vector<int> ids = {0, 1, 2, 3, 4};
  const std::vector<CoordinateChoice> choices = {CoordinateChoice::Euclidean,
                                                  CoordinateChoice::InvDepth};
  for (CoordinateChoice choice : choices) {
    const EqFCoordinateSuite* suite = getCoordinates(choice);
    EXPECT(suite != nullptr);
    if (!suite) continue;

    for (int rep = 0; rep < kTestReps; ++rep) {
      const VIOState xi0 = ReasonableStateElement(ids);
      const VIOGroup X = ReasonableGroupElement(ids);
      const IMUInput vel = RandomVelocityElement();
      const Matrix A0t = suite->stateMatrixA(X, xi0, vel);

      const auto a0 = [&](const Vector& epsilon) {
        const VIOState xiHat = stateGroupAction(X, xi0);
        const VIOState xiE = suite->stateChartInv(epsilon, xi0);
        const VIOState xi = stateGroupAction(X, xiE);
        const Vector lambdaTilde =
            liftVelocity(xi, vel) - liftVelocity(xiHat, vel);
        const VIOState xiHat1 =
            stateGroupAction(VIOGroup::Expmap(lambdaTilde), xiHat);
        const VIOState xiE1 = stateGroupAction(X.inverse(), xiHat1);
        return suite->stateChart(xiE1, xi0);
      };

      const Vector a0AtZero = a0(Vector::Zero(xi0.dim()));
      EXPECT(a0AtZero.norm() < 1e-10);
      EXPECT(IsDifferentialClose(
          a0, Vector::Zero(xi0.dim()), A0t,
          std::cbrt(std::numeric_limits<double>::epsilon())));
    }
  }
}

//******************************************************************************
TEST(VIOEqFMatricesParity, InputMatrixB) {
  srand(0);
  const std::vector<int> ids = {0, 1, 2, 3, 4};
  const std::vector<CoordinateChoice> choices = {CoordinateChoice::Euclidean,
                                                  CoordinateChoice::InvDepth};
  for (CoordinateChoice choice : choices) {
    const EqFCoordinateSuite* suite = getCoordinates(choice);
    EXPECT(suite != nullptr);
    if (!suite) continue;

    for (int rep = 0; rep < kTestReps; ++rep) {
      const VIOState xi0 = ReasonableStateElement(ids);
      const VIOGroup X = ReasonableGroupElement(ids);
      const IMUInput vel = RandomVelocityElement();
      const Matrix Bt = suite->inputMatrixB(X, xi0);

      const auto b0 = [&](const Vector& velErrVec) {
        IMUInput::Vector12 velErr12 = velErrVec;
        IMUInput velErr(velErr12);
        const VIOState xiHat = stateGroupAction(X, xi0);
        const Vector lambdaTilde =
            liftVelocity(xiHat, vel + velErr) - liftVelocity(xiHat, vel);
        const VIOState xiHat1 =
            stateGroupAction(VIOGroup::Expmap(lambdaTilde), xiHat);
        const VIOState xiE1 = stateGroupAction(X.inverse(), xiHat1);
        return suite->stateChart(xiE1, xi0);
      };

      const Vector b0AtZero = b0(Vector::Zero(IMUInput::CompDim));
      EXPECT(b0AtZero.norm() < 1e-10);
      EXPECT(IsDifferentialClose(
          b0, Vector::Zero(IMUInput::CompDim), Bt,
          std::cbrt(std::numeric_limits<double>::epsilon())));
    }
  }
}

//******************************************************************************
TEST(VIOEqFMatricesParity, OutputMatrixC) {
  srand(0);
  const std::vector<int> ids = {5, 0, 1, 2, 3, 4};
  const auto camera = std::make_shared<SimplePinholeCamera>();
  const std::vector<CoordinateChoice> choices = {CoordinateChoice::Euclidean,
                                                  CoordinateChoice::InvDepth};
  for (CoordinateChoice choice : choices) {
    const EqFCoordinateSuite* suite = getCoordinates(choice);
    EXPECT(suite != nullptr);
    if (!suite) continue;

    for (int rep = 0; rep < kTestReps; ++rep) {
      const VIOState xi0 = ReasonableStateElement(ids);
      const VIOGroup X = ReasonableGroupElement(ids);
      const VIOState xiHat = stateGroupAction(X, xi0);
      const VisionMeasurement yHat = measureSystemState(xiHat, camera);

      for (size_t i = 0; i < xi0.n(); ++i) {
        const int id = xi0.cameraLandmarks[i].id;
        const Point2 yFromQ =
            camera->projectPoint(
                SOT3ApplyInverse(Q_landmarkTransforms(X)[i], xi0.cameraLandmarks[i].p));
        const Point2 yStored = yHat.at(id);
        EXPECT((yFromQ - yStored).norm() < 1e-10);
      }

      const Matrix Ct = suite->outputMatrixC(xi0, X, yHat, camera);
      const Matrix Ct2 = suite->outputMatrixC(xi0, X, yHat, camera, false);
      EXPECT_LONGS_EQUAL(2 * static_cast<long>(yHat.size()), Ct.rows());
      EXPECT_LONGS_EQUAL(xi0.dim(), Ct.cols());
      EXPECT_LONGS_EQUAL(2 * static_cast<long>(yHat.size()), Ct2.rows());
      EXPECT_LONGS_EQUAL(xi0.dim(), Ct2.cols());
      EXPECT(Ct.array().isFinite().all());
      EXPECT(Ct2.array().isFinite().all());
      assert_equal(Ct, Ct2, 1e-8);

      const auto ct = [&](const Vector& epsilon) {
        const VIOState xiE = suite->stateChartInv(epsilon, xi0);
        const VIOState xi = stateGroupAction(X, xiE);
        const VisionMeasurement y = measureSystemState(xi, camera);
        return measurementDifference(y, yHat);
      };

      const Vector c0AtZero = ct(Vector::Zero(xi0.dim()));
      EXPECT(c0AtZero.norm() < 1e-10);

      const double floatStep = std::cbrt(std::numeric_limits<float>::epsilon());
      const Matrix CtNumerical =
          NumericalDifferential(ct, Vector::Zero(xi0.dim()), floatStep);
      EXPECT_LONGS_EQUAL(Ct.rows(), CtNumerical.rows());
      EXPECT_LONGS_EQUAL(Ct.cols(), CtNumerical.cols());
      EXPECT(CtNumerical.array().isFinite().all());
      EXPECT(IsMatrixClose(Ct, CtNumerical, floatStep));
    }
  }
}

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
