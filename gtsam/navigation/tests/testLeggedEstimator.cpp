/* ----------------------------------------------------------------------------
 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)
 *
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file testLeggedEstimator.cpp
 * @date February 2026
 * @brief Unit tests for the legged estimator variants.
 */

#include <CppUnitLite/TestHarness.h>
#include <gtsam/base/TestableAssertions.h>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/navigation/LeggedEstimator.h>
#include <gtsam/navigation/NavStateImuEKF.h>

using namespace gtsam;

namespace {

class ExposedLeggedInvariantEKF : public LeggedInvariantEKF {
 public:
  using LeggedInvariantEKF::LeggedInvariantEKF;
  using LeggedInvariantEKF::marginalizeFoot;
};

constexpr size_t kTwoFeet = 2;
constexpr int kExtendedPoseDim = 3 + 3 * static_cast<int>(2 + kTwoFeet);

LeggedEstimatorParams makeParams(bool marginalizeLeavingFoot = false) {
  LeggedEstimatorParams params;
  params.preintegrationParams = PreintegrationParams::MakeSharedU(9.81);
  params.preintegrationParams->setGyroscopeCovariance(I_3x3 * 1e-4);
  params.preintegrationParams->setIntegrationCovariance(I_3x3 * 1e-4);
  params.preintegrationParams->setAccelerometerCovariance(I_3x3 * 1e-4);
  params.body_P_imu = Pose3::Identity();
  params.footholdInitSigma = 0.25;
  params.footholdProcessSigma = 1e-3;
  params.contactCovariance = I_3x3 * (0.05 * 0.05);
  params.heightPriorSigma = 0.05;
  params.useFullContactInitialization = false;
  params.marginalizeLeavingFoot = marginalizeLeavingFoot;
  return params;
}

Matrix initialCovariance(size_t numFeet) {
  const int dim = 9 + 3 * static_cast<int>(numFeet);
  Matrix P = Matrix::Identity(dim, dim) * 0.1;
  P.block(0, 0, 3, 3) = I_3x3 * 0.01;
  P.block(3, 3, 3, 3) = I_3x3 * 0.04;
  P.block(6, 6, 3, 3) = I_3x3 * 0.04;
  for (size_t foot = 0; foot < numFeet; ++foot) {
    P.block(9 + 3 * static_cast<int>(foot), 9 + 3 * static_cast<int>(foot), 3,
            3) = I_3x3 * 0.09;
  }
  return P;
}

Matrix denseCorrelatedCovariance(size_t numFeet) {
  const int dim = 9 + 3 * static_cast<int>(numFeet);
  Matrix basis(dim, dim);
  for (int row = 0; row < dim; ++row) {
    for (int col = 0; col < dim; ++col) {
      basis(row, col) =
          0.13 * (row + 1) + 0.07 * (col + 1) + 0.01 * (row + 1) * (col + 1);
    }
  }
  return basis * basis.transpose() + Matrix::Identity(dim, dim);
}

Eigen::PermutationMatrix<Eigen::Dynamic, Eigen::Dynamic>
makeMarginalizationPermutation(size_t numFeet, size_t marginalizedFoot) {
  const int dim = 9 + 3 * static_cast<int>(numFeet);
  const int footStart = 9 + 3 * static_cast<int>(marginalizedFoot);
  Eigen::PermutationMatrix<Eigen::Dynamic, Eigen::Dynamic> permutation(dim);
  Eigen::VectorXi indices(dim);
  int next = 0;
  for (int index = 0; index < dim; ++index) {
    if (index < footStart || index >= footStart + 3) {
      indices(next++) = index;
    }
  }
  for (int index = footStart; index < footStart + 3; ++index) {
    indices(next++) = index;
  }
  for (int row = 0; row < dim; ++row) {
    permutation.indices()(row) = indices(row);
  }
  return permutation;
}

Matrix permuteSymmetricMatrix(
    const Matrix& matrix,
    const Eigen::PermutationMatrix<Eigen::Dynamic, Eigen::Dynamic>&
        permutation) {
  return permutation.transpose() * matrix * permutation;
}

Matrix oneFoot(const Vector3& foothold) {
  Matrix footholds(3, 1);
  footholds.col(0) = foothold;
  return footholds;
}

Matrix twoFootholds() {
  return (Matrix(3, 2) << 1.0, -0.2, 0.3, 0.4, 0.5, -0.1).finished();
}

Matrix fourFootholds() { return Matrix::Zero(3, 4); }

NavState identityState() {
  return NavState(Rot3(), Point3(0.0, 0.0, 0.0), Vector3::Zero());
}

NavState sampleNavState() {
  return NavState(Rot3::RzRyRx(0.1, -0.2, 0.05), Point3(0.2, -0.4, 0.3),
                  Vector3(0.6, -0.1, 0.2));
}

NavState navStateFromEstimate(const ExtendedPose3d& estimate) {
  return NavState(estimate.rotation(), estimate.x(0), estimate.x(1));
}

Matrix footholdsFromEstimate(const ExtendedPose3d& estimate) {
  const Eigen::Index numFeet =
      static_cast<Eigen::Index>(estimate.k() - static_cast<size_t>(2));
  return estimate.xMatrix().rightCols(numFeet);
}

Matrix fullContactFilterCovariance(double poseSigma, double velocitySigma,
                                   double footholdSigma) {
  Matrix covariance = Matrix::Zero(21, 21);
  covariance.block(0, 0, 6, 6) =
      Matrix::Identity(6, 6) * (poseSigma * poseSigma);
  covariance.block(6, 6, 3, 3) = I_3x3 * (velocitySigma * velocitySigma);
  covariance.block(9, 9, 12, 12) =
      Matrix::Identity(12, 12) * (footholdSigma * footholdSigma);
  return covariance;
}

Matrix9 fullContactBaseCovariance(double poseSigma, double velocitySigma) {
  Matrix9 covariance = Matrix9::Zero();
  covariance.topLeftCorner<6, 6>() =
      Matrix::Identity(6, 6) * (poseSigma * poseSigma);
  covariance.bottomRightCorner<3, 3>() =
      I_3x3 * (velocitySigma * velocitySigma);
  return covariance;
}

std::vector<ContactMeasurement> realReplayContacts(size_t eventIndex) {
  switch (eventIndex) {
    case 0:
      return {{0, Vector3(0.374083578587, 0.130614116788, -0.503917515278)},
              {1, Vector3(0.249635994434, -0.170448854566, -0.510778665543)},
              {2, Vector3(-0.347598612309, 0.177350610495, -0.507786512375)},
              {3, Vector3(-0.341215342283, -0.164554029703, -0.511552393436)}};
    case 1:
      return {{0, Vector3(0.140942052007, 0.116847425699, -0.497972786427)},
              {1, Vector3(0.429882109165, -0.062552683055, -0.505733430386)},
              {2, Vector3(-0.162623643875, 0.106095127761, -0.499895572662)},
              {3, Vector3(-0.568916738033, -0.166614994407, -0.492177367210)}};
    case 2:
      return {{0, Vector3(0.455747604370, 0.078563340008, -0.505808234215)},
              {3, Vector3(-0.145139902830, -0.099087588489, -0.511903226376)}};
    case 3:
      return {{1, Vector3(0.481080681086, -0.087105244398, -0.510063111782)},
              {2, Vector3(-0.115411847830, 0.083123117685, -0.506291151047)}};
    default:
      throw std::invalid_argument("Unsupported real replay event index.");
  }
}

LeggedEstimatorParams makeRealReplayParams() {
  LeggedEstimatorParams params = makeParams();
  params.body_P_imu = Pose3(Rot3(), Point3(0.3, 0.0, 0.15));
  params.footholdInitSigma = 0.282842712474619;
  params.contactCovariance = I_3x3 * (0.005 * 0.005);
  params.heightPriorSigma = 0.05;
  return params;
}

}  // namespace

/* ************************************************************************* */
TEST(LeggedEstimator, PredictSubtractsConfiguredImuBias) {
  LeggedEstimatorParams biasedParams = makeParams();
  biasedParams.imuBias = imuBias::ConstantBias(Vector3(0.05, -0.04, 0.03),
                                               Vector3(0.02, -0.01, 0.03));
  const LeggedEstimatorParams unbiasedParams = makeParams();

  const NavState X0 = sampleNavState();
  const Matrix footholds = twoFootholds();
  const Matrix P0 = initialCovariance(2);
  LeggedInvariantEKF biasedEstimator(X0, footholds, P0, biasedParams);
  LeggedInvariantEKF unbiasedEstimator(X0, footholds, P0, unbiasedParams);

  const Vector3 omegaBody(0.2, -0.3, 0.1);
  const Vector3 specificForceBody(0.4, -0.2, 0.5);
  const double dt = 0.02;

  biasedEstimator.predict(
      omegaBody + biasedParams.imuBias.gyroscope(),
      specificForceBody + biasedParams.imuBias.accelerometer(), dt);
  unbiasedEstimator.predict(omegaBody, specificForceBody, dt);

  EXPECT(assert_equal(navStateFromEstimate(unbiasedEstimator.estimate()),
                      navStateFromEstimate(biasedEstimator.estimate()), 1e-9));
  EXPECT(assert_equal(footholdsFromEstimate(unbiasedEstimator.estimate()),
                      footholdsFromEstimate(biasedEstimator.estimate()),
                      1e-12));
}

/* ************************************************************************* */
TEST(LeggedEstimator, FullContactInitializationKeepsUnobservedFilterVelocity) {
  LeggedEstimatorParams params = makeRealReplayParams();
  params.useFullContactInitialization = true;
  const NavState priorState(Rot3(), Point3(0.0, 0.0, 0.72),
                            Vector3(0.35, -0.10, 0.08));
  const Matrix P0 = fullContactFilterCovariance(0.2, 1e-3, 0.5);
  LeggedInvariantEKF estimator(priorState, fourFootholds(), P0, params,
                               {"FL", "FR", "RL", "RR"});

  estimator.processContacts(realReplayContacts(0));

  const NavState initializedState = navStateFromEstimate(estimator.estimate());
  EXPECT(
      assert_equal(priorState.velocity(), initializedState.velocity(), 1e-6));
}

/* ************************************************************************* */
TEST(LeggedEstimator, FilterWaitsForFullContactInitialization) {
  LeggedEstimatorParams params = makeRealReplayParams();
  params.useFullContactInitialization = true;
  const NavState priorState = sampleNavState();
  LeggedInvariantEKF estimator(priorState, fourFootholds(),
                               fullContactFilterCovariance(0.2, 0.1, 0.5),
                               params, {"FL", "FR", "RL", "RR"});

  estimator.predict(Vector3(0.3, -0.1, 0.2), Vector3(0.2, -0.4, 9.7), 0.2);
  estimator.processContacts(realReplayContacts(2));

  EXPECT(assert_equal(priorState, navStateFromEstimate(estimator.estimate()),
                      1e-12));
  EXPECT(assert_equal(fourFootholds(),
                      footholdsFromEstimate(estimator.estimate()), 1e-12));
}

/* ************************************************************************* */
TEST(LeggedEstimator, FixedLagSmootherWaitsForFullContactInitialization) {
  LeggedEstimatorParams params = makeRealReplayParams();
  params.useFullContactInitialization = true;
  const NavState priorState = sampleNavState();
  LeggedFixedLagSmoother estimator(priorState, fourFootholds(),
                                   fullContactBaseCovariance(0.2, 0.1), params,
                                   0.15, {"FL", "FR", "RL", "RR"});

  estimator.predict(Vector3(0.3, -0.1, 0.2), Vector3(0.2, -0.4, 9.7), 0.2);
  estimator.processContacts(realReplayContacts(2));

  EXPECT(assert_equal(priorState, navStateFromEstimate(estimator.estimate()),
                      1e-12));
  EXPECT(assert_equal(fourFootholds(),
                      footholdsFromEstimate(estimator.estimate()), 1e-12));
}

/* ************************************************************************* */
TEST(LeggedEstimator, FullContactInitializationFilterDependsOnPriorCovariance) {
  LeggedEstimatorParams params = makeRealReplayParams();
  params.useFullContactInitialization = true;
  const double priorPitch = 0.25;
  const NavState priorState(Rot3::Ypr(0.0, priorPitch, 0.0),
                            Point3(0.0, 0.0, 0.72), Vector3::Zero());

  LeggedInvariantEKF tightEstimator(
      priorState, fourFootholds(), fullContactFilterCovariance(0.02, 0.05, 0.5),
      params, {"FL", "FR", "RL", "RR"});
  LeggedInvariantEKF looseEstimator(priorState, fourFootholds(),
                                    fullContactFilterCovariance(2.0, 0.05, 0.5),
                                    params, {"FL", "FR", "RL", "RR"});

  tightEstimator.processContacts(realReplayContacts(0));
  looseEstimator.processContacts(realReplayContacts(0));

  const double tightPitch =
      navStateFromEstimate(tightEstimator.estimate()).attitude().rpy().y();
  const double loosePitch =
      navStateFromEstimate(looseEstimator.estimate()).attitude().rpy().y();
  CHECK(std::abs(tightPitch - loosePitch) > 1e-2);
  CHECK(std::abs(tightPitch - priorPitch) < std::abs(loosePitch - priorPitch));
}

/* ************************************************************************* */
TEST(LeggedEstimator,
     FullContactInitializationFixedLagSmootherDependsOnPriorCovariance) {
  LeggedEstimatorParams params = makeRealReplayParams();
  params.useFullContactInitialization = true;
  const double priorPitch = 0.25;
  const NavState priorState(Rot3::Ypr(0.0, priorPitch, 0.0),
                            Point3(0.0, 0.0, 0.72), Vector3::Zero());

  LeggedFixedLagSmoother tightEstimator(priorState, fourFootholds(),
                                        fullContactBaseCovariance(0.02, 0.05),
                                        params, 0.15, {"FL", "FR", "RL", "RR"});
  LeggedFixedLagSmoother looseEstimator(priorState, fourFootholds(),
                                        fullContactBaseCovariance(2.0, 0.05),
                                        params, 0.15, {"FL", "FR", "RL", "RR"});

  tightEstimator.processContacts(realReplayContacts(0));
  looseEstimator.processContacts(realReplayContacts(0));

  const double tightPitch =
      navStateFromEstimate(tightEstimator.estimate()).attitude().rpy().y();
  const double loosePitch =
      navStateFromEstimate(looseEstimator.estimate()).attitude().rpy().y();
  CHECK(std::abs(tightPitch - loosePitch) > 1e-2);
  CHECK(std::abs(tightPitch - priorPitch) < std::abs(loosePitch - priorPitch));
}

/* ************************************************************************* */
TEST(LeggedEstimator,
     FullContactInitializationCombinedSmootherDependsOnPriorCovariance) {
  LeggedEstimatorParams params = makeRealReplayParams();
  params.useFullContactInitialization = true;
  const double priorPitch = 0.25;
  const NavState priorState(Rot3::Ypr(0.0, priorPitch, 0.0),
                            Point3(0.0, 0.0, 0.72), Vector3::Zero());

  LeggedCombinedFixedLagSmoother tightEstimator(
      priorState, fourFootholds(), fullContactBaseCovariance(0.02, 0.05),
      params, 0.15, {"FL", "FR", "RL", "RR"});
  LeggedCombinedFixedLagSmoother looseEstimator(
      priorState, fourFootholds(), fullContactBaseCovariance(2.0, 0.05), params,
      0.15, {"FL", "FR", "RL", "RR"});

  tightEstimator.processContacts(realReplayContacts(0));
  looseEstimator.processContacts(realReplayContacts(0));

  const double tightPitch =
      navStateFromEstimate(tightEstimator.estimate()).attitude().rpy().y();
  const double loosePitch =
      navStateFromEstimate(looseEstimator.estimate()).attitude().rpy().y();
  CHECK(std::abs(tightPitch - loosePitch) > 1e-2);
  CHECK(std::abs(tightPitch - priorPitch) < std::abs(loosePitch - priorPitch));
}

/* ************************************************************************* */
TEST(LeggedEstimator, LeggedInvariantDynamicsJacobian) {
  const NavState X0 = sampleNavState();
  const Matrix footholds0 = twoFootholds();
  const ExtendedPose3d state = LeggedInvariantEKF::MakeState(X0, footholds0);
  const Vector3 omegaBody(0.2, -0.3, 0.1);
  const Vector3 specificForceBody(0.4, -0.2, 0.5);
  const Vector3 gravity(0.0, 0.0, -9.81);
  const double dt = 0.02;

  const ExtendedPose3d W =
      LeggedInvariantEKF::GravityIncrement(kTwoFeet, gravity, dt);
  const ExtendedPose3d U = LeggedInvariantEKF::ImuIncrement(
      kTwoFeet, omegaBody, specificForceBody, dt);
  const LeggedInvariantEKF::AutonomousFlow phi(kTwoFeet, dt);

  Matrix A;
  (void)LeftLinearEKF<ExtendedPose3d>::template Dynamics<
      LeggedInvariantEKF::AutonomousFlow>(W, phi, state, U, A);
  const auto f = [&](const ExtendedPose3d& X) {
    return LeftLinearEKF<ExtendedPose3d>::template Dynamics<
        LeggedInvariantEKF::AutonomousFlow>(W, phi, X, U);
  };
  const Matrix expected =
      numericalDerivative11<ExtendedPose3d, ExtendedPose3d, kExtendedPoseDim>(
          f, state, 1e-6);

  EXPECT(assert_equal(expected, A, 1e-6));
}

/* ************************************************************************* */
TEST(LeggedEstimator, MarginalizeLeavingFootResetsCovarianceBlock) {
  const LeggedEstimatorParams params = makeParams(true);
  LeggedInvariantEKF estimator(identityState(), oneFoot(Vector3(1.0, 0.0, 0.0)),
                               initialCovariance(1), params);

  estimator.processContacts({ContactMeasurement{0, Vector3(1.0, 0.0, 0.0)}});
  const Matrix afterContact = estimator.covariance();
  CHECK(afterContact.block(0, 9, 9, 3).norm() > 1e-6);

  estimator.processContacts({});
  const Matrix afterLeaving = estimator.covariance();
  const Matrix3 expectedFoot =
      I_3x3 * (params.footholdInitSigma * params.footholdInitSigma);

  EXPECT_DOUBLES_EQUAL(0.0, afterLeaving.block(0, 9, 9, 3).norm(), 1e-12);
  EXPECT_DOUBLES_EQUAL(0.0, afterLeaving.block(9, 0, 3, 9).norm(), 1e-12);
  EXPECT(assert_equal(Matrix(expectedFoot), afterLeaving.block(9, 9, 3, 3),
                      1e-12));
  EXPECT_DOUBLES_EQUAL(
      0.0, Point3(footholdsFromEstimate(estimator.estimate()).col(0)).norm(),
      1e-12);
}

/* ************************************************************************* */
TEST(LeggedEstimator, MarginalizeFootPreservesRetainedSchurComplement) {
  const size_t numFeet = 2;
  const size_t marginalizedFoot = 0;
  LeggedEstimatorParams params = makeParams(true);
  params.footholdInitSigma = 0.3;
  const Matrix P0 = denseCorrelatedCovariance(numFeet);
  const Matrix footholds =
      (Matrix(3, 2) << 0.4, -0.2, 1.1, 0.7, -0.5, 0.3).finished();
  ExposedLeggedInvariantEKF estimator(identityState(), footholds, P0, params);

  estimator.marginalizeFoot(marginalizedFoot);
  const Matrix marginalized = estimator.covariance();

  const int retainedDim = static_cast<int>(P0.rows()) - 3;
  const auto permutation =
      makeMarginalizationPermutation(numFeet, marginalizedFoot);
  const Matrix permutedInformation =
      permuteSymmetricMatrix(P0.inverse(), permutation);
  const Matrix Lambda_rr =
      permutedInformation.topLeftCorner(retainedDim, retainedDim);
  const Matrix Lambda_rl = permutedInformation.topRightCorner(retainedDim, 3);
  const Matrix Lambda_lr = permutedInformation.bottomLeftCorner(3, retainedDim);
  const Matrix Lambda_ll = permutedInformation.bottomRightCorner(3, 3);
  const Matrix expectedRetained =
      (Lambda_rr - Lambda_rl * Lambda_ll.inverse() * Lambda_lr).inverse();
  const Matrix permutedMarginalized =
      permuteSymmetricMatrix(marginalized, permutation);
  const Matrix actualRetained =
      permutedMarginalized.topLeftCorner(retainedDim, retainedDim);
  const Matrix actualCross =
      permutedMarginalized.topRightCorner(retainedDim, 3);
  Matrix expectedPermuted = Matrix::Zero(P0.rows(), P0.cols());
  expectedPermuted.topLeftCorner(retainedDim, retainedDim) = expectedRetained;
  expectedPermuted.bottomRightCorner(3, 3) =
      I_3x3 * (params.footholdInitSigma * params.footholdInitSigma);
  const Matrix expectedMarginalized =
      permutation * expectedPermuted * permutation.transpose();

  EXPECT(assert_equal(expectedRetained, actualRetained, 1e-8));
  EXPECT_DOUBLES_EQUAL(0.0, actualCross.norm(), 1e-12);
  EXPECT_DOUBLES_EQUAL(0.0, actualCross.transpose().norm(), 1e-12);
  EXPECT(assert_equal(expectedMarginalized, marginalized, 1e-8));
}

/* ************************************************************************* */
TEST(LeggedEstimator, TouchdownFlagReinitializesFootWithoutIntermediateSwing) {
  const LeggedEstimatorParams params = makeParams();
  LeggedInvariantEKF estimator(identityState(), oneFoot(Vector3::Zero()),
                               initialCovariance(1), params);

  estimator.processContacts(
      {ContactMeasurement{0, Vector3(1.0, 0.0, 0.0), true}});
  const Point3 firstFoothold =
      Point3(footholdsFromEstimate(estimator.estimate()).col(0));

  estimator.processContacts(
      {ContactMeasurement{0, Vector3(2.0, 0.0, 0.0), true}});
  const Point3 secondFoothold =
      Point3(footholdsFromEstimate(estimator.estimate()).col(0));

  EXPECT(assert_equal(Point3(1.0, 0.0, 0.0), firstFoothold, 1e-6));
  EXPECT(assert_equal(Point3(2.0, 0.0, 0.0), secondFoothold, 1e-6));
}

/* ************************************************************************* */
TEST(LeggedEstimator, HeightPriorChangesInvariantFilterContactUpdate) {
  LeggedEstimatorParams params = makeParams();
  params.body_P_imu = Pose3(Rot3(), Point3(0.30, 0.0, 0.15));
  params.footholdInitSigma = 0.5;
  params.contactCovariance =
      (Vector3(0.03 * 0.03, 0.03 * 0.03, 0.02 * 0.02)).asDiagonal();
  params.heightPriorSigma = 0.02;

  const NavState navState0(Rot3(), Point3(0.0, 0.0, 0.12),
                           Vector3(0.0, 0.0, -0.12));
  Matrix P0 = Matrix::Zero(12, 12);
  P0.block(0, 0, 9, 9) = I_9x9 * 1e-3;
  P0.block(9, 9, 3, 3) =
      I_3x3 * (params.footholdInitSigma * params.footholdInitSigma);

  const ContactMeasurement contact{0, Vector3::Zero(), false};

  LeggedInvariantEKF noPrior(navState0, oneFoot(Vector3::Zero()), P0, params,
                             {"foot"});
  noPrior.turnHeightPriorOff();
  noPrior.processContacts({contact});
  const double noPriorFootHeight =
      Point3(footholdsFromEstimate(noPrior.estimate()).col(0)).z();

  LeggedInvariantEKF withPrior(navState0, oneFoot(Vector3::Zero()), P0, params,
                               {"foot"});
  withPrior.turnHeightPriorOn(10.0);
  withPrior.processContacts({contact});
  const double withPriorFootHeight =
      Point3(footholdsFromEstimate(withPrior.estimate()).col(0)).z();

  CHECK(std::abs(withPriorFootHeight - noPriorFootHeight) > 1.0);
}

/* ************************************************************************* */
TEST(LeggedEstimator, ExtendedPoseSequentialAndGraphUpdatesAgree) {
  const LeggedEstimatorParams params = makeParams();
  const Matrix P0 = initialCovariance(1);
  LeggedInvariantEKF sequential(identityState(),
                                oneFoot(Vector3(1.0, 0.0, 0.0)), P0, params);
  LeggedInvariantIEKF graph(identityState(), oneFoot(Vector3(1.0, 0.0, 0.0)),
                            P0, params);

  const ContactMeasurement first{0, Vector3(1.0, 0.0, 0.0)};
  const ContactMeasurement second{0, Vector3(0.9, 0.1, 0.0)};
  sequential.processContacts({first});
  graph.processContacts({first});
  sequential.processContacts({second});
  graph.processContacts({second});

  EXPECT(assert_equal(navStateFromEstimate(sequential.estimate()),
                      navStateFromEstimate(graph.estimate()), 3e-3));
  EXPECT(assert_equal(footholdsFromEstimate(sequential.estimate()),
                      footholdsFromEstimate(graph.estimate()), 3e-3));
  EXPECT(assert_equal(sequential.covariance(), graph.covariance(), 5e-4));
}

/* ************************************************************************* */
TEST(LeggedEstimator, FixedLagEstimatorReplacesContactEpisode) {
  const LeggedEstimatorParams params = makeParams();
  LeggedFixedLagSmoother estimator(identityState(), oneFoot(Vector3::Zero()),
                                   I_9x9 * 1e-3, params, 0.15);

  estimator.processContacts({ContactMeasurement{0, Vector3(1.0, 0.0, 0.0)}});
  EXPECT(assert_equal(
      Point3(1.0, 0.0, 0.0),
      Point3(footholdsFromEstimate(estimator.estimate()).col(0)), 5e-3));

  const Vector3 stationarySpecificForce(0.0, 0.0, 9.81);
  estimator.predict(Vector3::Zero(), stationarySpecificForce, 0.1);
  const NavState expectedDeadReckoned = NavStateImuEKF::Dynamics(
      params.preintegrationParams->n_gravity, identityState(), Vector3::Zero(),
      stationarySpecificForce, 0.1);
  EXPECT(assert_equal(expectedDeadReckoned,
                      navStateFromEstimate(estimator.estimate()), 1e-6));
  estimator.processContacts({ContactMeasurement{0, Vector3(1.0, 0.0, 0.0)}});

  estimator.predict(Vector3::Zero(), stationarySpecificForce, 0.1);
  estimator.processContacts({});
  EXPECT_DOUBLES_EQUAL(
      0.0, Point3(footholdsFromEstimate(estimator.estimate()).col(0)).norm(),
      1e-12);

  estimator.predict(Vector3::Zero(), stationarySpecificForce, 0.1);
  estimator.processContacts({ContactMeasurement{0, Vector3(2.0, 0.0, 0.0)}});

  EXPECT(assert_equal(
      Point3(2.0, 0.0, 0.0),
      Point3(footholdsFromEstimate(estimator.estimate()).col(0)), 5e-3));
}

/* ************************************************************************* */
TEST(LeggedEstimator, CombinedFixedLagEstimatorReplacesContactEpisode) {
  LeggedEstimatorParams params = makeParams();
  params.imuBias = imuBias::ConstantBias(Vector3(0.02, -0.01, 0.03),
                                         Vector3(0.001, -0.002, 0.0015));
  LeggedCombinedFixedLagSmoother estimator(
      identityState(), oneFoot(Vector3::Zero()), I_9x9 * 1e-3, params, 0.15);

  estimator.processContacts({ContactMeasurement{0, Vector3(1.0, 0.0, 0.0)}});
  EXPECT(assert_equal(
      Point3(1.0, 0.0, 0.0),
      Point3(footholdsFromEstimate(estimator.estimate()).col(0)), 5e-3));

  const Vector3 stationarySpecificForce(0.0, 0.0, 9.81);
  estimator.predict(Vector3::Zero(), stationarySpecificForce, 0.1);
  CHECK(navStateFromEstimate(estimator.estimate()).position().allFinite());
  estimator.processContacts({ContactMeasurement{0, Vector3(1.0, 0.0, 0.0)}});

  estimator.predict(Vector3::Zero(), stationarySpecificForce, 0.1);
  estimator.processContacts({});
  EXPECT_DOUBLES_EQUAL(
      0.0, Point3(footholdsFromEstimate(estimator.estimate()).col(0)).norm(),
      1e-12);

  estimator.predict(Vector3::Zero(), stationarySpecificForce, 0.1);
  estimator.processContacts({ContactMeasurement{0, Vector3(2.0, 0.0, 0.0)}});

  EXPECT(assert_equal(
      Point3(2.0, 0.0, 0.0),
      Point3(footholdsFromEstimate(estimator.estimate()).col(0)), 5e-3));
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
