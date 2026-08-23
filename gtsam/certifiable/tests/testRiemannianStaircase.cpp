/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    testRiemannianStaircase.cpp
 * @brief   Unit tests for the Burer-Monteiro Riemannian Staircase solver.
 * @author  Zhexin Xu
 */

#include <CppUnitLite/TestHarness.h>
#include <gtsam/base/TestableAssertions.h>
#include <gtsam/certifiable/RiemannianStaircaseOptimizer.h>
#include <gtsam/constrained/AugmentedLagrangianOptimizer.h>
#include <gtsam/constrained/LinearConstraint.h>
#include <gtsam/constrained/QcqpProblem.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/geometry/Rot2.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/FastSync.h>
#include <gtsam/slam/FrobeniusFactor.h>

#include <Eigen/Eigenvalues>

#include <array>
#include <cmath>
#include <limits>

using namespace gtsam;

// MSVC doesn't define M_PI by default; use a local constant for portability.
constexpr double kPi = 3.14159265358979323846;

/* ************************************************************************* */
namespace RingFixture {

NonlinearFactorGraph RingGraph(size_t numPoses, double delta) {
  NonlinearFactorGraph graph;
  auto noise = noiseModel::Isotropic::Sigma(1, 1.0);
  for (size_t i = 0; i < numPoses; ++i) {
    graph.emplace_shared<FrobeniusBetweenFactor<Rot2>>(
        Symbol('x', i), Symbol('x', (i + 1) % numPoses),
        Rot2::fromAngle(delta), noise);
  }
  return graph;
}

// Matrix-form (D=2) fixture: each value is a 2x2 row-orthonormal matrix.
Values RingQcqpValuesD2(size_t numPoses, double delta, double perturbation) {
  Values values;
  for (size_t i = 0; i < numPoses; ++i) {
    InsertQcqpValue<Rot2, 2>(
        Symbol('x', i),
        Rot2::fromAngle(i * delta + perturbation * static_cast<double>(i)),
        &values);
  }
  return values;
}

// Shared BCL tuning used by most tests. The tighter feasibility tolerance
// makes the ring fixture suitable for certificate checks.
AugmentedLagrangianParams::shared_ptr DefaultAlmParams() {
  auto p = std::make_shared<AugmentedLagrangianParams>();
  p->maxIterations = 50;
  p->absoluteViolationTolerance = 1e-8;
  return p;
}

}  // namespace RingFixture

/* ************************************************************************* */
namespace CachedQcqpFixture {

class CountingFrobeniusBetweenFactor : public FrobeniusBetweenFactor<Rot2> {
 public:
  using Base = FrobeniusBetweenFactor<Rot2>;

  CountingFrobeniusBetweenFactor(Key key1, Key key2, const Rot2& measured,
                                 const SharedNoiseModel& noise,
                                 const std::shared_ptr<size_t>& buildCount)
      : Base(key1, key2, measured, noise), buildCount_(buildCount) {}

  void qcqpFactors(NonlinearFactorGraph* costs,
                   NonlinearEqualityConstraints* constraints,
                   size_t columnDimension) const override {
    ++*buildCount_;
    Base::qcqpFactors(costs, constraints, columnDimension);
  }

 private:
  std::shared_ptr<size_t> buildCount_;
};

// The constructor's pMin QCQP is reused, and each subsequently visited rank
// lowers every source factor exactly once.
TEST(RiemannianStaircase, ReusesCachedPMinQcqp) {
  constexpr size_t kNumPoses = 5;
  constexpr double kDelta = 2.0 * kPi / static_cast<double>(kNumPoses);
  const auto noise = noiseModel::Isotropic::Sigma(1, 1.0);
  const auto buildCount = std::make_shared<size_t>(0);
  NonlinearFactorGraph graph;
  for (size_t i = 0; i < kNumPoses; ++i) {
    graph.emplace_shared<CountingFrobeniusBetweenFactor>(
        Symbol('x', i), Symbol('x', (i + 1) % kNumPoses),
        Rot2::fromAngle(kDelta), noise, buildCount);
  }

  RiemannianStaircaseParams params;
  params.pMin = 2;
  params.pMax = 3;
  params.eta = -1.0;
  params.almParams = RingFixture::DefaultAlmParams();
  const Values initial = RingFixture::RingQcqpValuesD2(kNumPoses, kDelta, 0.01);

  const RiemannianStaircaseOptimizer optimizer(graph, initial, params);
  EXPECT_LONGS_EQUAL(kNumPoses, *buildCount);
  const auto result = optimizer.optimize();

  EXPECT_LONGS_EQUAL(2, result.ranksVisited.size());
  EXPECT_LONGS_EQUAL(kNumPoses * result.ranksVisited.size(), *buildCount);
  EXPECT_LONGS_EQUAL(result.ranksVisited.size(),
                     result.qcqpBuildTimePerLevel.size());
  EXPECT_LONGS_EQUAL(result.ranksVisited.size(), result.nlpTimePerLevel.size());
  EXPECT_LONGS_EQUAL(result.ranksVisited.size(),
                     result.verifyTimePerLevel.size());
  EXPECT_LONGS_EQUAL(result.ranksVisited.size(), result.costPerLevel.size());
  EXPECT_LONGS_EQUAL(result.ranksVisited.size(),
                     result.minEigenvaluePerLevel.size());
  EXPECT_LONGS_EQUAL(result.ranksVisited.size(),
                     result.stationarityPerLevel.size());

  double accountedTime = 0.0;
  for (size_t i = 0; i < result.ranksVisited.size(); ++i) {
    accountedTime += result.qcqpBuildTimePerLevel[i] +
                     result.nlpTimePerLevel[i] + result.verifyTimePerLevel[i];
  }
  EXPECT(result.totalTime >= accountedTime);
}

}  // namespace CachedQcqpFixture

/* ************************************************************************* */
namespace FastSyncRot3Fixture {

Values noncontiguousTriangleRotations() {
  Values rotations;
  rotations.insert(Symbol('x', 2), Rot3::RzRyRx(0.1, -0.2, 0.3));
  rotations.insert(Symbol('x', 7), Rot3::RzRyRx(-0.2, 0.4, -0.1));
  rotations.insert(Symbol('x', 11), Rot3::RzRyRx(0.3, 0.1, -0.4));
  return rotations;
}

NonlinearFactorGraph noncontiguousTriangle() {
  const auto noise = noiseModel::Isotropic::Sigma(Rot3::dimension, 1.0);
  const Key x2 = Symbol('x', 2), x7 = Symbol('x', 7), x11 = Symbol('x', 11);
  const Values rotations = noncontiguousTriangleRotations();
  const std::array<std::pair<Key, Key>, 3> edges{
      std::make_pair(x2, x7), std::make_pair(x7, x11), std::make_pair(x11, x2)};
  NonlinearFactorGraph graph;
  for (const auto& [key1, key2] : edges) {
    const Rot3 measured =
        rotations.at<Rot3>(key1).between(rotations.at<Rot3>(key2));
    graph.emplace_shared<FrobeniusBetweenFactor<Rot3>>(key1, key2, measured,
                                                       noise);
  }
  return graph;
}

// FAST-Sync values converted through the public QCQP API certify with the
// generic BM/ALM staircase, without a rotation-specific solver path.
TEST(RiemannianStaircase, FastSyncRot3QcqpInitialization) {
  const NonlinearFactorGraph graph = noncontiguousTriangle();
  const Values rotations = fastSync<Rot3>(graph);
  Values initial;
  for (const auto& [key, rotation] : rotations.extract<Rot3>()) {
    InsertQcqpValue<Rot3, 3>(key, rotation, &initial);
  }

  RiemannianStaircaseParams params;
  params.pMin = 3;
  params.pMax = 3;
  params.eta = 1e-4;
  params.almParams = RingFixture::DefaultAlmParams();

  const auto result =
      RiemannianStaircaseOptimizer(graph, initial, params).optimize();

  EXPECT(result.certified);
  EXPECT_LONGS_EQUAL(3, static_cast<long>(result.finalRank));
  EXPECT_LONGS_EQUAL(3, static_cast<long>(result.roundedValues().size()));
  EXPECT(result.costPerLevel.front() < 1e-8);
}

}  // namespace FastSyncRot3Fixture

/* ************************************************************************* */
// At an ALM-converged Ystar, the certificate S is PSD and S * Y ≈ 0
// (1st-order KKT for the BM problem at eq. (16)).
TEST(RiemannianStaircase, CertificateMatchesKKT) {
  using namespace RingFixture;
  constexpr size_t N = 5;
  constexpr double delta = 2.0 * kPi / static_cast<double>(N);
  constexpr size_t K = 2;

  const NonlinearFactorGraph graph = RingGraph(N, delta);
  const QcqpProblem qcqp(graph, K);
  const Values initialValues = RingQcqpValuesD2(N, delta, 0.01);

  auto params = DefaultAlmParams();
  params->storeOptProgress = true;
  params->maxIterations = 100;
  params->absoluteViolationTolerance = 1e-10;
  params->relativeViolationTolerance = 1e-10;
  params->absoluteCostTolerance = 1e-12;
  params->relativeCostTolerance = 1e-12;

  AugmentedLagrangianOptimizer alm(qcqp, initialValues, params);
  const Values Ystar = alm.optimize();
  EXPECT(!alm.progress().empty());

  const auto& lambdaEq = alm.progress().back().lambdaEq;
  EXPECT_LONGS_EQUAL(static_cast<long>(qcqp.eConstraints().size()),
                     static_cast<long>(lambdaEq.size()));

  const auto layout = RiemannianStaircaseOptimizer::Layout::From(Ystar);
  LONGS_EQUAL(N * 2, layout.totalDim);

  const Eigen::SparseMatrix<double> S =
      RiemannianStaircaseOptimizer::buildCertificate(qcqp, layout, lambdaEq);
  LONGS_EQUAL(static_cast<long>(layout.totalDim), S.rows());
  LONGS_EQUAL(static_cast<long>(layout.totalDim), S.cols());

  // (a) SDP dual cert PSD up to tolerance.
  const Matrix Sdense(S);
  Eigen::SelfAdjointEigenSolver<Matrix> es(Sdense);
  const double lambdaMin = es.eigenvalues().minCoeff();
  EXPECT(lambdaMin > -1e-3);

  // (b) 1st-order KKT stationarity: ||S * Y|| / ||Y|| ~= 0.
  // S is exactly the matrix assembled above; no rebuild needed.
  const Matrix Ymat = layout.stack(Ystar);
  const Matrix SY = S * Ymat;
  const double stationarityRatio = SY.norm() / Ymat.norm();
  EXPECT(stationarityRatio < 1e-3);
}

/* ************************************************************************* */
// liftWithDescent widens each value by one column, leaving the first
// p columns unchanged and writing alpha * v into the new column.
TEST(RiemannianStaircase, LiftWithDescentShape) {
  using namespace RingFixture;
  constexpr size_t N = 5;
  constexpr double delta = 2.0 * kPi / static_cast<double>(N);

  const Values Ystar = RingQcqpValuesD2(N, delta, 0.0);
  const auto layout = RiemannianStaircaseOptimizer::Layout::From(Ystar);
  LONGS_EQUAL(N * 2, layout.totalDim);

  Vector v =
      Vector::LinSpaced(static_cast<int>(layout.totalDim), 1.0, layout.totalDim);
  v.normalize();

  const double alpha = 1e-2;
  const Values Ynext =
      RiemannianStaircaseOptimizer::liftWithDescent(Ystar, v, layout, alpha);

  EXPECT_LONGS_EQUAL(static_cast<long>(Ystar.size()),
                     static_cast<long>(Ynext.size()));
  for (const auto& [key, Yi] : Ystar.extract<Matrix>()) {
    const Matrix YiNew = Ynext.at<Matrix>(key);
    EXPECT_LONGS_EQUAL(Yi.rows(), YiNew.rows());
    EXPECT_LONGS_EQUAL(Yi.cols() + 1, YiNew.cols());
    EXPECT(assert_equal(Matrix(Yi), Matrix(YiNew.leftCols(Yi.cols())), 1e-12));

    const auto& slice = layout.sliceOf(key);
    const Matrix expectedCol = alpha * v.segment(slice.offset, slice.rowDim);
    EXPECT(assert_equal(expectedCol, Matrix(YiNew.rightCols(1)), 1e-12));
  }
}

/* ************************************************************************* */
// Constraint violation at the lifted point grows like O(alpha^2):
// feasible Ystar at rank p + h(Y) = trace(Y' A Y) - b = 0,
// lifting to [Ystar | alpha v] gives h = alpha^2 v' A v.
TEST(RiemannianStaircase, LiftWithDescentViolationIsQuadratic) {
  using namespace RingFixture;
  constexpr size_t N = 5;
  constexpr double delta = 2.0 * kPi / static_cast<double>(N);

  const NonlinearFactorGraph graph = RingGraph(N, delta);
  const Values Ystar = RingQcqpValuesD2(N, delta, 0.0);
  const auto layout = RiemannianStaircaseOptimizer::Layout::From(Ystar);

  const QcqpProblem qcqpBase(graph, 2);
  EXPECT_DOUBLES_EQUAL(0.0, qcqpBase.eConstraints().violationNorm(Ystar),
                       1e-12);

  Vector v = Vector::LinSpaced(static_cast<int>(layout.totalDim), 1.0,
                               layout.totalDim);
  v.normalize();

  const QcqpProblem qcqpLifted(graph, 3);
  const double alphaSmall = 1e-3;
  const double alphaLarge = 2e-3;
  const Values Yh = RiemannianStaircaseOptimizer::liftWithDescent(
      Ystar, v, layout, alphaSmall);
  const Values YH = RiemannianStaircaseOptimizer::liftWithDescent(
      Ystar, v, layout, alphaLarge);

  const double vioSmall = qcqpLifted.eConstraints().violationNorm(Yh);
  const double vioLarge = qcqpLifted.eConstraints().violationNorm(YH);
  EXPECT(vioSmall > 0.0);
  // alphaLarge/alphaSmall = 2, so vioLarge/vioSmall should be 4 (O(alpha^2)).
  // 4% tolerance absorbs noise in violationNorm at this alpha regime.
  EXPECT_DOUBLES_EQUAL(4.0, vioLarge / vioSmall, 4e-2);
}

/* ************************************************************************* */
// Both verifier methods reach the same verdict + final rank on an easy ring.
// Precise eigenvalue agreement is in VerifyStage2OnNegativeEigenvalue.
TEST(RiemannianStaircase, BothVerifyMethodsCertifyEasyRing) {
  using namespace RingFixture;
  constexpr size_t N = 5;
  constexpr double delta = 2.0 * kPi / static_cast<double>(N);

  const NonlinearFactorGraph graph = RingGraph(N, delta);
  const Values initial = RingQcqpValuesD2(N, delta, /*perturbation=*/0.01);

  RiemannianStaircaseParams paramsS;
  paramsS.pMin = 2;
  paramsS.pMax = 5;
  paramsS.alpha = 1e-2;
  paramsS.eta = 1e-3;
  paramsS.verificationMethod =
      RiemannianStaircaseParams::VerificationMethod::Spectra;
  paramsS.almParams = DefaultAlmParams();
  paramsS.almParams->maxIterations = 100;

  RiemannianStaircaseParams paramsD = paramsS;
  paramsD.verificationMethod =
      RiemannianStaircaseParams::VerificationMethod::DenseEigen;
  paramsD.almParams =
      std::make_shared<AugmentedLagrangianParams>(*paramsS.almParams);

  const auto resultS =
      RiemannianStaircaseOptimizer(graph, initial, paramsS).optimize();
  const auto resultD =
      RiemannianStaircaseOptimizer(graph, initial, paramsD).optimize();

  EXPECT(resultS.certified == resultD.certified);
  EXPECT_LONGS_EQUAL(static_cast<long>(resultS.finalRank),
                     static_cast<long>(resultD.finalRank));
  EXPECT(resultS.certified);
  // Spectra certifies through Cholesky, which proves only lambda_min >= -eta,
  // so it reports exactly that bound rather than an estimate. DenseEigen has
  // the actual eigenvalue and reports it.
  EXPECT_DOUBLES_EQUAL(-paramsS.eta, resultS.minEigenvalue, 1e-15);
  EXPECT(resultD.minEigenvalue >= -paramsD.eta);
}

/* ************************************************************************* */
// Easy case: well-initialized 5-Rot2 ring. The relaxation is tight; the
// certificate should pass at the starting rank or after at most one lift.
TEST(RiemannianStaircase, Rot2RingEasy) {
  using namespace RingFixture;
  constexpr size_t N = 5;
  constexpr double delta = 2.0 * kPi / static_cast<double>(N);

  const NonlinearFactorGraph graph = RingGraph(N, delta);
  const Values initial = RingQcqpValuesD2(N, delta, /*perturbation=*/0.01);

  RiemannianStaircaseParams params;
  params.pMin = 2;
  params.pMax = 5;
  params.alpha = 1e-2;
  params.eta = 1e-3;
  params.almParams = DefaultAlmParams();
  params.almParams->maxIterations = 100;
  params.almParams->absoluteCostTolerance = 1e-10;
  params.almParams->relativeCostTolerance = 1e-10;

  const auto result =
      RiemannianStaircaseOptimizer(graph, initial, params).optimize();

  EXPECT(result.certified);
  EXPECT(result.finalRank <= 3);
  EXPECT(result.minEigenvalue >= -params.eta);
  EXPECT(result.hasRoundedSolution());

  const Values roundedValues = result.roundedValues();
  EXPECT_LONGS_EQUAL(static_cast<long>(N),
                     static_cast<long>(roundedValues.size()));
  for (const auto& [key, roundedMatrix] :
       roundedValues.extract<Matrix>()) {
    EXPECT_LONGS_EQUAL(2, roundedMatrix.rows());
    EXPECT_LONGS_EQUAL(2, roundedMatrix.cols());
  }
}

/* ************************************************************************* */
// Layout::From walks variables in sorted-Key order with contiguous offsets
// and totalDim equal to the sum of row dims.
TEST(Layout, OrderAndTotalDim) {
  Values values;
  const Matrix zero4x3 = Matrix::Zero(4, 3);
  values.insert(Symbol('x', 2), zero4x3);
  values.insert(Symbol('x', 0), zero4x3);
  values.insert(Symbol('x', 1), zero4x3);

  const auto layout = RiemannianStaircaseOptimizer::Layout::From(values);
  LONGS_EQUAL(12, layout.totalDim);
  LONGS_EQUAL(3, layout.size());
  LONGS_EQUAL(0, layout.offsetOf(Symbol('x', 0)));
  LONGS_EQUAL(4, layout.offsetOf(Symbol('x', 1)));
  LONGS_EQUAL(8, layout.offsetOf(Symbol('x', 2)));
}

/* ************************************************************************* */
// Layout::From handles heterogeneous row dims (Rot2 + Rot3 shapes mixed).
TEST(Layout, HeterogeneousRowDims) {
  Values values;
  values.insert(Symbol('x', 0), Matrix(Matrix::Zero(2, 2)));
  values.insert(Symbol('x', 1), Matrix(Matrix::Zero(3, 2)));
  values.insert(Symbol('x', 2), Matrix(Matrix::Zero(2, 2)));

  const auto layout = RiemannianStaircaseOptimizer::Layout::From(values);
  LONGS_EQUAL(2 + 3 + 2, layout.totalDim);
  LONGS_EQUAL(0, layout.offsetOf(Symbol('x', 0)));
  LONGS_EQUAL(2, layout.offsetOf(Symbol('x', 1)));
  LONGS_EQUAL(5, layout.offsetOf(Symbol('x', 2)));
  LONGS_EQUAL(3, layout.maxRowDim());
}

/* ************************************************************************* */
// stack/unstack round-trip is exact.
TEST(Layout, StackUnstackRoundTrip) {
  Values values;
  const Matrix A{{1, 2, 3}, {4, 5, 6}};
  const Matrix B{{7, 8, 9}, {10, 11, 12}, {13, 14, 15}};
  const Matrix C{{16, 17, 18}, {19, 20, 21}};
  values.insert(Symbol('x', 0), A);
  values.insert(Symbol('x', 1), B);
  values.insert(Symbol('x', 2), C);

  const auto layout = RiemannianStaircaseOptimizer::Layout::From(values);
  const Matrix Y = layout.stack(values);
  LONGS_EQUAL(7, Y.rows());
  LONGS_EQUAL(3, Y.cols());

  const Values back = layout.unstack(Y);
  EXPECT(assert_equal(A, back.at<Matrix>(Symbol('x', 0)), 1e-15));
  EXPECT(assert_equal(B, back.at<Matrix>(Symbol('x', 1)), 1e-15));
  EXPECT(assert_equal(C, back.at<Matrix>(Symbol('x', 2)), 1e-15));
}

/* ************************************************************************* */
// Non-QCQP-representable factors are rejected at construction (probe build
// at pMin throws).
TEST(RiemannianStaircase, RejectsNonQcqpFactor) {
  NonlinearFactorGraph graph;
  // BetweenFactor<Pose2> doesn't override qcqpFactors — the base throws.
  graph.emplace_shared<BetweenFactor<Pose2>>(
      Symbol('x', 0), Symbol('x', 1), Pose2(),
      noiseModel::Isotropic::Sigma(3, 1.0));
  Values values;
  InsertQcqpValue<Rot2, 2>(Symbol('x', 0), Rot2::fromAngle(0.0), &values);
  InsertQcqpValue<Rot2, 2>(Symbol('x', 1), Rot2::fromAngle(0.0), &values);

  CHECK_EXCEPTION(
      RiemannianStaircaseOptimizer(graph, values, RiemannianStaircaseParams{}),
      std::invalid_argument);
}

/* ************************************************************************* */
// pMin > pMax is rejected at construction.
TEST(RiemannianStaircase, RejectsInvertedPMinPMax) {
  NonlinearFactorGraph graph;
  auto noise = noiseModel::Isotropic::Sigma(1, 1.0);
  graph.emplace_shared<FrobeniusBetweenFactor<Rot2>>(
      Symbol('x', 0), Symbol('x', 1), Rot2::fromAngle(0.3), noise);
  Values values;
  InsertQcqpValue<Rot2, 2>(Symbol('x', 0), Rot2::fromAngle(0.0), &values);
  InsertQcqpValue<Rot2, 2>(Symbol('x', 1), Rot2::fromAngle(0.0), &values);

  RiemannianStaircaseParams params;
  params.pMin = 3;
  params.pMax = 2;
  CHECK_EXCEPTION(
      RiemannianStaircaseOptimizer(graph, values, params),
      std::invalid_argument);
}

/* ************************************************************************* */
// truncateToRankD rejects d < 1 and d > stacked-matrix column count.
TEST(RiemannianStaircase, TruncateToRankDRejectsBadD) {
  Values values;
  values.insert(Symbol('x', 0), Matrix(Matrix::Zero(2, 4)));
  values.insert(Symbol('x', 1), Matrix(Matrix::Zero(2, 4)));
  const auto layout = RiemannianStaircaseOptimizer::Layout::From(values);

  CHECK_EXCEPTION(
      RiemannianStaircaseOptimizer::truncateToRankD(values, layout, /*d=*/0),
      std::invalid_argument);
  CHECK_EXCEPTION(
      RiemannianStaircaseOptimizer::truncateToRankD(values, layout, /*d=*/5),
      std::invalid_argument);
}

// truncateToRankD supports rank-one generic QCQPs.
TEST(RiemannianStaircase, TruncateToRankDAllowsRankOne) {
  Values values;
  values.insert(Symbol('x', 0), (Matrix(1, 2) << 3.0, 4.0).finished());
  values.insert(Symbol('x', 1), (Matrix(1, 2) << 0.0, 5.0).finished());
  const auto layout = RiemannianStaircaseOptimizer::Layout::From(values);

  const auto rounded =
      RiemannianStaircaseOptimizer::truncateToRankD(values, layout, /*d=*/1);
  EXPECT_LONGS_EQUAL(2, rounded.Yd.rows());
  EXPECT_LONGS_EQUAL(1, rounded.Yd.cols());
}

/* ************************************************************************* */
// optimize() zero-pads initial values narrower than pMin and still certifies.
// XX' is invariant under zero-column padding, so feasibility is preserved.
TEST(RiemannianStaircase, AutoPadsNarrowInitialValues) {
  using namespace RingFixture;
  constexpr size_t N = 5;
  constexpr double delta = 2.0 * kPi / static_cast<double>(N);

  const NonlinearFactorGraph graph = RingGraph(N, delta);
  // Initial values stored as 2x2 (rank-2 form).
  const Values initial = RingQcqpValuesD2(N, delta, /*perturbation=*/0.01);

  // Ask the staircase to start at rank 3 — wider than the supplied 2x2.
  RiemannianStaircaseParams params;
  params.pMin = 3;
  params.pMax = 4;
  params.almParams = DefaultAlmParams();
  params.almParams->maxIterations = 100;

  const auto result =
      RiemannianStaircaseOptimizer(graph, initial, params).optimize();

  // Auto-pad to rank 3 must not crash and the run must certify on the ring.
  EXPECT(result.certified);
  EXPECT(result.finalRank >= 3 && result.finalRank <= params.pMax);
}

/* ************************************************************************* */
// padInitialValues zero-pads narrower entries to pMin columns; matching
// entries pass through; wider entries throw.
TEST(RiemannianStaircase, PadInitialValuesWidens) {
  Values Y;
  Y.insert(Symbol('x', 0), Matrix(Matrix::Identity(2, 2)));
  Y.insert(Symbol('x', 1), Matrix(Matrix::Identity(3, 3)));

  const Values padded =
      RiemannianStaircaseOptimizer::padInitialValues(Y, /*pMin=*/4);

  const Matrix M0 = padded.at<Matrix>(Symbol('x', 0));
  EXPECT_LONGS_EQUAL(2, M0.rows());
  EXPECT_LONGS_EQUAL(4, M0.cols());
  EXPECT(assert_equal(Matrix(Matrix::Identity(2, 2)),
                      Matrix(M0.leftCols<2>()), 1e-15));
  EXPECT(assert_equal(Matrix(Matrix::Zero(2, 2)),
                      Matrix(M0.rightCols<2>()), 1e-15));

  const Matrix M1 = padded.at<Matrix>(Symbol('x', 1));
  EXPECT_LONGS_EQUAL(3, M1.rows());
  EXPECT_LONGS_EQUAL(4, M1.cols());
  EXPECT(assert_equal(Matrix(Matrix::Identity(3, 3)),
                      Matrix(M1.leftCols<3>()), 1e-15));
  EXPECT(assert_equal(Matrix(Matrix::Zero(3, 1)),
                      Matrix(M1.rightCols<1>()), 1e-15));
}

// Rejects initial values that are already wider than the requested rank.
TEST(RiemannianStaircase, PadInitialValuesRejectsTooWide) {
  Values Y;
  Y.insert(Symbol('x', 0), Matrix(Matrix::Zero(2, 5)));
  CHECK_EXCEPTION(
      RiemannianStaircaseOptimizer::padInitialValues(Y, /*pMin=*/3),
      std::invalid_argument);
}

// Rejects wrapper-style negative ranks converted to an unsigned size_t.
TEST(RiemannianStaircase, PadInitialValuesRejectsUnrepresentableRank) {
  Values Y;
  Y.insert(Symbol('x', 0), Matrix(Matrix::Identity(2, 2)));
  CHECK_EXCEPTION(
      RiemannianStaircaseOptimizer::padInitialValues(
          Y, std::numeric_limits<size_t>::max()),
      std::invalid_argument);
}

/* ************************************************************************* */
// runLocalSolver returns lambdaEq aligned 1:1 with qcqp.eConstraints()
// and a Y conforming to the input layout.
TEST(RiemannianStaircase, RunLocalSolverAlignsLambdaEq) {
  using namespace RingFixture;
  constexpr size_t N = 5;
  constexpr double delta = 2.0 * kPi / static_cast<double>(N);

  const NonlinearFactorGraph graph = RingGraph(N, delta);
  const QcqpProblem qcqp(graph, 2);
  const Values initial = RingQcqpValuesD2(N, delta, 0.01);

  const auto inner =
      RiemannianStaircaseOptimizer::runLocalSolver(qcqp, initial,
                                                    DefaultAlmParams());

  EXPECT_LONGS_EQUAL(static_cast<long>(qcqp.eConstraints().size()),
                     static_cast<long>(inner.lambdaEq.size()));
  const auto layout = RiemannianStaircaseOptimizer::Layout::From(initial);
  EXPECT(layout.conformsTo(inner.Y));
}

/* ************************************************************************* */
// buildDataMatrix returns a square symmetric matrix of size totalDim, with
// the K=p cost rolled into its 2x2 (rowDim=2) blocks.
TEST(RiemannianStaircase, BuildDataMatrixShape) {
  using namespace RingFixture;
  constexpr size_t N = 3;
  constexpr double delta = 2.0 * kPi / static_cast<double>(N);

  const NonlinearFactorGraph graph = RingGraph(N, delta);
  const QcqpProblem qcqp(graph, 2);
  const Values values = RingQcqpValuesD2(N, delta, 0.0);
  const auto layout = RiemannianStaircaseOptimizer::Layout::From(values);

  const auto Q =
      RiemannianStaircaseOptimizer::buildDataMatrix(qcqp, layout);
  EXPECT_LONGS_EQUAL(static_cast<long>(layout.totalDim), Q.rows());
  EXPECT_LONGS_EQUAL(static_cast<long>(layout.totalDim), Q.cols());

  const Matrix Qdense(Q);
  EXPECT(assert_equal(Qdense, Matrix(Qdense.transpose()), 1e-12));
}

/* ************************************************************************* */
// buildCertificate throws when lambdaEq is shorter than the number of
// quadratic equality constraints.
TEST(RiemannianStaircase, BuildCertificateShortLambdaEqThrows) {
  using namespace RingFixture;
  const NonlinearFactorGraph graph = RingGraph(3, 2.0 * kPi / 3.0);
  const QcqpProblem qcqp(graph, 2);
  const Values values = RingQcqpValuesD2(3, 2.0 * kPi / 3.0, 0.0);
  const auto layout = RiemannianStaircaseOptimizer::Layout::From(values);

  const std::vector<Vector> empty;
  CHECK_EXCEPTION(
      RiemannianStaircaseOptimizer::buildCertificate(qcqp, layout, empty),
      std::runtime_error);
}

// Certificate construction rejects equality constraints that do not have a
// quadratic matrix representation.
TEST(RiemannianStaircase, BuildMultiplierMatrixRejectsLinearEquality) {
  const Key x0 = Symbol('x', 0);
  QcqpProblem qcqp;
  qcqp.addConstraint(LinearConstraint::Equal(
      JacobianFactor(x0, Matrix::Identity(1, 1), Vector1(0.0))));

  Values values;
  values.insert(x0, Matrix(Matrix::Zero(1, 1)));
  const auto layout = RiemannianStaircaseOptimizer::Layout::From(values);
  const std::vector<Vector> lambdaEq{Vector1(0.0)};

  CHECK_EXCEPTION(
      RiemannianStaircaseOptimizer::buildMultiplierMatrix(qcqp, layout,
                                                          lambdaEq),
      std::runtime_error);
}

/* ************************************************************************* */
// buildMultiplierMatrix throws when the constraint's A size doesn't match the
// per-key rowDim recorded in the layout.
TEST(RiemannianStaircase, BuildMultiplierMatrixRejectsASizeMismatch) {
  using namespace RingFixture;
  constexpr size_t N = 3;
  constexpr double delta = 2.0 * kPi / static_cast<double>(N);

  // QCQP built from a Rot2 graph — each equality constraint's A is 2 × 2.
  const NonlinearFactorGraph graph = RingGraph(N, delta);
  const QcqpProblem qcqp(graph, /*K=*/2);

  // But supply a layout where each key has rowDim = 3, mismatching A.
  Values badValues;
  for (size_t i = 0; i < N; ++i) {
    badValues.insert(Symbol('x', i), Matrix(Matrix::Zero(3, 2)));
  }
  const auto badLayout =
      RiemannianStaircaseOptimizer::Layout::From(badValues);

  std::vector<Vector> lambdaEq(qcqp.eConstraints().size(),
                               Vector::Constant(1, 1.0));
  CHECK_EXCEPTION(
      RiemannianStaircaseOptimizer::buildMultiplierMatrix(
          qcqp, badLayout, lambdaEq),
      std::runtime_error);
}

/* ************************************************************************* */
// verify on a synthetic S = diag(1,1,1,1,-0.5) — Cholesky on S + eta*I
// fails, Stage-2 runs. Spectra and DenseEigen agree on the verdict and
// lambda_min to Lanczos tolerance.
TEST(RiemannianStaircase, VerifyStage2OnNegativeEigenvalue) {
  const int n = 5;
  std::vector<Eigen::Triplet<double>> tri = {
      {0, 0, 1.0}, {1, 1, 1.0}, {2, 2, 1.0}, {3, 3, 1.0}, {4, 4, -0.5}};
  Eigen::SparseMatrix<double> S(n, n);
  S.setFromTriplets(tri.begin(), tri.end());
  S.makeCompressed();

  RiemannianStaircaseParams paramsS;
  paramsS.eta = 1e-3;
  paramsS.numLanczosVectors = 5;
  paramsS.maxSpectraIters = 500;
  paramsS.spectraTol = 1e-8;
  paramsS.verificationMethod =
      RiemannianStaircaseParams::VerificationMethod::Spectra;

  RiemannianStaircaseParams paramsD = paramsS;
  paramsD.verificationMethod =
      RiemannianStaircaseParams::VerificationMethod::DenseEigen;

  auto [passS, lamS, vS] =
      RiemannianStaircaseOptimizer::verify(S, paramsS);
  auto [passD, lamD, vD] =
      RiemannianStaircaseOptimizer::verify(S, paramsD);

  EXPECT(!passS);
  EXPECT(!passD);
  EXPECT_DOUBLES_EQUAL(-0.5, lamD, 1e-9);
  EXPECT(std::abs(lamS - lamD) < 1e-4);
  // v_min concentrated on the negative-eigenvalue index, up to sign.
  EXPECT(std::abs(std::abs(vD(4)) - 1.0) < 1e-9);
  EXPECT(std::abs(std::abs(vS(4)) - 1.0) < 1e-3);
}

/* ************************************************************************* */
// pMin == 0 is rejected at construction.
TEST(RiemannianStaircase, RejectsZeroPMin) {
  using namespace RingFixture;
  const NonlinearFactorGraph graph = RingGraph(3, 2.0 * kPi / 3.0);
  const Values values = RingQcqpValuesD2(3, 2.0 * kPi / 3.0, 0.0);
  RiemannianStaircaseParams params;
  params.pMin = 0;
  params.pMax = 2;
  CHECK_EXCEPTION(
      RiemannianStaircaseOptimizer(graph, values, params),
      std::invalid_argument);
}

/* ************************************************************************* */
// Layout::stack throws when entries have inconsistent column counts.
TEST(Layout, StackColumnMismatchThrows) {
  Values values;
  values.insert(Symbol('x', 0), Matrix(Matrix::Zero(2, 3)));
  values.insert(Symbol('x', 1), Matrix(Matrix::Zero(2, 4)));
  const auto layout =
      RiemannianStaircaseOptimizer::Layout::From(values);
  CHECK_EXCEPTION(layout.stack(values), std::invalid_argument);
}

/* ************************************************************************* */
// Layout::unstack throws when the input row count doesn't match totalDim.
TEST(Layout, UnstackRowMismatchThrows) {
  Values values;
  values.insert(Symbol('x', 0), Matrix(Matrix::Zero(2, 3)));
  values.insert(Symbol('x', 1), Matrix(Matrix::Zero(2, 3)));
  const auto layout =
      RiemannianStaircaseOptimizer::Layout::From(values);
  const Matrix wrong = Matrix::Zero(7, 3);  // totalDim is 4
  CHECK_EXCEPTION(layout.unstack(wrong), std::invalid_argument);
}

/* ************************************************************************* */
// liftWithDescent error paths: short vMin and non-conforming layout.
TEST(RiemannianStaircase, LiftWithDescentErrorPaths) {
  using namespace RingFixture;
  constexpr size_t N = 5;
  constexpr double delta = 2.0 * kPi / static_cast<double>(N);
  const Values Ystar = RingQcqpValuesD2(N, delta, 0.0);
  const auto layout = RiemannianStaircaseOptimizer::Layout::From(Ystar);

  const Vector shortV = Vector::Zero(layout.totalDim - 1);
  CHECK_EXCEPTION(
      RiemannianStaircaseOptimizer::liftWithDescent(
          Ystar, shortV, layout, 1e-2),
      std::invalid_argument);

  Values nonConforming;
  nonConforming.insert(Symbol('z', 0), Matrix(Matrix::Zero(3, 2)));
  const Vector v = Vector::Zero(layout.totalDim);
  CHECK_EXCEPTION(
      RiemannianStaircaseOptimizer::liftWithDescent(
          nonConforming, v, layout, 1e-2),
      std::invalid_argument);
}

/* ************************************************************************* */
// pMax exhausted without certification: result.certified == false,
// result.rounded is empty, finalRank == pMax.
TEST(RiemannianStaircase, UncertifiedReturnWhenPMaxTooLow) {
  using namespace RingFixture;
  constexpr size_t N = 5;
  constexpr double delta = 2.0 * kPi / static_cast<double>(N);

  const NonlinearFactorGraph graph = RingGraph(N, delta);
  const Values initial = RingQcqpValuesD2(N, delta, 0.01);

  // pMin = pMax = 2, but force a very strict eta so the verifier rejects
  // any non-negative lambda_min that isn't *strongly* positive — guarantees
  // the Cholesky stage fails and the staircase exhausts pMax with one level.
  RiemannianStaircaseParams params;
  params.pMin = 2;
  params.pMax = 2;
  params.eta = -1.0;  // require lambda_min >= 1.0, impossible at KKT
  params.almParams = DefaultAlmParams();

  const auto result =
      RiemannianStaircaseOptimizer(graph, initial, params).optimize();

  EXPECT(!result.certified);
  EXPECT_LONGS_EQUAL(2, static_cast<long>(result.finalRank));
  EXPECT(!result.rounded.has_value());
  EXPECT(!result.hasRoundedSolution());
  CHECK_EXCEPTION(result.roundedValues(), std::runtime_error);
  EXPECT_LONGS_EQUAL(1, static_cast<long>(result.ranksVisited.size()));
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}

/* ************************************************************************* */
namespace least_squares_multipliers {
using namespace RingFixture;

// A QCQP over one variable whose constraints span the symmetric 2x2 matrices,
// so the closed form below applies.
QcqpProblem OneVariableProblem(const Matrix2& Q, double sigma) {
  const Key key = 0;
  QcqpProblem qcqp;
  qcqp.addCost(QpCost(KeyVector{key},
                      SymmetricBlockMatrix(std::vector<DenseIndex>{2},
                                           Matrix(Q)),
                      3));
  for (const auto& [A, b] : traits<Rot2>::QcqpConstraints<2>()) {
    qcqp.addConstraint(QuadraticConstraint::Equal(key, A, b, sigma));
  }
  return qcqp;
}

// The defining property: lambda minimizes ||S(lambda) Y||, so the
// residual it leaves is orthogonal to every direction the multipliers can move
// in. Checked away from a stationary point, where the residual is nonzero and
// the condition has content.
TEST(RiemannianStaircase, LeastSquaresMultipliersSolveTheNormalEquations) {
  constexpr size_t N = 5;
  constexpr double delta = 2.0 * kPi / static_cast<double>(N);
  const Values Y = RingQcqpValuesD2(N, delta, 0.35);
  const auto layout = RiemannianStaircaseOptimizer::Layout::From(Y);
  const QcqpProblem qcqp(RingGraph(N, delta), 2);

  const auto ls = RiemannianStaircaseOptimizer::leastSquaresMultipliers(
      qcqp, layout, Y);
  const auto S = RiemannianStaircaseOptimizer::buildCertificate(qcqp, layout,
                                                                ls.lambdaEq);
  EXPECT(ls.totalResidual > 1e-6);

  // d/d lambda_m of ||S Y||^2 vanishes: <A_m Y_n, (S Y)_n> = 0 for every m.
  const Matrix SY = Matrix(S) * layout.stack(Y);
  for (const auto& factor : qcqp.eConstraints()) {
    const auto quadratic =
        std::dynamic_pointer_cast<const QuadraticEqualityConstraintFactor>(
            factor);
    const auto& constraint = quadratic->quadraticConstraint();
    const auto& slice = layout.sliceOf(constraint.key());
    const Matrix Yn = layout.stack(Y).middleRows(slice.offset, slice.rowDim);
    const Matrix Gn = SY.middleRows(slice.offset, slice.rowDim);
    EXPECT_DOUBLES_EQUAL(0.0, (constraint.A() * Yn).cwiseProduct(Gn).sum(),
                         1e-8);
  }
}

// The recovered multipliers are the ones that annihilate Y, which is the
// property the certificate depends on.
TEST(RiemannianStaircase, LeastSquaresMultipliersAnnihilateY) {
  constexpr size_t N = 5;
  constexpr double delta = 2.0 * kPi / static_cast<double>(N);
  const Values Y = RingQcqpValuesD2(N, delta, 0.0);
  const auto layout = RiemannianStaircaseOptimizer::Layout::From(Y);
  const QcqpProblem qcqp(RingGraph(N, delta), 2);

  const auto ls = RiemannianStaircaseOptimizer::leastSquaresMultipliers(
      qcqp, layout, Y);
  const auto S = RiemannianStaircaseOptimizer::buildCertificate(qcqp, layout,
                                                                ls.lambdaEq);
  EXPECT_DOUBLES_EQUAL(0.0, (Matrix(S) * layout.stack(Y)).norm(), 1e-8);
  EXPECT_DOUBLES_EQUAL(0.0, ls.totalResidual, 1e-8);
}

// Away from a stationary point the residual is large, which is what lets it
// serve as a gate. A zero-multiplier certificate cannot express this: S would
// collapse to the positive semidefinite data matrix Q.
TEST(RiemannianStaircase, LeastSquaresMultipliersFlagNonStationaryPoint) {
  constexpr size_t N = 4;
  constexpr double delta = 2.0 * kPi / static_cast<double>(N);
  const Values Y = RingQcqpValuesD2(N, delta, 0.6);
  const auto layout = RiemannianStaircaseOptimizer::Layout::From(Y);
  const QcqpProblem qcqp(RingGraph(N, delta), 2);

  const auto ls = RiemannianStaircaseOptimizer::leastSquaresMultipliers(
      qcqp, layout, Y);
  EXPECT(ls.totalResidual > 1e-3);

  // Zero multipliers leave S = Q, which is positive semidefinite by
  // construction and so certifies regardless of the point.
  const std::vector<Vector> zeros(qcqp.eConstraints().size(), Vector1(0.0));
  const auto Sz = RiemannianStaircaseOptimizer::buildCertificate(qcqp, layout,
                                                                 zeros);
  RiemannianStaircaseParams params;
  const auto [certified, lambdaMin, vMin] =
      RiemannianStaircaseOptimizer::verify(Sz, params);
  EXPECT(certified);
}

// Every variable gets a residual entry, including unconstrained ones, whose
// entry is ||(QY)_n|| since no multiplier can reduce it.
TEST(RiemannianStaircase, LeastSquaresMultipliersCoverEveryVariable) {
  constexpr size_t N = 4;
  constexpr double delta = 2.0 * kPi / static_cast<double>(N);
  const Values Y = RingQcqpValuesD2(N, delta, 0.25);
  const auto layout = RiemannianStaircaseOptimizer::Layout::From(Y);
  const QcqpProblem qcqp(RingGraph(N, delta), 2);

  const auto ls = RiemannianStaircaseOptimizer::leastSquaresMultipliers(
      qcqp, layout, Y);
  LONGS_EQUAL(layout.slices.size(), ls.residual.size());
  for (const auto& [key, slice] : layout.slices) {
    EXPECT(ls.residual.count(key) == 1);
  }
  LONGS_EQUAL(qcqp.eConstraints().size(), ls.lambdaEq.size());
}

// A case with a closed-form answer. For a single variable whose constraints
// span the symmetric matrices, KKT stationarity (Q + 2*Lambda) X = 0 with
// X X' = I forces Lambda = -Q/2, hence S = 0 and zero residual, whatever Q is.
TEST(RiemannianStaircase, LeastSquaresMultipliersMatchClosedFormOnOneVariable) {
  const Matrix2 Q{{2.0, -0.7}, {-0.7, 1.5}};
  const QcqpProblem qcqp = OneVariableProblem(Q, 1.0);

  Values Y;
  Y.insert(Key(0), Matrix(traits<Rot2>::QcqpValue<3>(Rot2(0.4))));
  const auto layout = RiemannianStaircaseOptimizer::Layout::From(Y);

  const auto ls = RiemannianStaircaseOptimizer::leastSquaresMultipliers(
      qcqp, layout, Y);
  const auto S = RiemannianStaircaseOptimizer::buildCertificate(qcqp, layout,
                                                                ls.lambdaEq);
  EXPECT_DOUBLES_EQUAL(0.0, Matrix(S).norm(), 1e-9);
  EXPECT_DOUBLES_EQUAL(0.0, ls.totalResidual, 1e-9);
}

// At a converged point the least-squares multipliers reproduce what the
// augmented-Lagrangian solver reports, since both satisfy the same
// stationarity conditions.
TEST(RiemannianStaircase, LeastSquaresMultipliersMatchConvergedAlm) {
  constexpr size_t N = 5;
  constexpr double delta = 2.0 * kPi / static_cast<double>(N);
  const NonlinearFactorGraph graph = RingGraph(N, delta);
  const Values initial = RingQcqpValuesD2(N, delta, 0.15);

  RiemannianStaircaseParams params;
  params.pMin = 2;
  params.pMax = 2;
  params.almParams = DefaultAlmParams();
  const auto result =
      RiemannianStaircaseOptimizer(graph, initial, params).optimize();

  const QcqpProblem qcqp(graph, 2);
  const auto ls = RiemannianStaircaseOptimizer::leastSquaresMultipliers(
      qcqp, result.layout, result.values);

  // A converged point is stationary, so the residual the multipliers leave is
  // essentially zero.
  EXPECT(ls.totalResidual < 1e-4);
}

// Cross-check against a reference computed a completely different way. S is
// affine in lambda, so probing buildCertificate with the zero vector and each
// basis vector recovers the whole least-squares system, which is then solved
// as one dense problem. That route shares no code with the block-wise
// grouping, the per-variable QR, or the whitening conversion inside the
// implementation, so agreement pins all three.
TEST(RiemannianStaircase, LeastSquaresMultipliersMatchDenseReference) {
  constexpr size_t N = 4;
  constexpr double delta = 2.0 * kPi / static_cast<double>(N);
  const Values Y = RingQcqpValuesD2(N, delta, 0.3);
  const auto layout = RiemannianStaircaseOptimizer::Layout::From(Y);
  const QcqpProblem qcqp(RingGraph(N, delta), 2);
  const size_t M = qcqp.eConstraints().size();
  const Matrix Ystack = layout.stack(Y);

  const std::vector<Vector> zeros(M, Vector1(0.0));
  const Matrix G0 =
      Matrix(RiemannianStaircaseOptimizer::buildCertificate(qcqp, layout,
                                                            zeros)) *
      Ystack;

  Matrix W(G0.size(), static_cast<DenseIndex>(M));
  for (size_t m = 0; m < M; ++m) {
    std::vector<Vector> basis(M, Vector1(0.0));
    basis[m] = Vector1(1.0);
    const Matrix column =
        Matrix(RiemannianStaircaseOptimizer::buildCertificate(qcqp, layout,
                                                              basis)) *
            Ystack -
        G0;
    W.col(static_cast<DenseIndex>(m)) =
        Eigen::Map<const Vector>(column.data(), column.size());
  }
  const Eigen::Map<const Vector> g(G0.data(), G0.size());
  const Vector reference = W.colPivHouseholderQr().solve(-g);

  const auto ls = RiemannianStaircaseOptimizer::leastSquaresMultipliers(
      qcqp, layout, Y);
  for (size_t m = 0; m < M; ++m) {
    EXPECT_DOUBLES_EQUAL(reference(static_cast<DenseIndex>(m)),
                         ls.lambdaEq[m](0), 1e-7);
  }
}

// The two conventions the implementation has to get right, neither of which
// the rotation fixtures exercise: every constraint there has sigma = 1 and an
// already symmetric A.
//
// sigma is whitening bookkeeping, so the unwhitened multiplier lambda/sigma is
// what pairs with A and must not move; lambda itself scales with sigma. And
// trace(X' A X) sees only the symmetric part of A, so a constraint stated with
// a non-symmetric A must behave exactly like its symmetrized form.
TEST(RiemannianStaircase, LeastSquaresMultipliersRespectSigmaAndAsymmetry) {
  const Key key = 0;
  const Matrix2 Q{{2.0, -0.7}, {-0.7, 1.5}};
  const Matrix symmetric(Matrix2{{0.0, 0.5}, {0.5, 0.0}});
  const Matrix skewed(Matrix2{{0.0, 1.3}, {-0.3, 0.0}});  // same symmetric part

  Matrix X(2, 3);
  X << 0.8, 0.1, -0.2, 0.3, 0.9, 0.4;  // deliberately infeasible
  Values Y;
  Y.insert(key, X);
  const auto layout = RiemannianStaircaseOptimizer::Layout::From(Y);

  const auto solve = [&](const Matrix& A, double sigma) {
    QcqpProblem qcqp;
    qcqp.addCost(QpCost(KeyVector{key},
                        SymmetricBlockMatrix(std::vector<DenseIndex>{2},
                                             Matrix(Q)),
                        3));
    qcqp.addConstraint(QuadraticConstraint::Equal(
        key, Matrix(Matrix2{{1.0, 0.0}, {0.0, 0.0}}), 1.0, sigma));
    qcqp.addConstraint(QuadraticConstraint::Equal(key, A, 0.0, sigma));
    return RiemannianStaircaseOptimizer::leastSquaresMultipliers(qcqp, layout,
                                                                 Y);
  };

  const auto unit = solve(symmetric, 1.0);
  const auto scaled = solve(symmetric, 4.0);
  const auto asymmetric = solve(skewed, 1.0);

  // Scaling sigma leaves the fit alone and scales lambda by exactly sigma.
  EXPECT_DOUBLES_EQUAL(unit.totalResidual, scaled.totalResidual, 1e-9);
  for (size_t m = 0; m < unit.lambdaEq.size(); ++m) {
    EXPECT_DOUBLES_EQUAL(4.0 * unit.lambdaEq[m](0), scaled.lambdaEq[m](0),
                         1e-9);
  }

  // Only the symmetric part of A is visible to the constraint.
  EXPECT_DOUBLES_EQUAL(unit.totalResidual, asymmetric.totalResidual, 1e-9);
  for (size_t m = 0; m < unit.lambdaEq.size(); ++m) {
    EXPECT_DOUBLES_EQUAL(unit.lambdaEq[m](0), asymmetric.lambdaEq[m](0), 1e-9);
  }
}

}  // namespace least_squares_multipliers
/* ************************************************************************* */

/* ************************************************************************* */
namespace inner_solver_state {
using namespace RingFixture;

constexpr size_t kNumPoses = 5;
const double kDelta = 2.0 * kPi / static_cast<double>(kNumPoses);

// BCL raises the penalty during the solve, so the value the saddle merit needs
// is the one the solver finished on, not `bclInitialPenalty`.
TEST(RiemannianStaircase, InnerSolveReportsFinalPenalty) {
  const QcqpProblem qcqp(RingGraph(kNumPoses, kDelta), 2);
  const Values initial = RingQcqpValuesD2(kNumPoses, kDelta, 0.3);

  auto almParams = DefaultAlmParams();
  almParams->bclInitialPenalty = 7.0;
  const auto inner =
      RiemannianStaircaseOptimizer::runLocalSolver(qcqp, initial, almParams);
  EXPECT(inner.penalty > 0.0);
  EXPECT(inner.penalty >= almParams->bclInitialPenalty);
}

// A null almParams means "use the defaults" to runLocalSolver, so it must mean
// the same to the staircase. It used to be dereferenced unchecked on the lift
// path, which crashed as soon as a level failed to certify.
TEST(RiemannianStaircase, NullAlmParamsIsNormalized) {
  RiemannianStaircaseParams params;
  params.pMin = 2;
  params.pMax = 3;
  params.eta = -1.0;  // Reject every level, so the lift path is exercised.
  params.almParams = nullptr;

  const RiemannianStaircaseOptimizer optimizer(
      RingGraph(kNumPoses, kDelta), RingQcqpValuesD2(kNumPoses, kDelta, 0.3),
      params);
  EXPECT(optimizer.params().almParams != nullptr);

  const auto result = optimizer.optimize();
  EXPECT(!result.certified);
  LONGS_EQUAL(2, result.ranksVisited.size());
}

}  // namespace inner_solver_state
/* ************************************************************************* */

/* ************************************************************************* */
namespace cached_data_matrix {
using namespace RingFixture;

constexpr size_t kNumPoses = 6;
const double kDelta = 2.0 * kPi / static_cast<double>(kNumPoses);

// Caching Q for the whole climb is only sound because Q does not depend on the
// rank: the layout is indexed by row dimension, and lifting adds columns only.
TEST(RiemannianStaircase, DataMatrixIsRankIndependent) {
  const NonlinearFactorGraph graph = RingGraph(kNumPoses, kDelta);
  Matrix reference;
  for (int p = 2; p <= 5; ++p) {
    const QcqpProblem qcqp(graph, p);
    Values Y;
    for (size_t i = 0; i < kNumPoses; ++i)
      Y.insert(Symbol('x', i), Matrix(Matrix::Zero(2, p)));
    const auto layout = RiemannianStaircaseOptimizer::Layout::From(Y);
    const Matrix Q =
        Matrix(RiemannianStaircaseOptimizer::buildDataMatrix(qcqp, layout));
    if (p == 2) {
      reference = Q;
    } else {
      EXPECT(assert_equal(reference, Q, 1e-12));
    }
  }
}

// The optimizer caches Q at construction; it must equal a freshly built one.
TEST(RiemannianStaircase, CachedDataMatrixMatchesFreshBuild) {
  const NonlinearFactorGraph graph = RingGraph(kNumPoses, kDelta);
  const Values initial = RingQcqpValuesD2(kNumPoses, kDelta, 0.01);
  RiemannianStaircaseParams params;
  params.pMin = 2;
  params.pMax = 3;
  params.almParams = DefaultAlmParams();

  const RiemannianStaircaseOptimizer optimizer(graph, initial, params);
  const QcqpProblem qcqp(graph, 2);
  const auto layout = RiemannianStaircaseOptimizer::Layout::From(initial);
  EXPECT(assert_equal(
      Matrix(RiemannianStaircaseOptimizer::buildDataMatrix(qcqp, layout)),
      Matrix(optimizer.dataMatrix()), 1e-12));
}

// Passing Q explicitly must give exactly what rebuilding it internally gives,
// for both routines that consume it.
TEST(RiemannianStaircase, ReusingDataMatrixMatchesRebuilding) {
  const NonlinearFactorGraph graph = RingGraph(kNumPoses, kDelta);
  const Values Y = RingQcqpValuesD2(kNumPoses, kDelta, 0.3);
  const auto layout = RiemannianStaircaseOptimizer::Layout::From(Y);
  const QcqpProblem qcqp(graph, 2);
  const auto Q = RiemannianStaircaseOptimizer::buildDataMatrix(qcqp, layout);

  const auto rebuilt =
      RiemannianStaircaseOptimizer::leastSquaresMultipliers(qcqp, layout, Y);
  const auto reused = RiemannianStaircaseOptimizer::leastSquaresMultipliers(
      qcqp, layout, Y, Q);
  EXPECT_DOUBLES_EQUAL(rebuilt.totalResidual, reused.totalResidual, 1e-12);
  LONGS_EQUAL(rebuilt.lambdaEq.size(), reused.lambdaEq.size());
  for (size_t m = 0; m < rebuilt.lambdaEq.size(); ++m) {
    EXPECT(assert_equal(rebuilt.lambdaEq[m], reused.lambdaEq[m], 1e-12));
  }

  EXPECT(assert_equal(
      Matrix(RiemannianStaircaseOptimizer::buildCertificate(qcqp, layout,
                                                            reused.lambdaEq)),
      Matrix(RiemannianStaircaseOptimizer::buildCertificate(
          qcqp, layout, reused.lambdaEq, Q)),
      1e-12));
}

}  // namespace cached_data_matrix
/* ************************************************************************* */

/* ************************************************************************* */
namespace stage_two_is_not_authoritative {

// Reaching Stage 2 means the Cholesky test already failed, so Stage 2 must
// never report a pass no matter what its estimate says. Soundness then rests
// only on the direct factorization, not on Lanczos accuracy.
TEST(RiemannianStaircase, SpectraStageTwoNeverCertifies) {
  constexpr int n = 40;
  // Indefinite, but the negative eigenvalue is buried in a cluster spanning
  // 16 orders of magnitude. Lanczos reports a positive lambda_min here, so a
  // stage that graded itself on that estimate would certify an indefinite S.
  Eigen::SparseMatrix<double> S(n, n);
  std::vector<Eigen::Triplet<double>> triplets;
  triplets.emplace_back(0, 0, -1e-9);
  for (int i = 1; i < n - 1; ++i) triplets.emplace_back(i, i, 1e-9 * i);
  triplets.emplace_back(n - 1, n - 1, 1e7);
  S.setFromTriplets(triplets.begin(), triplets.end());
  S.makeCompressed();

  RiemannianStaircaseParams params;
  params.eta = 1e-10;
  const auto [passed, lambdaMin, vMin] =
      RiemannianStaircaseOptimizer::verify(S, params);

  // Cholesky on S + eta*I fails, so the verdict is settled before Stage 2
  // reports anything; what it estimates cannot flip that. The estimate itself
  // is not asserted: whether Lanczos resolves a cluster this tight varies with
  // the BLAS in use, and the point of the test is that the verdict does not.
  EXPECT(!passed);
  EXPECT_LONGS_EQUAL(n, static_cast<long>(vMin.size()));
}

}  // namespace stage_two_is_not_authoritative
/* ************************************************************************* */

/* ************************************************************************* */
namespace saddle_line_search {
using namespace RingFixture;

// The search only chooses the step; the lift itself must be untouched, so the
// first p columns still carry Y* and only a column is appended.
TEST(RiemannianStaircase, LineSearchPreservesLiftShape) {
  constexpr size_t N = 4;
  constexpr double delta = 2.0 * kPi / static_cast<double>(N);
  const NonlinearFactorGraph graph = RingGraph(N, delta);
  const Values Ystar = RingQcqpValuesD2(N, delta, 0.2);
  const auto layout = RiemannianStaircaseOptimizer::Layout::From(Ystar);

  const QcqpProblem lifted(graph, 3);
  const Vector vMin = Vector::Ones(layout.totalDim).normalized();
  std::vector<Vector> lambdaEq(lifted.eConstraints().size(), Vector1(0.1));

  RiemannianStaircaseParams params;
  params.almParams = DefaultAlmParams();
  params.useSaddleLineSearch = true;  // off by default
  const auto escape = RiemannianStaircaseOptimizer::saddleEscapeWithLineSearch(
      lifted, layout, Ystar, vMin, lambdaEq, 10.0, -0.5, params);

  for (const auto& [key, X] : escape.lifted.extract<Matrix>()) {
    const Matrix before = Ystar.at<Matrix>(key);
    LONGS_EQUAL(before.cols() + 1, X.cols());
    EXPECT(assert_equal(before, Matrix(X.leftCols(before.cols())), 1e-12));
  }
}

// With the search disabled the fixed step is used verbatim, so the two paths
// can be compared on the same problem.
TEST(RiemannianStaircase, LineSearchTogglesOff) {
  constexpr size_t N = 4;
  constexpr double delta = 2.0 * kPi / static_cast<double>(N);
  const NonlinearFactorGraph graph = RingGraph(N, delta);
  const Values Ystar = RingQcqpValuesD2(N, delta, 0.2);
  const auto layout = RiemannianStaircaseOptimizer::Layout::From(Ystar);

  const QcqpProblem lifted(graph, 3);
  const Vector vMin = Vector::Ones(layout.totalDim).normalized();
  std::vector<Vector> lambdaEq(lifted.eConstraints().size(), Vector1(0.1));

  RiemannianStaircaseParams params;
  params.almParams = DefaultAlmParams();
  params.alpha = 3e-2;
  params.useSaddleLineSearch = false;

  const auto escape = RiemannianStaircaseOptimizer::saddleEscapeWithLineSearch(
      lifted, layout, Ystar, vMin, lambdaEq, 10.0, -0.5, params);
  EXPECT_DOUBLES_EQUAL(params.alpha, escape.alpha, 1e-12);
  EXPECT(!escape.descentFound);
  EXPECT(assert_equal(RiemannianStaircaseOptimizer::liftWithDescent(
                          Ystar, vMin, layout, params.alpha),
                      escape.lifted, 1e-12));
}

// Along the certificate's own minimum eigenvector the merit does decrease, so
// this covers the accepting branch. An arbitrary direction does not: the lift
// adds constraint violation the merit charges for, and the search declines.
TEST(RiemannianStaircase, LineSearchFindsDescentAlongMinEigenvector) {
  constexpr size_t N = 4;
  constexpr double delta = 2.0 * kPi / static_cast<double>(N);
  const NonlinearFactorGraph graph = RingGraph(N, delta);
  const Values Ystar = RingQcqpValuesD2(N, delta, 0.2);
  const auto layout = RiemannianStaircaseOptimizer::Layout::From(Ystar);

  const QcqpProblem atRank(graph, 2);
  const auto ls = RiemannianStaircaseOptimizer::leastSquaresMultipliers(
      atRank, layout, Ystar);
  const auto S = RiemannianStaircaseOptimizer::buildCertificate(atRank, layout,
                                                                ls.lambdaEq);
  RiemannianStaircaseParams verifyParams;
  verifyParams.eta = 1e-9;
  const auto [certified, lambdaMin, vMin] =
      RiemannianStaircaseOptimizer::verify(S, verifyParams);
  EXPECT(!certified);
  EXPECT(lambdaMin < 0.0);

  const QcqpProblem lifted(graph, 3);
  RiemannianStaircaseParams params;
  params.almParams = DefaultAlmParams();
  params.useSaddleLineSearch = true;
  const auto escape = RiemannianStaircaseOptimizer::saddleEscapeWithLineSearch(
      lifted, layout, Ystar, vMin, ls.lambdaEq, 10.0, lambdaMin, params);

  EXPECT(escape.descentFound);
  EXPECT(escape.meritDecrease > 0.0);
  EXPECT(escape.alpha > params.alpha);
}

// lambda_min of exactly zero divides to an infinite starting step, which
// halving never brings back under alphaMin.
TEST(RiemannianStaircase, LineSearchTerminatesAtZeroEigenvalue) {
  constexpr size_t N = 4;
  constexpr double delta = 2.0 * kPi / static_cast<double>(N);
  const NonlinearFactorGraph graph = RingGraph(N, delta);
  const Values Ystar = RingQcqpValuesD2(N, delta, 0.2);
  const auto layout = RiemannianStaircaseOptimizer::Layout::From(Ystar);

  const QcqpProblem lifted(graph, 3);
  const Vector vMin = Vector::Ones(layout.totalDim).normalized();
  std::vector<Vector> lambdaEq(lifted.eConstraints().size(), Vector1(0.1));

  RiemannianStaircaseParams params;
  params.almParams = DefaultAlmParams();
  params.useSaddleLineSearch = true;

  const auto escape = RiemannianStaircaseOptimizer::saddleEscapeWithLineSearch(
      lifted, layout, Ystar, vMin, lambdaEq, 10.0, 0.0, params);
  EXPECT(std::isfinite(escape.alpha));
  for (const auto& [key, X] : escape.lifted.extract<Matrix>()) {
    EXPECT(X.allFinite());
  }
}

}  // namespace saddle_line_search
/* ************************************************************************* */

/* ************************************************************************* */
namespace certificate_fixes {

using RingFixture::DefaultAlmParams;
using RingFixture::RingGraph;
using RingFixture::RingQcqpValuesD2;

constexpr size_t kNumPoses = 5;
constexpr double kDelta = 2.0 * kPi / kNumPoses;

// buildDataMatrix and QpCost::error read the same HessianFactor through
// different views: the certificate takes the natural top-left corner of each
// expanded block, while the solver uses the whole expansion. Nothing in the
// type system ties those views together, so check the assembled Q against the
// cost it is supposed to represent. QpCost evaluates 0.5 * tr(Y' Q Y), hence
// the factor of 2.
TEST(RiemannianStaircase, DataMatrixAgreesWithQcqpCost) {
  const NonlinearFactorGraph graph = RingGraph(kNumPoses, kDelta);
  const QcqpProblem qcqp(graph, 2);
  const Values values = RingQcqpValuesD2(kNumPoses, kDelta, 0.05);
  const auto layout = RiemannianStaircaseOptimizer::Layout::From(values);

  const auto Q = RiemannianStaircaseOptimizer::buildDataMatrix(qcqp, layout);
  const Matrix Y = layout.stack(values);
  const double fromDataMatrix = (Y.transpose() * (Q * Y)).trace();

  EXPECT_DOUBLES_EQUAL(2.0 * qcqp.costs().error(values), fromDataMatrix, 1e-9);
}

// lambda_min comes out of a shift trick as the difference lambda_max -
// shiftedMax, so an unscaled relative tolerance leaves an absolute error of
// tol * lambda_max. On a spectrum with a wide dynamic range that error can
// exceed the eigenvalue itself and flip its sign, which is what produced false
// certificates. Build a matrix whose spectrum is known exactly and check both
// the sign and the value.
TEST(RiemannianStaircase, SpectraResolvesTinyNegativeEigenvalueOnWideSpectrum) {
  constexpr int n = 60;
  const double trueMin = -2.4e-02;
  Eigen::SparseMatrix<double> S(n, n);
  std::vector<Eigen::Triplet<double>> triplets;
  triplets.emplace_back(0, 0, trueMin);
  // Remaining eigenvalues span up to ~5.9e+03, the ratio seen on intel.
  for (int i = 1; i < n; ++i) {
    triplets.emplace_back(i, i, 1.0 + 100.0 * static_cast<double>(i));
  }
  S.setFromTriplets(triplets.begin(), triplets.end());
  S.makeCompressed();

  RiemannianStaircaseParams params;
  params.eta = 1e-4;  // far tighter than tol * lambda_max would allow
  // verify() dispatches to the Spectra path, which is the default.
  const auto [passed, lambdaMin, vMin] =
      RiemannianStaircaseOptimizer::verify(S, params);

  EXPECT(!passed);
  EXPECT_DOUBLES_EQUAL(trueMin, lambdaMin, 1e-6);
  EXPECT(lambdaMin < 0.0);
}

}  // namespace certificate_fixes
/* ************************************************************************* */
