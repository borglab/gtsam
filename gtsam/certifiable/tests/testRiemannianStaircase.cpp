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
#include <gtsam/slam/FrobeniusFactor.h>

#include <Eigen/Eigenvalues>

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

// Shared ALM tuning used by most tests. Tighter than library defaults so the
// ring fixture converges to a 1st-order KKT point in <100 iterations.
AugmentedLagrangianParams::shared_ptr DefaultAlmParams() {
  auto p = std::make_shared<AugmentedLagrangianParams>();
  p->maxIterations = 50;
  p->initialMuEq = 10.0;
  p->muEqIncreaseRate = 2.0;
  p->absoluteViolationTolerance = 1e-8;
  p->relativeViolationTolerance = 1e-8;
  return p;
}

}  // namespace RingFixture

/* ************************************************************************* */
// At an ALM-converged Ystar, S = Q + sum_m lambda_m * A_m is PSD and S * Y ≈ 0
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
// escapeSaddleAndLift widens each value by one column, leaving the first
// p columns unchanged and writing alpha * v into the new column.
TEST(RiemannianStaircase, EscapeSaddleAndLiftShape) {
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
      RiemannianStaircaseOptimizer::escapeSaddleAndLift(Ystar, v, layout, alpha);

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
TEST(RiemannianStaircase, EscapeSaddleViolationIsQuadratic) {
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
  const Values Yh = RiemannianStaircaseOptimizer::escapeSaddleAndLift(
      Ystar, v, layout, alphaSmall);
  const Values YH = RiemannianStaircaseOptimizer::escapeSaddleAndLift(
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
  // Spectra takes the Cholesky shortcut and reports lambda_min = -eta as a
  // bound; both methods land in the cert-passing region.
  EXPECT(resultS.minEigenvalue >= -paramsS.eta);
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
// escapeSaddleAndLift error paths: short vMin and non-conforming layout.
TEST(RiemannianStaircase, EscapeSaddleErrorPaths) {
  using namespace RingFixture;
  constexpr size_t N = 5;
  constexpr double delta = 2.0 * kPi / static_cast<double>(N);
  const Values Ystar = RingQcqpValuesD2(N, delta, 0.0);
  const auto layout = RiemannianStaircaseOptimizer::Layout::From(Ystar);

  const Vector shortV = Vector::Zero(layout.totalDim - 1);
  CHECK_EXCEPTION(
      RiemannianStaircaseOptimizer::escapeSaddleAndLift(
          Ystar, shortV, layout, 1e-2),
      std::invalid_argument);

  Values nonConforming;
  nonConforming.insert(Symbol('z', 0), Matrix(Matrix::Zero(3, 2)));
  const Vector v = Vector::Zero(layout.totalDim);
  CHECK_EXCEPTION(
      RiemannianStaircaseOptimizer::escapeSaddleAndLift(
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
