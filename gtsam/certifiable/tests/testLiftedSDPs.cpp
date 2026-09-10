/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file testLiftedSDPs.cpp
 * @brief Tests for QCQP-backed lifted SDP objective assembly.
 */

#include <CppUnitLite/TestHarness.h>
#include <gtsam/base/TestableAssertions.h>
#include <gtsam/certifiable/LiftedSDPProblem.h>
#include <gtsam/certifiable/internal/DirectSDP.h>
#include <gtsam/constrained/QcqpProblem.h>
#include <gtsam/constrained/QpCost.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot2.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/FrobeniusFactor.h>

#include <algorithm>
#include <cmath>
#include <iostream>
#include <map>
#include <stdexcept>
#include <vector>

using namespace gtsam;

/* ************************************************************************* */
namespace lifted_sdp_tests {

constexpr double kPi = 3.141592653589793238462643383279502884;

// Build a cycle of relative Rot2 measurements.
NonlinearFactorGraph Rot2RingGraph(size_t numPoses, double delta) {
  NonlinearFactorGraph graph;
  for (size_t i = 0; i < numPoses; ++i) {
    graph.emplace_shared<FrobeniusBetweenFactor<Rot2>>(
        Symbol('x', i), Symbol('x', (i + 1) % numPoses),
        Rot2::fromAngle(delta));
  }
  return graph;
}

// Create lifted QCQP values with a controlled angular perturbation.
Values Rot2RingQcqpValues(size_t numPoses, double delta, double perturbation) {
  Values values;
  for (size_t i = 0; i < numPoses; ++i) {
    InsertQcqpValue<Rot2, 1>(
        Symbol('x', i),
        Rot2::fromAngle(i * delta + perturbation * static_cast<double>(i)),
        &values);
  }
  return values;
}

// Build a cycle using relative measurements from the supplied poses.
template <typename T>
NonlinearFactorGraph PoseRingGraph(const std::vector<T>& poses) {
  NonlinearFactorGraph graph;
  for (size_t i = 0; i < poses.size(); ++i) {
    const size_t j = (i + 1) % poses.size();
    graph.emplace_shared<FrobeniusBetweenFactor<T>>(
        Symbol('x', i), Symbol('x', j), poses[i].between(poses[j]));
  }
  return graph;
}

// Lift a sequence of manifold values into D=1 QCQP coordinates.
template <typename T>
Values PoseRingQcqpValues(const std::vector<T>& poses) {
  Values values;
  for (size_t i = 0; i < poses.size(); ++i) {
    InsertQcqpValue<T, 1>(Symbol('x', i), poses[i], &values);
  }
  return values;
}

// Create a closed Pose2 ring by repeatedly composing one body-frame step.
std::vector<Pose2> Pose2RingPoses(size_t numPoses) {
  const Pose2 step(2.0, 0.0, 2.0 * kPi / static_cast<double>(numPoses));
  std::vector<Pose2> poses(numPoses);
  for (size_t i = 1; i < numPoses; ++i) {
    poses[i] = poses[i - 1].compose(step);
  }
  return poses;
}

// Create a closed Pose3 ring by repeatedly composing one body-frame step.
std::vector<Pose3> Pose3RingPoses(size_t numPoses) {
  const Pose3 step(Rot3::Rz(2.0 * kPi / static_cast<double>(numPoses)),
                   Point3(2.0, 0.0, 0.0));
  std::vector<Pose3> poses(numPoses);
  for (size_t i = 1; i < numPoses; ++i) {
    poses[i] = poses[i - 1].compose(step);
  }
  return poses;
}

// Apply deterministic local perturbations so the objective is nonzero.
std::vector<Pose2> PerturbedPose2Values(const std::vector<Pose2>& poses) {
  std::vector<Pose2> perturbed = poses;
  for (size_t i = 0; i < perturbed.size(); ++i) {
    const double scale = static_cast<double>(i);
    perturbed[i] =
        poses[i].retract(Vector3(0.01 * scale, -0.005 * scale, 0.003 * scale));
  }
  return perturbed;
}

// Apply deterministic local perturbations so the objective is nonzero.
std::vector<Pose3> PerturbedPose3Values(const std::vector<Pose3>& poses) {
  std::vector<Pose3> perturbed = poses;
  for (size_t i = 0; i < perturbed.size(); ++i) {
    const double scale = static_cast<double>(i);
    Vector6 delta{0.002 * scale, -0.001 * scale, 0.0015 * scale,
                  0.01 * scale,  -0.005 * scale, 0.004 * scale};
    perturbed[i] = poses[i].retract(delta);
  }
  return perturbed;
}

// Assemble the local rank-one matrix in a Hessian factor's key order.
Matrix BuildLocalX(const HessianFactor& H, const Values& values) {
  std::vector<Vector> localValues;
  DenseIndex totalDim = 0;
  for (auto it = H.begin(); it != H.end(); ++it) {
    Vector x = values.at<Matrix>(*it).col(0);
    if (x.size() != H.getDim(it)) {
      throw std::runtime_error("BuildLocalX: QCQP value dimension mismatch.");
    }
    totalDim += x.size();
    localValues.push_back(std::move(x));
  }

  Matrix X_f = Matrix::Zero(totalDim, totalDim);
  DenseIndex rowStart = 0;
  for (const Vector& x_i : localValues) {
    DenseIndex colStart = 0;
    for (const Vector& x_j : localValues) {
      X_f.block(rowStart, colStart, x_i.size(), x_j.size()) =
          x_i * x_j.transpose();
      colStart += x_j.size();
    }
    rowStart += x_i.size();
  }
  return X_f;
}

// Evaluate the lifted trace objective directly from rank-one matrices.
double ComputeLiftedObjective(const QcqpProblem& problem,
                              const Values& values) {
  double objective = 0.0;
  for (const auto& factor : problem.costs()) {
    if (!factor) {
      continue;
    }

    const auto* cost = dynamic_cast<const QpCost*>(factor.get());
    if (!cost) {
      throw std::runtime_error("ComputeLiftedObjective: expected QpCost.");
    }

    const HessianFactor& H = cost->hessianFactor();
    if (H.linearTerm().norm() > 0.0 || H.constantTerm() != 0.0) {
      throw std::runtime_error(
          "ComputeLiftedObjective: linear/constant terms are not supported.");
    }

    const Matrix Q_f = H.information();
    const Matrix X_f = BuildLocalX(H, values);
    objective += 0.5 * Q_f.cwiseProduct(X_f).sum();
  }
  return objective;
}

// Verifies that lifting preserves the QCQP objective at a feasible assignment.
TEST(LiftedSDPs, Rot2_QcqpObjectiveMatchesLiftedObjective) {
  constexpr size_t N = 20;
  const double delta = 2.0 * kPi / static_cast<double>(N);
  constexpr double perturbation = 0.03;

  const NonlinearFactorGraph graph = Rot2RingGraph(N, delta);
  const QcqpProblem problem(graph);
  const Values qcqpValues = Rot2RingQcqpValues(N, delta, perturbation);

  const double qcqpObjective = problem.costs().error(qcqpValues);
  const double sdpObjective = ComputeLiftedObjective(problem, qcqpValues);

  std::cout << "qcqpObjective: " << qcqpObjective << std::endl;
  std::cout << "sdpObjective: " << sdpObjective << std::endl;

  EXPECT_DOUBLES_EQUAL(qcqpObjective, sdpObjective, 1e-12);
}

// Verifies that the Pose2 QCQP and its rank-one SDP lift have equal costs.
TEST(LiftedSDPs, Pose2_QcqpObjectiveMatchesLiftedObjective) {
  constexpr size_t N = 20;
  const std::vector<Pose2> groundTruth = Pose2RingPoses(N);
  const NonlinearFactorGraph graph = PoseRingGraph(groundTruth);
  const QcqpProblem problem(graph);
  const Values qcqpValues =
      PoseRingQcqpValues(PerturbedPose2Values(groundTruth));

  const double qcqpObjective = problem.costs().error(qcqpValues);
  const double sdpObjective = ComputeLiftedObjective(problem, qcqpValues);

  EXPECT_DOUBLES_EQUAL(qcqpObjective, sdpObjective, 1e-10);
}

// Verifies that the Pose3 QCQP and its rank-one SDP lift have equal costs.
TEST(LiftedSDPs, Pose3_QcqpObjectiveMatchesLiftedObjective) {
  constexpr size_t N = 20;
  const std::vector<Pose3> groundTruth = Pose3RingPoses(N);
  const NonlinearFactorGraph graph = PoseRingGraph(groundTruth);
  const QcqpProblem problem(graph);
  const Values qcqpValues =
      PoseRingQcqpValues(PerturbedPose3Values(groundTruth));

  const double qcqpObjective = problem.costs().error(qcqpValues);
  const double sdpObjective = ComputeLiftedObjective(problem, qcqpValues);

  EXPECT_DOUBLES_EQUAL(qcqpObjective, sdpObjective, 1e-10);
}

}  // namespace lifted_sdp_tests
/* ************************************************************************* */

#ifdef GTSAM_USE_MOSEK
/* ************************************************************************* */
namespace pose_ring_sdp_fixture {

constexpr size_t kNumPoses = 20;
constexpr double kRankOneEigenRatioThreshold = 1e5;
constexpr double kPoseErrorTolerance = 1e-4;
constexpr double kObjectiveTolerance = 1e-3;

struct SdpSolutionSummary {
  double objective = 0.0;
  double minimumEigenvalueRatio = 0.0;
  double maximumPoseError = 0.0;
  bool finiteEigenvalueRatios = false;
  bool repeatedQueriesMatch = false;
};

// Build an exactly consistent ring with a hard Frobenius prior on the first
// pose.
template <typename T>
NonlinearFactorGraph ExactPoseRingGraph(const std::vector<T>& poses,
                                        size_t frobeniusDimension) {
  NonlinearFactorGraph graph;
  const auto priorNoise = noiseModel::Constrained::All(frobeniusDimension);
  const auto betweenNoise = noiseModel::Isotropic::Sigma(T::dimension, 0.01);
  graph.emplace_shared<FrobeniusPrior<T>>(0, poses[0].matrix(), priorNoise);
  for (size_t i = 0; i < poses.size(); ++i) {
    const size_t j = (i + 1) % poses.size();
    graph.emplace_shared<FrobeniusBetweenFactor<T>>(
        i, j, poses[i].between(poses[j]), betweenNoise);
  }
  return graph;
}

// Solve an SDP and summarize its diagonal-block rank and recovery accuracy.
template <typename T, typename SdpProblem>
SdpSolutionSummary SolveAndSummarize(SdpProblem* sdp,
                                     const std::vector<T>& groundTruth) {
  const std::map<std::string, double> mosekParams{
      {"intpntCoTolRelGap", 1e-10},
      {"optimizerMaxTime", 1500.0},
  };
  if (!sdp->solve(mosekParams)) {
    throw std::runtime_error("MOSEK did not return a readable solution.");
  }

  const std::vector<double> eigenvalueRatios = sdp->variableEVRs();
  const Values qcqpValues = sdp->qcqpValues();
  const Values repeatedQcqpValues = sdp->qcqpValues();
  const std::vector<double> repeatedEigenvalueRatios = sdp->variableEVRs();
  const bool finiteEigenvalueRatios =
      std::all_of(eigenvalueRatios.begin(), eigenvalueRatios.end(),
                  [](double ratio) { return std::isfinite(ratio); });
  const bool repeatedRatiosMatch =
      eigenvalueRatios.size() == repeatedEigenvalueRatios.size() &&
      std::equal(eigenvalueRatios.begin(), eigenvalueRatios.end(),
                 repeatedEigenvalueRatios.begin(),
                 [](double first, double second) {
                   return std::abs(first - second) <= 1e-12;
                 });
  const bool repeatedQueriesMatch =
      repeatedRatiosMatch &&
      assert_equal(qcqpValues, repeatedQcqpValues, 1e-12);

  const auto recovered = ExtractQcqpValues<T, 1>(qcqpValues);
  if (recovered.size() != groundTruth.size()) {
    throw std::runtime_error(
        "Recovered QCQP value count does not match ground truth.");
  }
  std::vector<double> poseErrors(recovered.size());
  for (size_t index = 0; index < recovered.size(); ++index) {
    poseErrors[index] =
        groundTruth[index].localCoordinates(recovered[index].second).norm();
  }

  return {sdp->objectiveValue(),
          *std::min_element(eigenvalueRatios.begin(), eigenvalueRatios.end()),
          *std::max_element(poseErrors.begin(), poseErrors.end()),
          finiteEigenvalueRatios, repeatedQueriesMatch};
}

// Recovery queries reject access before either SDP formulation has been solved.
TEST(LiftedSDPs, RecoveryQueriesRequireSolve) {
  const std::vector<Pose2> groundTruth =
      lifted_sdp_tests::Pose2RingPoses(kNumPoses);
  const QcqpProblem problem(ExactPoseRingGraph(groundTruth, 9));
  LiftedSDPProblem<MonolithicSDP, MosekSDPSolver> monolithic(problem);
  LiftedSDPProblem<ChordalSDP, MosekSDPSolver> chordal(
      problem, ChordalOrderingType::Metis);

  CHECK_EXCEPTION(monolithic.qcqpValues(), std::runtime_error);
  CHECK_EXCEPTION(monolithic.variableEVRs(), std::runtime_error);
  CHECK_EXCEPTION(chordal.qcqpValues(), std::runtime_error);
  CHECK_EXCEPTION(chordal.variableEVRs(), std::runtime_error);
}

// Verifies rank-one Pose2 slices and matching monolithic/chordal solutions.
TEST(LiftedSDPs, Pose2_MonolithicAndChordal) {
  const std::vector<Pose2> groundTruth =
      lifted_sdp_tests::Pose2RingPoses(kNumPoses);
  const QcqpProblem problem(ExactPoseRingGraph(groundTruth, 9));

  LiftedSDPProblem<MonolithicSDP, MosekSDPSolver> monolithic(problem);
  LiftedSDPProblem<ChordalSDP, MosekSDPSolver> chordal(
      problem, ChordalOrderingType::Metis);
  const SdpSolutionSummary monolithicResult =
      SolveAndSummarize(&monolithic, groundTruth);
  const SdpSolutionSummary chordalResult =
      SolveAndSummarize(&chordal, groundTruth);

  EXPECT(monolithicResult.minimumEigenvalueRatio > kRankOneEigenRatioThreshold);
  EXPECT(chordalResult.minimumEigenvalueRatio > kRankOneEigenRatioThreshold);
  EXPECT(monolithicResult.finiteEigenvalueRatios);
  EXPECT(chordalResult.finiteEigenvalueRatios);
  EXPECT(monolithicResult.repeatedQueriesMatch);
  EXPECT(chordalResult.repeatedQueriesMatch);
  EXPECT(monolithicResult.maximumPoseError < kPoseErrorTolerance);
  EXPECT(chordalResult.maximumPoseError < kPoseErrorTolerance);
  EXPECT(monolithicResult.objective < kObjectiveTolerance);
  EXPECT(chordalResult.objective < kObjectiveTolerance);
  EXPECT_DOUBLES_EQUAL(monolithicResult.objective, chordalResult.objective,
                       kObjectiveTolerance);
}

// Verifies rank-one Pose3 slices and matching monolithic/chordal solutions.
TEST(LiftedSDPs, Pose3_MonolithicAndChordal) {
  const std::vector<Pose3> groundTruth =
      lifted_sdp_tests::Pose3RingPoses(kNumPoses);
  const QcqpProblem problem(ExactPoseRingGraph(groundTruth, 16));

  LiftedSDPProblem<MonolithicSDP, MosekSDPSolver> monolithic(problem);
  LiftedSDPProblem<ChordalSDP, MosekSDPSolver> chordal(
      problem, ChordalOrderingType::Metis);
  const SdpSolutionSummary monolithicResult =
      SolveAndSummarize(&monolithic, groundTruth);
  const SdpSolutionSummary chordalResult =
      SolveAndSummarize(&chordal, groundTruth);

  EXPECT(monolithicResult.minimumEigenvalueRatio > kRankOneEigenRatioThreshold);
  EXPECT(chordalResult.minimumEigenvalueRatio > kRankOneEigenRatioThreshold);
  EXPECT(monolithicResult.finiteEigenvalueRatios);
  EXPECT(chordalResult.finiteEigenvalueRatios);
  EXPECT(monolithicResult.repeatedQueriesMatch);
  EXPECT(chordalResult.repeatedQueriesMatch);
  EXPECT(monolithicResult.maximumPoseError < kPoseErrorTolerance);
  EXPECT(chordalResult.maximumPoseError < kPoseErrorTolerance);
  EXPECT(monolithicResult.objective < kObjectiveTolerance);
  EXPECT(chordalResult.objective < kObjectiveTolerance);
  EXPECT_DOUBLES_EQUAL(monolithicResult.objective, chordalResult.objective,
                       kObjectiveTolerance);
}

}  // namespace pose_ring_sdp_fixture
/* ************************************************************************* */
#endif

/* ************************************************************************* */
namespace direct_sdp_fixture {

std::vector<Pose2> poses2() {
  return {Pose2(1.2, -0.7, 0.4), Pose2(2.0, 0.3, -0.6), Pose2(-0.8, 1.1, 1.0)};
}

std::vector<Pose3> poses3() {
  return {Pose3(Rot3::RzRyRx(0.3, -0.2, 0.4), Point3(1.2, -0.7, 0.2)),
          Pose3(Rot3::RzRyRx(-0.5, 0.4, -0.6), Point3(2.0, 0.3, 1.0)),
          Pose3(Rot3::RzRyRx(0.2, 0.7, 1.0), Point3(-0.8, 1.1, -0.3))};
}

template <typename T>
NonlinearFactorGraph graphForPoses(const std::vector<T>& poses, bool anchor) {
  constexpr int N = T::LieAlgebra::RowsAtCompileTime;
  const KeyVector keys{19, 3, 81};
  NonlinearFactorGraph graph;
  if (anchor) {
    graph.emplace_shared<FrobeniusPrior<T>>(
        keys[0], poses[0].matrix(), noiseModel::Constrained::All(N * N));
  }
  Matrix covariance = Matrix::Identity(N * N, N * N);
  covariance(0, 1) = covariance(1, 0) = 0.2;
  covariance(1, 1) = 2.0;
  const auto noise = noiseModel::Gaussian::Covariance(covariance);
  for (size_t i = 0; i < poses.size(); ++i) {
    auto measurement = poses[i].matrix().eval();
    measurement(0, N - 1) += 0.15;
    graph.emplace_shared<FrobeniusPrior<T>>(keys[i], measurement, noise);
    if (i + 1 < poses.size()) {
      graph.emplace_shared<FrobeniusBetweenFactor<T>>(
          keys[i + 1], keys[i], poses[i + 1].between(poses[i]), noise);
    }
  }
  return graph;
}

template <typename T>
bool coefficientsMatch(const std::vector<T>& poses) {
  const auto graph = graphForPoses(poses, false);
  const QcqpProblem qcqp(graph);
  const auto direct = internal::buildDirectSDP(graph);
  if (direct.costs.size() != qcqp.costs().size() ||
      direct.constraints.size() != qcqp.eConstraints().size()) return false;
  for (size_t i = 0; i < direct.costs.size(); ++i) {
    const auto& cost = dynamic_cast<const QpCost&>(*qcqp.costs()[i]);
    if (cost.keys() != direct.costs[i].keys ||
        !assert_equal(cost.hessianFactor().information(), direct.costs[i].matrix, 1e-12)) {
      return false;
    }
    // Equality of full coefficient matrices also covers higher-rank Gram
    // matrices, including directions that vanish on the pose manifold.
    const Matrix& Q = direct.costs[i].matrix;
    const Matrix gram = Matrix::Identity(Q.rows(), Q.cols());
    if (std::abs(Q.cwiseProduct(gram).sum() -
        cost.hessianFactor().information().cwiseProduct(gram).sum()) > 1e-12) return false;
  }
  for (const auto& constraint : direct.constraints) {
    bool found = false;
    for (const auto& factor : qcqp.eConstraints()) {
      const auto& other = dynamic_cast<const QuadraticEqualityConstraintFactor&>(*factor)
                              .quadraticConstraint();
      if (other.key() == constraint.key && other.b() == constraint.rhs &&
          other.A().isApprox(constraint.matrix, 1e-14)) found = true;
    }
    if (!found) return false;
  }
  Values values, lifted;
  const KeyVector keys{19, 3, 81};
  for (size_t i = 0; i < poses.size(); ++i) {
    values.insert(keys[i], poses[i]);
    InsertQcqpValue<T, 1>(keys[i], poses[i], &lifted);
  }
  return std::abs(graph.error(values) - qcqp.costs().error(lifted)) < 1e-10;
}

// Independent direct coefficients agree with QCQP lowering for nontrivial SE(2).
TEST(LiftedSDPs, DirectPose2Coefficients) {
  EXPECT(coefficientsMatch(poses2()));
}

// Full 3D rotations and correlated ambient noise expose ordering/whitening errors.
TEST(LiftedSDPs, DirectPose3Coefficients) {
  EXPECT(coefficientsMatch(poses3()));
}

// An arbitrary ambient prior's fixed-row mismatch remains part of its cost.
TEST(LiftedSDPs, DirectPriorFixedRowCost) {
  Matrix3 measurement = Pose2(1.0, 2.0, 0.3).matrix();
  measurement(2, 0) = 0.4;
  measurement(2, 2) = 1.2;
  NonlinearFactorGraph graph;
  graph.emplace_shared<FrobeniusPrior<Pose2>>(7, measurement);
  Values values, lifted;
  values.insert(7, Pose2(-1.0, 0.7, -0.2));
  InsertQcqpValue<Pose2, 1>(7, values.at<Pose2>(7), &lifted);
  const QcqpProblem qcqp(graph);
  const auto direct = internal::buildDirectSDP(graph);
  const Vector x = lifted.at<Matrix>(7).col(0);
  EXPECT_DOUBLES_EQUAL(graph.error(values), qcqp.costs().error(lifted), 1e-12);
  EXPECT_DOUBLES_EQUAL(graph.error(values), 0.5 * x.dot(direct.costs[0].matrix * x), 1e-12);
}

// Partial hard priors and mixed pose types must not silently change semantics.
TEST(LiftedSDPs, DirectRejectsUnsupportedGraphs) {
  NonlinearFactorGraph partial;
  Vector sigmas = Vector::Ones(9);
  sigmas(0) = 0.0;
  partial.emplace_shared<FrobeniusPrior<Pose2>>(
      1, Pose2().matrix(), noiseModel::Constrained::MixedSigmas(sigmas));
  CHECK_EXCEPTION(internal::buildDirectSDP(partial), std::invalid_argument);
  CHECK_EXCEPTION({ QcqpProblem problem(partial); }, std::invalid_argument);
  NonlinearFactorGraph mixed;
  mixed.emplace_shared<FrobeniusPrior<Pose2>>(1, Pose2().matrix());
  mixed.emplace_shared<FrobeniusPrior<Pose3>>(2, Pose3().matrix());
  CHECK_EXCEPTION(internal::buildDirectSDP(mixed), std::invalid_argument);
}

#ifdef GTSAM_USE_MOSEK
template <typename T>
bool solvesMatch(const std::vector<T>& poses, ChordalOrderingType ordering) {
  const auto graph = graphForPoses(poses, true);
  const QcqpProblem qcqp(graph);
  MosekMonolithicSDP qm(qcqp), dm(graph);
  MosekChordalSDP qc(qcqp, ordering), dc(graph, ordering);
  if (!qm.solve() || !dm.solve() || !qc.solve() || !dc.solve()) return false;
  for (const auto& residuals : {qm.primalResiduals(), dm.primalResiduals(),
                                qc.primalResiduals(), dc.primalResiduals()}) {
    for (const auto& [name, residual] : residuals) {
      if (!std::isfinite(residual) || residual > 1e-7) return false;
    }
  }
  for (const auto& status : {qm.solutionStatus(), dm.solutionStatus(),
                             qc.solutionStatus(), dc.solutionStatus()}) {
    if (status != "SolutionStatus::Optimal") return false;
  }
  const double objective = qm.objectiveValue();
  for (double other : {dm.objectiveValue(), qc.objectiveValue(), dc.objectiveValue()}) {
    if (std::abs(objective - other) > 1e-6) return false;
  }
  const Vector target = traits<T>::template QcqpValue<1>(poses[0]).col(0);
  const Matrix anchor = target * target.transpose();
  return assert_equal(anchor, dm.liftedBlock(19, 19), 1e-7) &&
         assert_equal(anchor, dc.liftedBlock(19, 19), 1e-7) &&
         assert_equal(anchor, qm.liftedBlock(19, 19), 1e-7) &&
         assert_equal(anchor, qc.liftedBlock(19, 19), 1e-7) &&
         std::abs(dc.liftedBlock(19, 3)(0, 0) - 1.0) < 1e-7;
}

// All four SE(2) solves agree with exact anchors and noisy absolute measurements.
TEST(LiftedSDPs, DirectPose2Solves) {
  EXPECT(solvesMatch(poses2(), ChordalOrderingType::Colamd));
#ifdef GTSAM_SUPPORT_NESTED_DISSECTION
  EXPECT(solvesMatch(poses2(), ChordalOrderingType::Metis));
#endif
}

// All four SE(3) solves agree away from planar or identity-only configurations.
TEST(LiftedSDPs, DirectPose3Solves) {
  EXPECT(solvesMatch(poses3(), ChordalOrderingType::Colamd));
}

// A unary-only variable still owns a PSD block in the direct chordal graph.
TEST(LiftedSDPs, DirectUnaryOnly) {
  NonlinearFactorGraph graph;
  graph.emplace_shared<FrobeniusPrior<Pose2>>(91, Pose2(1, 2, 0.4).matrix());
  MosekChordalSDP problem(graph, ChordalOrderingType::Colamd);
  CHECK_EXCEPTION(problem.liftedBlock(91, 91), std::runtime_error);
  EXPECT(problem.solve());
  EXPECT(std::abs(problem.objectiveValue()) < 1e-7);
}

// Cross blocks preserve row/column key orientation when read from Fusion.
TEST(LiftedSDPs, DirectCrossBlockOrientation) {
  const auto poses = poses2();
  NonlinearFactorGraph graph;
  for (size_t i = 0; i < 2; ++i) {
    graph.emplace_shared<FrobeniusPrior<Pose2>>(
        i, poses[i].matrix(), noiseModel::Constrained::All(9));
  }
  graph.emplace_shared<FrobeniusBetweenFactor<Pose2>>(
      0, 1, poses[0].between(poses[1]));
  MosekMonolithicSDP problem(graph);
  EXPECT(problem.solve());
  const Vector first = traits<Pose2>::QcqpValue<1>(poses[0]).col(0);
  const Vector second = traits<Pose2>::QcqpValue<1>(poses[1]).col(0);
  EXPECT(assert_equal(Matrix(first * second.transpose()), problem.liftedBlock(0, 1), 1e-7));
}
#endif

}  // namespace direct_sdp_fixture
/* ************************************************************************* */

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
