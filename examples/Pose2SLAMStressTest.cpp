/**
 * @file Pose2SLAMStressTest.cpp
 * @brief Test GTSAM on large open-loop chains
 * @date May 23, 2018
 * @author Wenqiang Zhou
 */

// Create N 3D poses, add relative motion between each consecutive poses. (The
// relative motion is simply a unit translation(1, 0, 0), no rotation). For each
// each pose, add some random noise to the x value of the translation part.
// Use gtsam to create a prior factor for the first pose and N-1 between factors
// and run optimization.

#include <gtsam/geometry/Cal3_S2Stereo.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/NonlinearEquality.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/StereoFactor.h>

#include <random>

using namespace std;
using namespace gtsam;

void testGtsam(int numberNodes) {
  std::random_device rd;
  std::mt19937 e2(rd());
  std::uniform_real_distribution<> dist(0, 1);

  vector<Pose3> poses;
  for (int i = 0; i < numberNodes; ++i) {
    Matrix4 M;
    double r = dist(e2);
    r = (r - 0.5) / 10 + i;
    M << 1, 0, 0, r, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1;
    poses.push_back(Pose3(M));
  }

  // prior factor for the first pose
  auto priorModel = noiseModel::Isotropic::Variance(6, 1e-4);
  Matrix4 first_M;
  first_M << 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1;
  Pose3 first = Pose3(first_M);

  NonlinearFactorGraph graph;
  graph.addPrior(0, first, priorModel);

  // vo noise model
  auto VOCovarianceModel = noiseModel::Isotropic::Variance(6, 1e-3);

  // relative VO motion
  Matrix4 vo_M;
  vo_M << 1, 0, 0, 1, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1;
  Pose3 relativeMotion(vo_M);
  for (int i = 0; i < numberNodes - 1; ++i) {
    graph.add(
        BetweenFactor<Pose3>(i, i + 1, relativeMotion, VOCovarianceModel));
  }

  // inital values
  Values initial;
  for (int i = 0; i < numberNodes; ++i) {
    initial.insert(i, poses[i]);
  }

  LevenbergMarquardtParams params;
  params.setVerbosity("ERROR");
  params.setOrderingType("METIS");
  params.setLinearSolverType("MULTIFRONTAL_CHOLESKY");
  LevenbergMarquardtOptimizer optimizer(graph, initial, params);
  auto result = optimizer.optimize();
}

int main(int args, char* argv[]) {
  int numberNodes = stoi(argv[1]);
  cout << "number of_nodes: " << numberNodes << endl;
  testGtsam(numberNodes);
  return 0;
}
