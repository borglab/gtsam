#include <boost/random.hpp>
#include <cstdlib>

#include <gtsam/geometry/Cal3_S2.h>
#include <gtsam/geometry/Line3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/ExpressionFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/expressions.h>
#include <gtsam/slam/PriorFactor.h>
#include "LineData.h"

using namespace std;
using namespace gtsam;

/**
 * Computes the point to line distance in a plane.
 * @param line
 * @param point
 * @param Dline
 * @param Dpoint
 * @return double - point to line distance
 */
double pointToLineDistance(const Unit3 &l, const Point2 &p,
                           OptionalJacobian<1, 2> Dl = boost::none,
                           OptionalJacobian<1, 2> Dp = boost::none) {
  Matrix32 Dl32;
  Point3 l3 = l.point3(Dl32);
  double l_dot_p = l3[0] * p.x() + l3[1] * p.y() + l3[2];
  double normalMag = sqrt(pow(l3[0], 2) + pow(l3[1], 2));
  double sign = abs(l_dot_p) / l_dot_p;

  if (Dl) {
    Matrix13 Dl3;
    Dl3 << sign * (p.x() * pow(l3[1], 2) - l3[0] * l3[1] * p.y() - l3[0] * l3[2]) / pow(normalMag, 3),
        sign * (p.y() * pow(l3[0], 2) - l3[0] * l3[1] * p.x() - l3[1] * l3[2]) / pow(normalMag, 3),
        sign / normalMag;
    *Dl = Dl3 * Dl32;
  }
  if (Dp) {
    *Dp << sign * l3[0] / normalMag, sign * l3[1] / normalMag;
  }
  return abs(l_dot_p) / normalMag;
}

int main() {
  // importing and visualizing the data
  LineData data = createLineDatasetRandom();

  // factor graph and initial values
  ExpressionFactorGraph graph;
  Values initialEstimate, groundTruth;

  // noise model for point to line distance, lines and poses
  noiseModel::Isotropic::shared_ptr measurementNoise = noiseModel::Isotropic::Sigma(1, 0.03);
  Vector6 pose_sigma;
  pose_sigma << 0.2, 0.2, 0.2, 0.06, 0.06, 0.06;
  noiseModel::Diagonal::shared_ptr posePriorNoise = noiseModel::Diagonal::Sigmas(pose_sigma);
  Vector4 line_sigma;
  line_sigma << 0.12, 0.12, 0.06, 0.06;
  noiseModel::Diagonal::shared_ptr linePriorNoise = noiseModel::Diagonal::Sigmas(line_sigma);

  // noise to be added to ground truth for initial estimate
  static Pose3 kDeltaPose(Rot3::Rodrigues(-0.3, 0.2, 0.25),
                          Point3(0.15, -0.05, 0.05));
  static Vector4 lineDelta(0.02, 0.03, 0.05, -.05);
  // all poses
  for (unsigned int i = 0; i < data.cameras.size(); i++) {
    Pose3_ x_('x', i);
    initialEstimate.insert(Symbol('x', i), data.cameras[i].pose());
    groundTruth.insert(Symbol('x', i), data.cameras[i].pose());

    // adding priors over the first pose (anchoring in 3D) and the first line (scale)
    if (i == 0) {
      graph.addExpressionFactor(x_, data.cameras[0].pose(), posePriorNoise);
      Line3_ l_('l', 0);
      graph.addExpressionFactor(l_, data.lines[0], linePriorNoise);
    }

    // for each line in the dataset
    for (unsigned int j = 0; j < data.lines.size(); j++) {
      // add initial estimate
      if (i == 0) {
        initialEstimate.insert(Symbol('l', j), data.lines[j]);
        groundTruth.insert(Symbol('l', j), data.lines[j]);
      }
      // project unknown line to image plane
      Line3_ l_('l', j);
      Line3_ transformed_line_ = transformTo(x_, l_);
      Unit3_ proj_line_(transformed_line_, &Line3::project);

      // dataset contains calibrated end points of lines on image plane
      Point2_ detected_start_(data.tracksEnd1[j][i]);
      Point2_ detected_end_(data.tracksEnd2[j][i]);

      Double_ start_dist_(&pointToLineDistance, proj_line_, detected_start_);
      Double_ end_dist_(&pointToLineDistance, proj_line_, detected_end_);

      // adding measurements to factor graph
      graph.addExpressionFactor(start_dist_, (double) 0.0, measurementNoise);
      graph.addExpressionFactor(end_dist_, (double) 0.0, measurementNoise);
    }
  }

  LevenbergMarquardtOptimizer optim(graph, initialEstimate);

  int iterations = 30;
  vector < vector < vector < double >>
      > errors(data.cameras.size(), vector < vector < double >> (data.lines.size(), vector<double>(iterations, 0.0)));

  for (int iter = 0; iter < iterations; iter++) {
    optim.iterate();
    std::cout << "iteration " << optim.iterations() << ", error: " << optim.error() << std::endl;
    Values currentEstimate = optim.values();

    for (size_t j = 0; j < data.cameras.size(); j++) {
      Pose3 current_cam_pose = currentEstimate.at<Pose3>(Symbol('x', j));
      for (size_t i = 0; i < data.lines.size(); i++) {
        Line3 line3d = currentEstimate.at<Line3>(Symbol('l', i));
        Unit3 projected_line = (transformTo(current_cam_pose, line3d)).project();
        errors[j][i][iter] = pointToLineDistance(projected_line, data.tracksEnd1[i][j]);
      }
    }
  }

  std::cout << "measurement errors for each measurement" << std::endl;
  for (size_t i = 0; i < errors.size(); i++) {
    for (size_t j = 0; j < errors[i].size(); j++) {
      std::cout << "camera " << i << " line " << j << " errors : " << std::setprecision(10);
      for (int k = 0; k < iterations; k++) {
        std::cout << errors[i][j][k] << " ";
      }
      std::cout << std::endl;
    }
  }
}