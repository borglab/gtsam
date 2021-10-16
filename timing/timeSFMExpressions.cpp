/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    timeSFMExpressions.cpp
 * @brief   time CalibratedCamera derivatives, realistic scenario
 * @author  Frank Dellaert
 * @date    October 3, 2014
 */

#include <gtsam/slam/expressions.h>
#include <gtsam/nonlinear/ExpressionFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/linear/GaussianFactorGraph.h>

#include <time.h>
#include <iostream>
#include <iomanip>      // std::setprecision

using namespace std;
using namespace gtsam;

//#define TERNARY

int main() {

  // number of cameras, and points
  static const size_t M=100, N = 10000, n = M*N;

  // Create leaves
  Cal3_S2_ K('K', 0);
  std::vector<Expression<Pose3> > x = createUnknowns<Pose3>(M, 'x');
  std::vector<Expression<Point3> > p = createUnknowns<Point3>(N, 'p');

  // Some parameters needed
  Point2 z(-17, 30);
  SharedNoiseModel model = noiseModel::Unit::Create(2);

  // Create values
  Values values;
  values.insert(Symbol('K', 0), Cal3_S2());
  for (size_t i = 0; i < M; i++)
    values.insert(Symbol('x', i), Pose3());
  for (size_t j = 0; j < N; j++)
    values.insert(Symbol('p', j), Point3(0, 0, 1));

  long timeLog = clock();
  NonlinearFactorGraph graph;
  for (size_t i = 0; i < M; i++) {
    for (size_t j = 0; j < N; j++) {
      NonlinearFactor::shared_ptr f = std::make_shared<
          ExpressionFactor<Point2> >
#ifdef TERNARY
          (model, z, project3(x[i], p[j], K));
#else
          (model, z, uncalibrate(K, project(transformTo(x[i], p[j]))));
#endif
      graph.push_back(f);
    }
  }
  long timeLog2 = clock();
  cout << setprecision(3);
  double seconds = (double) (timeLog2 - timeLog) / CLOCKS_PER_SEC;
  cout << seconds << " seconds to build" << endl;

  timeLog = clock();
  GaussianFactorGraph::shared_ptr gfg = graph.linearize(values);
  timeLog2 = clock();
  seconds = (double) (timeLog2 - timeLog) / CLOCKS_PER_SEC;
  cout << seconds << " seconds to linearize" << endl;
  cout << ((double) seconds * 1000000 / n) << " musecs/call" << endl;

  return 0;
}
