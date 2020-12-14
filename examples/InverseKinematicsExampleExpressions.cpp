/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file InverseKinematicsExampleExpressions.cpp
 * @brief Implement inverse kinematics on a three-link arm using expressions.
 * @date April 15, 2019
 * @author Frank Dellaert
 */

#include <gtsam/geometry/Pose2.h>
#include <gtsam/nonlinear/ExpressionFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/expressions.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/expressions.h>

#include <cmath>

using namespace std;
using namespace gtsam;

// Scalar multiplication of a vector, with derivatives.
inline Vector3 scalarMultiply(const double& s, const Vector3& v,
                              OptionalJacobian<3, 1> Hs,
                              OptionalJacobian<3, 3> Hv) {
  if (Hs) *Hs = v;
  if (Hv) *Hv = s * I_3x3;
  return s * v;
}

// Expression version of scalar product, using above function.
inline Vector3_ operator*(const Double_& s, const Vector3_& v) {
  return Vector3_(&scalarMultiply, s, v);
}

// Expression version of Pose2::Expmap
inline Pose2_ Expmap(const Vector3_& xi) { return Pose2_(&Pose2::Expmap, xi); }

// Main function
int main(int argc, char** argv) {
  // Three-link planar manipulator specification.
  const double L1 = 3.5, L2 = 3.5, L3 = 2.5;    // link lengths
  const Pose2 sXt0(0, L1 + L2 + L3, M_PI / 2);  // end-effector pose at rest
  const Vector3 xi1(0, 0, 1), xi2(L1, 0, 1),
      xi3(L1 + L2, 0, 1);  // screw axes at rest

  // Create Expressions for unknowns
  using symbol_shorthand::Q;
  Double_ q1(Q(1)), q2(Q(2)), q3(Q(3));

  // Forward kinematics expression as product of exponentials
  Pose2_ l1Zl1 = Expmap(q1 * Vector3_(xi1));
  Pose2_ l2Zl2 = Expmap(q2 * Vector3_(xi2));
  Pose2_ l3Zl3 = Expmap(q3 * Vector3_(xi3));
  Pose2_ forward = compose(compose(l1Zl1, l2Zl2), compose(l3Zl3, Pose2_(sXt0)));

  // Create a factor graph with a a single expression factor.
  ExpressionFactorGraph graph;
  Pose2 desiredEndEffectorPose(3, 2, 0);
  auto model = noiseModel::Diagonal::Sigmas(Vector3(0.2, 0.2, 0.1));
  graph.addExpressionFactor(forward, desiredEndEffectorPose, model);

  // Create initial estimate
  Values initial;
  initial.insert(Q(1), 0.1);
  initial.insert(Q(2), 0.2);
  initial.insert(Q(3), 0.3);
  initial.print("\nInitial Estimate:\n");  // print
  GTSAM_PRINT(forward.value(initial));

  // Optimize the initial values using a Levenberg-Marquardt nonlinear optimizer
  LevenbergMarquardtParams params;
  params.setlambdaInitial(1e6);
  LevenbergMarquardtOptimizer optimizer(graph, initial, params);
  Values result = optimizer.optimize();
  result.print("Final Result:\n");

  GTSAM_PRINT(forward.value(result));

  return 0;
}
