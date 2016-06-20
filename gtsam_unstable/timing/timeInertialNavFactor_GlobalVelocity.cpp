/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    testInertialNavFactor_GlobalVelocity.cpp
 * @brief   Unit test for the InertialNavFactor_GlobalVelocity
 * @author  Vadim Indelman, Stephen Williams
 */

#include <iostream>
#include <gtsam/navigation/ImuBias.h>
#include <gtsam_unstable/slam/InertialNavFactor_GlobalVelocity.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/inference/Key.h>

using namespace std;
using namespace gtsam;

gtsam::Rot3 world_R_ECEF(
    0.31686,      0.51505,      0.79645,
    0.85173,     -0.52399,            0,
    0.41733,      0.67835,     -0.60471);

gtsam::Vector ECEF_omega_earth(Vector3(0.0, 0.0, 7.292115e-5));
gtsam::Vector world_omega_earth(world_R_ECEF.matrix() * ECEF_omega_earth);

/* ************************************************************************* */
gtsam::Pose3 predictionErrorPose(const Pose3& p1, const Vector3& v1, const imuBias::ConstantBias& b1, const Pose3& p2, const Vector3& v2, const InertialNavFactor_GlobalVelocity<Pose3, Vector3, imuBias::ConstantBias>& factor) {
  return Pose3::Expmap(factor.evaluateError(p1, v1, b1, p2, v2).head(6));
}

gtsam::Vector3 predictionErrorVel(const Pose3& p1, const Vector3& v1, const imuBias::ConstantBias& b1, const Pose3& p2, const Vector3& v2, const InertialNavFactor_GlobalVelocity<Pose3, Vector3, imuBias::ConstantBias>& factor) {
  return factor.evaluateError(p1, v1, b1, p2, v2).tail(3);
}

#include <gtsam/linear/GaussianFactorGraph.h>
/* ************************************************************************* */
int main() {
  gtsam::Key PoseKey1(11);
  gtsam::Key PoseKey2(12);
  gtsam::Key VelKey1(21);
  gtsam::Key VelKey2(22);
  gtsam::Key BiasKey1(31);

  double measurement_dt(0.1);
  Vector world_g(Vector3(0.0, 0.0, 9.81));
  Vector world_rho(Vector3(0.0, -1.5724e-05, 0.0)); // NED system
  gtsam::Vector ECEF_omega_earth(Vector3(0.0, 0.0, 7.292115e-5));
  gtsam::Vector world_omega_earth(world_R_ECEF.matrix() * ECEF_omega_earth);

  SharedGaussian model(noiseModel::Isotropic::Sigma(9, 0.1));

  // Second test: zero angular motion, some acceleration - generated in matlab
  Vector measurement_acc(Vector3(6.501390843381716,  -6.763926150509185,  -2.300389940090343));
  Vector measurement_gyro(Vector3(0.1, 0.2, 0.3));

  InertialNavFactor_GlobalVelocity<Pose3, Vector3, imuBias::ConstantBias> f(PoseKey1, VelKey1, BiasKey1, PoseKey2, VelKey2, measurement_acc, measurement_gyro, measurement_dt, world_g, world_rho, world_omega_earth, model);

  Rot3 R1(0.487316618,   0.125253866,   0.86419557,
       0.580273724,  0.693095498, -0.427669306,
      -0.652537293,  0.709880342,  0.265075427);
  Point3 t1(2.0,1.0,3.0);
  Pose3 Pose1(R1, t1);
  Vector3 Vel1 = Vector(Vector3(0.5,-0.5,0.4));
  Rot3 R2(0.473618898,   0.119523052,  0.872582019,
       0.609241153,   0.67099888, -0.422594037,
      -0.636011287,  0.731761397,  0.244979388);
  Point3 t2 = t1 + Point3(Vel1*measurement_dt);
  Pose3 Pose2(R2, t2);
  Vector dv = measurement_dt * (R1.matrix() * measurement_acc + world_g);
  Vector3 Vel2 = Vel1 + dv;
  imuBias::ConstantBias Bias1;

  Values values;
  values.insert(PoseKey1, Pose1);
  values.insert(PoseKey2, Pose2);
  values.insert(VelKey1,  Vel1);
  values.insert(VelKey2,  Vel2);
  values.insert(BiasKey1, Bias1);

  Ordering ordering;
  ordering.push_back(PoseKey1);
  ordering.push_back(VelKey1);
  ordering.push_back(BiasKey1);
  ordering.push_back(PoseKey2);
  ordering.push_back(VelKey2);

  GaussianFactorGraph graph;
  gttic_(LinearizeTiming);
  for(size_t i = 0; i < 100000; ++i) {
    GaussianFactor::shared_ptr g = f.linearize(values);
    graph.push_back(g);
  }
  gttoc_(LinearizeTiming);
  tictoc_finishedIteration_();
  tictoc_print_();
}

/* ************************************************************************* */
