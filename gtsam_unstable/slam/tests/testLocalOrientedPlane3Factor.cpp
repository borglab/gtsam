/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/*
 * @file testOrientedPlane3Factor.cpp
 * @date Dec 19, 2012
 * @author Alex Trevor
 * @brief Tests the OrientedPlane3Factor class
 */

#include <gtsam_unstable/slam/LocalOrientedPlane3Factor.h>

#include <gtsam/slam/OrientedPlane3Factor.h>

#include <gtsam/base/numericalDerivative.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/nonlinear/ISAM2.h>

#include <CppUnitLite/TestHarness.h>

#include <boost/assign/std/vector.hpp>
#include <boost/assign/std.hpp>
#include <boost/bind.hpp>

using namespace boost::assign;
using namespace gtsam;
using namespace std;

GTSAM_CONCEPT_TESTABLE_INST(OrientedPlane3)
GTSAM_CONCEPT_MANIFOLD_INST(OrientedPlane3)

using symbol_shorthand::P;  //< Planes
using symbol_shorthand::X;  //< Pose3

// *************************************************************************
TEST(LocalOrientedPlane3Factor, lm_translation_error) {
  // Tests one pose, two measurements of the landmark that differ in range only.
  // Normal along -x, 3m away
  OrientedPlane3 test_lm0(-1.0, 0.0, 0.0, 3.0);

  NonlinearFactorGraph graph;
 
  // Init pose and prior.  Pose Prior is needed since a single plane measurement
  // does not fully constrain the pose
  Pose3 init_pose = Pose3::identity();
  graph.addPrior(X(0), init_pose, noiseModel::Isotropic::Sigma(6, 0.001));
 
  // Add two landmark measurements, differing in range
  Vector4 measurement0 {-1.0, 0.0, 0.0, 3.0};
  Vector4 measurement1 {-1.0, 0.0, 0.0, 1.0};
  LocalOrientedPlane3Factor factor0(
      measurement0, noiseModel::Isotropic::Sigma(3, 0.1), X(0), X(0), P(0));
  LocalOrientedPlane3Factor factor1(
      measurement1, noiseModel::Isotropic::Sigma(3, 0.1), X(0), X(0), P(0));
  graph.add(factor0);
  graph.add(factor1);

  // Initial Estimate
  Values values;
  values.insert(X(0), init_pose);
  values.insert(P(0), test_lm0);

  // Optimize
  ISAM2 isam2;
  isam2.update(graph, values);
  Values result_values = isam2.calculateEstimate();
  auto optimized_plane_landmark = result_values.at<OrientedPlane3>(P(0));

  // Given two noisy measurements of equal weight, expect result between the two
  OrientedPlane3 expected_plane_landmark(-1.0, 0.0, 0.0, 2.0);
  EXPECT(assert_equal(optimized_plane_landmark, expected_plane_landmark));
}

// *************************************************************************
TEST (LocalOrientedPlane3Factor, lm_rotation_error) {
  // Tests one pose, two measurements of the landmark that differ in angle only.
  // Normal along -x, 3m away
  OrientedPlane3 test_lm0(-1.0/sqrt(1.01), -0.1/sqrt(1.01), 0.0, 3.0);

  NonlinearFactorGraph graph;

  // Init pose and prior.  Pose Prior is needed since a single plane measurement does not fully constrain the pose
  Pose3 init_pose = Pose3::identity();
  graph.addPrior(X(0), init_pose, noiseModel::Isotropic::Sigma(6, 0.001));

  // Add two landmark measurements, differing in angle
  Vector4 measurement0 {-1.0, 0.0, 0.0, 3.0};
  Vector4 measurement1 {0.0, -1.0, 0.0, 3.0};
  LocalOrientedPlane3Factor factor0(measurement0,
      noiseModel::Isotropic::Sigma(3, 0.1), X(0), X(0), P(0));
  LocalOrientedPlane3Factor factor1(measurement1,
      noiseModel::Isotropic::Sigma(3, 0.1), X(0), X(0), P(0));
  graph.add(factor0);
  graph.add(factor1);

  // Initial Estimate
  Values values;
  values.insert(X(0), init_pose);
  values.insert(P(0), test_lm0);

  // Optimize
  ISAM2 isam2;
  isam2.update(graph, values);
  Values result_values = isam2.calculateEstimate();
  auto optimized_plane_landmark = result_values.at<OrientedPlane3>(P(0));

  values.print();
  result_values.print();

  // HessianFactor::shared_ptr hessianFactor = graph.linearizeToHessianFactor(values);
  // const auto hessian = hessianFactor->hessianBlockDiagonal();

  // Matrix hessianP0 = hessian.at(P(0)), hessianX0 = hessian.at(X(0));

  // Eigen::JacobiSVD<Matrix> svdP0(hessianP0, Eigen::ComputeThinU),
  //                          svdX0(hessianX0, Eigen::ComputeThinU);

  // double conditionNumberP0 = svdP0.singularValues()[0] / svdP0.singularValues()[2],
  //        conditionNumberX0 = svdX0.singularValues()[0] / svdX0.singularValues()[5];

  // std::cout << "Hessian P0:\n" << hessianP0 << "\n"
  //     << "Condition number:\n" << conditionNumberP0 << "\n"
  //     << "Singular values:\n" << svdP0.singularValues().transpose() << "\n"
  //     << "SVD U:\n" << svdP0.matrixU() << "\n" << std::endl;

  // std::cout << "Hessian X0:\n" << hessianX0 << "\n"
  //     << "Condition number:\n" << conditionNumberX0 << "\n"
  //     << "Singular values:\n" << svdX0.singularValues().transpose() << "\n"
  //     << "SVD U:\n" << svdX0.matrixU() << "\n" << std::endl;

  // Given two noisy measurements of equal weight, expect result between the two
  OrientedPlane3 expected_plane_landmark(-sqrt(2.0) / 2.0, -sqrt(2.0) / 2.0,
      0.0, 3.0);
  EXPECT(assert_equal(optimized_plane_landmark, expected_plane_landmark));
}

// *************************************************************************
TEST(LocalOrientedPlane3Factor, Derivatives) {
  // Measurement
  OrientedPlane3 p(sqrt(2)/2, -sqrt(2)/2, 0, 5);

  // Linearisation point
  OrientedPlane3 pLin(sqrt(3)/3, -sqrt(3)/3, sqrt(3)/3, 7);
  Pose3 poseLin(Rot3::RzRyRx(0.5*M_PI, -0.3*M_PI, 1.4*M_PI), Point3(1, 2, -4));
  Pose3 anchorPoseLin(Rot3::RzRyRx(-0.1*M_PI, 0.2*M_PI, 1.0*M_PI), Point3(-5, 0, 1));

  // Factor
  Key planeKey(1), poseKey(2), anchorPoseKey(3);
  SharedGaussian noise = noiseModel::Isotropic::Sigma(3, 0.1);
  LocalOrientedPlane3Factor factor(p, noise, poseKey, anchorPoseKey, planeKey);

  // Calculate numerical derivatives
  auto f = boost::bind(&LocalOrientedPlane3Factor::evaluateError, factor,
    _1, _2, _3, boost::none, boost::none, boost::none);
  Matrix numericalH1 = numericalDerivative31<Vector3, Pose3, Pose3, OrientedPlane3>(f, poseLin, anchorPoseLin, pLin);
  Matrix numericalH2 = numericalDerivative32<Vector3, Pose3, Pose3, OrientedPlane3>(f, poseLin, anchorPoseLin, pLin);
  Matrix numericalH3 = numericalDerivative33<Vector3, Pose3, Pose3, OrientedPlane3>(f, poseLin, anchorPoseLin, pLin);

  // Use the factor to calculate the derivative
  Matrix actualH1, actualH2, actualH3;
  factor.evaluateError(poseLin, anchorPoseLin, pLin, actualH1, actualH2, actualH3);

  // Verify we get the expected error
  EXPECT(assert_equal(numericalH1, actualH1, 1e-8));
  EXPECT(assert_equal(numericalH2, actualH2, 1e-8));
  EXPECT(assert_equal(numericalH3, actualH3, 1e-8));
}


// /* ************************************************************************* */
// // Simplified version of the test by Marco Camurri to debug issue #561
// TEST(OrientedPlane3Factor, Issue561Simplified) {
//   // Typedefs
//   using Plane = OrientedPlane3;

//   NonlinearFactorGraph graph;

//   // Setup prior factors
//   Pose3 x0(Rot3::identity(), Vector3(0, 0, 10));
//   auto x0_noise = noiseModel::Isotropic::Sigma(6, 0.01);
//   graph.addPrior<Pose3>(X(0), x0, x0_noise);

//   // Two horizontal planes with different heights, in the world frame.
//   const Plane p1(0,0,1,1), p2(0,0,1,2);

//   auto p1_noise = noiseModel::Diagonal::Sigmas(Vector3{1, 1, 5});
//   graph.addPrior<Plane>(P(1), p1, p1_noise);

//   // ADDING THIS PRIOR MAKES OPTIMIZATION FAIL
//   auto p2_noise = noiseModel::Diagonal::Sigmas(Vector3{1, 1, 5});
//   graph.addPrior<Plane>(P(2), p2, p2_noise);

//   // First plane factor
//   auto p1_measured_from_x0 = p1.transform(x0); // transform p1 to pose x0 as a measurement
//   const auto x0_p1_noise = noiseModel::Isotropic::Sigma(3, 0.05);
//   graph.emplace_shared<OrientedPlane3Factor>(
//       p1_measured_from_x0.planeCoefficients(), x0_p1_noise, X(0), P(1));

//   // Second plane factor
//   auto p2_measured_from_x0 = p2.transform(x0); // transform p2 to pose x0 as a measurement
//   const auto x0_p2_noise = noiseModel::Isotropic::Sigma(3, 0.05);
//   graph.emplace_shared<OrientedPlane3Factor>(
//       p2_measured_from_x0.planeCoefficients(), x0_p2_noise, X(0), P(2));

//   GTSAM_PRINT(graph);

//   // Initial values
//   // Just offset the initial pose by 1m. This is what we are trying to optimize.
//   Values initialEstimate;
//   Pose3 x0_initial = x0.compose(Pose3(Rot3::identity(), Vector3(1,0,0)));
//   initialEstimate.insert(P(1), p1);
//   initialEstimate.insert(P(2), p2);
//   initialEstimate.insert(X(0), x0_initial);

//   // Print Jacobian
//   cout << graph.linearize(initialEstimate)->augmentedJacobian() << endl << endl;

//   // For testing only
//   HessianFactor::shared_ptr hessianFactor = graph.linearizeToHessianFactor(initialEstimate);
//   const auto hessian = hessianFactor->hessianBlockDiagonal();

//   Matrix hessianP1 = hessian.at(P(1)),
//          hessianP2 = hessian.at(P(2)),
// 	 hessianX0 = hessian.at(X(0));

//   Eigen::JacobiSVD<Matrix> svdP1(hessianP1, Eigen::ComputeThinU),
// 	                   svdP2(hessianP2, Eigen::ComputeThinU),
// 			   svdX0(hessianX0, Eigen::ComputeThinU);

//   double conditionNumberP1 = svdP1.singularValues()[0] / svdP1.singularValues()[2],
//          conditionNumberP2 = svdP2.singularValues()[0] / svdP2.singularValues()[2],
// 	 conditionNumberX0 = svdX0.singularValues()[0] / svdX0.singularValues()[5];

//   std::cout << "Hessian P1:\n" << hessianP1 << "\n"
// 	    << "Condition number:\n" << conditionNumberP1 << "\n"
// 	    << "Singular values:\n" << svdP1.singularValues().transpose() << "\n"
// 	    << "SVD U:\n" << svdP1.matrixU() << "\n" << std::endl;

//   std::cout << "Hessian P2:\n" << hessianP2 << "\n"
// 	    << "Condition number:\n" << conditionNumberP2 << "\n"
// 	    << "Singular values:\n" << svdP2.singularValues().transpose() << "\n"
// 	    << "SVD U:\n" << svdP2.matrixU() << "\n" << std::endl;

//   std::cout << "Hessian X0:\n" << hessianX0 << "\n"
// 	    << "Condition number:\n" << conditionNumberX0 << "\n"
// 	    << "Singular values:\n" << svdX0.singularValues().transpose() << "\n"
// 	    << "SVD U:\n" << svdX0.matrixU() << "\n" << std::endl;

//   // std::cout << "Hessian P2:\n" << hessianP2 << std::endl;
//   // std::cout << "Hessian X0:\n" << hessianX0 << std::endl;
  
//   // For testing only

//   // Optimize
//   try {
//     GaussNewtonParams params;
//     //GTSAM_PRINT(graph);
//     //Ordering ordering = list_of(P(1))(P(2))(X(0));  // make sure P1 eliminated first
//     //params.setOrdering(ordering);
//     // params.setLinearSolverType("SEQUENTIAL_QR");  // abundance of caution
//     params.setVerbosity("TERMINATION");  // show info about stopping conditions
//     GaussNewtonOptimizer optimizer(graph, initialEstimate, params);
//     Values result = optimizer.optimize();
//     EXPECT_DOUBLES_EQUAL(0, graph.error(result), 0.1);
//     EXPECT(x0.equals(result.at<Pose3>(X(0))));
//     EXPECT(p1.equals(result.at<Plane>(P(1))));
//     EXPECT(p2.equals(result.at<Plane>(P(2))));
//   } catch (const IndeterminantLinearSystemException &e) {
//     std::cerr << "CAPTURED THE EXCEPTION: " << DefaultKeyFormatter(e.nearbyVariable()) << std::endl;
//     EXPECT(false); // fail if this happens
//   }
// }

/* ************************************************************************* */
int main() {
  srand(time(nullptr));
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
