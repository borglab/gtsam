/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 *  @file  InitializePose3.h
 *  @author Luca Carlone
 *  @author Frank Dellaert
 *  @date   August, 2014
 */

#include <gtsam/slam/InitializePose3.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/base/timing.h>

#include <boost/math/special_functions.hpp>

using namespace std;

namespace gtsam {
namespace InitializePose3 {

//static const Matrix I = eye(1);
static const Matrix I9 = eye(9);
static const Vector zero9 = Vector::Zero(9);
static const Matrix zero33= Matrix::Zero(3,3);

static const Key keyAnchor = symbol('Z', 9999999);

/* ************************************************************************* */
GaussianFactorGraph buildLinearOrientationGraph(const NonlinearFactorGraph& g) {

  GaussianFactorGraph linearGraph;
  noiseModel::Unit::shared_ptr model = noiseModel::Unit::Create(9);

  BOOST_FOREACH(const boost::shared_ptr<NonlinearFactor>& factor, g) {
    Matrix3 Rij;

    boost::shared_ptr<BetweenFactor<Pose3> > pose3Between =
        boost::dynamic_pointer_cast<BetweenFactor<Pose3> >(factor);
    if (pose3Between)
      Rij = pose3Between->measured().rotation().matrix();
    else
      std::cout << "Error in buildLinearOrientationGraph" << std::endl;

    // std::cout << "Rij \n" << Rij << std::endl;

    const FastVector<Key>& keys = factor->keys();
    Key key1 = keys[0], key2 = keys[1];
    Matrix M9 = Matrix::Zero(9,9);
    M9.block(0,0,3,3) = Rij;
    M9.block(3,3,3,3) = Rij;
    M9.block(6,6,3,3) = Rij;
    linearGraph.add(key1, -I9, key2, M9, zero9, model);
  }
  // prior on the anchor orientation
  linearGraph.add(keyAnchor, I9, (Vector(9) << 1.0, 0.0, 0.0,/*  */ 0.0, 1.0, 0.0, /*  */ 0.0, 0.0, 1.0), model);
  return linearGraph;
}

/* ************************************************************************* */
// Transform VectorValues into valid Rot3
Values normalizeRelaxedRotations(const VectorValues& relaxedRot3) {
  gttic(InitializePose3_computeOrientations);

  Matrix ppm = Matrix::Zero(3,3); // plus plus minus
  ppm(0,0) = 1; ppm(1,1) = 1; ppm(2,2) = -1;

  Values validRot3;
  BOOST_FOREACH(const VectorValues::value_type& it, relaxedRot3) {
    Key key = it.first;
    if (key != keyAnchor) {
      const Vector& rotVector = it.second;
      Matrix3 rotMat;
      rotMat(0,0) = rotVector(0); rotMat(0,1) = rotVector(1); rotMat(0,2) = rotVector(2);
      rotMat(1,0) = rotVector(3); rotMat(1,1) = rotVector(4); rotMat(1,2) = rotVector(5);
      rotMat(2,0) = rotVector(6); rotMat(2,1) = rotVector(7); rotMat(2,2) = rotVector(8);

      Matrix U, V; Vector s;
      svd(rotMat, U, s, V);
      Matrix3 normalizedRotMat = U * V.transpose();

      //      std::cout << "rotMat \n" << rotMat << std::endl;
      //      std::cout << "U V' \n" << U * V.transpose() << std::endl;
      //      std::cout << "V \n" << V << std::endl;

      if(normalizedRotMat.determinant() < 0)
        normalizedRotMat = U * ppm * V.transpose();

      Rot3 initRot = Rot3(normalizedRotMat);
      validRot3.insert(key, initRot);
    }
  }
  return validRot3;
}

/* ************************************************************************* */
// Select the subgraph of betweenFactors and transforms priors into between wrt a fictitious node
NonlinearFactorGraph buildPose3graph(const NonlinearFactorGraph& graph) {
  gttic(InitializePose3_buildPose3graph);
  NonlinearFactorGraph pose3Graph;

  BOOST_FOREACH(const boost::shared_ptr<NonlinearFactor>& factor, graph) {

    // recast to a between on Pose3
    boost::shared_ptr<BetweenFactor<Pose3> > pose3Between =
        boost::dynamic_pointer_cast<BetweenFactor<Pose3> >(factor);
    if (pose3Between)
      pose3Graph.add(pose3Between);

    // recast PriorFactor<Pose3> to BetweenFactor<Pose3>
    boost::shared_ptr<PriorFactor<Pose3> > pose3Prior =
        boost::dynamic_pointer_cast<PriorFactor<Pose3> >(factor);
    if (pose3Prior)
      pose3Graph.add(
          BetweenFactor<Pose3>(keyAnchor, pose3Prior->keys()[0],
              pose3Prior->prior(), pose3Prior->get_noiseModel()));
  }
  return pose3Graph;
}

/* ************************************************************************* */
// Return the orientations of a graph including only BetweenFactors<Pose2>
Values computeOrientations(const NonlinearFactorGraph& pose3Graph) {
  gttic(InitializePose3_computeOrientations);

  // regularize measurements and plug everything in a factor graph
  GaussianFactorGraph relaxedGraph = buildLinearOrientationGraph(pose3Graph);

  // Solve the LFG
  VectorValues relaxedRot3 = relaxedGraph.optimize();

  // normalize and compute Rot3
  return normalizeRelaxedRotations(relaxedRot3);
}

/* ************************************************************************* */
Values initializeOrientations(const NonlinearFactorGraph& graph) {

  // We "extract" the Pose3 subgraph of the original graph: this
  // is done to properly model priors and avoiding operating on a larger graph
  NonlinearFactorGraph pose3Graph = buildPose3graph(graph);

  // Get orientations from relative orientation measurements
  return computeOrientations(pose3Graph);
}

///* ************************************************************************* */
//Values computePoses(const NonlinearFactorGraph& pose2graph,
//    VectorValues& orientationsLago) {
//  gttic(lago_computePoses);
//
//  // Linearized graph on full poses
//  GaussianFactorGraph linearPose2graph;
//
//  // We include the linear version of each between factor
//  BOOST_FOREACH(const boost::shared_ptr<NonlinearFactor>& factor, pose2graph) {
//
//    boost::shared_ptr<BetweenFactor<Pose2> > pose2Between =
//        boost::dynamic_pointer_cast<BetweenFactor<Pose2> >(factor);
//
//    if (pose2Between) {
//      Key key1 = pose2Between->keys()[0];
//      double theta1 = orientationsLago.at(key1)(0);
//      double s1 = sin(theta1);
//      double c1 = cos(theta1);
//
//      Key key2 = pose2Between->keys()[1];
//      double theta2 = orientationsLago.at(key2)(0);
//
//      double linearDeltaRot = theta2 - theta1
//          - pose2Between->measured().theta();
//      linearDeltaRot = Rot2(linearDeltaRot).theta(); // to normalize
//
//      double dx = pose2Between->measured().x();
//      double dy = pose2Between->measured().y();
//
//      Vector globalDeltaCart = //
//          (Vector(2) << c1 * dx - s1 * dy, s1 * dx + c1 * dy);
//      Vector b = (Vector(3) << globalDeltaCart, linearDeltaRot); // rhs
//      Matrix J1 = -I3;
//      J1(0, 2) = s1 * dx + c1 * dy;
//      J1(1, 2) = -c1 * dx + s1 * dy;
//      // Retrieve the noise model for the relative rotation
//      boost::shared_ptr<noiseModel::Diagonal> diagonalModel =
//          boost::dynamic_pointer_cast<noiseModel::Diagonal>(
//              pose2Between->get_noiseModel());
//
//      linearPose2graph.add(key1, J1, key2, I3, b, diagonalModel);
//    } else {
//      throw invalid_argument(
//          "computeLagoPoses: cannot manage non between factor here!");
//    }
//  }
//  // add prior
//  linearPose2graph.add(keyAnchor, I3, (Vector(3) << 0.0, 0.0, 0.0),
//      priorPose2Noise);
//
//  // optimize
//  VectorValues posesLago = linearPose2graph.optimize();
//
//  // put into Values structure
//  Values initialGuessLago;
//  BOOST_FOREACH(const VectorValues::value_type& it, posesLago) {
//    Key key = it.first;
//    if (key != keyAnchor) {
//      const Vector& poseVector = it.second;
//      Pose2 poseLago = Pose2(poseVector(0), poseVector(1),
//          orientationsLago.at(key)(0) + poseVector(2));
//      initialGuessLago.insert(key, poseLago);
//    }
//  }
//  return initialGuessLago;
//}

///* ************************************************************************* */
//Values initialize(const NonlinearFactorGraph& graph, bool useOdometricPath) {
//  gttic(lago_initialize);
//
//  // We "extract" the Pose2 subgraph of the original graph: this
//  // is done to properly model priors and avoiding operating on a larger graph
//  NonlinearFactorGraph pose2Graph = buildPose2graph(graph);
//
//  // Get orientations from relative orientation measurements
//  VectorValues orientationsLago = computeOrientations(pose2Graph,
//      useOdometricPath);
//
//  // Compute the full poses
//  return computePoses(pose2Graph, orientationsLago);
//}
//
///* ************************************************************************* */
//Values initialize(const NonlinearFactorGraph& graph,
//    const Values& initialGuess) {
//  Values initialGuessLago;
//
//  // get the orientation estimates from LAGO
//  VectorValues orientations = initializeOrientations(graph);
//
//  // for all nodes in the tree
//  BOOST_FOREACH(const VectorValues::value_type& it, orientations) {
//    Key key = it.first;
//    if (key != keyAnchor) {
//      const Pose2& pose = initialGuess.at<Pose2>(key);
//      const Vector& orientation = it.second;
//      Pose2 poseLago = Pose2(pose.x(), pose.y(), orientation(0));
//      initialGuessLago.insert(key, poseLago);
//    }
//  }
//  return initialGuessLago;
//}

} // end of namespace lago
} // end of namespace gtsam
