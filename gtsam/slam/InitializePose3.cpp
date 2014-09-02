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
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
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

typedef std::map<Key, vector<size_t> > KeyVectorMap;

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
  gttic(InitializePose3_computeOrientationsChordal);

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
// Return the orientations of a graph including only BetweenFactors<Pose3>
Values computeOrientationsChordal(const NonlinearFactorGraph& pose3Graph) {
  gttic(InitializePose3_computeOrientationsChordal);

  // regularize measurements and plug everything in a factor graph
  GaussianFactorGraph relaxedGraph = buildLinearOrientationGraph(pose3Graph);

  // Solve the LFG
  VectorValues relaxedRot3 = relaxedGraph.optimize();

  // normalize and compute Rot3
  return normalizeRelaxedRotations(relaxedRot3);
}

/* ************************************************************************* */
// Return the orientations of a graph including only BetweenFactors<Pose3>
Values computeOrientationsGradient(const NonlinearFactorGraph& pose3Graph, const Values& givenGuess) {
  gttic(InitializePose3_computeOrientationsGradient);

  // this works on the inverse rotations, according to Tron&Vidal,2011
  Values inverseRot;
  BOOST_FOREACH(const Values::ConstKeyValuePair& key_value, givenGuess) {
    Key key = key_value.key;
    const Pose3& pose = givenGuess.at<Pose3>(key);
    inverseRot.insert(key, pose.rotation().inverse());
  }

  // Create the map of edges incident on each node
  KeyVectorMap adjEdgesMap;
//
//  // regularize measurements and plug everything in a factor graph
//  GaussianFactorGraph relaxedGraph = buildLinearOrientationGraph(pose3Graph);
//
//  // Solve the LFG
//  VectorValues relaxedRot3 = relaxedGraph.optimize();

  // Return correct rotations
    Values estimateRot;
    BOOST_FOREACH(const Values::ConstKeyValuePair& key_value, inverseRot) {
      Key key = key_value.key;
      const Rot3& R = inverseRot.at<Rot3>(key);
      estimateRot.insert(key, R.inverse());
    }
  return estimateRot;
}

/* ************************************************************************* */
Values initializeOrientations(const NonlinearFactorGraph& graph) {

  // We "extract" the Pose3 subgraph of the original graph: this
  // is done to properly model priors and avoiding operating on a larger graph
  NonlinearFactorGraph pose3Graph = buildPose3graph(graph);

  // Get orientations from relative orientation measurements
  return computeOrientationsChordal(pose3Graph);
}

///* ************************************************************************* */
Values computePoses(NonlinearFactorGraph& pose3graph,  Values& initialRot) {
  gttic(InitializePose3_computePoses);

  // put into Values structure
  Values initialPose;
  BOOST_FOREACH(const Values::ConstKeyValuePair& key_value, initialRot){
    Key key = key_value.key;
    const Rot3& rot = initialRot.at<Rot3>(key);
    Pose3 initializedPose = Pose3(rot, Point3());
    initialPose.insert(key, initializedPose);
  }
  // add prior
  noiseModel::Unit::shared_ptr priorModel = noiseModel::Unit::Create(6);
  initialPose.insert(keyAnchor, Pose3());
  pose3graph.add(PriorFactor<Pose3>(keyAnchor, Pose3(), priorModel));

  // Create optimizer
  GaussNewtonParams params;
  params.maxIterations = 1;
  GaussNewtonOptimizer optimizer(pose3graph, initialPose, params);
  Values GNresult = optimizer.optimize();

  // put into Values structure
  Values estimate;
  BOOST_FOREACH(const Values::ConstKeyValuePair& key_value, GNresult) {
    Key key = key_value.key;
    if (key != keyAnchor) {
      const Pose3& pose = GNresult.at<Pose3>(key);
      estimate.insert(key, pose);
    }
  }
  return estimate;
}

/* ************************************************************************* */
Values initialize(const NonlinearFactorGraph& graph) {
  gttic(InitializePose3_initialize);

  // We "extract" the Pose3 subgraph of the original graph: this
  // is done to properly model priors and avoiding operating on a larger graph
  NonlinearFactorGraph pose3Graph = buildPose3graph(graph);

  // Get orientations from relative orientation measurements
  Values valueRot3 = computeOrientationsChordal(pose3Graph);

  // Compute the full poses (1 GN iteration on full poses)
  return computePoses(pose3Graph, valueRot3);
}

/* ************************************************************************* */
Values initialize(const NonlinearFactorGraph& graph, const Values& givenGuess, bool useGradient) {
  Values initialValues;

  // We "extract" the Pose3 subgraph of the original graph: this
  // is done to properly model priors and avoiding operating on a larger graph
  NonlinearFactorGraph pose3Graph = buildPose3graph(graph);

  // Get orientations from relative orientation measurements
  Values orientations;
  if(useGradient)
    orientations = computeOrientationsGradient(pose3Graph, givenGuess);
  else
    orientations = computeOrientationsChordal(pose3Graph);

  BOOST_FOREACH(const Values::ConstKeyValuePair& key_value, orientations) {
    Key key = key_value.key;
    if (key != keyAnchor) {
      const Point3& pos = givenGuess.at<Pose3>(key).translation();
      const Rot3& rot = orientations.at<Rot3>(key);
      Pose3 initializedPoses = Pose3(rot, pos);
      initialValues.insert(key, initializedPoses);
    }
  }
  return initialValues;
}

} // end of namespace lago
} // end of namespace gtsam
