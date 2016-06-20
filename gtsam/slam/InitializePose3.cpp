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

static const Matrix I9 = I_9x9;
static const Vector zero9 = Vector::Zero(9);
static const Matrix zero33 = Z_3x3;

static const Key keyAnchor = symbol('Z', 9999999);

/* ************************************************************************* */
GaussianFactorGraph buildLinearOrientationGraph(const NonlinearFactorGraph& g) {

  GaussianFactorGraph linearGraph;
  noiseModel::Unit::shared_ptr model = noiseModel::Unit::Create(9);

  for(const boost::shared_ptr<NonlinearFactor>& factor: g) {
    Matrix3 Rij;

    boost::shared_ptr<BetweenFactor<Pose3> > pose3Between =
        boost::dynamic_pointer_cast<BetweenFactor<Pose3> >(factor);
    if (pose3Between)
      Rij = pose3Between->measured().rotation().matrix();
    else
      std::cout << "Error in buildLinearOrientationGraph" << std::endl;

    const FastVector<Key>& keys = factor->keys();
    Key key1 = keys[0], key2 = keys[1];
    Matrix M9 = Z_9x9;
    M9.block(0,0,3,3) = Rij;
    M9.block(3,3,3,3) = Rij;
    M9.block(6,6,3,3) = Rij;
    linearGraph.add(key1, -I9, key2, M9, zero9, model);
  }
  // prior on the anchor orientation
  linearGraph.add(keyAnchor, I9, (Vector(9) << 1.0, 0.0, 0.0,/*  */ 0.0, 1.0, 0.0, /*  */ 0.0, 0.0, 1.0).finished(), model);
  return linearGraph;
}

/* ************************************************************************* */
// Transform VectorValues into valid Rot3
Values normalizeRelaxedRotations(const VectorValues& relaxedRot3) {
  gttic(InitializePose3_computeOrientationsChordal);

  Matrix ppm = Z_3x3; // plus plus minus
  ppm(0,0) = 1; ppm(1,1) = 1; ppm(2,2) = -1;

  Values validRot3;
  for(const VectorValues::value_type& it: relaxedRot3) {
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

  for(const boost::shared_ptr<NonlinearFactor>& factor: graph) {

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
              pose3Prior->prior(), pose3Prior->noiseModel()));
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
Values computeOrientationsGradient(const NonlinearFactorGraph& pose3Graph, const Values& givenGuess, const size_t maxIter, const bool setRefFrame) {
  gttic(InitializePose3_computeOrientationsGradient);

  // this works on the inverse rotations, according to Tron&Vidal,2011
  Values inverseRot;
  inverseRot.insert(keyAnchor, Rot3());
  for(const Values::ConstKeyValuePair& key_value: givenGuess) {
    Key key = key_value.key;
    const Pose3& pose = givenGuess.at<Pose3>(key);
    inverseRot.insert(key, pose.rotation().inverse());
  }

  // Create the map of edges incident on each node
  KeyVectorMap adjEdgesMap;
  KeyRotMap factorId2RotMap;

  createSymbolicGraph(adjEdgesMap, factorId2RotMap, pose3Graph);

  // calculate max node degree & allocate gradient
  size_t maxNodeDeg = 0;
  VectorValues grad;
  for(const Values::ConstKeyValuePair& key_value: inverseRot) {
    Key key = key_value.key;
    grad.insert(key,Vector3::Zero());
    size_t currNodeDeg = (adjEdgesMap.at(key)).size();
    if(currNodeDeg > maxNodeDeg)
      maxNodeDeg = currNodeDeg;
  }

  // Create parameters
  double b = 1;
  double f0 = 1/b - (1/b + M_PI) * exp(-b*M_PI);
  double a = (M_PI*M_PI)/(2*f0);
  double rho = 2*a*b;
  double mu_max = maxNodeDeg * rho;
  double stepsize = 2/mu_max; // = 1/(a b dG)

  std::cout <<" b " << b <<" f0 " << f0 <<" a " << a <<" rho " << rho <<" stepsize " << stepsize << " maxNodeDeg "<< maxNodeDeg << std::endl;
  double maxGrad;
  // gradient iterations
  size_t it;
  for(it=0; it < maxIter; it++){
    //////////////////////////////////////////////////////////////////////////
    // compute the gradient at each node
    //std::cout << "it  " << it <<" b " << b <<" f0 " << f0 <<" a " << a
    //   <<" rho " << rho <<" stepsize " << stepsize << " maxNodeDeg "<< maxNodeDeg << std::endl;
    maxGrad = 0;
    for(const Values::ConstKeyValuePair& key_value: inverseRot) {
      Key key = key_value.key;
      //std::cout << "---------------------------key  " << DefaultKeyFormatter(key) << std::endl;
      Vector gradKey = Vector3::Zero();
      // collect the gradient for each edge incident on key
      for(const size_t& factorId: adjEdgesMap.at(key)){
        Rot3 Rij = factorId2RotMap.at(factorId);
        Rot3 Ri = inverseRot.at<Rot3>(key);
        if( key == (pose3Graph.at(factorId))->keys()[0] ){
          Key key1 = (pose3Graph.at(factorId))->keys()[1];
          Rot3 Rj = inverseRot.at<Rot3>(key1);
          gradKey = gradKey + gradientTron(Ri, Rij  * Rj, a, b);
          //std::cout << "key1 " << DefaultKeyFormatter(key1) << " gradientTron(Ri, Rij  * Rj, a, b) \n " << gradientTron(Ri, Rij  * Rj, a, b) << std::endl;
        }else if( key == (pose3Graph.at(factorId))->keys()[1] ){
          Key key0 = (pose3Graph.at(factorId))->keys()[0];
          Rot3 Rj = inverseRot.at<Rot3>(key0);
          gradKey = gradKey + gradientTron(Ri, Rij.between(Rj), a, b);
          //std::cout << "key0 " << DefaultKeyFormatter(key0) << " gradientTron(Ri, Rij.inverse()  * Rj, a, b) \n " << gradientTron(Ri, Rij.between(Rj), a, b) << std::endl;
        }else{
          std::cout << "Error in gradient computation" << std::endl;
        }
      } // end of i-th gradient computation
      grad.at(key) = stepsize *  gradKey;

      double normGradKey = (gradKey).norm();
      //std::cout << "key  " << DefaultKeyFormatter(key) <<" \n grad \n" << grad.at(key) << std::endl;
      if(normGradKey>maxGrad)
        maxGrad = normGradKey;
    } // end of loop over nodes

    //////////////////////////////////////////////////////////////////////////
    // update estimates
    inverseRot = inverseRot.retract(grad);

    //////////////////////////////////////////////////////////////////////////
    // check stopping condition
    if (it>20 && maxGrad < 5e-3)
      break;
  } // enf of gradient iterations

  std::cout << "nr of gradient iterations " << it << "maxGrad " << maxGrad <<  std::endl;

  // Return correct rotations
  const Rot3& Rref = inverseRot.at<Rot3>(keyAnchor); // This will be set to the identity as so far we included no prior
  Values estimateRot;
  for(const Values::ConstKeyValuePair& key_value: inverseRot) {
    Key key = key_value.key;
    if (key != keyAnchor) {
      const Rot3& R = inverseRot.at<Rot3>(key);
      if(setRefFrame)
        estimateRot.insert(key, Rref.compose(R.inverse()));
      else
        estimateRot.insert(key, R.inverse());
    }
  }
  return estimateRot;
}

/* ************************************************************************* */
void createSymbolicGraph(KeyVectorMap& adjEdgesMap, KeyRotMap& factorId2RotMap, const NonlinearFactorGraph& pose3Graph){
  size_t factorId = 0;
  for(const boost::shared_ptr<NonlinearFactor>& factor: pose3Graph) {
    boost::shared_ptr<BetweenFactor<Pose3> > pose3Between =
        boost::dynamic_pointer_cast<BetweenFactor<Pose3> >(factor);
    if (pose3Between){
      Rot3 Rij = pose3Between->measured().rotation();
      factorId2RotMap.insert(pair<Key, Rot3 >(factorId,Rij));

      Key key1 = pose3Between->key1();
      if (adjEdgesMap.find(key1) != adjEdgesMap.end()){  // key is already in
        adjEdgesMap.at(key1).push_back(factorId);
      }else{
        vector<size_t> edge_id;
        edge_id.push_back(factorId);
        adjEdgesMap.insert(pair<Key, vector<size_t> >(key1, edge_id));
      }
      Key key2 = pose3Between->key2();
      if (adjEdgesMap.find(key2) != adjEdgesMap.end()){  // key is already in
        adjEdgesMap.at(key2).push_back(factorId);
      }else{
        vector<size_t> edge_id;
        edge_id.push_back(factorId);
        adjEdgesMap.insert(pair<Key, vector<size_t> >(key2, edge_id));
      }
    }else{
      std::cout << "Error in computeOrientationsGradient" << std::endl;
    }
    factorId++;
  }
}

/* ************************************************************************* */
Vector3 gradientTron(const Rot3& R1, const Rot3& R2, const double a, const double b) {
  Vector3 logRot = Rot3::Logmap(R1.between(R2));

  double th = logRot.norm();
  if(th != th){ // the second case means that th = nan (logRot does not work well for +/-pi)
    Rot3 R1pert = R1.compose( Rot3::Expmap(Vector3(0.01, 0.01, 0.01)) ); // some perturbation
    logRot = Rot3::Logmap(R1pert.between(R2));
    th = logRot.norm();
  }
  // exclude small or invalid rotations
  if (th > 1e-5 && th == th){ // nonzero valid rotations
    logRot = logRot / th;
  }else{
    logRot = Vector3::Zero();
    th = 0.0;
  }

  double fdot = a*b*th*exp(-b*th);
  return fdot*logRot;
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
  for(const Values::ConstKeyValuePair& key_value: initialRot){
    Key key = key_value.key;
    const Rot3& rot = initialRot.at<Rot3>(key);
    Pose3 initializedPose = Pose3(rot, Point3(0, 0, 0));
    initialPose.insert(key, initializedPose);
  }
  // add prior
  noiseModel::Unit::shared_ptr priorModel = noiseModel::Unit::Create(6);
  initialPose.insert(keyAnchor, Pose3());
  pose3graph.add(PriorFactor<Pose3>(keyAnchor, Pose3(), priorModel));

  // Create optimizer
  GaussNewtonParams params;
  bool singleIter = true;
  if(singleIter){
    params.maxIterations = 1;
  }else{
    std::cout << " \n\n\n\n  performing more than 1 GN iterations \n\n\n" <<std::endl;
    params.setVerbosity("TERMINATION");
  }
  GaussNewtonOptimizer optimizer(pose3graph, initialPose, params);
  Values GNresult = optimizer.optimize();

  // put into Values structure
  Values estimate;
  for(const Values::ConstKeyValuePair& key_value: GNresult) {
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

//  orientations.print("orientations\n");

  // Compute the full poses (1 GN iteration on full poses)
  return computePoses(pose3Graph, orientations);

  //  for(const Values::ConstKeyValuePair& key_value: orientations) {
  //    Key key = key_value.key;
  //    if (key != keyAnchor) {
  //      const Point3& pos = givenGuess.at<Pose3>(key).translation();
  //      const Rot3& rot = orientations.at<Rot3>(key);
  //      Pose3 initializedPoses = Pose3(rot, pos);
  //      initialValues.insert(key, initializedPoses);
  //    }
  //  }
  //  return initialValues;
}

} // end of namespace lago
} // end of namespace gtsam
