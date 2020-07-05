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
#include <gtsam/nonlinear/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/base/timing.h>

#include <boost/math/special_functions.hpp>

using namespace std;

namespace gtsam {

static const Key kAnchorKey = symbol('Z', 9999999);

/* ************************************************************************* */
GaussianFactorGraph InitializePose3::buildLinearOrientationGraph(const NonlinearFactorGraph& g) {

  GaussianFactorGraph linearGraph;

  for(const auto& factor: g) {
    Matrix3 Rij;
    double rotationPrecision = 1.0;

    auto pose3Between = boost::dynamic_pointer_cast<BetweenFactor<Pose3> >(factor);
    if (pose3Between){
      Rij = pose3Between->measured().rotation().matrix();
      Vector precisions = Vector::Zero(6);
      precisions[0] = 1.0; // vector of all zeros except first entry equal to 1
      pose3Between->noiseModel()->whitenInPlace(precisions); // gets marginal precision of first variable
      rotationPrecision = precisions[0]; // rotations first
    }else{
      cout << "Error in buildLinearOrientationGraph" << endl;
    }

    const auto& keys = factor->keys();
    Key key1 = keys[0], key2 = keys[1];
    Matrix M9 = Z_9x9;
    M9.block(0,0,3,3) = Rij;
    M9.block(3,3,3,3) = Rij;
    M9.block(6,6,3,3) = Rij;
    linearGraph.add(key1, -I_9x9, key2, M9, Z_9x1, noiseModel::Isotropic::Precision(9, rotationPrecision));
  }
  // prior on the anchor orientation
  linearGraph.add(
      kAnchorKey, I_9x9,
      (Vector(9) << 1.0, 0.0, 0.0, /*  */ 0.0, 1.0, 0.0, /*  */ 0.0, 0.0, 1.0)
          .finished(),
          noiseModel::Isotropic::Precision(9, 1));
  return linearGraph;
}

/* ************************************************************************* */
// Transform VectorValues into valid Rot3
Values InitializePose3::normalizeRelaxedRotations(
    const VectorValues& relaxedRot3) {
  gttic(InitializePose3_computeOrientationsChordal);

  Values validRot3;
  for(const auto& it: relaxedRot3) {
    Key key = it.first;
    if (key != kAnchorKey) {
      Matrix3 M;
      M << Eigen::Map<const Matrix3>(it.second.data()); // Recover M from vectorized

      // ClosestTo finds rotation matrix closest to H in Frobenius sense
      Rot3 initRot = Rot3::ClosestTo(M.transpose());
      validRot3.insert(key, initRot);
    }
  }
  return validRot3;
}

/* ************************************************************************* */
NonlinearFactorGraph InitializePose3::buildPose3graph(const NonlinearFactorGraph& graph) {
  gttic(InitializePose3_buildPose3graph);
  NonlinearFactorGraph pose3Graph;

  for(const auto& factor: graph) {

    // recast to a between on Pose3
    const auto pose3Between = boost::dynamic_pointer_cast<BetweenFactor<Pose3> >(factor);
    if (pose3Between)
      pose3Graph.add(pose3Between);

    // recast PriorFactor<Pose3> to BetweenFactor<Pose3>
    const auto pose3Prior = boost::dynamic_pointer_cast<PriorFactor<Pose3> >(factor);
    if (pose3Prior)
      pose3Graph.emplace_shared<BetweenFactor<Pose3> >(kAnchorKey, pose3Prior->keys()[0],
              pose3Prior->prior(), pose3Prior->noiseModel());
  }
  return pose3Graph;
}

/* ************************************************************************* */
Values InitializePose3::computeOrientationsChordal(
    const NonlinearFactorGraph& pose3Graph) {
  gttic(InitializePose3_computeOrientationsChordal);

  // regularize measurements and plug everything in a factor graph
  GaussianFactorGraph relaxedGraph = buildLinearOrientationGraph(pose3Graph);

  // Solve the LFG
  VectorValues relaxedRot3 = relaxedGraph.optimize();

  // normalize and compute Rot3
  return normalizeRelaxedRotations(relaxedRot3);
}

/* ************************************************************************* */
Values InitializePose3::computeOrientationsGradient(
    const NonlinearFactorGraph& pose3Graph, const Values& givenGuess,
    const size_t maxIter, const bool setRefFrame) {
  gttic(InitializePose3_computeOrientationsGradient);

  // this works on the inverse rotations, according to Tron&Vidal,2011
  Values inverseRot;
  inverseRot.insert(kAnchorKey, Rot3());
  for(const auto& key_value: givenGuess) {
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
  for(const auto& key_value: inverseRot) {
    Key key = key_value.key;
    grad.insert(key,Z_3x1);
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

  double maxGrad;
  // gradient iterations
  size_t it;
  for (it = 0; it < maxIter; it++) {
    //////////////////////////////////////////////////////////////////////////
    // compute the gradient at each node
    maxGrad = 0;
    for (const auto& key_value : inverseRot) {
      Key key = key_value.key;
      Vector gradKey = Z_3x1;
      // collect the gradient for each edge incident on key
      for (const size_t& factorId : adjEdgesMap.at(key)) {
        Rot3 Rij = factorId2RotMap.at(factorId);
        Rot3 Ri = inverseRot.at<Rot3>(key);
        auto factor = pose3Graph.at(factorId);
        const auto& keys = factor->keys();
        if (key == keys[0]) {
          Key key1 = keys[1];
          Rot3 Rj = inverseRot.at<Rot3>(key1);
          gradKey = gradKey + gradientTron(Ri, Rij * Rj, a, b);
        } else if (key == keys[1]) {
          Key key0 = keys[0];
          Rot3 Rj = inverseRot.at<Rot3>(key0);
          gradKey = gradKey + gradientTron(Ri, Rij.between(Rj), a, b);
        } else {
          cout << "Error in gradient computation" << endl;
        }
      }  // end of i-th gradient computation
      grad.at(key) = stepsize * gradKey;

      double normGradKey = (gradKey).norm();
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

  // Return correct rotations
  const Rot3& Rref = inverseRot.at<Rot3>(kAnchorKey); // This will be set to the identity as so far we included no prior
  Values estimateRot;
  for(const auto& key_value: inverseRot) {
    Key key = key_value.key;
    if (key != kAnchorKey) {
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
void InitializePose3::createSymbolicGraph(KeyVectorMap& adjEdgesMap, KeyRotMap& factorId2RotMap,
                         const NonlinearFactorGraph& pose3Graph) {
  size_t factorId = 0;
  for(const auto& factor: pose3Graph) {
    auto pose3Between = boost::dynamic_pointer_cast<BetweenFactor<Pose3> >(factor);
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
      cout << "Error in createSymbolicGraph" << endl;
    }
    factorId++;
  }
}

/* ************************************************************************* */
Vector3 InitializePose3::gradientTron(const Rot3& R1, const Rot3& R2, const double a, const double b) {
  Vector3 logRot = Rot3::Logmap(R1.between(R2));

  double th = logRot.norm();
  if(th != th){ // the second case means that th = nan (logRot does not work well for +/-pi)
    Rot3 R1pert = R1.compose( Rot3::Expmap(Vector3(0.01, 0.01, 0.01)) ); // some perturbation
    logRot = Rot3::Logmap(R1pert.between(R2));
    th = logRot.norm();
  }
  // exclude small or invalid rotations
  if (th > 1e-5 && th == th) {  // nonzero valid rotations
    logRot = logRot / th;
  } else {
    logRot = Z_3x1;
    th = 0.0;
  }

  double fdot = a*b*th*exp(-b*th);
  return fdot*logRot;
}

/* ************************************************************************* */
Values InitializePose3::initializeOrientations(const NonlinearFactorGraph& graph) {
  // We "extract" the Pose3 subgraph of the original graph: this
  // is done to properly model priors and avoiding operating on a larger graph
  NonlinearFactorGraph pose3Graph = buildPose3graph(graph);

  // Get orientations from relative orientation measurements
  return computeOrientationsChordal(pose3Graph);
}

///* ************************************************************************* */
Values InitializePose3::computePoses(NonlinearFactorGraph& pose3graph,  Values& initialRot) {
  gttic(InitializePose3_computePoses);

  // put into Values structure
  Values initialPose;
  for (const auto& key_value : initialRot) {
    Key key = key_value.key;
    const Rot3& rot = initialRot.at<Rot3>(key);
    Pose3 initializedPose = Pose3(rot, Point3(0, 0, 0));
    initialPose.insert(key, initializedPose);
  }

  // add prior
  noiseModel::Unit::shared_ptr priorModel = noiseModel::Unit::Create(6);
  initialPose.insert(kAnchorKey, Pose3());
  pose3graph.emplace_shared<PriorFactor<Pose3> >(kAnchorKey, Pose3(), priorModel);

  // Create optimizer
  GaussNewtonParams params;
  bool singleIter = true;
  if (singleIter) {
    params.maxIterations = 1;
  } else {
    cout << " \n\n\n\n  performing more than 1 GN iterations \n\n\n" << endl;
    params.setVerbosity("TERMINATION");
  }
  GaussNewtonOptimizer optimizer(pose3graph, initialPose, params);
  Values GNresult = optimizer.optimize();

  // put into Values structure
  Values estimate;
  for (const auto& key_value : GNresult) {
    Key key = key_value.key;
    if (key != kAnchorKey) {
      const Pose3& pose = GNresult.at<Pose3>(key);
      estimate.insert(key, pose);
    }
  }
  return estimate;
}

/* ************************************************************************* */
Values InitializePose3::initialize(const NonlinearFactorGraph& graph, const Values& givenGuess,
                  bool useGradient) {
  gttic(InitializePose3_initialize);
  Values initialValues;

  // We "extract" the Pose3 subgraph of the original graph: this
  // is done to properly model priors and avoiding operating on a larger graph
  NonlinearFactorGraph pose3Graph = buildPose3graph(graph);

  // Get orientations from relative orientation measurements
  Values orientations;
  if (useGradient)
    orientations = computeOrientationsGradient(pose3Graph, givenGuess);
  else
    orientations = computeOrientationsChordal(pose3Graph);

  // Compute the full poses (1 GN iteration on full poses)
  return computePoses(pose3Graph, orientations);
}

/* ************************************************************************* */
Values InitializePose3::initialize(const NonlinearFactorGraph& graph) {
  return initialize(graph, Values(), false);
}

} // namespace gtsam
