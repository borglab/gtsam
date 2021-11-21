/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 *  @file  lago.h
 *  @author Luca Carlone
 *  @author Frank Dellaert
 *  @date   May 14, 2014
 */

#include <gtsam/slam/lago.h>

#include <gtsam/slam/InitializePose.h>
#include <gtsam/nonlinear/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/base/timing.h>

#include <boost/math/special_functions.hpp>

using namespace std;

namespace gtsam {
namespace lago {

static const Matrix I = I_1x1;
static const Matrix I3 = I_3x3;

static const noiseModel::Diagonal::shared_ptr priorOrientationNoise =
    noiseModel::Diagonal::Sigmas(Vector1(0));
static const noiseModel::Diagonal::shared_ptr priorPose2Noise =
    noiseModel::Diagonal::Variances(Vector3(1e-6, 1e-6, 1e-8));

/* ************************************************************************* */
/**
 * Compute the cumulative orientation (without wrapping) wrt the root of a
 * spanning tree (tree) for a node (nodeKey). The function starts at the nodes and
 * moves towards the root summing up the (directed) rotation measurements.
 * Relative measurements are encoded in "deltaThetaMap".
 * The root is assumed to have orientation zero.
 */
static double computeThetaToRoot(const Key nodeKey,
    const PredecessorMap<Key>& tree, const key2doubleMap& deltaThetaMap,
    const key2doubleMap& thetaFromRootMap) {

  double nodeTheta = 0;
  Key key_child = nodeKey; // the node
  Key key_parent = 0; // the initialization does not matter
  while (1) {
    // We check if we reached the root
    if (tree.at(key_child) == key_child) // if we reached the root
      break;
    // we sum the delta theta corresponding to the edge parent->child
    nodeTheta += deltaThetaMap.at(key_child);
    // we get the parent
    key_parent = tree.at(key_child); // the parent
    // we check if we connected to some part of the tree we know
    if (thetaFromRootMap.find(key_parent) != thetaFromRootMap.end()) {
      nodeTheta += thetaFromRootMap.at(key_parent);
      break;
    }
    key_child = key_parent; // we move upwards in the tree
  }
  return nodeTheta;
}

/* ************************************************************************* */
key2doubleMap computeThetasToRoot(const key2doubleMap& deltaThetaMap,
    const PredecessorMap<Key>& tree) {

  key2doubleMap thetaToRootMap;

  // Orientation of the roo
  thetaToRootMap.insert(pair<Key, double>(initialize::kAnchorKey, 0.0));

  // for all nodes in the tree
  for(const key2doubleMap::value_type& it: deltaThetaMap) {
    // compute the orientation wrt root
    Key nodeKey = it.first;
    double nodeTheta = computeThetaToRoot(nodeKey, tree, deltaThetaMap,
        thetaToRootMap);
    thetaToRootMap.insert(pair<Key, double>(nodeKey, nodeTheta));
  }
  return thetaToRootMap;
}

/* ************************************************************************* */
void getSymbolicGraph(
/*OUTPUTS*/vector<size_t>& spanningTreeIds, vector<size_t>& chordsIds,
    key2doubleMap& deltaThetaMap,
    /*INPUTS*/const PredecessorMap<Key>& tree, const NonlinearFactorGraph& g) {

  // Get keys for which you want the orientation
  size_t id = 0;
  // Loop over the factors
  for(const std::shared_ptr<NonlinearFactor>& factor: g) {
    if (factor->keys().size() == 2) {
      Key key1 = factor->keys()[0];
      Key key2 = factor->keys()[1];
      // recast to a between
      std::shared_ptr<BetweenFactor<Pose2> > pose2Between =
          std::dynamic_pointer_cast<BetweenFactor<Pose2> >(factor);
      if (!pose2Between)
        continue;
      // get the orientation - measured().theta();
      double deltaTheta = pose2Between->measured().theta();
      // insert (directed) orientations in the map "deltaThetaMap"
      bool inTree = false;
      if (tree.at(key1) == key2) { // key2 -> key1
        deltaThetaMap.insert(pair<Key, double>(key1, -deltaTheta));
        inTree = true;
      } else if (tree.at(key2) == key1) { // key1 -> key2
        deltaThetaMap.insert(pair<Key, double>(key2, deltaTheta));
        inTree = true;
      }
      // store factor slot, distinguishing spanning tree edges from chordsIds
      if (inTree == true)
        spanningTreeIds.push_back(id);
      else
        // it's a chord!
        chordsIds.push_back(id);
    }
    id++;
  }
}

/* ************************************************************************* */
// Retrieve the deltaTheta and the corresponding noise model from a BetweenFactor<Pose2>
static void getDeltaThetaAndNoise(NonlinearFactor::shared_ptr factor,
    Vector& deltaTheta, noiseModel::Diagonal::shared_ptr& model_deltaTheta) {

  // Get the relative rotation measurement from the between factor
  std::shared_ptr<BetweenFactor<Pose2> > pose2Between =
      std::dynamic_pointer_cast<BetweenFactor<Pose2> >(factor);
  if (!pose2Between)
    throw invalid_argument(
        "buildLinearOrientationGraph: invalid between factor!");
  deltaTheta = (Vector(1) << pose2Between->measured().theta()).finished();

  // Retrieve the noise model for the relative rotation
  SharedNoiseModel model = pose2Between->noiseModel();
  std::shared_ptr<noiseModel::Diagonal> diagonalModel =
      std::dynamic_pointer_cast<noiseModel::Diagonal>(model);
  if (!diagonalModel)
    throw invalid_argument("buildLinearOrientationGraph: invalid noise model "
        "(current version assumes diagonal noise model)!");
  Vector std_deltaTheta = (Vector(1) << diagonalModel->sigma(2)).finished(); // std on the angular measurement
  model_deltaTheta = noiseModel::Diagonal::Sigmas(std_deltaTheta);
}

/* ************************************************************************* */
GaussianFactorGraph buildLinearOrientationGraph(
    const vector<size_t>& spanningTreeIds, const vector<size_t>& chordsIds,
    const NonlinearFactorGraph& g, const key2doubleMap& orientationsToRoot,
    const PredecessorMap<Key>& tree) {

  GaussianFactorGraph lagoGraph;
  Vector deltaTheta;
  noiseModel::Diagonal::shared_ptr model_deltaTheta;

  // put original measurements in the spanning tree
  for(const size_t& factorId: spanningTreeIds) {
    const KeyVector& keys = g[factorId]->keys();
    Key key1 = keys[0], key2 = keys[1];
    getDeltaThetaAndNoise(g[factorId], deltaTheta, model_deltaTheta);
    lagoGraph.add(key1, -I, key2, I, deltaTheta, model_deltaTheta);
  }
  // put regularized measurements in the chordsIds
  for(const size_t& factorId: chordsIds) {
    const KeyVector& keys = g[factorId]->keys();
    Key key1 = keys[0], key2 = keys[1];
    getDeltaThetaAndNoise(g[factorId], deltaTheta, model_deltaTheta);
    double key1_DeltaTheta_key2 = deltaTheta(0);
    ///cout << "REG: key1= " << DefaultKeyFormatter(key1) << " key2= " << DefaultKeyFormatter(key2) << endl;
    double k2pi_noise = key1_DeltaTheta_key2 + orientationsToRoot.at(key1)
        - orientationsToRoot.at(key2); // this coincides to summing up measurements along the cycle induced by the chord
    double k = boost::math::round(k2pi_noise / (2 * M_PI));
    //if (k2pi_noise - 2*k*M_PI > 1e-5) cout << k2pi_noise - 2*k*M_PI << endl; // for debug
    Vector deltaThetaRegularized = (Vector(1)
        << key1_DeltaTheta_key2 - 2 * k * M_PI).finished();
    lagoGraph.add(key1, -I, key2, I, deltaThetaRegularized, model_deltaTheta);
  }
  // prior on the anchor orientation
  lagoGraph.add(initialize::kAnchorKey, I, (Vector(1) << 0.0).finished(), priorOrientationNoise);
  return lagoGraph;
}

/* ************************************************************************* */
static PredecessorMap<Key> findOdometricPath(
    const NonlinearFactorGraph& pose2Graph) {

  PredecessorMap<Key> tree;
  Key minKey = initialize::kAnchorKey; // this initialization does not matter
  bool minUnassigned = true;

  for(const std::shared_ptr<NonlinearFactor>& factor: pose2Graph) {

    Key key1 = std::min(factor->keys()[0], factor->keys()[1]);
    Key key2 = std::max(factor->keys()[0], factor->keys()[1]);
    if (minUnassigned) {
      minKey = key1;
      minUnassigned = false;
    }
    if (key2 - key1 == 1) { // consecutive keys
      tree.insert(key2, key1);
      if (key1 < minKey)
        minKey = key1;
    }
  }
  tree.insert(minKey, initialize::kAnchorKey);
  tree.insert(initialize::kAnchorKey, initialize::kAnchorKey); // root
  return tree;
}

/* ************************************************************************* */
// Return the orientations of a graph including only BetweenFactors<Pose2>
static VectorValues computeOrientations(const NonlinearFactorGraph& pose2Graph,
    bool useOdometricPath) {
  gttic(lago_computeOrientations);

  // Find a minimum spanning tree
  PredecessorMap<Key> tree;
  if (useOdometricPath)
    tree = findOdometricPath(pose2Graph);
  else
    tree = findMinimumSpanningTree<NonlinearFactorGraph, Key,
        BetweenFactor<Pose2> >(pose2Graph);

  // Create a linear factor graph (LFG) of scalars
  key2doubleMap deltaThetaMap;
  vector<size_t> spanningTreeIds; // ids of between factors forming the spanning tree T
  vector<size_t> chordsIds; // ids of between factors corresponding to chordsIds wrt T
  getSymbolicGraph(spanningTreeIds, chordsIds, deltaThetaMap, tree, pose2Graph);

  // temporary structure to correct wraparounds along loops
  key2doubleMap orientationsToRoot = computeThetasToRoot(deltaThetaMap, tree);

  // regularize measurements and plug everything in a factor graph
  GaussianFactorGraph lagoGraph = buildLinearOrientationGraph(spanningTreeIds,
      chordsIds, pose2Graph, orientationsToRoot, tree);

  // Solve the LFG
  VectorValues orientationsLago = lagoGraph.optimize();

  return orientationsLago;
}

/* ************************************************************************* */
VectorValues initializeOrientations(const NonlinearFactorGraph& graph,
    bool useOdometricPath) {

  // We "extract" the Pose2 subgraph of the original graph: this
  // is done to properly model priors and avoiding operating on a larger graph
  NonlinearFactorGraph pose2Graph = initialize::buildPoseGraph<Pose2>(graph);

  // Get orientations from relative orientation measurements
  return computeOrientations(pose2Graph, useOdometricPath);
}

/* ************************************************************************* */
Values computePoses(const NonlinearFactorGraph& pose2graph,
    VectorValues& orientationsLago) {
  gttic(lago_computePoses);

  // Linearized graph on full poses
  GaussianFactorGraph linearPose2graph;

  // We include the linear version of each between factor
  for(const std::shared_ptr<NonlinearFactor>& factor: pose2graph) {

    std::shared_ptr<BetweenFactor<Pose2> > pose2Between =
        std::dynamic_pointer_cast<BetweenFactor<Pose2> >(factor);

    if (pose2Between) {
      Key key1 = pose2Between->keys()[0];
      double theta1 = orientationsLago.at(key1)(0);
      double s1 = sin(theta1);
      double c1 = cos(theta1);

      Key key2 = pose2Between->keys()[1];
      double theta2 = orientationsLago.at(key2)(0);

      double linearDeltaRot = theta2 - theta1
          - pose2Between->measured().theta();
      linearDeltaRot = Rot2(linearDeltaRot).theta(); // to normalize

      double dx = pose2Between->measured().x();
      double dy = pose2Between->measured().y();

      Vector globalDeltaCart = //
          (Vector(2) << c1 * dx - s1 * dy, s1 * dx + c1 * dy).finished();
      Vector b = (Vector(3) << globalDeltaCart, linearDeltaRot).finished(); // rhs
      Matrix J1 = -I3;
      J1(0, 2) = s1 * dx + c1 * dy;
      J1(1, 2) = -c1 * dx + s1 * dy;
      // Retrieve the noise model for the relative rotation
      std::shared_ptr<noiseModel::Diagonal> diagonalModel =
          std::dynamic_pointer_cast<noiseModel::Diagonal>(
              pose2Between->noiseModel());

      linearPose2graph.add(key1, J1, key2, I3, b, diagonalModel);
    } else {
      throw invalid_argument(
          "computeLagoPoses: cannot manage non between factor here!");
    }
  }
  // add prior
  linearPose2graph.add(initialize::kAnchorKey, I3, Vector3(0.0, 0.0, 0.0),
      priorPose2Noise);

  // optimize
  VectorValues posesLago = linearPose2graph.optimize();

  // put into Values structure
  Values initialGuessLago;
  for(const VectorValues::value_type& it: posesLago) {
    Key key = it.first;
    if (key != initialize::kAnchorKey) {
      const Vector& poseVector = it.second;
      Pose2 poseLago = Pose2(poseVector(0), poseVector(1),
          orientationsLago.at(key)(0) + poseVector(2));
      initialGuessLago.insert(key, poseLago);
    }
  }
  return initialGuessLago;
}

/* ************************************************************************* */
Values initialize(const NonlinearFactorGraph& graph, bool useOdometricPath) {
  gttic(lago_initialize);

  // We "extract" the Pose2 subgraph of the original graph: this
  // is done to properly model priors and avoiding operating on a larger graph
  NonlinearFactorGraph pose2Graph = initialize::buildPoseGraph<Pose2>(graph);

  // Get orientations from relative orientation measurements
  VectorValues orientationsLago = computeOrientations(pose2Graph,
      useOdometricPath);

  // Compute the full poses
  return computePoses(pose2Graph, orientationsLago);
}

/* ************************************************************************* */
Values initialize(const NonlinearFactorGraph& graph,
    const Values& initialGuess) {
  Values initialGuessLago;

  // get the orientation estimates from LAGO
  VectorValues orientations = initializeOrientations(graph);

  // for all nodes in the tree
  for(const VectorValues::value_type& it: orientations) {
    Key key = it.first;
    if (key != initialize::kAnchorKey) {
      const Pose2& pose = initialGuess.at<Pose2>(key);
      const Vector& orientation = it.second;
      Pose2 poseLago = Pose2(pose.x(), pose.y(), orientation(0));
      initialGuessLago.insert(key, poseLago);
    }
  }
  return initialGuessLago;
}

} // end of namespace lago
} // end of namespace gtsam
