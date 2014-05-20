/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 *  @file  testPlanarSLAMExample_lago.cpp
 *  @brief Initialize Pose2 in a factor graph using LAGO
 *  (Linear Approximation for Graph Optimization). see papers:
 *
 *  L. Carlone, R. Aragues, J. Castellanos, and B. Bona, A fast and accurate
 *  approximation for planar pose graph optimization, IJRR, 2014.
 *
 *  L. Carlone, R. Aragues, J.A. Castellanos, and B. Bona, A linear approximation
 *  for graph-based simultaneous localization and mapping, RSS, 2011.
 *
 *  @param graph: nonlinear factor graph (can include arbitrary factors but we assume
 *  that there is a subgraph involving Pose2 and betweenFactors)
 *  @return Values: initial guess from LAGO (only pose2 are initialized)
 *
 *  @author Luca Carlone
 *  @author Frank Dellaert
 *  @date   May 14, 2014
 */

#pragma once

#include <gtsam/geometry/Pose2.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/GaussianFactorGraph.h>
#include <gtsam/linear/VectorValues.h>
#include <gtsam/inference/graph.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>

namespace gtsam {

typedef std::map<Key,double> key2doubleMap;
const Key keyAnchor = symbol('Z',9999999);
noiseModel::Diagonal::shared_ptr priorOrientationNoise = noiseModel::Diagonal::Variances((Vector(1) << 1e-8));
noiseModel::Diagonal::shared_ptr priorPose2Noise = noiseModel::Diagonal::Variances((Vector(3) << 1e-6, 1e-6, 1e-8));

/*
 *  This function computes the cumulative orientation (without wrapping) wrt the root of a spanning tree (tree)
 *  for a node (nodeKey). The function starts at the nodes and moves towards the root
 *  summing up the (directed) rotation measurements. Relative measurements are encoded in "deltaThetaMap"
 *  The root is assumed to have orientation zero.
 */
double computeThetaToRoot(const Key nodeKey, const PredecessorMap<Key>& tree,
    const key2doubleMap& deltaThetaMap, const key2doubleMap& thetaFromRootMap) {

  double nodeTheta = 0;
  Key key_child = nodeKey; // the node
  Key key_parent = 0; // the initialization does not matter
  while(1){
    // We check if we reached the root
    if(tree.at(key_child)==key_child) // if we reached the root
      break;
    // we sum the delta theta corresponding to the edge parent->child
    nodeTheta += deltaThetaMap.at(key_child);
    // we get the parent
    key_parent = tree.at(key_child); // the parent
    // we check if we connected to some part of the tree we know
    if(thetaFromRootMap.find(key_parent)!=thetaFromRootMap.end()){
      nodeTheta += thetaFromRootMap.at(key_parent);
      break;
    }
    key_child = key_parent; // we move upwards in the tree
  }
  return nodeTheta;
}

/*
 *  This function computes the cumulative orientations (without wrapping)
 *  for all node wrt the root (root has zero orientation)
 */
key2doubleMap computeThetasToRoot(const key2doubleMap& deltaThetaMap,
    const PredecessorMap<Key>& tree) {

  key2doubleMap thetaToRootMap;
  key2doubleMap::const_iterator it;
  // for all nodes in the tree
  for(it = deltaThetaMap.begin(); it != deltaThetaMap.end(); ++it )
  {
    // compute the orientation wrt root
    Key nodeKey = it->first;
    double nodeTheta = computeThetaToRoot(nodeKey, tree, deltaThetaMap,
        thetaToRootMap);
    thetaToRootMap.insert(std::pair<Key, double>(nodeKey, nodeTheta));
  }
  return thetaToRootMap;
}

/*
 *  Given a factor graph "g", and a spanning tree "tree", the function selects the nodes belonging to the tree and to g,
 *  and stores the factor slots corresponding to edges in the tree and to chordsIds wrt this tree
 *  Also it computes deltaThetaMap which is a fast way to encode relative orientations along the tree:
 *  for a node key2, s.t. tree[key2]=key1, the values deltaThetaMap[key2] is the relative orientation theta[key2]-theta[key1]
 */
void getSymbolicGraph(
    /*OUTPUTS*/ std::vector<size_t>& spanningTreeIds, std::vector<size_t>& chordsIds, key2doubleMap& deltaThetaMap,
    /*INPUTS*/ const PredecessorMap<Key>& tree, const NonlinearFactorGraph& g){

  // Get keys for which you want the orientation
  size_t id=0;
  // Loop over the factors
  BOOST_FOREACH(const boost::shared_ptr<NonlinearFactor>& factor, g){
    if (factor->keys().size() == 2){
      Key key1 = factor->keys()[0];
      Key key2 = factor->keys()[1];

      // recast to a between
      boost::shared_ptr< BetweenFactor<Pose2> > pose2Between =
          boost::dynamic_pointer_cast< BetweenFactor<Pose2> >(factor);
      if (!pose2Between) continue;

      // get the orientation - measured().theta();
      double deltaTheta = pose2Between->measured().theta();

      // insert (directed) orientations in the map "deltaThetaMap"
      bool inTree=false;
      if(tree.at(key1)==key2){
        deltaThetaMap.insert(std::pair<Key, double>(key1, -deltaTheta));
        inTree = true;
      } else if(tree.at(key2)==key1){
        deltaThetaMap.insert(std::pair<Key, double>(key2,  deltaTheta));
        inTree = true;
      }

      // store factor slot, distinguishing spanning tree edges from chordsIds
      if(inTree == true)
        spanningTreeIds.push_back(id);
      else // it's a chord!
        chordsIds.push_back(id);
    }
    id++;
  }
}

/*
 *  Retrieves the deltaTheta and the corresponding noise model from a BetweenFactor<Pose2>
 */
void getDeltaThetaAndNoise(NonlinearFactor::shared_ptr factor,
    Vector& deltaTheta, noiseModel::Diagonal::shared_ptr& model_deltaTheta) {

  // Get the relative rotation measurement from the between factor
  boost::shared_ptr<BetweenFactor<Pose2> > pose2Between =
      boost::dynamic_pointer_cast<BetweenFactor<Pose2> >(factor);
  if (!pose2Between)
    throw std::invalid_argument("buildLinearOrientationGraph: invalid between factor!");
  deltaTheta = (Vector(1) << pose2Between->measured().theta());

  // Retrieve the noise model for the relative rotation
  SharedNoiseModel model = pose2Between->get_noiseModel();
  boost::shared_ptr<noiseModel::Diagonal> diagonalModel =
      boost::dynamic_pointer_cast<noiseModel::Diagonal>(model);
  if (!diagonalModel)
    throw std::invalid_argument("buildLinearOrientationGraph: invalid noise model "
        "(current version assumes diagonal noise model)!");
  Vector std_deltaTheta = (Vector(1) << diagonalModel->sigma(2) ); // std on the angular measurement
  model_deltaTheta = noiseModel::Diagonal::Sigmas(std_deltaTheta);
}

/*
 *  Linear factor graph with regularized orientation measurements
 */
GaussianFactorGraph buildLinearOrientationGraph(const std::vector<size_t>& spanningTreeIds, const std::vector<size_t>& chordsIds,
    const NonlinearFactorGraph& g, const key2doubleMap& orientationsToRoot, const PredecessorMap<Key>& tree){

  GaussianFactorGraph lagoGraph;
  Vector deltaTheta;
  noiseModel::Diagonal::shared_ptr model_deltaTheta;

  Matrix I = eye(1);
  // put original measurements in the spanning tree
  BOOST_FOREACH(const size_t& factorId, spanningTreeIds){
    const FastVector<Key>& keys = g[factorId]->keys();
    Key key1 = keys[0], key2 = keys[1];
    getDeltaThetaAndNoise(g[factorId], deltaTheta, model_deltaTheta);
    lagoGraph.add(JacobianFactor(key1, -I, key2, I, deltaTheta, model_deltaTheta));
  }
  // put regularized measurements in the chordsIds
  BOOST_FOREACH(const size_t& factorId, chordsIds){
    const FastVector<Key>& keys = g[factorId]->keys();
    Key key1 = keys[0], key2 = keys[1];
    getDeltaThetaAndNoise(g[factorId], deltaTheta, model_deltaTheta);
    double key1_DeltaTheta_key2 = deltaTheta(0);
    double k2pi_noise = key1_DeltaTheta_key2 + orientationsToRoot.at(key1) - orientationsToRoot.at(key2); // this coincides to summing up measurements along the cycle induced by the chord
    double k = round(k2pi_noise/(2*M_PI));
    Vector deltaThetaRegularized = (Vector(1) << key1_DeltaTheta_key2 - 2*k*M_PI);
    lagoGraph.add(JacobianFactor(key1, -I, key2, I, deltaThetaRegularized, model_deltaTheta));
  }
  // prior on the anchor orientation
  lagoGraph.add(JacobianFactor(keyAnchor, I, (Vector(1) << 0.0), priorOrientationNoise));
  return lagoGraph;
}

/*
 *  Selects the subgraph of betweenFactors and transforms priors into between wrt a fictitious node
 */
NonlinearFactorGraph buildPose2graph(const NonlinearFactorGraph& graph){
  NonlinearFactorGraph pose2Graph;

  BOOST_FOREACH(const boost::shared_ptr<NonlinearFactor>& factor, graph){

    // recast to a between on Pose2
    boost::shared_ptr< BetweenFactor<Pose2> > pose2Between =
        boost::dynamic_pointer_cast< BetweenFactor<Pose2> >(factor);
    if (pose2Between)
      pose2Graph.add(pose2Between);

    // recast PriorFactor<Pose2> to BetweenFactor<Pose2>
    boost::shared_ptr< PriorFactor<Pose2> > pose2Prior =
        boost::dynamic_pointer_cast< PriorFactor<Pose2> >(factor);
    if (pose2Prior)
      pose2Graph.add(BetweenFactor<Pose2>(keyAnchor, pose2Prior->keys()[0],
          pose2Prior->prior(), pose2Prior->get_noiseModel()));
  }
  return pose2Graph;
}

/*
 *  Returns the orientations of a graph including only BetweenFactors<Pose2>
 */
VectorValues computeLagoOrientations(const NonlinearFactorGraph& pose2Graph){

  // Find a minimum spanning tree
  PredecessorMap<Key> tree = findMinimumSpanningTree<NonlinearFactorGraph, Key, BetweenFactor<Pose2> >(pose2Graph);

  // Create a linear factor graph (LFG) of scalars
  key2doubleMap deltaThetaMap;
  std::vector<size_t> spanningTreeIds; // ids of between factors forming the spanning tree T
  std::vector<size_t> chordsIds;       // ids of between factors corresponding to chordsIds wrt T
  getSymbolicGraph(spanningTreeIds, chordsIds, deltaThetaMap, tree, pose2Graph);

  // temporary structure to correct wraparounds along loops
  key2doubleMap orientationsToRoot = computeThetasToRoot(deltaThetaMap, tree);

  // regularize measurements and plug everything in a factor graph
  GaussianFactorGraph lagoGraph = buildLinearOrientationGraph(spanningTreeIds, chordsIds, pose2Graph, orientationsToRoot, tree);

  // Solve the LFG
  VectorValues orientationsLago = lagoGraph.optimize();

  return orientationsLago;
}

/*
 *  Returns the orientations of the Pose2 in a generic factor graph
 */
VectorValues initializeOrientationsLago(const NonlinearFactorGraph& graph) {

  // We "extract" the Pose2 subgraph of the original graph: this
  // is done to properly model priors and avoiding operating on a larger graph
  NonlinearFactorGraph pose2Graph = buildPose2graph(graph);

  // Get orientations from relative orientation measurements
  return computeLagoOrientations(pose2Graph);
}

/*
 *  Returns the values for the Pose2 in a generic factor graph
 */
Values initializeLago(const NonlinearFactorGraph& graph) {

  // We "extract" the Pose2 subgraph of the original graph: this
  // is done to properly model priors and avoiding operating on a larger graph
  NonlinearFactorGraph pose2Graph = buildPose2graph(graph);

  // Get orientations from relative orientation measurements
  VectorValues orientationsLago = computeLagoOrientations(pose2Graph);

  Values initialGuessLago;
  // for all nodes in the tree
  for(VectorValues::const_iterator it = orientationsLago.begin(); it != orientationsLago.end(); ++it ){
    Key key = it->first;
    Vector orientation = orientationsLago.at(key);
    Pose2 poseLago = Pose2(0.0,0.0,orientation(0));
    initialGuessLago.insert(key, poseLago);
  }
  pose2Graph.add(PriorFactor<Pose2>(keyAnchor, Pose2(), priorPose2Noise));
  GaussNewtonOptimizer pose2optimizer(pose2Graph, initialGuessLago);
  initialGuessLago = pose2optimizer.optimize();
  initialGuessLago.erase(keyAnchor); // that was fictitious
  return initialGuessLago;
}

/*
 *  Only corrects the orientation part in initialGuess
 */
Values initializeLago(const NonlinearFactorGraph& graph, const Values& initialGuess) {
  Values initialGuessLago;

  // get the orientation estimates from LAGO
  VectorValues orientations = initializeOrientationsLago(graph);

  // for all nodes in the tree
  for(VectorValues::const_iterator it = orientations.begin(); it != orientations.end(); ++it ){
    Key key = it->first;
    if (key != keyAnchor){
      Pose2 pose = initialGuess.at<Pose2>(key);
      Vector orientation = orientations.at(key);
      Pose2 poseLago = Pose2(pose.x(),pose.y(),orientation(0));
      initialGuessLago.insert(key, poseLago);
    }
  }
  return initialGuessLago;
}

} // end of namespace gtsam
