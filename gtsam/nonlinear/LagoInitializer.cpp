/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 *  @file  LagoInitializer.h
 *  @author Luca Carlone
 *  @author Frank Dellaert
 *  @date   May 14, 2014
 */

#include <gtsam/nonlinear/LagoInitializer.h>
#include <gtsam/slam/dataset.h>

namespace gtsam {

Matrix I = eye(1);
Matrix I3 = eye(3);

/* ************************************************************************* */
double computeThetaToRoot(const Key nodeKey, const PredecessorMap<Key>& tree,
    const key2doubleMap& deltaThetaMap, const key2doubleMap& thetaFromRootMap) {

  double nodeTheta = 0;
  Key key_child = nodeKey; // the node
  Key key_parent = 0; // the initialization does not matter
  ///std::cout << "start" << std::endl;
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
  ///std::cout << "end" << std::endl;
  return nodeTheta;
}

/* ************************************************************************* */
key2doubleMap computeThetasToRoot(const key2doubleMap& deltaThetaMap,
    const PredecessorMap<Key>& tree) {

  key2doubleMap thetaToRootMap;
  key2doubleMap::const_iterator it;

  // Orientation of the roo
  thetaToRootMap.insert(std::pair<Key, double>(keyAnchor, 0.0));

  // for all nodes in the tree
  for(it = deltaThetaMap.begin(); it != deltaThetaMap.end(); ++it ){
    // compute the orientation wrt root
    Key nodeKey = it->first;
    double nodeTheta = computeThetaToRoot(nodeKey, tree, deltaThetaMap,
        thetaToRootMap);
    thetaToRootMap.insert(std::pair<Key, double>(nodeKey, nodeTheta));
  }
  return thetaToRootMap;
}

/* ************************************************************************* */
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
      if(tree.at(key1)==key2){ // key2 -> key1
        deltaThetaMap.insert(std::pair<Key, double>(key1, -deltaTheta));
        inTree = true;
      } else if(tree.at(key2)==key1){ // key1 -> key2
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

  ///g.print("Before detlta map \n");

  key2doubleMap::const_iterator it;
  for(it = deltaThetaMap.begin(); it != deltaThetaMap.end(); ++it ){
    Key nodeKey = it->first;
    ///std::cout << "deltaThMAP = key " << DefaultKeyFormatter(nodeKey) << " th= " << it->second << std::endl;
  }

}

/* ************************************************************************* */
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

/* ************************************************************************* */
GaussianFactorGraph buildLinearOrientationGraph(const std::vector<size_t>& spanningTreeIds, const std::vector<size_t>& chordsIds,
    const NonlinearFactorGraph& g, const key2doubleMap& orientationsToRoot, const PredecessorMap<Key>& tree){

  GaussianFactorGraph lagoGraph;
  Vector deltaTheta;
  noiseModel::Diagonal::shared_ptr model_deltaTheta;

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
    ///std::cout << "REG: key1= " << DefaultKeyFormatter(key1) << " key2= " << DefaultKeyFormatter(key2) << std::endl;
    double k2pi_noise = key1_DeltaTheta_key2 + orientationsToRoot.at(key1) - orientationsToRoot.at(key2); // this coincides to summing up measurements along the cycle induced by the chord
    double k = round(k2pi_noise/(2*M_PI));
    //if (k2pi_noise - 2*k*M_PI > 1e-5) std::cout << k2pi_noise - 2*k*M_PI << std::endl; // for debug
    Vector deltaThetaRegularized = (Vector(1) << key1_DeltaTheta_key2 - 2*k*M_PI);
    lagoGraph.add(JacobianFactor(key1, -I, key2, I, deltaThetaRegularized, model_deltaTheta));
  }
  // prior on the anchor orientation
  lagoGraph.add(JacobianFactor(keyAnchor, I, (Vector(1) << 0.0), priorOrientationNoise));
  return lagoGraph;
}

/* ************************************************************************* */
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

/* ************************************************************************* */
PredecessorMap<Key> findOdometricPath(const NonlinearFactorGraph& pose2Graph) {

  PredecessorMap<Key> tree;
  Key minKey;
  bool minUnassigned = true;

  BOOST_FOREACH(const boost::shared_ptr<NonlinearFactor>& factor, pose2Graph){

    Key key1 = std::min(factor->keys()[0], factor->keys()[1]);
    Key key2 = std::max(factor->keys()[0], factor->keys()[1]);
    if(minUnassigned){
      minKey = key1;
      minUnassigned = false;
    }
    if( key2 - key1 == 1){ // consecutive keys
      tree.insert(key2, key1);
      if(key1 < minKey)
        minKey = key1;
    }
  }
  tree.insert(minKey,keyAnchor);
  tree.insert(keyAnchor,keyAnchor); // root
  return tree;
}

/* ************************************************************************* */
VectorValues computeLagoOrientations(const NonlinearFactorGraph& pose2Graph){

  bool useOdometricPath = true;
  // Find a minimum spanning tree
  PredecessorMap<Key> tree;
  if (useOdometricPath)
    tree = findOdometricPath(pose2Graph);
  else
    tree = findMinimumSpanningTree<NonlinearFactorGraph, Key, BetweenFactor<Pose2> >(pose2Graph);

  ///std::cout << "found spanning tree"  << std::endl;

  // Create a linear factor graph (LFG) of scalars
  key2doubleMap deltaThetaMap;
  std::vector<size_t> spanningTreeIds; // ids of between factors forming the spanning tree T
  std::vector<size_t> chordsIds;       // ids of between factors corresponding to chordsIds wrt T
  getSymbolicGraph(spanningTreeIds, chordsIds, deltaThetaMap, tree, pose2Graph);

  ///std::cout << "found symbolic graph"  << std::endl;

  // temporary structure to correct wraparounds along loops
  key2doubleMap orientationsToRoot = computeThetasToRoot(deltaThetaMap, tree);

  ///std::cout << "computed orientations from root"  << std::endl;

  // regularize measurements and plug everything in a factor graph
  GaussianFactorGraph lagoGraph = buildLinearOrientationGraph(spanningTreeIds, chordsIds, pose2Graph, orientationsToRoot, tree);

  // Solve the LFG
  VectorValues orientationsLago = lagoGraph.optimize();

  return orientationsLago;
}

/* ************************************************************************* */
VectorValues initializeOrientationsLago(const NonlinearFactorGraph& graph) {

  // We "extract" the Pose2 subgraph of the original graph: this
  // is done to properly model priors and avoiding operating on a larger graph
  NonlinearFactorGraph pose2Graph = buildPose2graph(graph);

  // Get orientations from relative orientation measurements
  return computeLagoOrientations(pose2Graph);
}

/* ************************************************************************* */
Values computeLagoPoses(const NonlinearFactorGraph& pose2graph, VectorValues& orientationsLago) {

  // Linearized graph on full poses
  GaussianFactorGraph linearPose2graph;

  // We include the linear version of each between factor
  BOOST_FOREACH(const boost::shared_ptr<NonlinearFactor>& factor, pose2graph){

    boost::shared_ptr< BetweenFactor<Pose2> > pose2Between =
        boost::dynamic_pointer_cast< BetweenFactor<Pose2> >(factor);

    if(pose2Between){
      Key key1 = pose2Between->keys()[0];
      double theta1 = orientationsLago.at(key1)(0);
      double s1 = sin(theta1); double c1 = cos(theta1);

      Key key2 = pose2Between->keys()[1];
      double theta2 = orientationsLago.at(key2)(0);

      double linearDeltaRot = theta2 - theta1 - pose2Between->measured().theta();
      linearDeltaRot = Rot2(linearDeltaRot).theta(); // to normalize
      if(fabs(linearDeltaRot)>M_PI)
        std::cout << "linearDeltaRot " << linearDeltaRot << std::endl;

      double dx = pose2Between->measured().x();
      double dy = pose2Between->measured().y();

      Vector globalDeltaCart = (Vector(2) << c1*dx - s1*dy, s1*dx + c1*dy);
      Vector b = (Vector(3) <<  globalDeltaCart, linearDeltaRot );// rhs
      Matrix J1 = - I3;
      J1(0,2) =  s1*dx + c1*dy;
      J1(1,2) = -c1*dx + s1*dy;
      // Retrieve the noise model for the relative rotation
      boost::shared_ptr<noiseModel::Diagonal> diagonalModel =
          boost::dynamic_pointer_cast<noiseModel::Diagonal>(pose2Between->get_noiseModel());

      linearPose2graph.add(JacobianFactor(key1, J1, key2, I3, b, diagonalModel));
    }else{
      throw std::invalid_argument("computeLagoPoses: cannot manage non between factor here!");
    }
  }
  // add prior
  noiseModel::Diagonal::shared_ptr priorModel = noiseModel::Diagonal::Variances((Vector(3) << 1e-2, 1e-2, 1e-4));
  linearPose2graph.add(JacobianFactor(keyAnchor, I3, (Vector(3) << 0.0,0.0,0.0), priorModel));

  // optimize
  VectorValues posesLago = linearPose2graph.optimize();

  // put into Values structure
  Values initialGuessLago;
  for(VectorValues::const_iterator it = posesLago.begin(); it != posesLago.end(); ++it ){
    Key key = it->first;
    if (key != keyAnchor){
      Vector poseVector = posesLago.at(key);
      Pose2 poseLago = Pose2(poseVector(0),poseVector(1),orientationsLago.at(key)(0)+poseVector(2));
      initialGuessLago.insert(key, poseLago);
    }
  }
  return initialGuessLago;
}

/* ************************************************************************* */
Values initializeLago(const NonlinearFactorGraph& graph) {

  // We "extract" the Pose2 subgraph of the original graph: this
  // is done to properly model priors and avoiding operating on a larger graph
  ///std::cout << "buildPose2graph" << std::endl;
  NonlinearFactorGraph pose2Graph = buildPose2graph(graph);

  // Get orientations from relative orientation measurements
  ///std::cout << "computeLagoOrientations" << std::endl;
  VectorValues orientationsLago = computeLagoOrientations(pose2Graph);

//  VectorValues orientationsLago;
//  NonlinearFactorGraph g;
//  Values orientationsLagoV;
//  readG2o("/home/aspn/Desktop/orientationsNoisyToyGraph.txt", g, orientationsLagoV);
//
//  BOOST_FOREACH(const Values::KeyValuePair& key_val, orientationsLagoV){
//    Key k = key_val.key;
//    double th = orientationsLagoV.at<Pose2>(k).theta();
//    orientationsLago.insert(k,(Vector(1) << th));
//  }
//  orientationsLago.insert(keyAnchor,(Vector(1) << 0.0));

  // Compute the full poses
  ///std::cout << "computeLagoPoses" << std::endl;
  return computeLagoPoses(pose2Graph, orientationsLago);
}

/* ************************************************************************* */
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
