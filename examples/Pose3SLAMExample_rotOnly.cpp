/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file Pose3SLAMExample_initializePose3.cpp
 * @brief A 3D Pose SLAM example that reads input from g2o, and initializes the Pose3 using InitializePose3
 * Syntax for the script is ./Pose3SLAMExample_initializePose3 input.g2o output.g2o
 * @date Aug 25, 2014
 * @author Luca Carlone
 */

#include <gtsam/slam/InitializePose3.h>
#include <gtsam/slam/dataset.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <fstream>

using namespace std;
using namespace gtsam;

int main(const int argc, const char *argv[]) {

  // Read graph from file
  string g2oFile;
  if (argc < 2)
    g2oFile = findExampleDataFile("pose3example.txt");
  else
    g2oFile = argv[1];

  NonlinearFactorGraph::shared_ptr graph;
  Values::shared_ptr initial;
  bool is3D = true;
  boost::tie(graph, initial) = readG2o(g2oFile, is3D);

  Values initialRot;
  BOOST_FOREACH(const Values::ConstKeyValuePair& key_value, *initial) {
    Key key = key_value.key;
    Pose3 pose = initial->at<Pose3>(key);
    Rot3 R = pose.rotation();
    initialRot.insert(key,R);
  }

  noiseModel::Unit::shared_ptr identityModel = noiseModel::Unit::Create(3);
  NonlinearFactorGraph graphRot;
  BOOST_FOREACH(const NonlinearFactor::shared_ptr& factor, *graph) {
    boost::shared_ptr<BetweenFactor<Pose3> > pose3Between =
        boost::dynamic_pointer_cast<BetweenFactor<Pose3> >(factor);
    if(pose3Between){
      Key key1 = pose3Between->key1();
      Key key2 = pose3Between->key2();
      Pose3 Pij = pose3Between->measured();
      Rot3 Rij = Pij.rotation();
      NonlinearFactor::shared_ptr factorRot(new BetweenFactor<Rot3>(key1, key2, Rij, identityModel));
      graphRot.add(factorRot);
    }else{
      std::cout << "Found a factor that is not a Between<Pose3>: not admitted" << std::endl;
      return 1;
    }
  }
  // Add prior on the first key
  graphRot.add(PriorFactor<Rot3>(0, Rot3(), identityModel));

  std::cout << "Optimizing Rot3 via GN" << std::endl;
  // GaussNewtonParams params;
  GaussNewtonOptimizer optimizer(graphRot, initialRot);
  Values GNrot = optimizer.optimize();
  std::cout << "done!" << std::endl;

  // Wrap estimate as poses to write in g2o format
  Values GNposes;
  BOOST_FOREACH(const Values::ConstKeyValuePair& key_value, GNrot) {
    Key key = key_value.key;
    Rot3 R = GNrot.at<Rot3>(key);
    GNposes.insert(key,Pose3(R,Point3()));
  }

  if (argc < 3) {
    GNrot.print("initialization");
  } else {
    const string outputFile = argv[2];
    std::cout << "Writing results to file: " << outputFile << std::endl;
    writeG2o(*graph, GNposes, outputFile);
    std::cout << "done! " << std::endl;
  }
  return 0;
}
