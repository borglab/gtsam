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
 * Syntax for the script is ./Pose3SLAMExample_changeKeys input.g2o rewritted.g2o
 * @date Aug 25, 2014
 * @author Luca Carlone
 */

#include <gtsam/slam/dataset.h>
#include <gtsam/slam/BetweenFactor.h>
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

  bool add = false;
  Key firstKey = 8646911284551352320;

  std::cout << "Using reference key: " <<  firstKey  << std::endl;
  if(add)
    std::cout << "adding key "  << std::endl;
  else
    std::cout << "subtracting key "  << std::endl;


  if (argc < 3) {
    std::cout << "Please provide output file to write "   << std::endl;
  } else {
    const string inputFileRewritten = argv[2];
    std::cout << "Rewriting input to file: " << inputFileRewritten << std::endl;
    // Additional: rewrite input with simplified keys 0,1,...
    Values simpleInitial;
    for(const auto key_value: *initial) {
      Key key;
      if(add)
        key = key_value.key + firstKey;
      else
        key = key_value.key - firstKey;

      simpleInitial.insert(key, initial->at(key_value.key));
    }
    NonlinearFactorGraph simpleGraph;
    for(const std::shared_ptr<NonlinearFactor>& factor: *graph) {
      std::shared_ptr<BetweenFactor<Pose3> > pose3Between =
          boost::dynamic_pointer_cast<BetweenFactor<Pose3> >(factor);
      if (pose3Between){
        Key key1, key2;
        if(add){
          key1 = pose3Between->key<1>() + firstKey;
          key2 = pose3Between->key<2>() + firstKey;
        }else{
          key1 = pose3Between->key<1>() - firstKey;
          key2 = pose3Between->key<2>() - firstKey;
        }
        NonlinearFactor::shared_ptr simpleFactor(
            new BetweenFactor<Pose3>(key1, key2, pose3Between->measured(), pose3Between->noiseModel()));
        simpleGraph.add(simpleFactor);
      }
    }
    writeG2o(simpleGraph, simpleInitial, inputFileRewritten);
  }
  return 0;
}
