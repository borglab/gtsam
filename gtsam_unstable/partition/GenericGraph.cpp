/*
 * GenericGraph2D.cpp
 *
 *   Created on: Nov 23, 2010
 *       Author: nikai
 *  Description: generic graph types used in partitioning
 */
#include <iostream>
#include <boost/lexical_cast.hpp>

#include <gtsam/base/DSFVector.h>

#include "GenericGraph.h"

using namespace std;

namespace gtsam { namespace partition {

  /**
   * Note: Need to be able to handle a graph with factors that involve variables not in the given {keys}
   */
  list<vector<size_t> > findIslands(const GenericGraph2D& graph, const vector<size_t>& keys, WorkSpace& workspace,
      const int minNrConstraintsPerCamera, const int minNrConstraintsPerLandmark)
  {
    typedef pair<int, int> IntPair;
    typedef list<sharedGenericFactor2D> FactorList;
    typedef map<IntPair, FactorList::iterator> Connections;

    // create disjoin set forest
    DSFVector dsf(workspace.dsf, keys);

    FactorList factors(graph.begin(), graph.end());
    size_t nrFactors = factors.size();
    FactorList::iterator itEnd;
    workspace.prepareDictionary(keys);
    while (nrFactors) {
      Connections connections;
      bool succeed = false;
      itEnd = factors.end();
      list<FactorList::iterator> toErase;
      for (FactorList::iterator itFactor=factors.begin(); itFactor!=itEnd; itFactor++) {

        // remove invalid factors
        GenericNode2D key1 = (*itFactor)->key1, key2 = (*itFactor)->key2;
        if (workspace.dictionary[key1.index]==-1 || workspace.dictionary[key2.index]==-1) {
          toErase.push_back(itFactor);  nrFactors--; continue;
        }

        size_t label1 = dsf.find(key1.index);
        size_t label2 = dsf.find(key2.index);
        if (label1 == label2) {  toErase.push_back(itFactor);  nrFactors--; continue; }

        // merge two trees if the connection is strong enough, otherwise cache it
        // an odometry factor always merges two islands
        if (key1.type == NODE_POSE_2D && key2.type  == NODE_POSE_2D) {
          toErase.push_back(itFactor); nrFactors--;
          dsf.merge(label1, label2);
          succeed = true;
          break;
        }

        // single landmark island only need one measurement
        if ((dsf.isSingleton(label1)==1 && key1.type == NODE_LANDMARK_2D) ||
            (dsf.isSingleton(label2)==1 && key2.type == NODE_LANDMARK_2D)) {
          toErase.push_back(itFactor); nrFactors--;
          dsf.merge(label1, label2);
          succeed = true;
          break;
        }

        // stack the current factor with the cached constraint
        IntPair labels = (label1 < label2) ? make_pair(label1, label2) : make_pair(label2, label1);
        Connections::iterator itCached = connections.find(labels);
        if (itCached == connections.end()) {
          connections.insert(make_pair(labels, itFactor));
          continue;
        } else {
          GenericNode2D key21 = (*itCached->second)->key1, key22 = (*itCached->second)->key2;
          // if observe the same landmark, we can not merge, abandon the current factor
          if ((key1.index == key21.index && key1.type == NODE_LANDMARK_2D) ||
              (key1.index == key22.index && key1.type == NODE_LANDMARK_2D) ||
              (key2.index == key21.index && key2.type == NODE_LANDMARK_2D) ||
              (key2.index == key22.index && key2.type == NODE_LANDMARK_2D)) {
            toErase.push_back(itFactor); nrFactors--;
            continue;
          } else {
            toErase.push_back(itFactor); nrFactors--;
            toErase.push_back(itCached->second); nrFactors--;
            dsf.merge(label1, label2);
            connections.erase(itCached);
            succeed = true;
            break;
          }
        }
      }

      // erase unused factors
      for(const FactorList::iterator& it: toErase)
        factors.erase(it);

      if (!succeed) break;
    }

    list<vector<size_t> > islands;
    map<size_t, vector<size_t> > arrays = dsf.arrays();
    for(const auto& kv : arrays)
      islands.push_back(kv.second);
    return islands;
  }


  /* ************************************************************************* */
  void print(const GenericGraph2D& graph, const std::string name) {
    cout << name << endl;
    for(const sharedGenericFactor2D& factor_: graph)
      cout << factor_->key1.index << " " << factor_->key2.index << endl;
  }

  /* ************************************************************************* */
  void print(const GenericGraph3D& graph, const std::string name) {
    cout << name << endl;
    for(const sharedGenericFactor3D& factor_: graph)
      cout << factor_->key1.index << " " << factor_->key2.index << " (" <<
      factor_->key1.type << ", " << factor_->key2.type <<")" << endl;
  }

  /* ************************************************************************* */
  // create disjoin set forest
  DSFVector createDSF(const GenericGraph3D& graph, const vector<size_t>& keys, const WorkSpace& workspace) {
    DSFVector dsf(workspace.dsf, keys);
    typedef list<sharedGenericFactor3D> FactorList;

    FactorList factors(graph.begin(), graph.end());
    size_t nrFactors = factors.size();
    FactorList::iterator itEnd;
    while (nrFactors) {

      bool succeed = false;
      itEnd = factors.end();
      list<FactorList::iterator> toErase;
      for (FactorList::iterator itFactor=factors.begin(); itFactor!=itEnd; itFactor++) {

        // remove invalid factors
        if (graph.size() == 178765) cout << "kai21" <<  endl;
        GenericNode3D key1 = (*itFactor)->key1, key2 = (*itFactor)->key2;
        if (graph.size() == 178765) cout << "kai21: " << key1.index << " " << key2.index << endl;
        if (workspace.dictionary[key1.index]==-1 || workspace.dictionary[key2.index]==-1) {
          toErase.push_back(itFactor);  nrFactors--; continue;
        }

        if (graph.size() == 178765) cout << "kai22" << endl;
        size_t label1 = dsf.find(key1.index);
        size_t label2 = dsf.find(key2.index);
        if (label1 == label2) {  toErase.push_back(itFactor);  nrFactors--; continue; }

        if (graph.size() == 178765) cout << "kai23" << endl;
        // merge two trees if the connection is strong enough, otherwise cache it
        // an odometry factor always merges two islands
        if ((key1.type == NODE_POSE_3D && key2.type  == NODE_LANDMARK_3D) ||
            (key1.type == NODE_POSE_3D && key2.type  == NODE_POSE_3D)) {
          toErase.push_back(itFactor); nrFactors--;
          dsf.merge(label1, label2);
          succeed = true;
          break;
        }

        if (graph.size() == 178765) cout << "kai24" << endl;


      }

      // erase unused factors
      for(const FactorList::iterator& it: toErase)
      factors.erase(it);

      if (!succeed) break;
    }
    return dsf;
  }

  /* ************************************************************************* */
  // first check the type of the key (pose or landmark), and then check whether it is singular
  inline bool isSingular(const set<size_t>& singularCameras, const set<size_t>& singularLandmarks, const GenericNode3D& node) {
    switch(node.type) {
    case NODE_POSE_3D:
      return singularCameras.find(node.index) != singularCameras.end(); break;
    case NODE_LANDMARK_3D:
      return singularLandmarks.find(node.index) != singularLandmarks.end(); break;
    default:
      throw runtime_error("unrecognized key type!");
    }
  }

  /* ************************************************************************* */
  void findSingularCamerasLandmarks(const GenericGraph3D& graph, const WorkSpace& workspace,
      const vector<bool>& isCamera, const vector<bool>& isLandmark,
      set<size_t>& singularCameras, set<size_t>& singularLandmarks,  vector<int>& nrConstraints,
      bool& foundSingularCamera, bool& foundSingularLandmark,
      const int minNrConstraintsPerCamera, const int minNrConstraintsPerLandmark) {

    // compute the constraint number per camera
    std::fill(nrConstraints.begin(),  nrConstraints.end(),    0);
    for(const sharedGenericFactor3D& factor_: graph) {
      const int& key1 = factor_->key1.index;
      const int& key2 = factor_->key2.index;
      if (workspace.dictionary[key1] != -1 &&  workspace.dictionary[key2] != -1 &&
          !isSingular(singularCameras, singularLandmarks, factor_->key1) &&
          !isSingular(singularCameras, singularLandmarks, factor_->key2)) {
        nrConstraints[key1]++;
        nrConstraints[key2]++;

        // a single pose constraint is sufficient for stereo, so we add 2 to the counter
        // for a total of 3, i.e. the same as 3 landmarks fully constraining the camera
        if(factor_->key1.type == NODE_POSE_3D && factor_->key2.type == NODE_POSE_3D){
          nrConstraints[key1]+=2;
          nrConstraints[key2]+=2;
        }
      }
    }

    // find singular cameras and landmarks
    foundSingularCamera = false;
    foundSingularLandmark = false;
    for (size_t i=0; i<nrConstraints.size(); i++) {
      if (isCamera[i] && nrConstraints[i] < minNrConstraintsPerCamera &&
          singularCameras.find(i) == singularCameras.end()) {
        singularCameras.insert(i);
        foundSingularCamera = true;
      }
      if (isLandmark[i] && nrConstraints[i] < minNrConstraintsPerLandmark &&
          singularLandmarks.find(i) == singularLandmarks.end()) {
        singularLandmarks.insert(i);
        foundSingularLandmark = true;
      }
    }
  }

  /* ************************************************************************* */
  list<vector<size_t> > findIslands(const GenericGraph3D& graph, const vector<size_t>& keys, WorkSpace& workspace,
      const size_t minNrConstraintsPerCamera, const size_t minNrConstraintsPerLandmark) {

    // create disjoint set forest
    workspace.prepareDictionary(keys);
    DSFVector dsf = createDSF(graph, keys, workspace);

    const bool verbose = false;
    bool foundSingularCamera = true;
    bool foundSingularLandmark = true;

    list<vector<size_t> > islands;
    set<size_t> singularCameras, singularLandmarks;
    vector<bool> isCamera(workspace.dictionary.size(), false);
    vector<bool> isLandmark(workspace.dictionary.size(), false);

    // check the constraint number of every variable
    // find the camera and landmark keys
    for(const sharedGenericFactor3D& factor_: graph) {
      //assert(factor_->key2.type == NODE_LANDMARK_3D); // only VisualSLAM should come here, not StereoSLAM
      if (workspace.dictionary[factor_->key1.index] != -1) {
        if (factor_->key1.type == NODE_POSE_3D)
          isCamera[factor_->key1.index] = true;
        else
          isLandmark[factor_->key1.index] = true;
      }
            if (workspace.dictionary[factor_->key2.index] != -1) {
        if (factor_->key2.type == NODE_POSE_3D)
          isCamera[factor_->key2.index] = true;
        else
          isLandmark[factor_->key2.index] = true;
            }
    }

    vector<int> nrConstraints(workspace.dictionary.size(), 0);
    // iterate until all singular variables have been removed. Removing a singular variable
    // can cause another to become singular, so this will probably run several times
    while (foundSingularCamera || foundSingularLandmark) {
      findSingularCamerasLandmarks(graph, workspace, isCamera, isLandmark,      // input
          singularCameras, singularLandmarks, nrConstraints,                    // output
          foundSingularCamera, foundSingularLandmark,                           // output
          minNrConstraintsPerCamera,  minNrConstraintsPerLandmark);             // input
    }

    // add singular variables directly as islands
    if (!singularCameras.empty()) {
      if (verbose) cout << "singular cameras:";
      for(const size_t i: singularCameras) {
        islands.push_back(vector<size_t>(1, i)); // <---------------------------
        if (verbose) cout << i << " ";
      }
      if (verbose) cout << endl;
    }
    if (!singularLandmarks.empty()) {
      if (verbose) cout << "singular landmarks:";
      for(const size_t i: singularLandmarks) {
        islands.push_back(vector<size_t>(1, i)); // <---------------------------
        if (verbose) cout << i << " ";
      }
      if (verbose) cout << endl;
    }


    // regenerating islands
    map<size_t, vector<size_t> > labelIslands = dsf.arrays();
    size_t label; vector<size_t> island;
    for(const auto& li: labelIslands) {
      tie(label, island) = li;
      vector<size_t> filteredIsland; // remove singular cameras from array
      filteredIsland.reserve(island.size());
      for(const size_t key: island) {
        if ((isCamera[key]   && singularCameras.find(key) == singularCameras.end()) ||        // not singular
            (isLandmark[key] && singularLandmarks.find(key) == singularLandmarks.end()) ||    // not singular
            (!isCamera[key] && !isLandmark[key])) {   // the key is not involved in any factor, so the type is undertermined
          filteredIsland.push_back(key);
        }
      }
      islands.push_back(filteredIsland);
    }

    // sanity check
    size_t nrKeys = 0;
    for(const vector<size_t>& island: islands)
      nrKeys += island.size();
    if (nrKeys != keys.size())  {
      cout << nrKeys << " vs " << keys.size() << endl;
      throw runtime_error("findIslands: the number of keys is inconsistent!");
    }


    if (verbose) cout << "found " << islands.size() << " islands!" << endl;
    return islands;
  }

  /* ************************************************************************* */
  // return the number of intersection between two **sorted** landmark vectors
  inline int getNrCommonLandmarks(const vector<size_t>& landmarks1, const vector<size_t>& landmarks2){
    size_t i1 = 0, i2 = 0;
    int nrCommonLandmarks = 0;
    while (i1 < landmarks1.size() && i2 < landmarks2.size()) {
      if (landmarks1[i1] < landmarks2[i2])
        i1 ++;
      else if (landmarks1[i1] > landmarks2[i2])
        i2 ++;
      else {
        i1++; i2++;
        nrCommonLandmarks ++;
      }
    }
    return nrCommonLandmarks;
  }

  /* ************************************************************************* */
  void reduceGenericGraph(const GenericGraph3D& graph, const std::vector<size_t>& cameraKeys,  const std::vector<size_t>& landmarkKeys,
      const std::vector<int>& dictionary,  GenericGraph3D& reducedGraph) {

    typedef size_t LandmarkKey;
    // get a mapping from each landmark to its connected cameras
    vector<vector<LandmarkKey> > cameraToLandmarks(dictionary.size());
    // for odometry xi-xj where i<j, we always store cameraToCamera[i] = j, otherwise equal to -1 if no odometry
    vector<int> cameraToCamera(dictionary.size(), -1);
    size_t key_i, key_j;
    for(const sharedGenericFactor3D& factor_: graph) {
      if (factor_->key1.type == NODE_POSE_3D) {
        if (factor_->key2.type == NODE_LANDMARK_3D) {// projection factor
          cameraToLandmarks[factor_->key1.index].push_back(factor_->key2.index);
        }
        else { // odometry factor
          if (factor_->key1.index < factor_->key2.index) {
            key_i = factor_->key1.index;
            key_j = factor_->key2.index;
          } else {
            key_i = factor_->key2.index;
            key_j = factor_->key1.index;
          }
          cameraToCamera[key_i] = key_j;
        }
      }
    }

    // sort the landmark keys for the late getNrCommonLandmarks call
    for(vector<LandmarkKey> &landmarks: cameraToLandmarks){
      if (!landmarks.empty())
        std::sort(landmarks.begin(), landmarks.end());
    }

    // generate the reduced graph
    reducedGraph.clear();
    int factorIndex = 0;
    int camera1, camera2, nrTotalConstraints;
    bool hasOdometry;
    for (size_t i1=0; i1<cameraKeys.size()-1; ++i1) {
      for (size_t i2=i1+1; i2<cameraKeys.size(); ++i2) {
        camera1 = cameraKeys[i1];
        camera2 = cameraKeys[i2];
        int nrCommonLandmarks = getNrCommonLandmarks(cameraToLandmarks[camera1], cameraToLandmarks[camera2]);
        hasOdometry =  cameraToCamera[camera1] == camera2;
        if (nrCommonLandmarks > 0 || hasOdometry) {
          nrTotalConstraints = 2 * nrCommonLandmarks + (hasOdometry ? 6 : 0);
          reducedGraph.push_back(std::make_shared<GenericFactor3D>(camera1, camera2,
              factorIndex++, NODE_POSE_3D, NODE_POSE_3D, nrTotalConstraints));
        }
      }
    }
  }

  /* ************************************************************************* */
  void checkSingularity(const GenericGraph3D& graph, const std::vector<size_t>& frontals,
      WorkSpace& workspace, const size_t minNrConstraintsPerCamera, const size_t minNrConstraintsPerLandmark) {
    workspace.prepareDictionary(frontals);
    vector<size_t> nrConstraints(workspace.dictionary.size(), 0);

    // summarize the constraint number
    const vector<int>& dictionary = workspace.dictionary;
    vector<bool> isValidCamera(workspace.dictionary.size(), false);
    vector<bool> isValidLandmark(workspace.dictionary.size(), false);
    for(const sharedGenericFactor3D& factor_: graph) {
      assert(factor_->key1.type == NODE_POSE_3D);
      //assert(factor_->key2.type == NODE_LANDMARK_3D);
      const size_t& key1 = factor_->key1.index;
      const size_t& key2 = factor_->key2.index;
      if (dictionary[key1] == -1 || dictionary[key2] == -1)
        continue;

      isValidCamera[key1] = true;
      if(factor_->key2.type == NODE_LANDMARK_3D)
        isValidLandmark[key2] = true;
      else
        isValidCamera[key2] = true;

      nrConstraints[key1]++;
      nrConstraints[key2]++;

      // a single pose constraint is sufficient for stereo, so we add 2 to the counter
      // for a total of 3, i.e. the same as 3 landmarks fully constraining the camera
      if(factor_->key1.type == NODE_POSE_3D && factor_->key2.type == NODE_POSE_3D){
        nrConstraints[key1]+=2;
        nrConstraints[key2]+=2;
      }
    }

    // find the minimum constraint for cameras and landmarks
    size_t minFoundConstraintsPerCamera = 10000;
    size_t minFoundConstraintsPerLandmark = 10000;

    for (size_t i=0; i<isValidCamera.size(); i++) {
      if (isValidCamera[i]) {
        minFoundConstraintsPerCamera   = std::min(nrConstraints[i], minFoundConstraintsPerCamera);
        if (nrConstraints[i] < minNrConstraintsPerCamera)
              cout << "!!!!!!!!!!!!!!!!!!! camera with " << nrConstraints[i] << " constraint: " << i << endl;
      }

    }
    for (size_t j=0; j<isValidLandmark.size(); j++) {
      if (isValidLandmark[j]) {
        minFoundConstraintsPerLandmark = std::min(nrConstraints[j], minFoundConstraintsPerLandmark);
        if (nrConstraints[j] < minNrConstraintsPerLandmark)
          cout << "!!!!!!!!!!!!!!!!!!! landmark with " << nrConstraints[j] << " constraint: " << j << endl;
      }
    }

    // debug info
    for(const size_t key: frontals) {
      if (isValidCamera[key] && nrConstraints[key] < minNrConstraintsPerCamera)
        cout << "singular camera:" << key << " with " << nrConstraints[key] << " constraints" << endl;
    }

     if (minFoundConstraintsPerCamera < minNrConstraintsPerCamera)
      throw runtime_error("checkSingularity:minConstraintsPerCamera < " + boost::lexical_cast<string>(minFoundConstraintsPerCamera));
    if (minFoundConstraintsPerLandmark < minNrConstraintsPerLandmark)
      throw runtime_error("checkSingularity:minConstraintsPerLandmark < " + boost::lexical_cast<string>(minFoundConstraintsPerLandmark));
  }

}} // namespace
