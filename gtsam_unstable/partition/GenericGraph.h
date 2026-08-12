/*
 * GenericGraph.h
 *
 *   Created on: Nov 22, 2010
 *       Author: nikai
 *  Description: generic graph types used in partitioning
 */

#pragma once

#include <set>
#include <list>
#include <vector>
#include <stdexcept>
#include <string>
#include <memory>
#include <gtsam_unstable/dllexport.h>

#include "PartitionWorkSpace.h"

namespace gtsam { namespace partition {

  /***************************************************
   * 2D generic factors and their factor graph
   ***************************************************/
  enum GenericNode2DType { NODE_POSE_2D, NODE_LANDMARK_2D };

  /** the index of the node and the type of the node */
  struct GenericNode2D {
    std::size_t index;
    GenericNode2DType type;
    GenericNode2D (const std::size_t& index_in, const GenericNode2DType& type_in) : index(index_in), type(type_in) {}
  };

  /** a factor always involves two nodes/variables for now */
  struct GenericFactor2D {
    GenericNode2D key1;
    GenericNode2D key2;
    int index;          // the factor index in the original nonlinear factor graph
    int weight;         // the weight of the edge
    GenericFactor2D(const size_t index1, const GenericNode2DType type1, const size_t index2, const GenericNode2DType type2, const int index_ = -1, const int weight_ = 1)
    : key1(index1, type1), key2(index2, type2), index(index_), weight(weight_) {}
    GenericFactor2D(const size_t index1, const char type1, const size_t index2, const char type2, const int index_ = -1, const int weight_ = 1)
        : key1(index1, type1 == 'x' ? NODE_POSE_2D : NODE_LANDMARK_2D),
          key2(index2, type2 == 'x' ? NODE_POSE_2D : NODE_LANDMARK_2D), index(index_), weight(weight_) {}
  };

  /** graph is a collection of factors */
  typedef std::shared_ptr<GenericFactor2D> sharedGenericFactor2D;
  typedef std::vector<sharedGenericFactor2D> GenericGraph2D;

  /** merge nodes in DSF using constraints captured by the given graph */
  std::list<std::vector<size_t> > GTSAM_UNSTABLE_EXPORT findIslands(const GenericGraph2D& graph, const std::vector<size_t>& keys, WorkSpace& workspace,
      const int minNrConstraintsPerCamera, const int minNrConstraintsPerLandmark);

  /** eliminate the sensors from generic graph */
  inline void reduceGenericGraph(const GenericGraph2D& graph, const std::vector<size_t>& cameraKeys,  const std::vector<size_t>& landmarkKeys,
      const std::vector<int>& dictionary,  GenericGraph2D& reducedGraph) {
    throw std::runtime_error("reduceGenericGraph 2d not implemented");
  }

  /** check whether the 2D graph is singular (under constrained) , Dummy function for 2D */
  inline void checkSingularity(const GenericGraph2D& graph, const std::vector<size_t>& frontals,
      WorkSpace& workspace, const int minNrConstraintsPerCamera, const int minNrConstraintsPerLandmark) {  return; }

  /** print the graph **/
  void print(const GenericGraph2D& graph, const std::string name = "GenericGraph2D");

  /***************************************************
   * 3D generic factors and their factor graph
   ***************************************************/
  enum GenericNode3DType { NODE_POSE_3D, NODE_LANDMARK_3D };

//  const int minNrConstraintsPerCamera = 7;
//  const int minNrConstraintsPerLandmark = 2;

  /** the index of the node and the type of the node */
  struct GenericNode3D {
    std::size_t index;
    GenericNode3DType type;
    GenericNode3D (const std::size_t& index_in, const GenericNode3DType& type_in) : index(index_in), type(type_in) {}
  };

  /** a factor always involves two nodes/variables for now */
  struct GenericFactor3D {
    GenericNode3D key1;
    GenericNode3D key2;
    int index; // the index in the entire graph, 0-based
    int weight; // the weight of the edge
    GenericFactor3D() :key1(-1, NODE_POSE_3D), key2(-1, NODE_LANDMARK_3D), index(-1), weight(1) {}
    GenericFactor3D(const size_t index1, const size_t index2, const int index_ = -1,
        const GenericNode3DType type1 = NODE_POSE_3D, const GenericNode3DType type2 = NODE_LANDMARK_3D, const int weight_ = 1)
    : key1(index1, type1), key2(index2, type2), index(index_), weight(weight_) {}
  };

  /** graph is a collection of factors */
  typedef std::shared_ptr<GenericFactor3D> sharedGenericFactor3D;
  typedef std::vector<sharedGenericFactor3D> GenericGraph3D;

  /** merge nodes in DSF using constraints captured by the given graph */
  std::list<std::vector<size_t> > GTSAM_UNSTABLE_EXPORT findIslands(const GenericGraph3D& graph, const std::vector<size_t>& keys, WorkSpace& workspace,
      const size_t minNrConstraintsPerCamera, const size_t minNrConstraintsPerLandmark);

  /** eliminate the sensors from generic graph */
  void GTSAM_UNSTABLE_EXPORT reduceGenericGraph(const GenericGraph3D& graph, const std::vector<size_t>& cameraKeys,  const std::vector<size_t>& landmarkKeys,
      const std::vector<int>& dictionary,  GenericGraph3D& reducedGraph);

  /** check whether the 3D graph is singular (under constrained) */
  void checkSingularity(const GenericGraph3D& graph, const std::vector<size_t>& frontals,
      WorkSpace& workspace, const size_t minNrConstraintsPerCamera, const size_t minNrConstraintsPerLandmark);


  /** print the graph **/
  void print(const GenericGraph3D& graph, const std::string name = "GenericGraph3D");

  /***************************************************
   * unary generic factors and their factor graph
   ***************************************************/
  /** a factor involves a single variable */
  struct GenericUnaryFactor {
    GenericNode2D key;
    int index;          // the factor index in the original nonlinear factor graph
    GenericUnaryFactor(const size_t key_, const GenericNode2DType type_, const int index_ = -1)
    : key(key_, type_), index(index_) {}
    GenericUnaryFactor(const size_t key_, const char type_, const int index_ = -1)
    : key(key_, type_ == 'x' ? NODE_POSE_2D : NODE_LANDMARK_2D), index(index_) {}
  };

  /** graph is a collection of factors */
  typedef std::shared_ptr<GenericUnaryFactor> sharedGenericUnaryFactor;
  typedef std::vector<sharedGenericUnaryFactor> GenericUnaryGraph;

  /***************************************************
   *               utility functions
  ***************************************************/
  inline bool hasCommonCamera(const std::set<size_t>& cameras1, const std::set<size_t>& cameras2) {
    if (cameras1.empty() || cameras2.empty())
      throw std::invalid_argument("hasCommonCamera: the input camera set is empty!");
    std::set<size_t>::const_iterator it1 = cameras1.begin();
    std::set<size_t>::const_iterator it2 = cameras2.begin();
    while (it1 != cameras1.end() && it2 != cameras2.end()) {
      if (*it1 == *it2)
        return true;
      else if (*it1 < *it2)
        it1++;
      else
        it2++;
    }
    return false;
  }

}} // namespace
