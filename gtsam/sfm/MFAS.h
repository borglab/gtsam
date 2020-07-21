/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010-2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

#ifndef __MFAS_H__
#define __MFAS_H__

#include <gtsam/geometry/Unit3.h>
#include <gtsam/inference/Key.h>

#include <map>
#include <memory>
#include <vector>

namespace gtsam {

// used to represent edges between two nodes in the graph
using KeyPair = std::pair<Key, Key>;
using TranslationEdges = std::map<KeyPair, Unit3>;

/**
  The MFAS class to solve a Minimum feedback arc set (MFAS)
  problem. We implement the solution from:
  Kyle Wilson and Noah Snavely, "Robust Global Translations with 1DSfM", 
  Proceedings of the European Conference on Computer Vision, ECCV 2014

  Given a weighted directed graph, the objective in a Minimum feedback arc set
  problem is to obtain a graph that does not contain any cycles by removing
  edges such that the total weight of removed edges is minimum.
  @addtogroup SFM
*/
class MFAS {
 public:
  /**
   * @brief Construct from the nodes in a graph (points in 3D), edges
   * that are translation directions in 3D and the direction in
   * which edges are to be projected.
   * @param nodes Nodes in the graph
   * @param relativeTranslations translation directions between nodes
   * @param projectionDirection direction in which edges are to be projected
   */
  MFAS(const std::shared_ptr<std::vector<Key>> &nodes,
       const std::shared_ptr<TranslationEdges> &relativeTranslations,
       const Unit3 &projectionDirection);

  /**
   * @brief Construct from the nodes in a graph and weighted directed edges
   * between the graph. Not recommended for any purpose other than unit testing. 
   * The computeOutlierWeights method will return an empty output if this constructor 
   * is used. 
   * When used in a translation averaging context, these weights are obtained
   * by projecting edges in a particular direction.
   * @param nodes: Nodes in the graph
   * @param edgeWeights: weights of edges in the graph (map from edge to signed double)
   */
  MFAS(const std::shared_ptr<std::vector<Key>> &nodes,
       const std::map<KeyPair, double> &edgeWeights);

  /**
   * @brief Computes the "outlier weights" of the graph. We define the outlier weight
   * of a edge to be zero if the edge in an inlier and the magnitude of its edgeWeight
   * if it is an outlier. This function can only be used when constructing the 
   * @return outlierWeights: map from an edge to its outlier weight.
   */
  std::map<KeyPair, double> computeOutlierWeights();

  /**
   * @brief Computes the 1D MFAS ordering of nodes in the graph
   * @return orderedNodes: vector of nodes in the obtained order
   */
  std::vector<Key> computeOrdering();

 private:
  // pointer to nodes in the graph
  const std::shared_ptr<std::vector<Key>> nodes_;
  // pointer to translation edges (translation directions between two node points)
  const std::shared_ptr<TranslationEdges> relativeTranslations_;

  // relative translations when the object is initialized without using the actual
  // relative translations, but with the weights from projecting in a certain 
  // direction. This is used for unit testing, but not in practice. 
  std::shared_ptr<TranslationEdges> relativeTranslationsForWeights_;

  // edges with a direction such that all weights are positive
  // i.e, edges that originally had negative weights are flipped
  std::map<KeyPair, double> positiveEdgeWeights_;

  // map from edges to their outlier weight
  std::map<KeyPair, double> outlierWeights_;

  // nodes arranged in the MFAS order
  std::vector<Key> orderedNodes_;

  // map from nodes to their position in the MFAS order
  // used to speed up computation (lookup) when computing outlierWeights_
  FastMap<Key, int> orderedPositions_;
};

}  // namespace gtsam

#endif  // __MFAS_H__
