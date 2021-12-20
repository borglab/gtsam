/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010-2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

#pragma once

/**
 *  @file  MFAS.h
 *  @brief MFAS class to solve Minimum Feedback Arc Set graph problem
 *  @author Akshay Krishnan
 *  @date September 2020
 */

#include <gtsam/geometry/Unit3.h>
#include <gtsam/inference/Key.h>
#include <gtsam/sfm/BinaryMeasurement.h>

#include <memory>
#include <unordered_map>
#include <vector>

namespace gtsam {

/**
  The MFAS class to solve a Minimum feedback arc set (MFAS)
  problem. We implement the solution from:
  Kyle Wilson and Noah Snavely, "Robust Global Translations with 1DSfM",
  Proceedings of the European Conference on Computer Vision, ECCV 2014

  Given a weighted directed graph, the objective in a Minimum feedback arc set
  problem is to obtain a directed acyclic graph by removing
  edges such that the total weight of removed edges is minimum.

  Although MFAS is a general graph problem and can be applied in many areas,
  this classed was designed for the purpose of outlier rejection in a
  translation averaging for SfM setting. For more details, refer to the above
  paper. The nodes of the graph in this context represents cameras in 3D and the
  edges between them represent unit translations in the world coordinate frame,
  i.e w_aZb is the unit translation from a to b expressed in the world
  coordinate frame. The weights for the edges are obtained by projecting the
  unit translations in a projection direction.
  @addtogroup SFM
*/
class MFAS {
 public:
  // used to represent edges between two nodes in the graph. When used in
  // translation averaging for global SfM
  using KeyPair = std::pair<Key, Key>;
  using TranslationEdges = std::vector<BinaryMeasurement<Unit3>>;

 private:
  // edges with a direction such that all weights are positive
  // i.e, edges that originally had negative weights are flipped
  std::map<KeyPair, double> edgeWeights_;

 public:
  /**
   * @brief Construct from the weighted directed edges
   * between the nodes. Each node is identified by a Key.
   * @param edgeWeights: weights of edges in the graph
   */
  MFAS(const std::map<KeyPair, double> &edgeWeights)
      : edgeWeights_(edgeWeights) {}

  /**
   * @brief Constructor to be used in the context of translation averaging.
   * Here, the nodes of the graph are cameras in 3D and the edges have a unit
   * translation direction between them. The weights of the edges is computed by
   * projecting them along a projection direction.
   * @param relativeTranslations translation directions between the cameras
   * @param projectionDirection direction in which edges are to be projected
   */
  MFAS(const TranslationEdges &relativeTranslations,
       const Unit3 &projectionDirection);

  /**
   * @brief Computes the 1D MFAS ordering of nodes in the graph
   * @return orderedNodes: vector of nodes in the obtained order
   */
  KeyVector computeOrdering() const;

  /**
   * @brief Computes the outlier weights of the graph. We define the outlier
   * weight of a edge to be zero if the edge is an inlier and the magnitude of
   * its edgeWeight if it is an outlier. This function internally calls
   * computeOrdering and uses the obtained ordering to identify outlier edges.
   * @return outlierWeights: map from an edge to its outlier weight.
   */
  std::map<KeyPair, double> computeOutlierWeights() const;
};

typedef std::map<std::pair<Key, Key>, double> KeyPairDoubleMap;

}  // namespace gtsam
