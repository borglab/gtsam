/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 *  @file graph.h
 *  @brief Graph algorithm using boost library
 *  @author: Kai Ni
 *  @date Jan 11, 2010
 */

#pragma once

#include <map>

#define BOOST_NO_HASH  // to pacify the warnings about depricated headers in boost.graph

#include <boost/graph/graph_traits.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <memory>
#include <gtsam/nonlinear/Values.h>

namespace gtsam {

  // type definitions :

  /**
   * SDGraph is undirected graph with variable keys and double edge weights
   */
  template<class KEY>
  class SDGraph: public boost::adjacency_list<boost::vecS, boost::vecS, boost::undirectedS,
  boost::property<boost::vertex_name_t, KEY>, boost::property<
  boost::edge_weight_t, double> > {
  public:
    typedef typename boost::graph_traits<SDGraph<KEY> >::vertex_descriptor Vertex;
  };

  template<class KEY>
  class SGraph : public boost::adjacency_list<boost::vecS, boost::vecS, boost::directedS,
      boost::property<boost::vertex_name_t, KEY> > {
  public:
    typedef typename boost::graph_traits<SGraph<KEY> >::vertex_descriptor Vertex;
  };

  //typedef boost::graph_traits<SGraph>::vertex_descriptor SVertex;

  /**
   * Map from variable key to parent key
   */
  template<class KEY>
  class PredecessorMap: public std::map<KEY, KEY> {
  public:
    /** convenience insert so we can pass ints for TypedSymbol keys */
    inline void insert(const KEY& key, const KEY& parent) {
      std::map<KEY, KEY>::insert(std::make_pair(key, parent));
    }
  };

  /**
   * Generate a list of keys from a spanning tree represented by its predecessor map
   */
  template<class KEY>
  std::list<KEY> predecessorMap2Keys(const PredecessorMap<KEY>& p_map);

  /**
   * Convert the factor graph to an SDGraph
   * G = Graph type
   * F = Factor type
   * Key = Key type
   */
    template<class G, class F, class KEY> SDGraph<KEY> toBoostGraph(const G& graph);

  /**
   * Build takes a predecessor map, and builds a directed graph corresponding to the tree.
   * G = Graph type
   * V = Vertex type
   */
  template<class G, class V, class KEY>
  std::tuple<G, V, std::map<KEY,V> >  predecessorMap2Graph(const PredecessorMap<KEY>& p_map);

  /**
   * Compose the poses by following the chain specified by the spanning tree
   */
  template<class G, class Factor, class POSE, class KEY>
  std::shared_ptr<Values>
    composePoses(const G& graph, const PredecessorMap<KEY>& tree, const POSE& rootPose);


  /**
   * find the minimum spanning tree using boost graph library
   */
  template<class G, class KEY, class FACTOR2>
  PredecessorMap<KEY> findMinimumSpanningTree(const G& g) ;

  /**
   * Split the graph into two parts: one corresponds to the given spanning tree,
   * and the other corresponds to the rest of the factors
   */
  template<class G, class KEY, class FACTOR2>
  void split(const G& g, const PredecessorMap<KEY>& tree, G& Ab1, G& Ab2) ;


} // namespace gtsam

#include <gtsam/inference/graph-inl.h>
