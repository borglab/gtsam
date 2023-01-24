/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/*
 * @file graph-inl.h
 * @brief Graph algorithm using boost library
 * @author Kai Ni
 */

#pragma once

#include <stdexcept>
#ifdef __GNUC__
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-variable"
//#pragma GCC diagnostic ignored "-Wunneeded-internal-declaration"
#endif
#include <boost/graph/breadth_first_search.hpp>
#ifdef __GNUC__
#pragma GCC diagnostic pop
#endif
#include <boost/graph/prim_minimum_spanning_tree.hpp>

#include <gtsam/inference/graph.h>

namespace gtsam {

/* ************************************************************************* */
template <class KEY>
class ordering_key_visitor : public boost::default_bfs_visitor {
public:
  ordering_key_visitor(std::list<KEY>& ordering_in) : ordering_(ordering_in) {}
  template <typename Vertex, typename Graph> void discover_vertex(Vertex v, const Graph& g) const {
    KEY key = boost::get(boost::vertex_name, g, v);
    ordering_.push_front(key);
  }
  std::list<KEY>& ordering_;
};

/* ************************************************************************* */
template<class KEY>
std::list<KEY> predecessorMap2Keys(const PredecessorMap<KEY>& p_map) {

  typedef typename SGraph<KEY>::Vertex SVertex;

  SGraph<KEY> g;
  SVertex root;
  std::map<KEY, SVertex> key2vertex;
  std::tie(g, root, key2vertex) = gtsam::predecessorMap2Graph<SGraph<KEY>, SVertex, KEY>(p_map);

  // breadth first visit on the graph
  std::list<KEY> keys;
  ordering_key_visitor<KEY> vis(keys);
  boost::breadth_first_search(g, root, boost::visitor(vis));
  return keys;
}

/* ************************************************************************* */
template<class G, class F, class KEY>
SDGraph<KEY> toBoostGraph(const G& graph) {
  // convert the factor graph to boost graph
  SDGraph<KEY> g;
  typedef typename boost::graph_traits<SDGraph<KEY> >::vertex_descriptor BoostVertex;
  std::map<KEY, BoostVertex> key2vertex;
  typename G::const_iterator itFactor;

  // Loop over the factors
  for(itFactor=graph.begin(); itFactor!=graph.end(); itFactor++) {

    // Ignore factors that are not binary
    if ((*itFactor)->keys().size() != 2)
      continue;

    // Cast the factor to the user-specified factor type F
    std::shared_ptr<F> factor = std::dynamic_pointer_cast<F>(*itFactor);
    // Ignore factors that are not of type F
    if (!factor) continue;

    // Retrieve the 2 keys (nodes) the factor (edge) is incident on
    KEY key1 = factor->keys()[0];
    KEY key2 = factor->keys()[1];

    BoostVertex v1, v2;

    // If key1 is a new key, add it to the key2vertex map, else get the corresponding vertex id
    if (key2vertex.find(key1) == key2vertex.end()) {
         v1 = add_vertex(key1, g);
         key2vertex.insert(std::pair<KEY,KEY>(key1, v1));
       } else
         v1 = key2vertex[key1];

    // If key2 is a new key, add it to the key2vertex map, else get the corresponding vertex id
    if (key2vertex.find(key2) == key2vertex.end()) {
       v2 = add_vertex(key2, g);
       key2vertex.insert(std::pair<KEY,KEY>(key2, v2));
     } else
       v2 = key2vertex[key2];

    // Add an edge with weight 1.0
    boost::property<boost::edge_weight_t, double> edge_property(1.0);  // assume constant edge weight here
    boost::add_edge(v1, v2, edge_property, g);
  }

  return g;
}

/* ************************************************************************* */
template<class G, class V, class KEY>
std::tuple<G, V, std::map<KEY,V> >
predecessorMap2Graph(const PredecessorMap<KEY>& p_map) {

  G g;
  std::map<KEY, V> key2vertex;
  V v1, v2, root;
  bool foundRoot = false;
  for(auto child_parent: p_map) {
    KEY child, parent;
    std::tie(child,parent) = child_parent;
    if (key2vertex.find(child) == key2vertex.end()) {
       v1 = add_vertex(child, g);
       key2vertex.insert(std::make_pair(child, v1));
     } else
       v1 = key2vertex[child];

    if (key2vertex.find(parent) == key2vertex.end()) {
       v2 = add_vertex(parent, g);
       key2vertex.insert(std::make_pair(parent, v2));
     } else
       v2 = key2vertex[parent];

    if (child==parent) {
      root = v1;
      foundRoot = true;
    } else
      boost::add_edge(v2, v1, g); // edge is from parent to child
  }

  if (!foundRoot)
    throw std::invalid_argument("predecessorMap2Graph: invalid predecessor map!");
  else
    return std::tuple<G, V, std::map<KEY, V> >(g, root, key2vertex);
}

/* ************************************************************************* */
template <class V, class POSE, class KEY>
class compose_key_visitor : public boost::default_bfs_visitor {

private:
  std::shared_ptr<Values> config_;

public:

  compose_key_visitor(std::shared_ptr<Values> config_in) {config_ = config_in;}

  template <typename Edge, typename Graph> void tree_edge(Edge edge, const Graph& g) const {
    KEY key_from = boost::get(boost::vertex_name, g, boost::source(edge, g));
    KEY key_to = boost::get(boost::vertex_name, g, boost::target(edge, g));
    POSE relativePose = boost::get(boost::edge_weight, g, edge);
    config_->insert(key_to, config_->at<POSE>(key_from).compose(relativePose));
  }

};

/* ************************************************************************* */
template<class G, class Factor, class POSE, class KEY>
std::shared_ptr<Values> composePoses(const G& graph, const PredecessorMap<KEY>& tree,
    const POSE& rootPose) {

  //TODO: change edge_weight_t to edge_pose_t
  typedef typename boost::adjacency_list<
    boost::vecS, boost::vecS, boost::directedS,
    boost::property<boost::vertex_name_t, KEY>,
    boost::property<boost::edge_weight_t, POSE> > PoseGraph;
  typedef typename boost::graph_traits<PoseGraph>::vertex_descriptor PoseVertex;
  typedef typename boost::graph_traits<PoseGraph>::edge_descriptor PoseEdge;

  PoseGraph g;
  PoseVertex root;
  std::map<KEY, PoseVertex> key2vertex;
  std::tie(g, root, key2vertex) =
      predecessorMap2Graph<PoseGraph, PoseVertex, KEY>(tree);

  // attach the relative poses to the edges
  PoseEdge edge12, edge21;
  bool found1, found2;
  for(typename G::sharedFactor nl_factor: graph) {

    if (nl_factor->keys().size() > 2)
      throw std::invalid_argument("composePoses: only support factors with at most two keys");

    // e.g. in pose2graph, nonlinear factor needs to be converted to pose2factor
    std::shared_ptr<Factor> factor = std::dynamic_pointer_cast<Factor>(nl_factor);
    if (!factor) continue;

    KEY key1 = factor->key1();
    KEY key2 = factor->key2();

    PoseVertex v1 = key2vertex.find(key1)->second;
    PoseVertex v2 = key2vertex.find(key2)->second;

    POSE l1Xl2 = factor->measured();
    std::tie(edge12, found1) = boost::edge(v1, v2, g);
    std::tie(edge21, found2) = boost::edge(v2, v1, g);
    if (found1 && found2) throw std::invalid_argument ("composePoses: invalid spanning tree");
    if (!found1 && !found2) continue;
    if (found1)
      boost::put(boost::edge_weight, g, edge12, l1Xl2);
    else if (found2)
      boost::put(boost::edge_weight, g, edge21, l1Xl2.inverse());
  }

  // compose poses
  std::shared_ptr<Values> config(new Values);
  KEY rootKey = boost::get(boost::vertex_name, g, root);
  config->insert(rootKey, rootPose);
  compose_key_visitor<PoseVertex, POSE, KEY> vis(config);
  boost::breadth_first_search(g, root, boost::visitor(vis));

  return config;
}

/* ************************************************************************* */
template<class G, class KEY, class FACTOR2>
PredecessorMap<KEY> findMinimumSpanningTree(const G& fg) {

  // Convert to a graph that boost understands
  SDGraph<KEY> g = gtsam::toBoostGraph<G, FACTOR2, KEY>(fg);

  // // find minimum spanning tree
  std::vector<typename SDGraph<KEY>::Vertex> p_map(boost::num_vertices(g));
  prim_minimum_spanning_tree(g, &p_map[0]);

  // convert edge to string pairs
  PredecessorMap<KEY> tree;
  typename SDGraph<KEY>::vertex_iterator itVertex = boost::vertices(g).first;
  for(const typename SDGraph<KEY>::Vertex& vi: p_map){
    KEY key = boost::get(boost::vertex_name, g, *itVertex);
    KEY parent = boost::get(boost::vertex_name, g, vi);
    tree.insert(key, parent);
    itVertex++;
  }
  return tree;
}

/* ************************************************************************* */
template<class G, class KEY, class FACTOR2>
void split(const G& g, const PredecessorMap<KEY>& tree, G& Ab1, G& Ab2) {

  typedef typename G::sharedFactor F ;

  for(const F& factor: g)
  {
    if (factor->keys().size() > 2)
      throw(std::invalid_argument("split: only support factors with at most two keys"));

    if (factor->keys().size() == 1) {
      Ab1.push_back(factor);
      continue;
    }

    std::shared_ptr<FACTOR2> factor2 = std::dynamic_pointer_cast<
        FACTOR2>(factor);
    if (!factor2) continue;

    KEY key1 = factor2->key1();
    KEY key2 = factor2->key2();
    // if the tree contains the key
    if ((tree.find(key1) != tree.end() &&
       tree.find(key1)->second.compare(key2) == 0) ||
        (tree.find(key2) != tree.end() &&
         tree.find(key2)->second.compare(key1)== 0) )
      Ab1.push_back(factor2);
    else
      Ab2.push_back(factor2);
  }
}

}
