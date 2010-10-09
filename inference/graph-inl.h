/*
 * graph-inl.h
 * @brief Graph algorithm using boost library
 * @author: Kai Ni
 */

#pragma once

#include <stdexcept>
#include <boost/foreach.hpp>
#include <boost/graph/breadth_first_search.hpp>

#include <gtsam/inference/graph.h>

using namespace std;

#define FOREACH_PAIR( KEY, VAL, COL) BOOST_FOREACH (boost::tie(KEY,VAL),COL)

namespace gtsam {

/* ************************************************************************* */
template <class Key>
class ordering_key_visitor : public boost::default_bfs_visitor {
public:
	ordering_key_visitor(std::list<Key>& ordering_in) : ordering_(ordering_in) {}
	template <typename Vertex, typename Graph> void discover_vertex(Vertex v, const Graph& g) const {
		Key key = boost::get(boost::vertex_name, g, v);
		ordering_.push_front(key);
	}
	std::list<Key>& ordering_;
};

/* ************************************************************************* */
template<class Key>
list<Key> predecessorMap2Keys(const PredecessorMap<Key>& p_map) {

	typedef typename SGraph<Key>::Vertex SVertex;

	SGraph<Key> g;
	SVertex root;
	std::map<Key, SVertex> key2vertex;
	boost::tie(g, root, key2vertex) = gtsam::predecessorMap2Graph<SGraph<Key>, SVertex, Key>(p_map);

	// breadth first visit on the graph
	std::list<Key> keys;
	ordering_key_visitor<Key> vis(keys);
	boost::breadth_first_search(g, root, boost::visitor(vis));
	return keys;
}

///* ************************************************************************* */
//template<class G, class F, class Key>
// SL-NEEDED? SDGraph<Key> toBoostGraph(const G& graph) {
//	// convert the factor graph to boost graph
//	SDGraph<Key> g;
//	typedef typename boost::graph_traits<SDGraph<Key> >::vertex_descriptor BoostVertex;
//	map<Key, BoostVertex> key2vertex;
//	BoostVertex v1, v2;
//	typename G::const_iterator itFactor;
//	for(itFactor=graph.begin(); itFactor!=graph.end(); itFactor++) {
//		if ((*itFactor)->keys().size() > 2)
//			throw(invalid_argument("toBoostGraph: only support factors with at most two keys"));
//
//		if ((*itFactor)->keys().size() == 1)
//			continue;
//
//		boost::shared_ptr<F> factor = boost::dynamic_pointer_cast<F>(*itFactor);
//		if (!factor) continue;
//
//		Key key1 = factor->key1();
//		Key key2 = factor->key2();
//
//		if (key2vertex.find(key1) == key2vertex.end()) {
//				 v1 = add_vertex(key1, g);
//				 key2vertex.insert(make_pair(key1, v1));
//			 } else
//				 v1 = key2vertex[key1];
//
//		if (key2vertex.find(key2) == key2vertex.end()) {
//			 v2 = add_vertex(key2, g);
//			 key2vertex.insert(make_pair(key2, v2));
//		 } else
//			 v2 = key2vertex[key2];
//
//		boost::property<boost::edge_weight_t, double> edge_property(1.0);  // assume constant edge weight here
//		boost::add_edge(v1, v2, edge_property, g);
//	}
//
//	return g;
//}

/* ************************************************************************* */
template<class G, class V, class Key>
boost::tuple<G, V, map<Key,V> >
predecessorMap2Graph(const PredecessorMap<Key>& p_map) {

	G g;
	map<Key, V> key2vertex;
	V v1, v2, root;
	Key child, parent;
	bool foundRoot = false;
	FOREACH_PAIR(child, parent, p_map) {
		if (key2vertex.find(child) == key2vertex.end()) {
			 v1 = add_vertex(child, g);
			 key2vertex.insert(make_pair(child, v1));
		 } else
			 v1 = key2vertex[child];

		if (key2vertex.find(parent) == key2vertex.end()) {
			 v2 = add_vertex(parent, g);
			 key2vertex.insert(make_pair(parent, v2));
		 } else
			 v2 = key2vertex[parent];

		if (child==parent) {
			root = v1;
			foundRoot = true;
		} else
			boost::add_edge(v2, v1, g); // edge is from parent to child
	}

	if (!foundRoot)
		throw invalid_argument("predecessorMap2Graph: invalid predecessor map!");
	else
	  return boost::tuple<G, V, std::map<Key, V> >(g, root, key2vertex);
}

/* ************************************************************************* */
template <class V,class Pose, class Values>
class compose_key_visitor : public boost::default_bfs_visitor {

private:
	boost::shared_ptr<Values> config_;

public:

	compose_key_visitor(boost::shared_ptr<Values> config_in) {config_ = config_in;}

	template <typename Edge, typename Graph> void tree_edge(Edge edge, const Graph& g) const {
		typename Values::Key key_from = boost::get(boost::vertex_name, g, boost::source(edge, g));
		typename Values::Key key_to = boost::get(boost::vertex_name, g, boost::target(edge, g));
		Pose relativePose = boost::get(boost::edge_weight, g, edge);
		config_->insert(key_to, (*config_)[key_from].compose(relativePose));
	}

};

/* ************************************************************************* */
template<class G, class Factor, class Pose, class Values>
boost::shared_ptr<Values> composePoses(const G& graph, const PredecessorMap<typename Values::Key>& tree,
		const Pose& rootPose) {

	//TODO: change edge_weight_t to edge_pose_t
	typedef typename boost::adjacency_list<
		boost::vecS, boost::vecS, boost::directedS,
		boost::property<boost::vertex_name_t, typename Values::Key>,
		boost::property<boost::edge_weight_t, Pose> > PoseGraph;
	typedef typename boost::graph_traits<PoseGraph>::vertex_descriptor PoseVertex;
	typedef typename boost::graph_traits<PoseGraph>::edge_descriptor PoseEdge;

	PoseGraph g;
	PoseVertex root;
	map<typename Values::Key, PoseVertex> key2vertex;
	boost::tie(g, root, key2vertex) =
			predecessorMap2Graph<PoseGraph, PoseVertex, typename Values::Key>(tree);

	// attach the relative poses to the edges
	PoseEdge edge12, edge21;
	bool found1, found2;
	BOOST_FOREACH(typename G::sharedFactor nl_factor, graph) {

		if (nl_factor->keys().size() > 2)
			throw invalid_argument("composePoses: only support factors with at most two keys");

		// e.g. in pose2graph, nonlinear factor needs to be converted to pose2factor
		boost::shared_ptr<Factor> factor = boost::dynamic_pointer_cast<Factor>(nl_factor);
		if (!factor) continue;

		typename Values::Key key1 = factor->key1();
		typename Values::Key key2 = factor->key2();

		PoseVertex v1 = key2vertex.find(key1)->second;
		PoseVertex v2 = key2vertex.find(key2)->second;

		Pose l1Xl2 = factor->measured();
		tie(edge12, found1) = boost::edge(v1, v2, g);
		tie(edge21, found2) = boost::edge(v2, v1, g);
		if (found1 && found2) throw invalid_argument ("composePoses: invalid spanning tree");
		if (!found1 && !found2) continue;
		if (found1)
			boost::put(boost::edge_weight, g, edge12, l1Xl2);
		else if (found2)
			boost::put(boost::edge_weight, g, edge21, l1Xl2.inverse());
	}

	// compose poses
	boost::shared_ptr<Values> config(new Values);
	typename Values::Key rootKey = boost::get(boost::vertex_name, g, root);
	config->insert(rootKey, rootPose);
	compose_key_visitor<PoseVertex, Pose, Values> vis(config);
	boost::breadth_first_search(g, root, boost::visitor(vis));

	return config;
}

/* ************************************************************************* */

}
