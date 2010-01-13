/*
 * graph-inl.h
 *
 *   Created on: Jan 11, 2010
 *       Author: nikai
 *  Description: Graph algorithm using boost library
 */

#pragma once

#include <boost/foreach.hpp>
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/breadth_first_search.hpp>

using namespace std;

#define FOREACH_PAIR( KEY, VAL, COL) BOOST_FOREACH (boost::tie(KEY,VAL),COL)

namespace gtsam {

/* ************************************************************************* */
/**
 * type definitions
 */
typedef boost::adjacency_list<
	boost::vecS, boost::vecS, boost::undirectedS,
	boost::property<boost::vertex_name_t, std::string>,
	boost::property<boost::edge_weight_t, double> > SDGraph;
typedef boost::graph_traits<SDGraph>::vertex_descriptor BoostVertex;
typedef boost::graph_traits<SDGraph>::vertex_iterator BoostVertexIterator;

typedef boost::adjacency_list<
	boost::vecS, boost::vecS, boost::directedS,
	boost::property<boost::vertex_name_t, string> > SGraph;
typedef boost::graph_traits<SGraph>::vertex_descriptor SVertex;

/* ************************************************************************* */
/**
 * Convert the factor graph to a boost undirected graph
 */
template<class G, class F>
SDGraph toBoostGraph(const G& graph) {
	// convert the factor graph to boost graph
	SDGraph g(0);
	map<string, BoostVertex> key2vertex;
	BoostVertex v1, v2;
	BOOST_FOREACH(F factor, graph) {
		if (factor->keys().size() > 2)
			throw(invalid_argument("toBoostGraph: only support factors with at most two keys"));

		if (factor->keys().size() == 1)
			continue;

		string key1 = factor->keys().front();
		string key2 = factor->keys().back();

		if (key2vertex.find(key1) == key2vertex.end()) {
				 v1 = add_vertex(key1, g);
				 key2vertex.insert(make_pair(key1, v1));
			 } else
				 v1 = key2vertex[key1];

		if (key2vertex.find(key2) == key2vertex.end()) {
			 v2 = add_vertex(key2, g);
			 key2vertex.insert(make_pair(key2, v2));
		 } else
			 v2 = key2vertex[key2];

		boost::property<boost::edge_weight_t, double> edge_property(1.0);  // assume constant edge weight here
		boost::add_edge(v1, v2, edge_property, g);
	}

	return g;
}

/* ************************************************************************* */
/**
 *  build the graph corresponding to the predecessor map. Excute action for each edge.
 */
template<class G, class V>
boost::tuple<G, V, map<string, V> > predecessorMap2Graph(const map<string, string>& p_map) {

	G g(0);
	map<string, V> key2vertex;
	V v1, v2, root;
	string child, parent;
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

		if (child.compare(parent) == 0) {
			root = v1;
			foundRoot = true;
		} else
			boost::add_edge(v2, v1, g); // edge is from parent to child
	}

	if (!foundRoot)
		throw invalid_argument("predecessorMap2Graph: invalid predecessor map!");

	return boost::tuple<G, V, map<string, V> >(g, root, key2vertex);
}

/* ************************************************************************* */
/**
 * Visit each edge and compose the poses
 */
template <class V, class Pose, class PoseConfig>
class compose_key_visitor : public boost::default_bfs_visitor {
public:
	compose_key_visitor(boost::shared_ptr<PoseConfig> config_in) { config = config_in; }
	template <typename Edge, typename Graph> void tree_edge(Edge edge, const Graph& g) const {
			string key_from = boost::get(boost::vertex_name, g, boost::source(edge, g));
			string key_to   = boost::get(boost::vertex_name, g, boost::target(edge, g));
			Pose relativePose = boost::get(boost::edge_weight, g, edge);
			config->insert(key_to, compose(relativePose, config->get(key_from)));
	}

private:
	boost::shared_ptr<PoseConfig> config;

};

/* ************************************************************************* */
/**
 * Compose the poses by following the chain sepcified by the spanning tree
 */
template<class G, class Factor, class Pose, class Config>
boost::shared_ptr<Config> composePoses(const G& graph, const map<string, string>& tree,
		const Pose& rootPose) {

	//TODO: change edge_weight_t to edge_pose_t
	typedef typename boost::adjacency_list<
		boost::vecS, boost::vecS, boost::directedS,
		boost::property<boost::vertex_name_t, string>,
		boost::property<boost::edge_weight_t, Pose> > PoseGraph;
	typedef typename boost::graph_traits<PoseGraph>::vertex_descriptor PoseVertex;
	typedef typename boost::graph_traits<PoseGraph>::edge_descriptor PoseEdge;

	PoseGraph g;
	PoseVertex root;
	map<string, PoseVertex> key2vertex;
	boost::tie(g, root, key2vertex) = predecessorMap2Graph<PoseGraph, PoseVertex>(tree);

	// attach the relative poses to the edges
	PoseEdge edge1, edge2;
	bool found1, found2;
	BOOST_FOREACH(typename G::sharedFactor nl_factor, graph) {

		if (nl_factor->keys().size() > 2)
			throw invalid_argument("composePoses: only support factors with at most two keys");

		// e.g. in pose2graph, nonlinear factor needs to be converted to pose2factor
		boost::shared_ptr<Factor> factor = boost::dynamic_pointer_cast<Factor>(nl_factor);
		if (!factor) continue;

		PoseVertex v_from = key2vertex.find(factor->keys().front())->second;
		PoseVertex v_to = key2vertex.find(factor->keys().back())->second;

		Pose measured = factor->measured();
		tie(edge1, found1) = boost::edge(v_from, v_to, g);
		tie(edge2, found2) = boost::edge(v_to, v_from, g);
		if (found1 && found2) throw invalid_argument ("composePoses: invalid spanning tree");
		if (!found1 && !found2) continue;
		if (found1)
			boost::put(boost::edge_weight, g, edge1, measured);
		else if (found2)
			boost::put(boost::edge_weight, g, edge2, inverse(measured));
	}

	// compose poses
	boost::shared_ptr<Config> config(new Config);
	config->insert(boost::get(boost::vertex_name, g, root), rootPose);
	compose_key_visitor<PoseVertex, Pose, Config> vis(config);
	boost::breadth_first_search(g, root, boost::visitor(vis));

	return config;
}

/* ************************************************************************* */

}
