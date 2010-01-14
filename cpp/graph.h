/*
 *  graph.h
 *  @brief Graph algorithm using boost library
 *  @author: Kai Ni
 *  Created on: Jan 11, 2010
 */

#pragma once

#include <map>

#include <boost/graph/graph_traits.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/shared_ptr.hpp>

namespace gtsam {

	// type definitions :

	/**
	 * SDGraph is undirected graph with variable keys and double edge weights
	 */
	template<class Key>
	class SDGraph: public boost::adjacency_list<boost::vecS, boost::vecS, boost::undirectedS,
	boost::property<boost::vertex_name_t, Key>, boost::property<
	boost::edge_weight_t, double> > {
	};

	template<class Key>
	class SGraph : public boost::adjacency_list<boost::vecS, boost::vecS, boost::directedS,
			boost::property<boost::vertex_name_t, std::string> > {
		typedef typename boost::graph_traits<SDGraph<Key> >::vertex_descriptor Vertex;
	};

	//typedef boost::graph_traits<SGraph>::vertex_descriptor SVertex;

	/**
	 * Map from variable key to parent key
	 */
	template <class Key>
	class PredecessorMap : public std::map<Key,Key> {};

	/**
	 * Convert the factor graph to an SDGraph
	 * G = Graph type
	 * F = Factor type
	 * Key = Key type
	 */
	template<class G, class F, class Key> SDGraph<Key> toBoostGraph(const G& graph);

	/**
	 * Build takes a predecessor map, and builds a directed graph corresponding to the tree.
	 * G = Graph type
	 * V = Vertex type
	 */
	template<class Key>
	boost::tuple<SDGraph<Key>, typename SDGraph<Key>::Vertex, std::map<Key, typename SDGraph<Key>::Vertex> >
		predecessorMap2Graph(const PredecessorMap<Key>& p_map);

	/**
	 * Compose the poses by following the chain specified by the spanning tree
	 */
	template<class G, class Factor, class Pose, class Config>
	boost::shared_ptr<Config>
		composePoses(const G& graph, const PredecessorMap<typename Config::Key>& tree, const Pose& rootPose);

} // namespace gtsam
