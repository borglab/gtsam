/*
 * Ordering-inl.h
 *
 *   Created on: Jan 14, 2010
 *       Author: nikai
 *  Description: the inline file for Ordering
 */

#pragma once

#include "graph-inl.h"
#include "Ordering.h"

using namespace std;

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

}
