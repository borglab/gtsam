/**
 * @file    Ordering.cpp
 * @brief   Ordering
 * @author  Christian Potthast
 */

#include <iostream>
#include <stdexcept>
#include <boost/assign/std/list.hpp> // for operator +=
#include <boost/foreach.hpp>
#include <boost/tuple/tuple.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/breadth_first_search.hpp>

#include "Ordering.h"

using namespace std;
using namespace gtsam;
using namespace boost::assign;

#define FOREACH_PAIR( KEY, VAL, COL) BOOST_FOREACH (boost::tie(KEY,VAL),COL)

class key_visitor : public boost::default_bfs_visitor {
public:
	key_visitor(Ordering& ordering_in) : ordering_(ordering_in) {}
	template <typename Vertex, typename Graph> void discover_vertex(Vertex u, const Graph& g) const {
		string key = boost::get(boost::vertex_name, g, u);
		ordering_.push_front(key);
	}
	Ordering& ordering_;
};

/* ************************************************************************* */
Ordering::Ordering(const map<string, string>& p_map) {

	typedef boost::adjacency_list<boost::vecS, boost::vecS, boost::undirectedS,
		boost::property<boost::vertex_name_t, string> > Graph;
	typedef boost::graph_traits<Graph>::vertex_descriptor Vertex;

	// build the graph corresponding to the predecessor map
	Graph g(0);
	map<string, Vertex> key2vertex;
	Vertex v1, v2, root;
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
			boost::add_edge(v1, v2, g);
	}

	if (!foundRoot)
		throw invalid_argument("Ordering: invalid predecessor map!");

	// breadth first visit on the graph
	key_visitor vis(*this);
	boost::breadth_first_search(g, root, boost::visitor(vis));
}

/* ************************************************************************* */
Ordering Ordering::subtract(const Ordering& keys) const {
	Ordering newOrdering = *this;
	BOOST_FOREACH(string key, keys) {
		newOrdering.remove(key);
	}
	return newOrdering;
}

/* ************************************************************************* */
void Ordering::print(const string& s) const {
  cout << s;
  BOOST_FOREACH(string key, *this)
    cout << " " << key;
  cout << endl;
}

/* ************************************************************************* */
bool Ordering::equals(const Ordering &other, double tol) const {
	return *this == other;
}

/* ************************************************************************* */



