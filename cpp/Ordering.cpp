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

#include "graph-inl.h"
#include "Ordering.h"


using namespace std;
using namespace gtsam;
using namespace boost::assign;

#define FOREACH_PAIR( KEY, VAL, COL) BOOST_FOREACH (boost::tie(KEY,VAL),COL)

class ordering_key_visitor : public boost::default_bfs_visitor {
public:
	ordering_key_visitor(Ordering& ordering_in) : ordering_(ordering_in) {}
	template <typename Vertex, typename Graph> void discover_vertex(Vertex v, const Graph& g) const {
		string key = boost::get(boost::vertex_name, g, v);
		ordering_.push_front(key);
	}
	Ordering& ordering_;
};

/* ************************************************************************* */
Ordering::Ordering(const PredecessorMap<string>& p_map) {

	typedef SGraph<string>::Vertex SVertex;

	SGraph<string> g;
	SVertex root;
	map<string, SVertex> key2vertex;
	boost::tie(g, root, key2vertex) = predecessorMap2Graph<SGraph<string>, SVertex, string>(p_map);

	// breadth first visit on the graph
	ordering_key_visitor vis(*this);
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



