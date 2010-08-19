/*
 * saveGraph.cpp
 * Author: Richard Roberts
 */

#include <iostream>
#include <fstream>
#include <boost/format.hpp>
#include <boost/foreach.hpp>
#include <gtsam/inference/Ordering.h>
#include <gtsam/geometry/Point2.h>
#include <gtsam/inference/SymbolicFactorGraph.h>
#include <gtsam/inference/SymbolicBayesNet.h>
#include <gtsam/inference/inference-inl.h>
#include <gtsam/nonlinear/LieConfig-inl.h>

using namespace std;

namespace gtsam {

	INSTANTIATE_LIE_CONFIG(Symbol, Point2)

	/* ************************************************************************* */
	void saveGraph(const SymbolicFactorGraph& fg, const SymbolicConfig& config, const std::string& s) {

		Symbol key;
		Point2 pt;
		float scale = 100;

		string dotfile = s + ".dot";
		ofstream of(dotfile.c_str());
		of << "graph G{" << endl;
		of << "bgcolor=\"transparent\";" << endl;

		BOOST_FOREACH(boost::tie(key, pt), config){
			of << (string)key << "[label=\"" << (string)key << "\"][pos=\"" << pt.x()*scale << "," << pt.y()*scale << "\"];" << endl;
		}

		int index = 0;
		BOOST_FOREACH(const SymbolicFactorGraph::sharedFactor& factor, fg) {
			index++;
			Point2 center;
			BOOST_FOREACH(const Symbol& key, factor->keys())
				center = center + config[key];
			center = Point2(center.x() / factor->keys().size(), center.y() / factor->keys().size());
			of << "f" << index << "[pos=\"" << center.x()*scale << "," << center.y()*scale << "\"][shape=\"point\"];" << endl;
			BOOST_FOREACH(const Symbol& key, factor->keys())
				of << "f" << index << "--" << (string)key << endl;
		}
		of<<"}";
		of.close();

		string cmd = boost::str(boost::format("neato -s -n -Tpdf %s -o %s.pdf") % dotfile % s);
		system(cmd.c_str());
	}

	/* ************************************************************************* */
}
