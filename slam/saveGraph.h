/*
 * h
 * Author: Richard Roberts
 */

#pragma once

#include <string>
#include <list>
#include <gtsam/inference/FactorGraph.h>
#include <gtsam/inference/SymbolicFactor.h>
#include <gtsam/inference/SymbolicBayesNet.h>
#include <gtsam/inference/Key.h>
#include <gtsam/geometry/Point2.h>
#include <gtsam/nonlinear/LieValues.h>

namespace gtsam {

	class Point2;
	typedef LieValues<Symbol, Point2> SymbolicValues;

	// save graph to the graphviz format
	void saveGraph(const SymbolicFactorGraph& fg, const SymbolicValues& config, const std::string& s);

} // namespace gtsam
