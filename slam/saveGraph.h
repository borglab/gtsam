/*
 * h
 * Author: Richard Roberts
 */

#pragma once

#include <string>
#include <list>
#include "FactorGraph.h"
#include "SymbolicFactor.h"
#include "SymbolicBayesNet.h"
#include "Key.h"
#include "Point2.h"
#include "LieConfig.h"

namespace gtsam {

	class Point2;
	typedef LieConfig<Symbol, Point2> SymbolicConfig;

	// save graph to the graphviz format
	void saveGraph(const SymbolicFactorGraph& fg, const SymbolicConfig& config, const std::string& s);

} // namespace gtsam
