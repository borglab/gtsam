/**
 * @file    NonlinearFactorGraph.h
 * @brief   Factor Graph Constsiting of non-linear factors
 * @author  Frank Dellaert
 * @author  Carlos Nieto
 * @author  Christian Potthast
 */

// \callgraph

#pragma once

#include <boost/serialization/base_object.hpp>
#include <colamd/colamd.h>
#include "FactorGraph.h"
#include "NonlinearFactor.h"
#include "LinearFactorGraph.h"
#include "ChordalBayesNet.h"

namespace gtsam {

typedef FactorGraph<NonlinearFactor> BaseFactorGraph;

/** Factor Graph consisting of non-linear factors */
class NonlinearFactorGraph : public BaseFactorGraph
{
public:
	// internal, exposed for testing only, doc in .cpp file

	Ordering getOrdering(const FGConfig& config) const;

public: // these you will probably want to use
	/**
	 * linearize a non linear factor
	 */
	LinearFactorGraph linearize(const FGConfig& config) const;

	/**
	 * Given two configs, check whether the error of config2 is
	 * different enough from the error for config1, or whether config1
	 * is essentially optimal
	 *
	 * @param config1  Reference to first configuration
	 * @param config2  Reference to second configuration
	 * @param relativeErrorTreshold
	 * @param absoluteErrorTreshold
	 * @param verbosity Integer specifying how much output to provide
	 */
	bool check_convergence(const FGConfig& config1,
			const FGConfig& config2,
			double relativeErrorTreshold, double absoluteErrorTreshold,
			int verbosity = 0) const;

private:

	/** Serialization function */
	friend class boost::serialization::access;
	template<class Archive>
	void serialize(Archive & ar, const unsigned int version) {
		// do not use BOOST_SERIALIZATION_NVP for this name-value-pair ! It will crash.
		ar & boost::serialization::make_nvp("BaseFactorGraph",
				boost::serialization::base_object<BaseFactorGraph>(*this));
	}

};

/**
 * Check convergence
 */
bool check_convergence (double relativeErrorTreshold,
		double absoluteErrorTreshold,
		double currentError, double newError,
		int verbosity);

/**
 * calculate error for current configuration
 */
double calculate_error (const NonlinearFactorGraph& fg, const FGConfig& config, int verbosity);
}
