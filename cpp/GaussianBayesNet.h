/**
 * @file    GaussianBayesNet.h
 * @brief   Chordal Bayes Net, the result of eliminating a factor graph
 * @brief   GaussianBayesNet
 * @author  Frank Dellaert
 */

// \callgraph

#pragma once

#include <list>

#include "ConditionalGaussian.h"
#include "BayesNet.h"

namespace gtsam {

/** Chordal Bayes Net, the result of eliminating a factor graph */
class GaussianBayesNet : public BayesNet<ConditionalGaussian>
{
public:
	typedef boost::shared_ptr<GaussianBayesNet> shared_ptr;

	/** Construct an empty net */
	GaussianBayesNet() {}

	/** Copy Constructor */
//	GaussianBayesNet(const GaussianBayesNet& cbn_in) :
//		keys_(cbn_in.keys_), nodes_(cbn_in.nodes_) {}

	/** Destructor */
	virtual ~GaussianBayesNet() {}

	/**
	 * optimize, i.e. return x = inv(R)*d
	 */
	boost::shared_ptr<VectorConfig> optimize() const;

	/**
	 * Return (dense) upper-triangular matrix representation
	 */
	std::pair<Matrix,Vector> matrix() const;
};

} /// namespace gtsam
