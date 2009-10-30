/**
 * @file    ChordalBayesNet.h
 * @brief   Chordal Bayes Net, the result of eliminating a factor graph
 * @brief   ChordalBayesNet
 * @author  Frank Dellaert
 */

// \callgraph

#pragma once

#include <list>

#include "ConditionalGaussian.h"
#include "BayesChain.h"

namespace gtsam {

/** Chordal Bayes Net, the result of eliminating a factor graph */
class ChordalBayesNet : public BayesChain<ConditionalGaussian>
{
public:
	typedef boost::shared_ptr<ChordalBayesNet> shared_ptr;

	/** Construct an empty net */
	ChordalBayesNet() {}

	/** Copy Constructor */
//	ChordalBayesNet(const ChordalBayesNet& cbn_in) :
//		keys_(cbn_in.keys_), nodes_(cbn_in.nodes_) {}

	/** Destructor */
	virtual ~ChordalBayesNet() {}

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
