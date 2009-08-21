/*
 * ConstrainedChordalBayesNet.h
 *
 *  Created on: Aug 11, 2009
 *      Author: alexgc
 */

#ifndef CONSTRAINEDCHORDALBAYESNET_H_
#define CONSTRAINEDCHORDALBAYESNET_H_

#include "ChordalBayesNet.h"
#include "DeltaFunction.h"

namespace gtsam {

class ConstrainedChordalBayesNet : public ChordalBayesNet{

protected:
	std::map<std::string, DeltaFunction::shared_ptr> delta_nodes;

public:
	typedef std::map<std::string, DeltaFunction::shared_ptr>::const_iterator const_delta_iterator;
	typedef std::map<std::string, DeltaFunction::shared_ptr>::iterator delta_iterator;

public:
	typedef boost::shared_ptr<ConstrainedChordalBayesNet> shared_ptr;

	/**
	 * Default Constructor
	 */
	ConstrainedChordalBayesNet();

	/**
	 * Copies an existing ChordalBayesNet
	 */
	ConstrainedChordalBayesNet(const ChordalBayesNet& cbn);

	virtual ~ConstrainedChordalBayesNet();

	/** insert: use reverse topological sort (i.e. parents last) */
	void insert_df(const std::string& key, DeltaFunction::shared_ptr node);
	void insert(const std::string& key, ConditionalGaussian::shared_ptr node);

	/** optimize the solution - just a wrapper on the existing optimize implementation */
	boost::shared_ptr<FGConfig> optimize();
	boost::shared_ptr<FGConfig> optimize(const boost::shared_ptr<FGConfig> &c);

	/** convert to a regular cbn - strips out the delta functions
	 * TODO: make this check whether this is a safe conversion
	 */
	ChordalBayesNet convert() const;

	/** get delta functions by key */
	DeltaFunction::shared_ptr get_delta(const std::string& key) {return delta_nodes[key];}

	/** check equality */
	bool equals(const ConstrainedChordalBayesNet& cbn) const;

	/** prints the contents */
	void print(const std::string& s="") const;
};


/** check equality for testing */
bool assert_equal(const ConstrainedChordalBayesNet& expected, const ConstrainedChordalBayesNet& actual, double tol=1e-9);

}

#endif /* CONSTRAINEDCHORDALBAYESNET_H_ */
