/**
 * @file    LinearFactorGraph.cpp
 * @brief   Linear Factor Graph where all factors are Gaussians
 * @author  Kai Ni
 * @author  Christian Potthast
 */

#include <boost/foreach.hpp>
#include <boost/tuple/tuple.hpp>
#include <boost/numeric/ublas/lu.hpp>
#include <boost/numeric/ublas/io.hpp>

#include <colamd/colamd.h>

#include "ChordalBayesNet.h"
#include "LinearFactorGraph.h"

using namespace std;
using namespace gtsam;

/* ************************************************************************* */
LinearFactorGraph::LinearFactorGraph(const ChordalBayesNet& CBN)
{
	setCBN(CBN);
}

/* ************************************************************************* */
void LinearFactorGraph::setCBN(const ChordalBayesNet& CBN)
{
	clear();
	ChordalBayesNet::const_iterator it = CBN.begin();
	for(; it != CBN.end(); it++) {
		LinearFactor::shared_ptr lf(new LinearFactor(it->first, it->second));
		push_back(lf);
	}
}

/* ************************************************************************* */
/* find the separators                                                       */
/* ************************************************************************* */
set<string> LinearFactorGraph::find_separator(const string& key) const
{
	set<string> separator;
	BOOST_FOREACH(shared_factor factor,factors_)
		factor->tally_separator(key,separator);

	return separator;
}

/* ************************************************************************* */
/** O(1)                                                                     */
/* ************************************************************************* */
list<int> LinearFactorGraph::factors(const string& key) const {
	Indices::const_iterator it = indices_.find(key);
	return it->second;
}


/* ************************************************************************* */
/** find all non-NULL factors for a variable, then set factors to NULL       */
/* ************************************************************************* */
LinearFactorSet LinearFactorGraph::find_factors_and_remove(const string& key) {
	LinearFactorSet found;

	Indices::iterator it = indices_.find(key);
	list<int> *indices_ptr; // pointer to indices list in indices_ map
	indices_ptr = &(it->second);

	for (list<int>::iterator it = indices_ptr->begin(); it != indices_ptr->end(); it++) {
		if(factors_[*it] == NULL){  // skip NULL factors
			continue;
		}
		found.push_back(factors_[*it]);
		factors_[*it].reset(); // set factor to NULL.
	}
	return found;
}

/* ************************************************************************* */
/* find factors and remove them from the factor graph: O(n)                  */
/* ************************************************************************* */ 
boost::shared_ptr<LinearFactor>
LinearFactorGraph::combine_factors(const string& key)
{
	LinearFactorSet found = find_factors_and_remove(key);
	boost::shared_ptr<LinearFactor> lf(new LinearFactor(found));
	return lf;
}

/* ************************************************************************* */
/* eliminate one node from the linear factor graph                           */ 
/* ************************************************************************* */
ConditionalGaussian::shared_ptr LinearFactorGraph::eliminate_one(const string& key)
{
	// combine the factors of all nodes connected to the variable to be eliminated
	// if no factors are connected to key, returns an empty factor
	boost::shared_ptr<LinearFactor> joint_factor = combine_factors(key);

	// eliminate that joint factor
	try {
		ConditionalGaussian::shared_ptr conditional;
		LinearFactor::shared_ptr factor;
		boost::tie(conditional,factor) = joint_factor->eliminate(key);

		if (!factor->empty())
			push_back(factor);

		// return the conditional Gaussian
		return conditional;
	}
	catch (domain_error&) {
		throw(domain_error("LinearFactorGraph::eliminate: singular graph"));
	}
}

/* ************************************************************************* */
// eliminate factor graph using the given (not necessarily complete)
// ordering, yielding a chordal Bayes net and partially eliminated FG
/* ************************************************************************* */
ChordalBayesNet::shared_ptr
LinearFactorGraph::eliminate_partially(const Ordering& ordering)
{
	ChordalBayesNet::shared_ptr chordalBayesNet (new ChordalBayesNet()); // empty

	BOOST_FOREACH(string key, ordering) {
		ConditionalGaussian::shared_ptr cg = eliminate_one(key);
		chordalBayesNet->insert(key,cg);
	}

	return chordalBayesNet;
}

/* ************************************************************************* */
/** eliminate factor graph in the given order, yielding a chordal Bayes net  */ 
/* ************************************************************************* */
ChordalBayesNet::shared_ptr
LinearFactorGraph::eliminate(const Ordering& ordering)
{
	ChordalBayesNet::shared_ptr chordalBayesNet = eliminate_partially(ordering);

	// after eliminate, only one zero indegree factor should remain
	// TODO: this check needs to exist - verify that unit tests work when this check is in place
	/*
  if (factors_.size() != 1) {
    print();
    throw(invalid_argument("LinearFactorGraph::eliminate: graph not empty after eliminate, ordering incomplete?"));
  }
	 */
	return chordalBayesNet;
}

/* ************************************************************************* */
/** optimize the linear factor graph                                          */ 
/* ************************************************************************* */
VectorConfig LinearFactorGraph::optimize(const Ordering& ordering)
{
	// eliminate all nodes in the given ordering -> chordal Bayes net
	ChordalBayesNet::shared_ptr chordalBayesNet = eliminate(ordering);

	// calculate new configuration (using backsubstitution)
	boost::shared_ptr<VectorConfig> newConfig = chordalBayesNet->optimize();

	return *newConfig;
}

/* ************************************************************************* */
/** combine two factor graphs                                                 */ 
/* ************************************************************************* */
void LinearFactorGraph::combine(const LinearFactorGraph &lfg){
	for(const_iterator factor=lfg.factors_.begin(); factor!=lfg.factors_.end(); factor++){
		push_back(*factor);
	}
}

/* ************************************************************************* */
/** combine two factor graphs                                                */ 
/* ************************************************************************* */

LinearFactorGraph LinearFactorGraph::combine2(const LinearFactorGraph& lfg1,
		const LinearFactorGraph& lfg2) {
	// create new linear factor graph equal to the first one
	LinearFactorGraph fg = lfg1;

	// add the second factors_ in the graph
	for (const_iterator factor = lfg2.factors_.begin(); factor
			!= lfg2.factors_.end(); factor++) {
		fg.push_back(*factor);
	}

	return fg;
}


/* ************************************************************************* */  
// find all variables and their dimensions
VariableSet LinearFactorGraph::variables() const {
	VariableSet result;
	BOOST_FOREACH(shared_factor factor,factors_) {
		VariableSet vs = factor->variables();
		BOOST_FOREACH(Variable v,vs) result.insert(v);
	}
	return result;
}

/* ************************************************************************* */  
LinearFactorGraph LinearFactorGraph::add_priors(double sigma) const {

	// start with this factor graph
	LinearFactorGraph result = *this;

	// find all variables and their dimensions
	VariableSet vs = variables();

	// for each of the variables, add a prior
	BOOST_FOREACH(Variable v,vs) {
		size_t n = v.dim();
		const string& key = v.key();
		Matrix A = sigma*eye(n);
		Vector b = zero(n);
		shared_factor prior(new LinearFactor(key,A,b));
		result.push_back(prior);
	}
	return result;
}

/* ************************************************************************* */  
pair<Matrix,Vector> LinearFactorGraph::matrix(const Ordering& ordering) const {

	// get all factors
	LinearFactorSet found;
	BOOST_FOREACH(shared_factor factor,factors_)
		found.push_back(factor);

	// combine them
	LinearFactor lf(found);

	// Return Matrix and Vector
	return lf.matrix(ordering);
}

/* ************************************************************************* */
