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
#include "FactorGraph-inl.h"
#include "LinearFactorGraph.h"

using namespace std;
using namespace gtsam;

// Explicitly instantiate so we don't have to include everywhere
template class FactorGraph<LinearFactor>;

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
// eliminate factor graph using the given (not necessarily complete)
// ordering, yielding a chordal Bayes net and partially eliminated FG
/* ************************************************************************* */
ChordalBayesNet::shared_ptr
LinearFactorGraph::eliminate_partially(const Ordering& ordering)
{
	ChordalBayesNet::shared_ptr chordalBayesNet (new ChordalBayesNet()); // empty

	BOOST_FOREACH(string key, ordering) {
		ConditionalGaussian::shared_ptr cg = eliminateOne<ConditionalGaussian>(key);
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
