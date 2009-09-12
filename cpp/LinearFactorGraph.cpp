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
/** O(n)                                                                     */
/* ************************************************************************* */ 
LinearFactorSet
LinearFactorGraph::find_factors_and_remove(const string& key)
{
	LinearFactorSet found;

	for(iterator factor=factors_.begin(); factor!=factors_.end(); )
		if ((*factor)->involves(key)) {
			found.insert(*factor);
			factor = factors_.erase(factor);
		} else {
			factor++; // important, erase will have effect of ++
		}

	return found;
}

/* ************************************************************************* */
/* find factors and remove them from the factor graph: O(n)                  */
/* ************************************************************************* */ 
boost::shared_ptr<MutableLinearFactor> 
LinearFactorGraph::combine_factors(const string& key)
{
	LinearFactorSet found = find_factors_and_remove(key);
	boost::shared_ptr<MutableLinearFactor> lf(new MutableLinearFactor(found));
	return lf;
}

/* ************************************************************************* */
/* eliminate one node from the linear factor graph                           */ 
/* ************************************************************************* */
ConditionalGaussian::shared_ptr LinearFactorGraph::eliminate_one(const string& key)
{
	// combine the factors of all nodes connected to the variable to be eliminated
	// if no factors are connected to key, returns an empty factor
	boost::shared_ptr<MutableLinearFactor> joint_factor = combine_factors(key);

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
FGConfig LinearFactorGraph::optimize(const Ordering& ordering)
{
	// eliminate all nodes in the given ordering -> chordal Bayes net
	ChordalBayesNet::shared_ptr chordalBayesNet = eliminate(ordering);

	// calculate new configuration (using backsubstitution)
	boost::shared_ptr<FGConfig> newConfig = chordalBayesNet->optimize();

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
		found.insert(factor);

	// combine them
	MutableLinearFactor lf(found);

	// Return Matrix and Vector
	return lf.matrix(ordering);
}

/* ************************************************************************* */  

/**
 * Call colamd given a column-major symbolic matrix A
 * @param n_col colamd arg 1: number of rows in A
 * @param n_row colamd arg 2: number of columns in A
 * @param nrNonZeros number of non-zero entries in A
 * @param columns map from keys to a sparse column of non-zero row indices
 */
template <class Key>
Ordering colamd(int n_col, int n_row, int nrNonZeros, const map<Key, vector<int> >& columns) {

	// Convert to compressed column major format colamd wants it in (== MATLAB format!)
	vector<Key> initialOrder;
	int Alen = nrNonZeros*30;     /* colamd arg 3: size of the array A TODO: use Tim's function ! */
	int * A = new int[Alen];      /* colamd arg 4: row indices of A, of size Alen */
	int * p = new int[n_col + 1]; /* colamd arg 5: column pointers of A, of size n_col+1 */
	{
		p[0] = 0;
		int j = 1;
		int count = 0;
		typedef typename map<Key, vector<int> >::const_iterator iterator;
		for(iterator it = columns.begin(); it != columns.end(); it++)
		{
			const Key& key = it->first;
			const vector<int>& column = it->second;
			initialOrder.push_back(key);
			BOOST_FOREACH(int i, column) A[count++] = i; // copy sparse column
			p[j] = count; // column j (base 1) goes from A[j-1] to A[j]-1
			j+=1;
		}
	}
	double* knobs = NULL;    /* colamd arg 6: parameters (uses defaults if NULL) */
	int stats[COLAMD_STATS]; /* colamd arg 7: colamd output statistics and error codes */

	// call colamd, result will be in p *************************************************
	/* TODO: returns (1) if successful, (0) otherwise*/
	colamd(n_row, n_col, Alen, A, p, knobs, stats);
	// **********************************************************************************
	delete [] A; // delete symbolic A

	// Convert elimination ordering in p to an ordering
	Ordering result;
	for(int j = 0; j < n_col; j++)
		result.push_back(initialOrder[j]);
	delete [] p; // delete colamd result vector

	return result;
}

/* ************************************************************************* */
Ordering LinearFactorGraph::getOrdering() const {

	// A factor graph is really laid out in row-major format, each factor a row
	// Below, we compute a symbolic matrix stored in sparse columns.
	typedef string Key;             // default case  with string keys
	map<Key, vector<int> > columns; // map from keys to a sparse column of non-zero row indices
	int nrNonZeros = 0;             // number of non-zero entries
	int n_row = factors_.size();    /* colamd arg 1: number of rows in A */

	// loop over all factors = rows
	for (int i = 0; i < n_row; i++) {
		shared_factor factor = factors_[i];
		for (map<Key, Matrix>::const_iterator lit = factor->begin(); lit	!= factor->end(); lit++) {
			const Key& key = lit->first;
			columns[key].push_back(i);
			nrNonZeros++;
		}
	}
	int n_col = (int)(columns.size()); /* colamd arg 2: number of columns in A */

	if(n_col == 0)
		return Ordering(); // empty ordering
	else
		return colamd(n_col, n_row, nrNonZeros, columns);
}

/* ************************************************************************* */
