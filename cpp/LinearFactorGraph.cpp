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

#include "FactorGraph-inl.h"
#include "LinearFactorGraph.h"
#include "LinearFactorSet.h"

using namespace std;
using namespace gtsam;

// trick from some reading group
#define FOREACH_PAIR( KEY, VAL, COL) BOOST_FOREACH (boost::tie(KEY,VAL),COL)

// Explicitly instantiate so we don't have to include everywhere
template class FactorGraph<LinearFactor>;

/* ************************************************************************* */
LinearFactorGraph::LinearFactorGraph(const GaussianBayesNet& CBN) :
	FactorGraph<LinearFactor> (CBN) {
}

/* ************************************************************************* */
set<string> LinearFactorGraph::find_separator(const string& key) const
{
	set<string> separator;
	BOOST_FOREACH(shared_factor factor,factors_)
		factor->tally_separator(key,separator);

	return separator;
}

/* ************************************************************************* */
GaussianBayesNet::shared_ptr
LinearFactorGraph::eliminate(const Ordering& ordering)
{
	GaussianBayesNet::shared_ptr chordalBayesNet (new GaussianBayesNet()); // empty
	BOOST_FOREACH(string key, ordering) {
		ConditionalGaussian::shared_ptr cg = eliminateOne<ConditionalGaussian>(key);
		chordalBayesNet->push_back(cg);
	}
	return chordalBayesNet;
}

/* ************************************************************************* */
VectorConfig LinearFactorGraph::optimize(const Ordering& ordering)
{
	// eliminate all nodes in the given ordering -> chordal Bayes net
	GaussianBayesNet::shared_ptr chordalBayesNet = eliminate(ordering);

	// calculate new configuration (using backsubstitution)
	boost::shared_ptr<VectorConfig> newConfig = chordalBayesNet->optimize();

	return *newConfig;
}

/* ************************************************************************* */
void LinearFactorGraph::combine(const LinearFactorGraph &lfg){
	for(const_iterator factor=lfg.factors_.begin(); factor!=lfg.factors_.end(); factor++){
		push_back(*factor);
	}
}

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
Dimensions LinearFactorGraph::dimensions() const {
	Dimensions result;
	BOOST_FOREACH(shared_factor factor,factors_) {
		Dimensions vs = factor->dimensions();
		string key; int dim;
		FOREACH_PAIR(key,dim,vs) result.insert(make_pair(key,dim));
	}
	return result;
}

/* ************************************************************************* */  
LinearFactorGraph LinearFactorGraph::add_priors(double sigma) const {

	// start with this factor graph
	LinearFactorGraph result = *this;

	// find all variables and their dimensions
	Dimensions vs = dimensions();

	// for each of the variables, add a prior
	string key; int dim;
	FOREACH_PAIR(key,dim,vs) {
		Matrix A = eye(dim);
		Vector b = zero(dim);
		shared_factor prior(new LinearFactor(key,A,b, sigma));
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
Matrix LinearFactorGraph::sparse(const Ordering& ordering) const {

	// return values
	list<int> I,J;
	list<double> S;

	// get the dimensions for all variables
	Dimensions variableSet = dimensions();

	// Collect the I,J,S lists for all factors
	int row_index = 0;
	BOOST_FOREACH(shared_factor factor,factors_) {

		// get sparse lists for the factor
		list<int> i1,j1;
		list<double> s1;
		boost::tie(i1,j1,s1) = factor->sparse(ordering,variableSet);

		// add row_start to every row index
		transform(i1.begin(), i1.end(), i1.begin(), bind2nd(plus<int>(), row_index));

		// splice lists from factor to the end of the global lists
		I.splice(I.end(), i1);
		J.splice(J.end(), j1);
		S.splice(S.end(), s1);

		// advance row start
		row_index += factor->numberOfRows();
	}

	// Convert them to vectors for MATLAB
	// TODO: just create a sparse matrix class already
	size_t nzmax = S.size();
	Matrix ijs(3,nzmax);
	copy(I.begin(),I.end(),ijs.begin2());
	copy(J.begin(),J.end(),ijs.begin2()+nzmax);
	copy(S.begin(),S.end(),ijs.begin2()+2*nzmax);

	// return the result
	return ijs;
}

/* ************************************************************************* */
