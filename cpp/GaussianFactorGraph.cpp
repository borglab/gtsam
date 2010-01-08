/**
 * @file    GaussianFactorGraph.cpp
 * @brief   Linear Factor Graph where all factors are Gaussians
 * @author  Kai Ni
 * @author  Christian Potthast
 */

#include <boost/foreach.hpp>
#include <boost/tuple/tuple.hpp>
#include <boost/numeric/ublas/lu.hpp>
#include <boost/numeric/ublas/io.hpp>

#include <colamd/colamd.h>

#include "GaussianFactorGraph.h"
#include "GaussianFactorSet.h"
#include "FactorGraph-inl.h"
#include "inference-inl.h"
#include "iterative.h"

using namespace std;
using namespace gtsam;

// trick from some reading group
#define FOREACH_PAIR( KEY, VAL, COL) BOOST_FOREACH (boost::tie(KEY,VAL),COL)

// Explicitly instantiate so we don't have to include everywhere
template class FactorGraph<GaussianFactor>;

/* ************************************************************************* */
GaussianFactorGraph::GaussianFactorGraph(const GaussianBayesNet& CBN) :
	FactorGraph<GaussianFactor> (CBN) {
}

/* ************************************************************************* */
double GaussianFactorGraph::error(const VectorConfig& x) const {
	double total_error = 0.;
	BOOST_FOREACH(sharedFactor factor,factors_)
		total_error += factor->error(x);
	return total_error;
}

/* ************************************************************************* */
Errors GaussianFactorGraph::errors(const VectorConfig& x) const {
	Errors e;
	BOOST_FOREACH(sharedFactor factor,factors_)
		e.push_back(factor->error_vector(x));
	return e;
}

/* ************************************************************************* */
Errors GaussianFactorGraph::operator*(const VectorConfig& x) const {
	Errors e;
	BOOST_FOREACH(sharedFactor Ai,factors_)
		e.push_back((*Ai)*x);
	return e;
}

/* ************************************************************************* */
VectorConfig GaussianFactorGraph::operator^(const Errors& e) const {
	VectorConfig x;
	// For each factor add the gradient contribution
	Errors::const_iterator it = e.begin();
	BOOST_FOREACH(sharedFactor Ai,factors_)
		x += (*Ai)^(*(it++));
	return x;
}

/* ************************************************************************* */
VectorConfig GaussianFactorGraph::gradient(const VectorConfig& x) const {
	const GaussianFactorGraph& A = *this;
	return A^errors(x);
}

/* ************************************************************************* */
set<string> GaussianFactorGraph::find_separator(const string& key) const
{
	set<string> separator;
	BOOST_FOREACH(sharedFactor factor,factors_)
		factor->tally_separator(key,separator);

	return separator;
}

/* ************************************************************************* */
GaussianConditional::shared_ptr
GaussianFactorGraph::eliminateOne(const std::string& key) {
	return gtsam::eliminateOne<GaussianFactor,GaussianConditional>(*this, key);
}

/* ************************************************************************* */
GaussianBayesNet
GaussianFactorGraph::eliminate(const Ordering& ordering)
{
	GaussianBayesNet chordalBayesNet; // empty
	BOOST_FOREACH(string key, ordering) {
		GaussianConditional::shared_ptr cg = eliminateOne(key);
		chordalBayesNet.push_back(cg);
	}
	return chordalBayesNet;
}

/* ************************************************************************* */
VectorConfig GaussianFactorGraph::optimize(const Ordering& ordering)
{
	// eliminate all nodes in the given ordering -> chordal Bayes net
	GaussianBayesNet chordalBayesNet = eliminate(ordering);

	// calculate new configuration (using backsubstitution)
	return ::optimize(chordalBayesNet);
}

/* ************************************************************************* */
boost::shared_ptr<GaussianBayesNet>
GaussianFactorGraph::eliminate_(const Ordering& ordering)
{
	boost::shared_ptr<GaussianBayesNet> chordalBayesNet(new GaussianBayesNet); // empty
	BOOST_FOREACH(string key, ordering) {
		GaussianConditional::shared_ptr cg = eliminateOne(key);
		chordalBayesNet->push_back(cg);
	}
	return chordalBayesNet;
}

/* ************************************************************************* */
boost::shared_ptr<VectorConfig>
GaussianFactorGraph::optimize_(const Ordering& ordering) {
	return boost::shared_ptr<VectorConfig>(new VectorConfig(optimize(ordering)));
}

/* ************************************************************************* */
void GaussianFactorGraph::combine(const GaussianFactorGraph &lfg){
	for(const_iterator factor=lfg.factors_.begin(); factor!=lfg.factors_.end(); factor++){
		push_back(*factor);
	}
}

/* ************************************************************************* */
GaussianFactorGraph GaussianFactorGraph::combine2(const GaussianFactorGraph& lfg1,
		const GaussianFactorGraph& lfg2) {

	// create new linear factor graph equal to the first one
	GaussianFactorGraph fg = lfg1;

	// add the second factors_ in the graph
	for (const_iterator factor = lfg2.factors_.begin(); factor
			!= lfg2.factors_.end(); factor++) {
		fg.push_back(*factor);
	}
	return fg;
}


/* ************************************************************************* */  
Dimensions GaussianFactorGraph::dimensions() const {
	Dimensions result;
	BOOST_FOREACH(sharedFactor factor,factors_) {
		Dimensions vs = factor->dimensions();
		string key; int dim;
		FOREACH_PAIR(key,dim,vs) result.insert(make_pair(key,dim));
	}
	return result;
}

/* ************************************************************************* */  
GaussianFactorGraph GaussianFactorGraph::add_priors(double sigma) const {

	// start with this factor graph
	GaussianFactorGraph result = *this;

	// find all variables and their dimensions
	Dimensions vs = dimensions();

	// for each of the variables, add a prior
	string key; int dim;
	FOREACH_PAIR(key,dim,vs) {
		Matrix A = eye(dim);
		Vector b = zero(dim);
		sharedFactor prior(new GaussianFactor(key,A,b, sigma));
		result.push_back(prior);
	}
	return result;
}

/* ************************************************************************* */
Errors GaussianFactorGraph::rhs() const {
	Errors e;
	BOOST_FOREACH(sharedFactor factor,factors_)
		e.push_back(ediv(factor->get_b(),factor->get_sigmas()));
	return e;
}

/* ************************************************************************* */  
pair<Matrix,Vector> GaussianFactorGraph::matrix(const Ordering& ordering) const {

	// get all factors
	GaussianFactorSet found;
	BOOST_FOREACH(sharedFactor factor,factors_)
		found.push_back(factor);

	// combine them
	GaussianFactor lf(found);

	// Return Matrix and Vector
	return lf.matrix(ordering);
}

/* ************************************************************************* */
Dimensions GaussianFactorGraph::columnIndices(const Ordering& ordering) const {

	// get the dimensions for all variables
	Dimensions variableSet = dimensions();

	// Find the starting index and dimensions for all variables given the order
	size_t j = 1;
	Dimensions result;
	BOOST_FOREACH(string key, ordering) {
		// associate key with first column index
		result.insert(make_pair(key,j));
		// find dimension for this key
		Dimensions::const_iterator it = variableSet.find(key);
		// advance column index to next block by adding dim(key)
		j += it->second;
	}

	return result;
}

/* ************************************************************************* */
Matrix GaussianFactorGraph::sparse(const Ordering& ordering) const {

	// return values
	list<int> I,J;
	list<double> S;

	// get the starting column indices for all variables
	Dimensions indices = columnIndices(ordering);

	// Collect the I,J,S lists for all factors
	int row_index = 0;
	BOOST_FOREACH(sharedFactor factor,factors_) {

		// get sparse lists for the factor
		list<int> i1,j1;
		list<double> s1;
		boost::tie(i1,j1,s1) = factor->sparse(indices);

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
VectorConfig GaussianFactorGraph::optimalUpdate(const VectorConfig& x,
		const VectorConfig& d) const {

	// create a new graph on step-size
	GaussianFactorGraph alphaGraph;
	BOOST_FOREACH(sharedFactor factor,factors_) {
		sharedFactor alphaFactor = factor->alphaFactor(x,d);
		alphaGraph.push_back(alphaFactor);
	}

	// solve it for optimal step-size alpha
	GaussianConditional::shared_ptr gc = alphaGraph.eliminateOne("alpha");
	double alpha = gc->get_d()(0);
	cout << alpha << endl;

	// return updated estimate by stepping in direction d
  return expmap(x, d.scale(alpha));
}

/* ************************************************************************* */
VectorConfig GaussianFactorGraph::steepestDescent(const VectorConfig& x0,
		bool verbose, double epsilon, size_t maxIterations) const {
	return gtsam::steepestDescent(*this, x0, verbose, epsilon, maxIterations);
}

/* ************************************************************************* */
boost::shared_ptr<VectorConfig> GaussianFactorGraph::steepestDescent_(
		const VectorConfig& x0, bool verbose, double epsilon, size_t maxIterations) const {
	return boost::shared_ptr<VectorConfig>(new VectorConfig(
			gtsam::conjugateGradientDescent(*this, x0, verbose, epsilon,
					maxIterations)));
}

/* ************************************************************************* */
VectorConfig GaussianFactorGraph::conjugateGradientDescent(
		const VectorConfig& x0, bool verbose, double epsilon, size_t maxIterations) const {
	return gtsam::conjugateGradientDescent(*this, x0, verbose, epsilon,
			maxIterations);
}

/* ************************************************************************* */
boost::shared_ptr<VectorConfig> GaussianFactorGraph::conjugateGradientDescent_(
		const VectorConfig& x0, bool verbose, double epsilon, size_t maxIterations) const {
	return boost::shared_ptr<VectorConfig>(new VectorConfig(
			gtsam::conjugateGradientDescent(*this, x0, verbose, epsilon,
					maxIterations)));
}

/* ************************************************************************* */

