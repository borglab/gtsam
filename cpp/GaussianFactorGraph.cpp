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
#include <boost/assign/std/list.hpp> // for operator += in Ordering

#include <colamd/colamd.h>

#include "GaussianFactorGraph.h"
#include "GaussianFactorSet.h"
#include "FactorGraph-inl.h"
#include "inference-inl.h"
#include "iterative.h"

#ifdef USE_SPQR
#include <spqr.hpp>;
#endif

using namespace std;
using namespace gtsam;
using namespace boost::assign;

//#define USE_FAST_ELIMINATE


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
	return *errors_(x);
}

/* ************************************************************************* */
boost::shared_ptr<Errors> GaussianFactorGraph::errors_(const VectorConfig& x) const {
	boost::shared_ptr<Errors> e(new Errors);
	BOOST_FOREACH(const sharedFactor& factor,factors_)
		e->push_back(factor->error_vector(x));
	return e;
}

/* ************************************************************************* */
Errors GaussianFactorGraph::operator*(const VectorConfig& x) const {
	Errors e;
	BOOST_FOREACH(const sharedFactor& Ai,factors_)
		e.push_back((*Ai)*x);
	return e;
}

/* ************************************************************************* */
void GaussianFactorGraph::multiplyInPlace(const VectorConfig& x, Errors& e) const {
	multiplyInPlace(x,e.begin());
}

/* ************************************************************************* */
void GaussianFactorGraph::multiplyInPlace(const VectorConfig& x,
		const Errors::iterator& e) const {
	Errors::iterator ei = e;
	BOOST_FOREACH(const sharedFactor& Ai,factors_) {
		*ei = (*Ai)*x;
		ei++;
	}
}

/* ************************************************************************* */
VectorConfig GaussianFactorGraph::operator^(const Errors& e) const {
	VectorConfig x;
	// For each factor add the gradient contribution
	Errors::const_iterator it = e.begin();
	BOOST_FOREACH(const sharedFactor& Ai,factors_) {
		VectorConfig xi = (*Ai)^(*(it++));
		x.insertAdd(xi);
	}
	return x;
}

/* ************************************************************************* */
// x += alpha*A'*e
void GaussianFactorGraph::transposeMultiplyAdd(double alpha, const Errors& e,
		VectorConfig& x) const {
	// For each factor add the gradient contribution
	Errors::const_iterator ei = e.begin();
	BOOST_FOREACH(const sharedFactor& Ai,factors_)
		Ai->transposeMultiplyAdd(alpha,*(ei++),x);
}

/* ************************************************************************* */
VectorConfig GaussianFactorGraph::gradient(const VectorConfig& x) const {
	// It is crucial for performance to make a zero-valued clone of x
	VectorConfig g = VectorConfig::zero(x);
	transposeMultiplyAdd(1.0, errors(x), g);
	return g;
}

/* ************************************************************************* */
set<Symbol> GaussianFactorGraph::find_separator(const Symbol& key) const
{
	set<Symbol> separator;
	BOOST_FOREACH(const sharedFactor& factor,factors_)
		factor->tally_separator(key,separator);

	return separator;
}

/* ************************************************************************* */
GaussianConditional::shared_ptr
GaussianFactorGraph::eliminateOne(const Symbol& key, bool old) {
#ifdef USE_FAST_ELIMINATE
	return eliminateOneMatrixJoin(key);
#else
	if (old)
		return gtsam::eliminateOne<GaussianFactor,GaussianConditional>(*this, key);
	else
		return eliminateOneMatrixJoin(key);
#endif
}

/* ************************************************************************* */
GaussianConditional::shared_ptr
GaussianFactorGraph::eliminateOneMatrixJoin(const Symbol& key) {
	// find and remove all factors connected to key
	vector<GaussianFactor::shared_ptr> factors = findAndRemoveFactors(key);

	// Collect all dimensions as well as the set of separator keys
	set<Symbol> separator;
	Dimensions dimensions;
	BOOST_FOREACH(const sharedFactor& factor, factors) {
		Dimensions factor_dim = factor->dimensions();
		dimensions.insert(factor_dim.begin(), factor_dim.end());
		BOOST_FOREACH(const Symbol& k, factor->keys())
			if (!k.equals(key))
				separator.insert(k);
	}

	// add the keys to the rendering
	Ordering render; render += key;
	BOOST_FOREACH(const Symbol& k, separator)
			if (k != key) render += k;

	// combine the factors to get a noisemodel and a combined matrix
	Matrix Ab; SharedDiagonal model;

	boost::tie(Ab, model) =
			GaussianFactor::combineFactorsAndCreateMatrix(factors,render,dimensions);

	// eliminate that joint factor
	GaussianFactor::shared_ptr factor;
	GaussianConditional::shared_ptr conditional;
	boost::tie(conditional, factor) =
			GaussianFactor::eliminateMatrix(Ab, model, render, dimensions);

	// add new factor on separator back into the graph
	if (!factor->empty()) push_back(factor);

	// return the conditional Gaussian
	return conditional;
}

/* ************************************************************************* */
GaussianBayesNet
GaussianFactorGraph::eliminate(const Ordering& ordering, bool old)
{
	GaussianBayesNet chordalBayesNet; // empty
	BOOST_FOREACH(const Symbol& key, ordering) {
		GaussianConditional::shared_ptr cg = eliminateOne(key, old);
		chordalBayesNet.push_back(cg);
	}
	return chordalBayesNet;
}

/* ************************************************************************* */
VectorConfig GaussianFactorGraph::optimize(const Ordering& ordering, bool old)
{
	bool verbose = false;
	if (verbose)
		BOOST_FOREACH(const sharedFactor& factor,factors_)
			factor->get_model()->print("Starting model");

	// eliminate all nodes in the given ordering -> chordal Bayes net
	GaussianBayesNet chordalBayesNet = eliminate(ordering, old);

	// calculate new configuration (using backsubstitution)
	VectorConfig delta = ::optimize(chordalBayesNet);
	return delta;
}

/* ************************************************************************* */
boost::shared_ptr<GaussianBayesNet>
GaussianFactorGraph::eliminate_(const Ordering& ordering)
{
	boost::shared_ptr<GaussianBayesNet> chordalBayesNet(new GaussianBayesNet); // empty
	BOOST_FOREACH(const Symbol& key, ordering) {
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
	BOOST_FOREACH(const sharedFactor& factor,factors_) {
		Dimensions vs = factor->dimensions();
		Symbol key; int dim;
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
	Symbol key; int dim;
	FOREACH_PAIR(key,dim,vs) {
		Matrix A = eye(dim);
		Vector b = zero(dim);
		SharedDiagonal model = noiseModel::Isotropic::Sigma(dim,sigma);
		sharedFactor prior(new GaussianFactor(key,A,b, model));
		result.push_back(prior);
	}
	return result;
}

/* ************************************************************************* */
Errors GaussianFactorGraph::rhs() const {
	Errors e;
	BOOST_FOREACH(const sharedFactor& factor,factors_)
		e.push_back(ediv(factor->get_b(),factor->get_sigmas()));
	return e;
}

/* ************************************************************************* */
Vector GaussianFactorGraph::rhsVector() const {
	Errors e = rhs();
	return concatVectors(e);
}

/* ************************************************************************* */  
pair<Matrix,Vector> GaussianFactorGraph::matrix(const Ordering& ordering) const {

	// get all factors
	GaussianFactorSet found;
	BOOST_FOREACH(const sharedFactor& factor,factors_)
		found.push_back(factor);

	// combine them
	GaussianFactor lf(found);

	// Return Matrix and Vector
	return lf.matrix(ordering);
}

/* ************************************************************************* */
VectorConfig GaussianFactorGraph::assembleConfig(const Vector& vs, const Ordering& ordering) const {
	Dimensions dims = dimensions();
	VectorConfig config;
	Vector::const_iterator itSrc = vs.begin();
	Vector::iterator itDst;
	BOOST_FOREACH(const Symbol& key, ordering){
		int dim = dims.find(key)->second;
		Vector v(dim);
		for (itDst=v.begin(); itDst!=v.end(); itDst++, itSrc++)
			*itDst = *itSrc;
		config.insert(key, v);
	}
	if (itSrc != vs.end())
		throw runtime_error("assembleConfig: input vector and ordering are not compatible with the graph");
	return config;
}

/* ************************************************************************* */
pair<Dimensions, size_t> GaussianFactorGraph::columnIndices_(const Ordering& ordering) const {

	// get the dimensions for all variables
	Dimensions variableSet = dimensions();

	// Find the starting index and dimensions for all variables given the order
	size_t j = 1;
	Dimensions result;
	BOOST_FOREACH(const Symbol& key, ordering) {
		// associate key with first column index
		result.insert(make_pair(key,j));
		// find dimension for this key
		Dimensions::const_iterator it = variableSet.find(key);
		if (it==variableSet.end()) // key not found, now what ?
				throw invalid_argument("ColumnIndices: this ordering contains keys not in the graph");
		// advance column index to next block by adding dim(key)
		j += it->second;
	}

	return make_pair(result, j-1);
}

/* ************************************************************************* */
Dimensions GaussianFactorGraph::columnIndices(const Ordering& ordering) const {
	return columnIndices_(ordering).first;
}

/* ************************************************************************* */
pair<size_t, size_t> GaussianFactorGraph::sizeOfA() const {
	size_t m = 0, n = 0;
	Dimensions variableSet = dimensions();
	BOOST_FOREACH(const Dimensions::value_type value, variableSet)
		n += value.second;
	BOOST_FOREACH(const sharedFactor& factor,factors_)
		m += factor->numberOfRows();
	return make_pair(m, n);
}

/* ************************************************************************* */
Matrix GaussianFactorGraph::sparse(const Ordering& ordering) const {

	// get the starting column indices for all variables
	Dimensions indices = columnIndices(ordering);

	return sparse(indices);
}

/* ************************************************************************* */
Matrix GaussianFactorGraph::sparse(const Dimensions& indices) const {

	// return values
	list<int> I,J;
	list<double> S;

	// Collect the I,J,S lists for all factors
	int row_index = 0;
	BOOST_FOREACH(const sharedFactor& factor,factors_) {

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
