/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    GaussianFactorGraph.cpp
 * @brief   Linear Factor Graph where all factors are Gaussians
 * @author  Kai Ni
 * @author  Christian Potthast
 */

#include <vector>

#include <boost/foreach.hpp>
#include <boost/make_shared.hpp>
#include <boost/tuple/tuple.hpp>
#include <boost/numeric/ublas/lu.hpp>
#include <boost/numeric/ublas/io.hpp>

#include <gtsam/linear/GaussianFactorGraph.h>
#include <gtsam/linear/GaussianFactorSet.h>
#include <gtsam/inference/FactorGraph-inl.h>
#include <gtsam/inference/inference-inl.h>
#include <gtsam/linear/iterative.h>


using namespace std;
using namespace gtsam;

// trick from some reading group
#define FOREACH_PAIR( KEY, VAL, COL) BOOST_FOREACH (boost::tie(KEY,VAL),COL)

namespace gtsam {

// Explicitly instantiate so we don't have to include everywhere
INSTANTIATE_FACTOR_GRAPH(GaussianFactor);

/* ************************************************************************* */
GaussianFactorGraph::GaussianFactorGraph(const GaussianBayesNet& CBN) :
	FactorGraph<GaussianFactor> (CBN) {
}

/* ************************************************************************* */
GaussianFactorGraph::Keys GaussianFactorGraph::keys() const {
  std::set<Index, std::less<Index>, boost::fast_pool_allocator<Index> > keys;
  BOOST_FOREACH(const sharedFactor& factor, *this) {
    if(factor) keys.insert(factor->begin(), factor->end()); }
  return keys;
}

/* ************************************************************************* */
void GaussianFactorGraph::permuteWithInverse(const Permutation& inversePermutation) {
  BOOST_FOREACH(const sharedFactor& factor, factors_) {
    factor->permuteWithInverse(inversePermutation);
  }
}

/* ************************************************************************* */
double GaussianFactorGraph::error(const VectorValues& x) const {
	double total_error = 0.;
	BOOST_FOREACH(sharedFactor factor,factors_)
		total_error += factor->error(x);
	return total_error;
}

/* ************************************************************************* */
Errors GaussianFactorGraph::errors(const VectorValues& x) const {
	return *errors_(x);
}

/* ************************************************************************* */
boost::shared_ptr<Errors> GaussianFactorGraph::errors_(const VectorValues& x) const {
	boost::shared_ptr<Errors> e(new Errors);
	BOOST_FOREACH(const sharedFactor& factor,factors_)
		e->push_back(factor->error_vector(x));
	return e;
}

/* ************************************************************************* */
Errors GaussianFactorGraph::operator*(const VectorValues& x) const {
	Errors e;
	BOOST_FOREACH(const sharedFactor& Ai,factors_)
		e.push_back((*Ai)*x);
	return e;
}

/* ************************************************************************* */
void GaussianFactorGraph::multiplyInPlace(const VectorValues& x, Errors& e) const {
	multiplyInPlace(x,e.begin());
}

/* ************************************************************************* */
void GaussianFactorGraph::multiplyInPlace(const VectorValues& x,
		const Errors::iterator& e) const {
	Errors::iterator ei = e;
	BOOST_FOREACH(const sharedFactor& Ai,factors_) {
		*ei = (*Ai)*x;
		ei++;
	}
}


/* ************************************************************************* */
// x += alpha*A'*e
void GaussianFactorGraph::transposeMultiplyAdd(double alpha, const Errors& e,
		VectorValues& x) const {
	// For each factor add the gradient contribution
	Errors::const_iterator ei = e.begin();
	BOOST_FOREACH(const sharedFactor& Ai,factors_)
		Ai->transposeMultiplyAdd(alpha,*(ei++),x);
}

/* ************************************************************************* */
VectorValues GaussianFactorGraph::gradient(const VectorValues& x) const {
	// It is crucial for performance to make a zero-valued clone of x
	VectorValues g = VectorValues::zero(x);
	transposeMultiplyAdd(1.0, errors(x), g);
	return g;
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
GaussianFactorGraph GaussianFactorGraph::add_priors(double sigma, const vector<size_t>& dimensions) const {

	// start with this factor graph
	GaussianFactorGraph result = *this;

	// for each of the variables, add a prior
	for(Index j=0; j<dimensions.size(); ++j) {
	  size_t dim = dimensions[j];
		Matrix A = eye(dim);
		Vector b = zero(dim);
		SharedDiagonal model = noiseModel::Isotropic::Sigma(dim,sigma);
		sharedFactor prior(new GaussianFactor(j, A, b, model));
		result.push_back(prior);
	}
	return result;
}

bool GaussianFactorGraph::split(const std::map<Index, Index> &M, GaussianFactorGraph &Ab1, GaussianFactorGraph &Ab2) const {

	//typedef sharedFactor F ;

	Ab1 = GaussianFactorGraph();
	Ab2 = GaussianFactorGraph();

	BOOST_FOREACH(const sharedFactor& factor, factors_) {

		if (factor->keys().size() > 2)
			throw(invalid_argument("split: only support factors with at most two keys"));
		if (factor->keys().size() == 1) {
			Ab1.push_back(factor);
			Ab2.push_back(factor);
			continue;
		}
		Index key1 = factor->keys_[0];
		Index key2 = factor->keys_[1];

		if ((M.find(key1) != M.end() && M.find(key1)->second == key2) ||
			(M.find(key2) != M.end() && M.find(key2)->second == key1))
			Ab1.push_back(factor);
		else
			Ab2.push_back(factor);
	}

	return true ;
}

VectorValues GaussianFactorGraph::allocateVectorValuesb() const {
	std::vector<size_t> dimensions(size()) ;
	Index i = 0 ;
	BOOST_FOREACH( const sharedFactor& factor, factors_) {
		dimensions[i] = factor->numberOfRows() ;
		i++;
	}

	return VectorValues(dimensions) ;
}


bool GaussianFactorGraph::getDiagonalOfHessian(VectorValues &values) const {

	values.makeZero() ;

	BOOST_FOREACH( const sharedFactor& factor, factors_) {
		Index i = 0 ;
		BOOST_FOREACH( const Index& idx, factor->keys_) {
			Vector v = columnNormSquare(factor->Ab_(i)) ;
			values[idx] += v;
			++i ;
		}
	}
	return true ;
}

void GaussianFactorGraph::residual(const VectorValues &x, VectorValues &r) const {

	getb(r) ;
	VectorValues Ax = VectorValues::SameStructure(r) ;
	multiply(x,Ax) ;
	axpy(-1.0,Ax,r) ;
}

void GaussianFactorGraph::multiply(const VectorValues &x, VectorValues &r) const {

	r.makeZero() ;
	Index i = 0 ;
	BOOST_FOREACH(const sharedFactor& factor, factors_) {
		Index j = 0 ;
		BOOST_FOREACH( const Index& idx, factor->keys_ ) {
			r[i] += prod(factor->Ab_(j), x[idx]) ;
			++j ;
		}
		++i ;
	}
}

void GaussianFactorGraph::transposeMultiply(const VectorValues &r, VectorValues &x) const {
	x.makeZero() ;
	Index i = 0 ;
	BOOST_FOREACH(const sharedFactor& factor, factors_) {
		Index j = 0 ;
		BOOST_FOREACH( const Index& idx, factor->keys_ ) {
			x[idx] += prod(trans(factor->Ab_(j)), r[i]) ;
			++j ;
		}
		++i ;
	}
}

void GaussianFactorGraph::getb(VectorValues &b) const {
	Index i = 0 ;
	BOOST_FOREACH( const sharedFactor& factor, factors_) {
		b[i] = factor->getb() ;
		i++;
	}
}

VectorValues GaussianFactorGraph::getb() const {
  VectorValues b = allocateVectorValuesb() ;
  getb(b) ;
  return b ;
}

} // namespace gtsam
