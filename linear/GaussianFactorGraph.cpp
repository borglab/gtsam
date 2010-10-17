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

#include <boost/foreach.hpp>
#include <boost/tuple/tuple.hpp>
#include <boost/numeric/ublas/lu.hpp>
#include <boost/numeric/ublas/io.hpp>

#include <gtsam/linear/GaussianFactorGraph.h>
#include <gtsam/linear/GaussianFactorSet.h>
#include <gtsam/inference/FactorGraph-inl.h>
#include <gtsam/inference/inference-inl.h>
#include <gtsam/linear/iterative.h>
//#include <gtsam/linear/GaussianJunctionTree.h>

using namespace std;
using namespace gtsam;
using namespace boost::assign;

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

///* ************************************************************************* */
//VectorValues GaussianFactorGraph::operator^(const Errors& e) const {
//	VectorValues x;
//	// For each factor add the gradient contribution
//	Errors::const_iterator it = e.begin();
//	BOOST_FOREACH(const sharedFactor& Ai,factors_) {
//		VectorValues xi = (*Ai)^(*(it++));
//		x.insertAdd(xi);
//	}
//	return x;
//}

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
GaussianFactorGraph GaussianFactorGraph::add_priors(double sigma, const GaussianVariableIndex<>& variableIndex) const {

	// start with this factor graph
	GaussianFactorGraph result = *this;

	// for each of the variables, add a prior
	for(Index var=0; var<variableIndex.size(); ++var) {
	  size_t dim = variableIndex.dim(var);
		Matrix A = eye(dim);
		Vector b = zero(dim);
		SharedDiagonal model = noiseModel::Isotropic::Sigma(dim,sigma);
		sharedFactor prior(new GaussianFactor(var,A,b, model));
		result.push_back(prior);
	}
	return result;
}

} // namespace gtsam
