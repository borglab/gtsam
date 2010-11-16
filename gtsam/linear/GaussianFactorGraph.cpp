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
    for(GaussianFactor::const_iterator j = factor->begin(); j != factor->end(); ++j) {
			r[i] += prod(factor->getA(j), x[*j]);
		}
		++i ;
	}
}

void GaussianFactorGraph::transposeMultiply(const VectorValues &r, VectorValues &x) const {
	x.makeZero() ;
	Index i = 0 ;
	BOOST_FOREACH(const sharedFactor& factor, factors_) {
    for(GaussianFactor::const_iterator j = factor->begin(); j != factor->end(); ++j) {
			x[*j] += prod(trans(factor->getA(j)), r[i]) ;
		}
		++i ;
	}
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
