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
#include <gtsam/linear/iterative.h>
#include <gtsam/linear/HessianFactor.h>


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
    FastSet<Index> keys;
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
  std::vector<boost::tuple<size_t,size_t,double> >
  GaussianFactorGraph::sparseJacobian(const std::vector<size_t>& columnIndices) const {
    std::vector<boost::tuple<size_t,size_t,double> > entries;
    size_t i = 0;
    BOOST_FOREACH(const sharedFactor& factor, *this) {
      // Convert to JacobianFactor if necessary
      JacobianFactor::shared_ptr jacobianFactor(boost::dynamic_pointer_cast<JacobianFactor>(factor));
      if(!jacobianFactor) {
        HessianFactor::shared_ptr hessianFactor(boost::dynamic_pointer_cast<HessianFactor>(factor));
        if(hessianFactor)
          jacobianFactor.reset(new JacobianFactor(*hessianFactor));
        else
          throw invalid_argument("GaussianFactorGraph contains a factor that is neither a JacobianFactor nor a HessianFactor.");
      }

      // Add entries, adjusting the row index i
      std::vector<boost::tuple<size_t,size_t,double> > factorEntries(jacobianFactor->sparse(columnIndices));
      entries.reserve(entries.size() + factorEntries.size());
      for(size_t entry=0; entry<factorEntries.size(); ++entry)
        entries.push_back(boost::make_tuple(factorEntries[entry].get<0>()+i, factorEntries[entry].get<1>(), factorEntries[entry].get<2>()));

      // Increment row index
      i += jacobianFactor->size1();
    }
    return entries;
  }

//  VectorValues GaussianFactorGraph::allocateVectorValuesb() const {
//    std::vector<size_t> dimensions(size()) ;
//    Index i = 0 ;
//    BOOST_FOREACH( const sharedFactor& factor, factors_) {
//      dimensions[i] = factor->numberOfRows() ;
//      i++;
//    }
//
//    return VectorValues(dimensions) ;
//  }
//
//  void GaussianFactorGraph::getb(VectorValues &b) const {
//    Index i = 0 ;
//    BOOST_FOREACH( const sharedFactor& factor, factors_) {
//      b[i] = factor->getb();
//      i++;
//    }
//  }
//
//  VectorValues GaussianFactorGraph::getb() const {
//    VectorValues b = allocateVectorValuesb() ;
//    getb(b) ;
//    return b ;
//  }

} // namespace gtsam
