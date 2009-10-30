/**
 * @file   ChordalBayesNet.cpp
 * @brief  Chordal Bayes Net, the result of eliminating a factor graph
 * @author Frank Dellaert
 */

#include <stdarg.h>
#include <boost/foreach.hpp>
#include <boost/tuple/tuple.hpp>

#include "ChordalBayesNet.h"
#include "VectorConfig.h"

using namespace std;
using namespace gtsam;

// Explicitly instantiate so we don't have to include everywhere
#include "BayesChain-inl.h"
template class BayesChain<ConditionalGaussian>;

// trick from some reading group
#define FOREACH_PAIR( KEY, VAL, COL) BOOST_FOREACH (boost::tie(KEY,VAL),COL) 

/* ************************************************************************* */
boost::shared_ptr<VectorConfig> ChordalBayesNet::optimize() const
{
  boost::shared_ptr<VectorConfig> result(new VectorConfig);
	
  /** solve each node in turn in topological sort order (parents first)*/
  BOOST_FOREACH(string key, keys_) {
    const_iterator cg = nodes_.find(key);		// get node
    assert( cg != nodes_.end() );						// make sure it exists
    Vector x = cg->second->solve(*result);  // Solve for that variable
    result->insert(key,x);                  // store result in partial solution
  }
  return result;
}

/* ************************************************************************* */  
pair<Matrix,Vector> ChordalBayesNet::matrix() const {

  // add the dimensions of all variables to get matrix dimension
  // and at the same time create a mapping from keys to indices
  size_t N=0; map<string,size_t> indices;
  BOOST_REVERSE_FOREACH(string key, keys_) {
    // find corresponding node
    const_iterator it = nodes_.find(key);
    indices.insert(make_pair(key,N));
    N += it->second->dim();
  }

  // create matrix and copy in values
  Matrix R = zeros(N,N);
  Vector d(N);
  string key; size_t I;
  FOREACH_PAIR(key,I,indices) {
    // find corresponding node
    const_iterator it = nodes_.find(key);
    ConditionalGaussian::shared_ptr cg = it->second;
    
    // get RHS and copy to d
    const Vector& d_ = cg->get_d();
    const size_t n = d_.size();
    for (size_t i=0;i<n;i++)
      d(I+i) = d_(i);

    // get leading R matrix and copy to R
    const Matrix& R_ = cg->get_R();
    for (size_t i=0;i<n;i++)
      for(size_t j=0;j<n;j++)
	R(I+i,I+j) = R_(i,j);

    // loop over S matrices and copy them into R
    ConditionalGaussian::const_iterator keyS = cg->parentsBegin();
    for (; keyS!=cg->parentsEnd(); keyS++) {
      Matrix S = keyS->second;                   // get S matrix      
      const size_t m = S.size1(), n = S.size2(); // find S size
      const size_t J = indices[keyS->first];     // find column index
      for (size_t i=0;i<m;i++)
	for(size_t j=0;j<n;j++)
	  R(I+i,J+j) = S(i,j);
    } // keyS

  } // keyI

  return make_pair(R,d);
}

/* ************************************************************************* */
