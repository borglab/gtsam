/**
 * @file   GaussianBayesNet.cpp
 * @brief  Chordal Bayes Net, the result of eliminating a factor graph
 * @author Frank Dellaert
 */

#include <stdarg.h>
#include <boost/foreach.hpp>
#include <boost/tuple/tuple.hpp>

#include "GaussianBayesNet.h"
#include "VectorConfig.h"

using namespace std;
using namespace gtsam;

// Explicitly instantiate so we don't have to include everywhere
#include "BayesNet-inl.h"
template class BayesNet<ConditionalGaussian>;

// trick from some reading group
#define FOREACH_PAIR( KEY, VAL, COL) BOOST_FOREACH (boost::tie(KEY,VAL),COL) 
#define REVERSE_FOREACH_PAIR( KEY, VAL, COL) BOOST_REVERSE_FOREACH (boost::tie(KEY,VAL),COL)

/* ************************************************************************* */
boost::shared_ptr<VectorConfig> GaussianBayesNet::optimize() const
{
  boost::shared_ptr<VectorConfig> result(new VectorConfig);
	
  /** solve each node in turn in topological sort order (parents first)*/
	BOOST_REVERSE_FOREACH(ConditionalGaussian::shared_ptr cg,conditionals_) {
    Vector x = cg->solve(*result); // Solve for that variable
    result->insert(cg->key(),x);   // store result in partial solution
  }
  return result;
}

/* ************************************************************************* */  
pair<Matrix,Vector> GaussianBayesNet::matrix() const {

  // add the dimensions of all variables to get matrix dimension
  // and at the same time create a mapping from keys to indices
  size_t N=0; map<string,size_t> mapping;
  BOOST_FOREACH(ConditionalGaussian::shared_ptr cg,conditionals_) {
    mapping.insert(make_pair(cg->key(),N));
    N += cg->dim();
  }

  // create matrix and copy in values
  Matrix R = zeros(N,N);
  Vector d(N);
	string key; size_t I;
  FOREACH_PAIR(key,I,mapping) {
    // find corresponding conditional
    ConditionalGaussian::shared_ptr cg = (*this)[key];
    
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
      const size_t J = mapping[keyS->first];     // find column index
      for (size_t i=0;i<m;i++)
      	for(size_t j=0;j<n;j++)
      		R(I+i,J+j) = S(i,j);
    } // keyS

  } // keyI

  return make_pair(R,d);
}

/* ************************************************************************* */
