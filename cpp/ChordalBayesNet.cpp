/**
 * @file   ChordalBayesNet.cpp
 * @brief  Chordal Bayes Net, the result of eliminating a factor graph
 * @author Frank Dellaert
 */

#include <stdarg.h>
#include <boost/foreach.hpp>
#include <boost/tuple/tuple.hpp>
#include "ChordalBayesNet.h"

using namespace std;
using namespace gtsam;

// trick from some reading group
#define FOREACH_PAIR( KEY, VAL, COL) BOOST_FOREACH (boost::tie(KEY,VAL),COL) 

/* ************************************************************************* */
void ChordalBayesNet::insert(const string& key, ConditionalGaussian::shared_ptr node)
{
  keys.push_front(key);
  nodes.insert(make_pair(key,node));
}

/* ************************************************************************* */
void ChordalBayesNet::erase(const string& key)
{
	list<string>::iterator it;
	for (it=keys.begin(); it != keys.end(); ++it){
	  if( strcmp(key.c_str(), (*it).c_str()) == 0 )
			break;
	}
	keys.erase(it);	
	nodes.erase(key);
}

/* ************************************************************************* */
// optimize, i.e. return x = inv(R)*d
/* ************************************************************************* */
boost::shared_ptr<FGConfig> ChordalBayesNet::optimize() const
{
  boost::shared_ptr<FGConfig> result(new FGConfig);
	result = optimize(result);
  return result;
}

/* ************************************************************************* */
boost::shared_ptr<FGConfig> ChordalBayesNet::optimize(const boost::shared_ptr<FGConfig> &c) const
{
  boost::shared_ptr<FGConfig> result(new FGConfig);
	result = c;
	
  /** solve each node in turn in topological sort order (parents first)*/
  BOOST_FOREACH(string key, keys) {
    const_iterator cg = nodes.find(key); // get node
    assert( cg != nodes.end() );
    Vector x = cg->second->solve(*result);                   // Solve it
    result->insert(key,x);   // store result in partial solution
  }
  return result;
}

/* ************************************************************************* */
void ChordalBayesNet::print() const {
  BOOST_FOREACH(string key, keys) {
    const_iterator it = nodes.find(key);
    it->second->print("\nNode[" + key + "]");
  }
}

/* ************************************************************************* */
bool ChordalBayesNet::equals(const ChordalBayesNet& cbn) const
{
  const_iterator it1 = nodes.begin(), it2 = cbn.nodes.begin();
 
  if(nodes.size() != cbn.nodes.size()) goto fail;
  for(; it1 != nodes.end(); it1++, it2++){
    const string& j1 = it1->first, j2 = it2->first;
    ConditionalGaussian::shared_ptr node1 = it1->second, node2 = it2->second;
    if (j1 != j2) goto fail;
    if (!node1->equals(*node2)) {
      cout << "node1[" << j1 << "] != node2[" << j2 << "]" << endl;
      goto fail;
    }
  }
  return true;

 fail:
  // they don't match, print out and fail
  print();
  cbn.print();
  return false;
}

/* ************************************************************************* */  
pair<Matrix,Vector> ChordalBayesNet::matrix() const {

  // add the dimensions of all variables to get matrix dimension
  // and at the same time create a mapping from keys to indices
  size_t N=0; map<string,size_t> indices;
  BOOST_REVERSE_FOREACH(string key, keys) {
    // find corresponding node
    const_iterator it = nodes.find(key);
    indices.insert(make_pair(key,N));
    N += it->second->dim();
  }

  // create matrix and copy in values
  Matrix R = zeros(N,N);
  Vector d(N);
  string key; size_t I;
  FOREACH_PAIR(key,I,indices) {
    // find corresponding node
    const_iterator it = nodes.find(key);
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
