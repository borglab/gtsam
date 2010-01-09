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
template class BayesNet<GaussianConditional>;

// trick from some reading group
#define FOREACH_PAIR( KEY, VAL, COL) BOOST_FOREACH (boost::tie(KEY,VAL),COL) 
#define REVERSE_FOREACH_PAIR( KEY, VAL, COL) BOOST_REVERSE_FOREACH (boost::tie(KEY,VAL),COL)

namespace gtsam {

/* ************************************************************************* */
GaussianBayesNet scalarGaussian(const string& key, double mu, double sigma) {
	GaussianBayesNet bn;
	GaussianConditional::shared_ptr
		conditional(new GaussianConditional(key, Vector_(1,mu), eye(1), Vector_(1,sigma)));
	bn.push_back(conditional);
	return bn;
}

/* ************************************************************************* */
GaussianBayesNet simpleGaussian(const string& key, const Vector& mu, double sigma) {
	GaussianBayesNet bn;
	size_t n = mu.size();
	GaussianConditional::shared_ptr
		conditional(new GaussianConditional(key, mu, eye(n), repeat(n,sigma)));
	bn.push_back(conditional);
	return bn;
}

/* ************************************************************************* */
void push_front(GaussianBayesNet& bn, const string& key, Vector d, Matrix R,
		const string& name1, Matrix S, Vector sigmas) {
	GaussianConditional::shared_ptr cg(new GaussianConditional(key, d, R, name1, S, sigmas));
	bn.push_front(cg);
}

/* ************************************************************************* */
void push_front(GaussianBayesNet& bn, const string& key, Vector d, Matrix R,
		const string& name1, Matrix S, const string& name2, Matrix T, Vector sigmas) {
	GaussianConditional::shared_ptr cg(new GaussianConditional(key, d, R, name1, S, name2, T, sigmas));
	bn.push_front(cg);
}

/* ************************************************************************* */
VectorConfig optimize(const GaussianBayesNet& bn)
{
  VectorConfig result;
	
  /** solve each node in turn in topological sort order (parents first)*/
	BOOST_REVERSE_FOREACH(GaussianConditional::shared_ptr cg, bn) {
    Vector x = cg->solve(result); // Solve for that variable
    result.insert(cg->key(),x);   // store result in partial solution
  }
  return result;
}

/* ************************************************************************* */
// (R*x)./sigmas = y by solving x=inv(R)*(y.*sigmas)
VectorConfig backSubstitute(const GaussianBayesNet& bn, const VectorConfig& y) {
	VectorConfig x;
	/** solve each node in turn in topological sort order (parents first)*/
	BOOST_REVERSE_FOREACH(GaussianConditional::shared_ptr cg, bn) {
		// i^th part of R*x=y, x=inv(R)*y
		// (Rii*xi + R_i*x(i+1:))./si = yi <-> xi = inv(Rii)*(yi.*si - R_i*x(i+1:))
		const string& i = cg->key();
		Vector zi = emul(y[i],cg->get_sigmas());
		GaussianConditional::const_iterator it;
		for (it = cg->parentsBegin(); it!= cg->parentsEnd(); it++) {
			const string& j = it->first;
			const Matrix& Rij = it->second;
			zi -= Rij * x[j];
		}
		Vector xi = gtsam::backSubstituteUpper(cg->get_R(), zi);
		x.insert(i,xi); // store result in partial solution
	}
	return x;
}

/* ************************************************************************* */
// gy=inv(L)*gx by solving L*gy=gx.
// gy=inv(R'*inv(Sigma))*gx
// gz'*R'=gx', gy = gz.*sigmas
VectorConfig backSubstituteTranspose(const GaussianBayesNet& bn,
		const VectorConfig& gx) {

	// Initialize gy from gx
	VectorConfig gy;
	BOOST_FOREACH(GaussianConditional::shared_ptr cg, bn) {
		const string& j = cg->key();
		Vector gyj = gx.contains(j) ? gx[j] : zero(cg->dim());
		gy.insert(j,gyj); // initialize result
	}

	// we loop from first-eliminated to last-eliminated
	// i^th part of L*gy=gx is done block-column by block-column of L
	BOOST_FOREACH(GaussianConditional::shared_ptr cg, bn) {
		const string& j = cg->key();
		Vector& gyj = gy.getReference(j);  // should never fail
		gyj = gtsam::backSubstituteUpper(gyj,cg->get_R());
		GaussianConditional::const_iterator it;
		for (it = cg->parentsBegin(); it!= cg->parentsEnd(); it++) {
			const string& i = it->first;
			const Matrix& Rij = it->second;
			Vector& gyi = gy.getReference(i);  // should never fail
			Matrix Lji = trans(Rij);  // TODO avoid transpose of matrix ?
			gyi -= Lji * gyj;
		}
	}

	// Scale gy
	BOOST_FOREACH(GaussianConditional::shared_ptr cg, bn) {
		const string& j = cg->key();
		Vector& gyj = gy.getReference(j);  // should never fail
		gyj = emul(gyj,cg->get_sigmas());
	}

	return gy;
}

/* ************************************************************************* */  
pair<Matrix,Vector> matrix(const GaussianBayesNet& bn)  {

  // add the dimensions of all variables to get matrix dimension
  // and at the same time create a mapping from keys to indices
  size_t N=0; map<string,size_t> mapping;
  BOOST_FOREACH(GaussianConditional::shared_ptr cg,bn) {
    mapping.insert(make_pair(cg->key(),N));
    N += cg->dim();
  }

  // create matrix and copy in values
  Matrix R = zeros(N,N);
  Vector d(N);
	string key; size_t I;
  FOREACH_PAIR(key,I,mapping) {
    // find corresponding conditional
    GaussianConditional::shared_ptr cg = bn[key];
    
    // get sigmas
    Vector sigmas = cg->get_sigmas();

    // get RHS and copy to d
    const Vector& d_ = cg->get_d();
    const size_t n = d_.size();
    for (size_t i=0;i<n;i++)
      d(I+i) = d_(i)/sigmas(i);

    // get leading R matrix and copy to R
    const Matrix& R_ = cg->get_R();
    for (size_t i=0;i<n;i++)
      for(size_t j=0;j<n;j++)
      	R(I+i,I+j) = R_(i,j)/sigmas(i);

    // loop over S matrices and copy them into R
    GaussianConditional::const_iterator keyS = cg->parentsBegin();
    for (; keyS!=cg->parentsEnd(); keyS++) {
      Matrix S = keyS->second;                   // get S matrix      
      const size_t m = S.size1(), n = S.size2(); // find S size
      const size_t J = mapping[keyS->first];     // find column index
      for (size_t i=0;i<m;i++)
      	for(size_t j=0;j<n;j++)
      		R(I+i,J+j) = S(i,j)/sigmas(i);
    } // keyS

  } // keyI

  return make_pair(R,d);
}

/* ************************************************************************* */

} // namespace gtsam
