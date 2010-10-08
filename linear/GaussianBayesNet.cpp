/**
 * @file   GaussianBayesNet.cpp
 * @brief  Chordal Bayes Net, the result of eliminating a factor graph
 * @author Frank Dellaert
 */

#include <stdarg.h>
#include <boost/foreach.hpp>
#include <boost/tuple/tuple.hpp>

#include <gtsam/base/Matrix-inl.h>
#include <gtsam/linear/GaussianBayesNet.h>
#include <gtsam/linear/VectorConfig.h>

using namespace std;
using namespace gtsam;

// Explicitly instantiate so we don't have to include everywhere
#include <gtsam/inference/BayesNet-inl.h>
template class BayesNet<GaussianConditional>;

// trick from some reading group
#define FOREACH_PAIR( KEY, VAL, COL) BOOST_FOREACH (boost::tie(KEY,VAL),COL) 
#define REVERSE_FOREACH_PAIR( KEY, VAL, COL) BOOST_REVERSE_FOREACH (boost::tie(KEY,VAL),COL)

namespace gtsam {

/* ************************************************************************* */
GaussianBayesNet scalarGaussian(varid_t key, double mu, double sigma) {
	GaussianBayesNet bn;
	GaussianConditional::shared_ptr
		conditional(new GaussianConditional(key, Vector_(1,mu)/sigma, eye(1)/sigma, ones(1)));
	bn.push_back(conditional);
	return bn;
}

/* ************************************************************************* */
GaussianBayesNet simpleGaussian(varid_t key, const Vector& mu, double sigma) {
	GaussianBayesNet bn;
	size_t n = mu.size();
	GaussianConditional::shared_ptr
		conditional(new GaussianConditional(key, mu/sigma, eye(n)/sigma, ones(n)));
	bn.push_back(conditional);
	return bn;
}

/* ************************************************************************* */
void push_front(GaussianBayesNet& bn, varid_t key, Vector d, Matrix R,
		varid_t name1, Matrix S, Vector sigmas) {
	GaussianConditional::shared_ptr cg(new GaussianConditional(key, d, R, name1, S, sigmas));
	bn.push_front(cg);
}

/* ************************************************************************* */
void push_front(GaussianBayesNet& bn, varid_t key, Vector d, Matrix R,
		varid_t name1, Matrix S, varid_t name2, Matrix T, Vector sigmas) {
	GaussianConditional::shared_ptr cg(new GaussianConditional(key, d, R, name1, S, name2, T, sigmas));
	bn.push_front(cg);
}

/* ************************************************************************* */
boost::shared_ptr<VectorConfig> allocateVectorConfig(const GaussianBayesNet& bn) {
  vector<size_t> dimensions(bn.size());
  varid_t var = 0;
  BOOST_FOREACH(const boost::shared_ptr<const GaussianConditional> conditional, bn) {
    dimensions[var++] = conditional->get_R().size1();
  }
  return boost::shared_ptr<VectorConfig>(new VectorConfig(dimensions));
}

/* ************************************************************************* */
VectorConfig optimize(const GaussianBayesNet& bn)
{
  return *optimize_(bn);
}

/* ************************************************************************* */
boost::shared_ptr<VectorConfig> optimize_(const GaussianBayesNet& bn)
{
	boost::shared_ptr<VectorConfig> result(allocateVectorConfig(bn));

  /** solve each node in turn in topological sort order (parents first)*/
	BOOST_REVERSE_FOREACH(GaussianConditional::shared_ptr cg, bn) {
    Vector x = cg->solve(*result); // Solve for that variable
    (*result)[cg->key()] = x;   // store result in partial solution
  }
  return result;
}

/* ************************************************************************* */
VectorConfig backSubstitute(const GaussianBayesNet& bn, const VectorConfig& y) {
	VectorConfig x(y);
	backSubstituteInPlace(bn,x);
	return x;
}

/* ************************************************************************* */
// (R*x)./sigmas = y by solving x=inv(R)*(y.*sigmas)
void backSubstituteInPlace(const GaussianBayesNet& bn, VectorConfig& y) {
	VectorConfig& x = y;
	/** solve each node in turn in topological sort order (parents first)*/
	BOOST_REVERSE_FOREACH(const boost::shared_ptr<const GaussianConditional> cg, bn) {
		// i^th part of R*x=y, x=inv(R)*y
		// (Rii*xi + R_i*x(i+1:))./si = yi <-> xi = inv(Rii)*(yi.*si - R_i*x(i+1:))
		varid_t i = cg->key();
		Vector zi = emul(y[i],cg->get_sigmas());
		GaussianConditional::const_iterator it;
		for (it = cg->beginParents(); it!= cg->endParents(); it++) {
			multiplyAdd(-1.0,cg->get_S(it),x[*it],zi);
		}
		x[i] = gtsam::backSubstituteUpper(cg->get_R(), zi);
	}
}

/* ************************************************************************* */
// gy=inv(L)*gx by solving L*gy=gx.
// gy=inv(R'*inv(Sigma))*gx
// gz'*R'=gx', gy = gz.*sigmas
VectorConfig backSubstituteTranspose(const GaussianBayesNet& bn,
		const VectorConfig& gx) {

	// Initialize gy from gx
	// TODO: used to insert zeros if gx did not have an entry for a variable in bn
	VectorConfig gy = gx;

	// we loop from first-eliminated to last-eliminated
	// i^th part of L*gy=gx is done block-column by block-column of L
	BOOST_FOREACH(const boost::shared_ptr<const GaussianConditional> cg, bn) {
		varid_t j = cg->key();
		gy[j] = gtsam::backSubstituteUpper(gy[j],cg->get_R());
		GaussianConditional::const_iterator it;
		for (it = cg->beginParents(); it!= cg->endParents(); it++) {
			const varid_t i = *it;
			transposeMultiplyAdd(-1.0,cg->get_S(it),gy[j],gy[i]);
		}
	}

	// Scale gy
	BOOST_FOREACH(GaussianConditional::shared_ptr cg, bn) {
		varid_t j = cg->key();
		gy[j] = emul(gy[j],cg->get_sigmas());
	}

	return gy;
}

/* ************************************************************************* */  
pair<Matrix,Vector> matrix(const GaussianBayesNet& bn)  {

  // add the dimensions of all variables to get matrix dimension
  // and at the same time create a mapping from keys to indices
  size_t N=0; map<varid_t,size_t> mapping;
  BOOST_FOREACH(GaussianConditional::shared_ptr cg,bn) {
    mapping.insert(make_pair(cg->key(),N));
    N += cg->dim();
  }

  // create matrix and copy in values
  Matrix R = zeros(N,N);
  Vector d(N);
  varid_t key; size_t I;
  FOREACH_PAIR(key,I,mapping) {
    // find corresponding conditional
    boost::shared_ptr<const GaussianConditional> cg = bn[key];
    
    // get sigmas
    Vector sigmas = cg->get_sigmas();

    // get RHS and copy to d
    GaussianConditional::const_d_type d_ = cg->get_d();
    const size_t n = d_.size();
    for (size_t i=0;i<n;i++)
      d(I+i) = d_(i)/sigmas(i);

    // get leading R matrix and copy to R
    GaussianConditional::const_r_type R_ = cg->get_R();
    for (size_t i=0;i<n;i++)
      for(size_t j=0;j<n;j++)
      	R(I+i,I+j) = R_(i,j)/sigmas(i);

    // loop over S matrices and copy them into R
    GaussianConditional::const_iterator keyS = cg->beginParents();
    for (; keyS!=cg->endParents(); keyS++) {
      Matrix S = cg->get_S(keyS);                   // get S matrix
      const size_t m = S.size1(), n = S.size2(); // find S size
      const size_t J = mapping[*keyS];     // find column index
      for (size_t i=0;i<m;i++)
      	for(size_t j=0;j<n;j++)
      		R(I+i,J+j) = S(i,j)/sigmas(i);
    } // keyS

  } // keyI

  return make_pair(R,d);
}

/* ************************************************************************* */
VectorConfig rhs(const GaussianBayesNet& bn) {
	boost::shared_ptr<VectorConfig> result(allocateVectorConfig(bn));
  BOOST_FOREACH(boost::shared_ptr<const GaussianConditional> cg,bn) {
  	varid_t key = cg->key();
  	// get sigmas
    const Vector& sigmas = cg->get_sigmas();

    // get RHS and copy to d
    GaussianConditional::const_d_type d = cg->get_d();
    (*result)[key] = ediv_(d,sigmas); // TODO ediv_? I think not
  }

  return *result;
}

/* ************************************************************************* */

} // namespace gtsam
