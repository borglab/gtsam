/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010-2019, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file SubgraphPreconditioner.cpp
 * @date Dec 31, 2009
 * @author Frank Dellaert, Yong-Dian Jian
 */

#include <gtsam/linear/SubgraphPreconditioner.h>

#include <gtsam/linear/SubgraphBuilder.h>
#include <gtsam/linear/GaussianFactorGraph.h>
#include <gtsam/linear/GaussianBayesNet.h>
#include <gtsam/linear/JacobianFactor.h>
#include <gtsam/base/types.h>
#include <gtsam/base/Vector.h>

#include <boost/algorithm/string.hpp>
#include <boost/range/adaptor/reversed.hpp>

#include <stdexcept>

using std::cout;
using std::endl;
using std::vector;
using std::ostream;

namespace gtsam {

/* ************************************************************************* */
static Vector getSubvector(const Vector &src, const KeyInfo &keyInfo,
                           const KeyVector &keys) {
  /* a cache of starting index and dim */
  vector<std::pair<size_t, size_t> > cache;
  cache.reserve(3);

  /* figure out dimension by traversing the keys */
  size_t dim = 0;
  for (const Key &key : keys) {
    const KeyInfoEntry &entry = keyInfo.find(key)->second;
    cache.emplace_back(entry.start, entry.dim);
    dim += entry.dim;
  }

  /* use the cache to fill the result */
  Vector result(dim);
  size_t index = 0;
  for (const auto &p : cache) {
    result.segment(index, p.second) = src.segment(p.first, p.second);
    index += p.second;
  }

  return result;
}

/*****************************************************************************/
static void setSubvector(const Vector &src, const KeyInfo &keyInfo,
                         const KeyVector &keys, Vector &dst) {
  size_t index = 0;
  for (const Key &key : keys) {
    const KeyInfoEntry &entry = keyInfo.find(key)->second;
    const size_t keyDim = entry.dim;
    dst.segment(entry.start, keyDim) = src.segment(index, keyDim);
    index += keyDim;
  }
}

/* ************************************************************************* */
// Convert any non-Jacobian factors to Jacobians (e.g. Hessian -> Jacobian with
// Cholesky)
static GaussianFactorGraph::shared_ptr convertToJacobianFactors(
    const GaussianFactorGraph &gfg) {
  auto result = boost::make_shared<GaussianFactorGraph>();
  for (const auto &factor : gfg) 
    if (factor) {
      auto jf = boost::dynamic_pointer_cast<JacobianFactor>(factor);
      if (!jf) {
        jf = boost::make_shared<JacobianFactor>(*factor);
      }
      result->push_back(jf);
    }
  return result;
}

/* ************************************************************************* */
SubgraphPreconditioner::SubgraphPreconditioner(const SubgraphPreconditionerParameters &p) :
         parameters_(p) {}

/* ************************************************************************* */
SubgraphPreconditioner::SubgraphPreconditioner(const sharedFG& Ab2,
    const sharedBayesNet& Rc1, const sharedValues& xbar, const SubgraphPreconditionerParameters &p) :
        Ab2_(convertToJacobianFactors(*Ab2)), Rc1_(Rc1), xbar_(xbar),
        b2bar_(new Errors(-Ab2_->gaussianErrors(*xbar))), parameters_(p) {
}

/* ************************************************************************* */
// x = xbar + inv(R1)*y
VectorValues SubgraphPreconditioner::x(const VectorValues& y) const {
  return *xbar_ + Rc1_->backSubstitute(y);
}

/* ************************************************************************* */
double SubgraphPreconditioner::error(const VectorValues& y) const {
  Errors e(y);
  VectorValues x = this->x(y);
  Errors e2 = Ab2()->gaussianErrors(x);
  return 0.5 * (dot(e, e) + dot(e2,e2));
}

/* ************************************************************************* */
// gradient is y + inv(R1')*A2'*(A2*inv(R1)*y-b2bar),
VectorValues SubgraphPreconditioner::gradient(const VectorValues &y) const {
  VectorValues x = Rc1()->backSubstitute(y); /* inv(R1)*y */
  Errors e = (*Ab2() * x - *b2bar());        /* (A2*inv(R1)*y-b2bar) */
  VectorValues v = VectorValues::Zero(x);
  Ab2()->transposeMultiplyAdd(1.0, e, v);    /* A2'*(A2*inv(R1)*y-b2bar) */
  return y + Rc1()->backSubstituteTranspose(v);
}

/* ************************************************************************* */
// Apply operator A, A*y = [I;A2*inv(R1)]*y = [y; A2*inv(R1)*y]
Errors SubgraphPreconditioner::operator*(const VectorValues& y) const {
  Errors e(y);
  VectorValues x = Rc1()->backSubstitute(y);   /* x=inv(R1)*y */
  Errors e2 = *Ab2() * x;                      /* A2*x */
  e.splice(e.end(), e2);
  return e;
}

/* ************************************************************************* */
// In-place version that overwrites e
void SubgraphPreconditioner::multiplyInPlace(const VectorValues& y, Errors& e) const {

  Errors::iterator ei = e.begin();
  for(const auto& key_value: y) {
    *ei = key_value.second;
    ++ei;
  }

  // Add A2 contribution
  VectorValues x = Rc1()->backSubstitute(y);      // x=inv(R1)*y
  Ab2()->multiplyInPlace(x, ei);                  // use iterator version
}

/* ************************************************************************* */
// Apply operator A', A'*e = [I inv(R1')*A2']*e = e1 + inv(R1')*A2'*e2
VectorValues SubgraphPreconditioner::operator^(const Errors& e) const {

  Errors::const_iterator it = e.begin();
  VectorValues y = zero();
  for(auto& key_value: y) {
    key_value.second = *it;
    ++it;
  }
  transposeMultiplyAdd2(1.0, it, e.end(), y);
  return y;
}

/* ************************************************************************* */
// y += alpha*A'*e
void SubgraphPreconditioner::transposeMultiplyAdd
(double alpha, const Errors& e, VectorValues& y) const {

  Errors::const_iterator it = e.begin();
  for(auto& key_value: y) {
    const Vector& ei = *it;
    key_value.second += alpha * ei;
    ++it;
  }
  transposeMultiplyAdd2(alpha, it, e.end(), y);
}

/* ************************************************************************* */
// y += alpha*inv(R1')*A2'*e2
void SubgraphPreconditioner::transposeMultiplyAdd2 (double alpha,
    Errors::const_iterator it, Errors::const_iterator end, VectorValues& y) const {

  // create e2 with what's left of e
  // TODO can we avoid creating e2 by passing iterator to transposeMultiplyAdd ?
  Errors e2;
  while (it != end) e2.push_back(*(it++));

  VectorValues x = VectorValues::Zero(y); // x = 0
  Ab2_->transposeMultiplyAdd(1.0,e2,x);   // x += A2'*e2
  y += alpha * Rc1_->backSubstituteTranspose(x); // y += alpha*inv(R1')*x
}

/* ************************************************************************* */
void SubgraphPreconditioner::print(const std::string& s) const {
  cout << s << endl;
  Ab2_->print();
}

/*****************************************************************************/
void SubgraphPreconditioner::solve(const Vector &y, Vector &x) const {
  assert(x.size() == y.size());

  /* back substitute */
  for (const auto &cg : boost::adaptors::reverse(*Rc1_)) {
    /* collect a subvector of x that consists of the parents of cg (S) */
    const KeyVector parentKeys(cg->beginParents(), cg->endParents());
    const KeyVector frontalKeys(cg->beginFrontals(), cg->endFrontals());
    const Vector xParent = getSubvector(x, keyInfo_, parentKeys);
    const Vector rhsFrontal = getSubvector(y, keyInfo_, frontalKeys);

    /* compute the solution for the current pivot */
    const Vector solFrontal = cg->R().triangularView<Eigen::Upper>().solve(
        rhsFrontal - cg->S() * xParent);

    /* assign subvector of sol to the frontal variables */
    setSubvector(solFrontal, keyInfo_, frontalKeys, x);
  }
}

/*****************************************************************************/
void SubgraphPreconditioner::transposeSolve(const Vector &y, Vector &x) const {
  /* copy first */
  assert(x.size() == y.size());
  std::copy(y.data(), y.data() + y.rows(), x.data());

  /* in place back substitute */
  for (const auto &cg : *Rc1_) {
    const KeyVector frontalKeys(cg->beginFrontals(), cg->endFrontals());
    const Vector rhsFrontal = getSubvector(x, keyInfo_, frontalKeys);
    const Vector solFrontal =
        cg->R().transpose().triangularView<Eigen::Lower>().solve(
            rhsFrontal);

    // Check for indeterminant solution
    if (solFrontal.hasNaN())
      throw IndeterminantLinearSystemException(cg->keys().front());

    /* assign subvector of sol to the frontal variables */
    setSubvector(solFrontal, keyInfo_, frontalKeys, x);

    /* substract from parent variables */
    for (auto it = cg->beginParents(); it != cg->endParents(); it++) {
      const KeyInfoEntry &entry = keyInfo_.find(*it)->second;
      Eigen::Map<Vector> rhsParent(x.data() + entry.start, entry.dim, 1);
      rhsParent -= Matrix(cg->getA(it)).transpose() * solFrontal;
    }
  }
}

/*****************************************************************************/
void SubgraphPreconditioner::build(const GaussianFactorGraph &gfg, const KeyInfo &keyInfo, const std::map<Key,Vector> &lambda)
{
  /* identify the subgraph structure */
  const SubgraphBuilder builder(parameters_.builderParams);
  auto subgraph = builder(gfg);

  keyInfo_ = keyInfo;

  /* build factor subgraph */
  GaussianFactorGraph::shared_ptr gfg_subgraph = buildFactorSubgraph(gfg, subgraph, true);

  /* factorize and cache BayesNet */
  Rc1_ = gfg_subgraph->eliminateSequential();
}

/*****************************************************************************/

} // nsamespace gtsam
