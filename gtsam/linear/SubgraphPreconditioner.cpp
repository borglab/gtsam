/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file SubgraphPreconditioner.cpp
 * @date Dec 31, 2009
 * @author: Frank Dellaert
 */

#include <gtsam/linear/SubgraphPreconditioner.h>
#include <gtsam/linear/GaussianFactorGraph.h>
#include <gtsam/linear/GaussianBayesNet.h>
#include <boost/foreach.hpp>

using namespace std;

namespace gtsam {

  /* ************************************************************************* */
  static GaussianFactorGraph::shared_ptr convertToJacobianFactors(const GaussianFactorGraph &gfg) {
    GaussianFactorGraph::shared_ptr result(new GaussianFactorGraph());
    BOOST_FOREACH(const GaussianFactor::shared_ptr &gf, gfg) {
      JacobianFactor::shared_ptr jf = boost::dynamic_pointer_cast<JacobianFactor>(gf);
      if( !jf ) {
        jf = boost::make_shared<JacobianFactor>(*gf); // Convert any non-Jacobian factors to Jacobians (e.g. Hessian -> Jacobian with Cholesky)
      }
      result->push_back(jf);
    }
    return result;
  }

  /* ************************************************************************* */
  SubgraphPreconditioner::SubgraphPreconditioner(const sharedFG& Ab2,
      const sharedBayesNet& Rc1, const sharedValues& xbar) :
    Ab2_(convertToJacobianFactors(*Ab2)), Rc1_(Rc1), xbar_(xbar), b2bar_(new Errors(-Ab2_->gaussianErrors(*xbar))) {
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
  VectorValues SubgraphPreconditioner::gradient(const VectorValues& y) const {
    VectorValues x = Rc1()->backSubstitute(y); /* inv(R1)*y */
    Errors e = (*Ab2()*x - *b2bar());               /* (A2*inv(R1)*y-b2bar) */
    VectorValues v = VectorValues::Zero(x);
    Ab2()->transposeMultiplyAdd(1.0, e, v);           /* A2'*(A2*inv(R1)*y-b2bar) */
    return y + Rc1()->backSubstituteTranspose(v);
  }

  /* ************************************************************************* */
  // Apply operator A, A*y = [I;A2*inv(R1)]*y = [y; A2*inv(R1)*y]
  Errors SubgraphPreconditioner::operator*(const VectorValues& y) const {

    Errors e(y);
    VectorValues x = Rc1()->backSubstitute(y);   /* x=inv(R1)*y */
    Errors e2 = *Ab2() * x;                              /* A2*x */
    e.splice(e.end(), e2);
    return e;
  }

  /* ************************************************************************* */
  // In-place version that overwrites e
  void SubgraphPreconditioner::multiplyInPlace(const VectorValues& y, Errors& e) const {

    Errors::iterator ei = e.begin();
    for ( Key i = 0 ; i < y.size() ; ++i, ++ei ) {
      *ei = y[i];
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
    for ( Key i = 0 ; i < y.size() ; ++i, ++it )
      y[i] = *it ;
    transposeMultiplyAdd2(1.0,it,e.end(),y);
    return y;
  }

  /* ************************************************************************* */
  // y += alpha*A'*e
  void SubgraphPreconditioner::transposeMultiplyAdd
    (double alpha, const Errors& e, VectorValues& y) const {

    Errors::const_iterator it = e.begin();
    for ( Key i = 0 ; i < y.size() ; ++i, ++it ) {
      const Vector& ei = *it;
      axpy(alpha, ei, y[i]);
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
    axpy(alpha, Rc1_->backSubstituteTranspose(x), y); // y += alpha*inv(R1')*x
  }

  /* ************************************************************************* */
  void SubgraphPreconditioner::print(const std::string& s) const {
    cout << s << endl;
    Ab2_->print();
  }
} // nsamespace gtsam
