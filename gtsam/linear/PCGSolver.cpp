/*
 * PCGSolver.cpp
 *
 *  Created on: Feb 14, 2012
 *      Author: Yong-Dian Jian
 *      Author: Sungtae An
 */

#include <gtsam/linear/GaussianFactorGraph.h>
#include <gtsam/linear/PCGSolver.h>
#include <gtsam/linear/Preconditioner.h>

#include <boost/algorithm/string.hpp>
#include <iostream>
#include <stdexcept>

using namespace std;

namespace gtsam {

/*****************************************************************************/
void PCGSolverParameters::print(ostream &os) const {
  Base::print(os);
  os << "PCGSolverParameters:" <<  endl;
  preconditioner_->print(os);
}

/*****************************************************************************/
PCGSolver::PCGSolver(const PCGSolverParameters &p) {
  parameters_ = p;
  preconditioner_ = createPreconditioner(p.preconditioner_);
}

/*****************************************************************************/
VectorValues PCGSolver::optimize (
  const GaussianFactorGraph &gfg,
  const KeyInfo &keyInfo,
  const std::map<Key, Vector> &lambda,
  const VectorValues &initial)
{
  /* build preconditioner */
  preconditioner_->build(gfg, keyInfo, lambda);

  /* apply pcg */
  const Vector sol = preconditionedConjugateGradient<GaussianFactorGraphSystem, Vector>(
        GaussianFactorGraphSystem(gfg, *preconditioner_, keyInfo, lambda),
        initial.vector(keyInfo.ordering()), parameters_);

  return buildVectorValues(sol, keyInfo);
}

/*****************************************************************************/
GaussianFactorGraphSystem::GaussianFactorGraphSystem(
    const GaussianFactorGraph &gfg,
    const Preconditioner &preconditioner,
    const KeyInfo &keyInfo,
    const std::map<Key, Vector> &lambda)
  : gfg_(gfg), preconditioner_(preconditioner), keyInfo_(keyInfo), lambda_(lambda) {}

/*****************************************************************************/
void GaussianFactorGraphSystem::residual(const Vector &x, Vector &r) const {
  /* implement b-Ax, assume x and r are pre-allocated */

  /* reset r to b */
  getb(r);

  /* substract A*x */
  Vector Ax = Vector::Zero(r.rows(), 1);
  multiply(x, Ax);
  r -= Ax ;
}

/*****************************************************************************/
void GaussianFactorGraphSystem::multiply(const Vector &x, Vector& AtAx) const {
  /* implement A^T*(A*x), assume x and AtAx are pre-allocated */

  // Build a VectorValues for Vector x
  VectorValues vvX = buildVectorValues(x,keyInfo_);

  // VectorValues form of A'Ax for multiplyHessianAdd
  VectorValues vvAtAx;

  // vvAtAx += 1.0 * A'Ax for each factor
  gfg_.multiplyHessianAdd(1.0, vvX, vvAtAx);

  // Make the result as Vector form
  AtAx = vvAtAx.vector();

}

/*****************************************************************************/
void GaussianFactorGraphSystem::getb(Vector &b) const {
  /* compute rhs, assume b pre-allocated */

  // Get whitened r.h.s (A^T * b) from each factor in the form of VectorValues
  VectorValues vvb = gfg_.gradientAtZero();

  // Make the result as Vector form
  b = -vvb.vector();
}

/**********************************************************************************/
void GaussianFactorGraphSystem::leftPrecondition(const Vector &x, Vector &y) const {
  // For a preconditioner M = L*L^T
  // Calculate y = L^{-1} x
  preconditioner_.solve(x, y);
}

/**********************************************************************************/
void GaussianFactorGraphSystem::rightPrecondition(const Vector &x, Vector &y) const {
  // For a preconditioner M = L*L^T
  // Calculate y = L^{-T} x
  preconditioner_.transposeSolve(x, y);
}

/**********************************************************************************/
VectorValues buildVectorValues(const Vector &v,
                               const Ordering &ordering,
                               const map<Key, size_t>  & dimensions) {
  VectorValues result;

  DenseIndex offset = 0;
  for ( size_t i = 0 ; i < ordering.size() ; ++i ) {
    const Key &key = ordering[i];
    map<Key, size_t>::const_iterator it = dimensions.find(key);
    if ( it == dimensions.end() ) {
      throw invalid_argument("buildVectorValues: inconsistent ordering and dimensions");
    }
    const size_t dim = it->second;
    result.insert(key, v.segment(offset, dim));
    offset += dim;
  }

  return result;
}

/**********************************************************************************/
VectorValues buildVectorValues(const Vector &v, const KeyInfo &keyInfo) {
  VectorValues result;
  BOOST_FOREACH ( const KeyInfo::value_type &item, keyInfo ) {
    result.insert(item.first, v.segment(item.second.colstart(), item.second.dim()));
  }
  return result;
}

}
