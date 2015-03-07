/*
 * PCGSolver.cpp
 *
 *  Created on: Feb 14, 2012
 *      Author: ydjian
 */

#include <gtsam/linear/GaussianFactorGraph.h>
//#include <gtsam/inference/FactorGraph-inst.h>
//#include <gtsam/linear/FactorGraphUtil-inl.h>
//#include <gtsam/linear/JacobianFactorGraph.h>
//#include <gtsam/linear/LSPCGSolver.h>
#include <gtsam/linear/PCGSolver.h>
#include <gtsam/linear/Preconditioner.h>
//#include <gtsam/linear/SuiteSparseUtil.h>
//#include <gtsam/linear/ConjugateGradientMethod-inl.h>
//#include <gsp2/gtsam-interface-sbm.h>
//#include <ydjian/tool/ThreadSafeTimer.h>
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
void GaussianFactorGraphSystem::multiply(const Vector &x, Vector& Ax) const {
  /* implement Ax, assume x and Ax are pre-allocated */

  /* reset y */
  Ax.setZero();

  BOOST_FOREACH ( const GaussianFactor::shared_ptr &gf, gfg_ ) {
    if ( JacobianFactor::shared_ptr jf = boost::dynamic_pointer_cast<JacobianFactor>(gf) ) {
      /* accumulate At A x*/
      for ( JacobianFactor::const_iterator it = jf->begin() ; it != jf->end() ; ++it ) {
        const Matrix Ai = jf->getA(it);
        /* this map lookup should be replaced */
        const KeyInfoEntry &entry = keyInfo_.find(*it)->second;
        Ax.segment(entry.colstart(), entry.dim())
            += Ai.transpose() * (Ai * x.segment(entry.colstart(), entry.dim()));
      }
    }
    else if ( HessianFactor::shared_ptr hf = boost::dynamic_pointer_cast<HessianFactor>(gf) ) {
      /* accumulate H x */

      /* use buffer to avoid excessive table lookups */
      const size_t sz = hf->size();
      vector<Vector> y;
      y.reserve(sz);
      for (HessianFactor::const_iterator it = hf->begin(); it != hf->end(); it++) {
        /* initialize y to zeros */
        y.push_back(zero(hf->getDim(it)));
      }

      for (HessianFactor::const_iterator j = hf->begin(); j != hf->end(); j++ ) {
        /* retrieve the key mapping */
        const KeyInfoEntry &entry = keyInfo_.find(*j)->second;
        // xj is the input vector
        const Vector xj = x.segment(entry.colstart(), entry.dim());
        size_t idx = 0;
        for (HessianFactor::const_iterator i = hf->begin(); i != hf->end(); i++, idx++ ) {
          if ( i == j ) y[idx] += hf->info(j, j).selfadjointView() * xj;
          else y[idx] += hf->info(i, j).knownOffDiagonal() * xj;
        }
      }

      /* accumulate to r */
      for(DenseIndex i = 0; i < (DenseIndex) sz; ++i) {
        /* retrieve the key mapping */
        const KeyInfoEntry &entry = keyInfo_.find(hf->keys()[i])->second;
        Ax.segment(entry.colstart(), entry.dim()) += y[i];
      }
    }
    else {
      throw invalid_argument("GaussianFactorGraphSystem::multiply gfg contains a factor that is neither a JacobianFactor nor a HessianFactor.");
    }
  }
}

/*****************************************************************************/
void GaussianFactorGraphSystem::getb(Vector &b) const {
  /* compute rhs, assume b pre-allocated */

  /* reset */
  b.setZero();

  BOOST_FOREACH ( const GaussianFactor::shared_ptr &gf, gfg_ ) {
    if ( JacobianFactor::shared_ptr jf = boost::dynamic_pointer_cast<JacobianFactor>(gf) ) {
      const Vector rhs = jf->getb();
      /* accumulate At rhs */
      for ( JacobianFactor::const_iterator it = jf->begin() ; it != jf->end() ; ++it ) {
        /* this map lookup should be replaced */
        const KeyInfoEntry &entry = keyInfo_.find(*it)->second;
        b.segment(entry.colstart(), entry.dim()) += jf->getA(it).transpose() * rhs ;
      }
    }
    else if ( HessianFactor::shared_ptr hf = boost::dynamic_pointer_cast<HessianFactor>(gf) ) {
      /* accumulate g */
      for (HessianFactor::const_iterator it = hf->begin(); it != hf->end(); it++) {
        const KeyInfoEntry &entry = keyInfo_.find(*it)->second;
        b.segment(entry.colstart(), entry.dim()) += hf->linearTerm(it);
      }
    }
    else {
      throw invalid_argument("GaussianFactorGraphSystem::getb gfg contains a factor that is neither a JacobianFactor nor a HessianFactor.");
    }
  }
}

/**********************************************************************************/
void GaussianFactorGraphSystem::leftPrecondition(const Vector &x, Vector &y) const
{ preconditioner_.transposeSolve(x, y); }

/**********************************************************************************/
void GaussianFactorGraphSystem::rightPrecondition(const Vector &x, Vector &y) const
{ preconditioner_.solve(x, y); }

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
