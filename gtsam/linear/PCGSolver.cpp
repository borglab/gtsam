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

///*****************************************************************************/
//std::string PCGSolverParameters::pcgTranslator(const PCGKernel value) {
//  std::string s;
//  switch (value) {
//  case PCGSolverParameters::PCG:      s = "PCG";         break;
//  case PCGSolverParameters::LSPCG:    s = "LSPCG";       break;
//  case PCGSolverParameters::SPCG:     s = "SPCG";        break;
//  default:                      s = "UNDEFINED";  break;
//  }
//  return s;
//}
//
///*****************************************************************************/
//PCGSolverParameters::PCGKernel PCGSolverParameters::pcgTranslator(const std::string &src) {
//  std::string s = src;  boost::algorithm::to_upper(s);
//  if (s == "PCG") return PCGSolverParameters::PCG;
//  if (s == "LSPCG") return PCGSolverParameters::LSPCG;
//  if (s == "SPCG") return PCGSolverParameters::SPCG;
//
//  /* default is LSPCG */
//  return PCGSolverParameters::LSPCG;
//}
//
///*****************************************************************************/
//std::string PCGSolverParameters::blasTranslator(const BLASKernel value) {
//  std::string s;
//  switch (value) {
//  case PCGSolverParameters::GTSAM:      s = "GTSAM" ;      break;
//  case PCGSolverParameters::SBM:        s = "SBM" ;        break;
//  case PCGSolverParameters::SBM_MT:     s = "SBM-MT" ;     break;
//  default:                        s = "UNDEFINED" ;  break;
//  }
//  return s;
//}
//
///*****************************************************************************/
//PCGSolverParameters::BLASKernel PCGSolverParameters::blasTranslator(const std::string &src) {
//  std::string s = src;  boost::algorithm::to_upper(s);
//  if (s == "GTSAM")  return PCGSolverParameters::GTSAM;
//  if (s == "SBM")    return PCGSolverParameters::SBM;
//  if (s == "SBM-MT") return PCGSolverParameters::SBM_MT;
//
//  /* default is SBM */
//  return PCGSolverParameters::SBM;
//}
//
///*****************************************************************************/
//std::string PCGSolverParameters::rbTranslator(const RegisterBlockKernel k) {
//  std::string s;
//    switch (k) {
//    case PCGSolverParameters::RB_NONE:    s = "RB_NONE" ;    break;
//    case PCGSolverParameters::SSE_RB22:   s = "SSE_RB22" ;   break;
//    case PCGSolverParameters::SSE_RB44:   s = "SSE_RB44" ;   break;
//    case PCGSolverParameters::AVX_RB44:   s = "AVX_RB44" ;   break;
//    default:                        s = "UNDEFINED" ;  break;
//    }
//    return s;
//}
//
///*****************************************************************************/
//PCGSolverParameters::RegisterBlockKernel PCGSolverParameters::rbTranslator(const std::string &src) {
//  std::string s = src;  boost::algorithm::to_upper(s);
//  if (s == "")  return PCGSolverParameters::RB_NONE;
//  if (s == "SSE_RB22") return PCGSolverParameters::SSE_RB22;
//  if (s == "SSE_RB44") return PCGSolverParameters::SSE_RB44;
//  if (s == "AVX_RB44") return PCGSolverParameters::AVX_RB44;
//
//  /* default is SBM */
//  return PCGSolverParameters::RB_NONE;
//}
//
///*****************************************************************************/
//PCGSolver::PCGSolver(const PCGSolverParameters &p) : Base(), parameters_(p), built_(false) {
//  preconditioner_ = createPreconditioner(p.preconditioner_);
//}
//
///*****************************************************************************/
//void PCGSolver::replaceFactors(
//    const Values &linearization_point,
//    const GaussianFactorGraph &gfg,
//    const double lambda)
//{
//  const JacobianFactorGraph jfg(gfg);
//
//  /* prepare the column structure */
//  if ( keyInfo_.size() == 0 ) {
//    keyInfo_ = KeyInfo(jfg, *orderingNatural(jfg));
//  }
//
//  /* implemented by subclass */
//  safe_tic_();
//  convertKernel(jfg, lambda);
//  safe_toc_("convert-kernel");
//
//  /* update the preconditioner as well */
//  preconditioner_->replaceFactors(jfg, lambda);
//  built_ = false;
//}
//
///*****************************************************************************/
//void PCGSolver::buildPreconditioner() {
//  if ( built_ == false ) {
//    safe_tic_();
//    preconditioner_->buildPreconditioner();
//    built_ = true;
//    safe_toc_("factorize");
//  }
//}
//
///*****************************************************************************/
//VectorValues PCGSolver::optimize() {
//
//  buildPreconditioner();
//
//  VectorValues zero;
//  BOOST_FOREACH ( const KeyInfo::value_type &item, keyInfo_ ) {
//    zero.insert(item.first, Vector::Zero(item.second.dim()));
//  }
//
//  return optimize(zero);
//}
//
///*****************************************************************************/
//VectorValues PCGSolver::optimize(const VectorValues &initial) {
//  safe_tic_();
//  VectorValues result = optimize_(initial);
//  safe_toc_("pcg");
//  return result ;
//}
//
///*****************************************************************************/
////void PCGSolver_FG::convertKernel(const JacobianFactorGraph &jfg, const double lambda) {
////  hfg_ = buildOuterProductJacobianFactorGraph(jfg, VariableIndex(jfg), *x0_, lambda);
////}
////
////VectorValues PCGSolver_FG::optimize_(const VectorValues &initial) {
////  System system(hfg_, preconditioner_);
////  return preconditionedConjugateGradient<System, VectorValues>(system, initial, parameters_);
////}
//
///*****************************************************************************/
//void PCGSolver_SBM::convertKernel(const JacobianFactorGraph &jfg, const double lambda) {
//  linear_ = buildSparseLinearSystem(jfg, keyInfo_, true /* AtA */, lambda, false /* column major */,
//                                    parameters_.blas_kernel_, parameters_.rb_kernel_);
//}
//
///*****************************************************************************/
//VectorValues PCGSolver_SBM::optimize_(const VectorValues &initial) {
//  System system(linear_, preconditioner_);
//  Vector solution = preconditionedConjugateGradient<System, Vector>(
//    system, initial.vector(keyInfo_.ordering()), parameters_);
//  return buildVectorValues(solution, keyInfo_);
//}
//
///*****************************************************************************/
//ydjian::SparseLinearSystem::shared_ptr
//buildSparseLinearSystem(const JacobianFactorGraph &jfg, const KeyInfo &keyInfo,
//  const bool AtA, const double lambda, const bool colMajor,
//  const PCGSolverParameters::BLASKernel blas, const PCGSolverParameters::RegisterBlockKernel rbSrc) {
//
//  std::map<PCGSolverParameters::RegisterBlockKernel, ydjian::SBM::Type> rbMap;
//  rbMap[PCGSolverParameters::RB_NONE] = ydjian::SBM::NAIVE;
//  rbMap[PCGSolverParameters::SSE_RB22] = ydjian::SBM::SSE_RB22;
//  rbMap[PCGSolverParameters::SSE_RB44] = ydjian::SBM::SSE_RB44;
//  rbMap[PCGSolverParameters::AVX_RB44] = ydjian::SBM::AVX_RB44;
//
//  ydjian::SBM::Type rb;
//  SC_ASSERT( rbMap.find(rbSrc) != rbMap.end(), "buildSparseLinearSystem: rbSrc is not supported..");
//  rb = rbMap[rbSrc];
//
//  ydjian::SparseLinearSystem::shared_ptr linear;
//
//  switch (blas) {
//
//  case PCGSolverParameters::SBM:
//    linear = gsp2::buildSparseLinearSystem(jfg, keyInfo, AtA,  lambda, colMajor, false /* multithread */, rb);
//    break;
//
//  case PCGSolverParameters::SBM_MT:
//    linear = gsp2::buildSparseLinearSystem(jfg, keyInfo, AtA, lambda, colMajor, true /* multithread */, rb);
//    break;
//
//  default:
//    throw std::invalid_argument("createSparseLinearSystem: unsupported blas kernel");
//    break;
//  }
//
//  return linear;
//}
//
///*****************************************************************************/
//PCGSolver::shared_ptr createPCGSolver(const PCGSolverParameters &parameters){
//
//  PCGSolver::shared_ptr solver;
//  switch ( parameters.pcg_kernel_ ) {
//  case PCGSolverParameters::PCG:
//    switch ( parameters.blas_kernel_ ) {
//
////    case PCGSolverParameters::GTSAM:
////      solver.reset(new PCGSolver_FG(parameters));
////      break;
//
//    case PCGSolverParameters::SBM:
//    case PCGSolverParameters::SBM_MT:
//      solver.reset(new PCGSolver_SBM(parameters));
//      break;
//
//    default:
//      throw std::invalid_argument("createPCGSolver: undefined blas_kernel for pcgsolver");
//      break;
//    }
//    break;
//
//  case PCGSolverParameters::LSPCG:
//    switch ( parameters.blas_kernel_ ) {
//    case PCGSolverParameters::GTSAM:
//      solver.reset(new LSPCGSolver_FG(parameters));
//      break;
//
//    case PCGSolverParameters::SBM:
//    case PCGSolverParameters::SBM_MT:
//      solver.reset(new LSPCGSolver_SBM(parameters));
//      break;
//
//    default:
//      throw std::invalid_argument("createPCGSolver: undefined blas_kernel for lspcg solver");
//      break;
//    }
//    break;
//
//  default:
//    throw std::invalid_argument("createPCGSolver: undefined pcg_kernel");
//    break;
//  }
//  return solver;
//}
}
