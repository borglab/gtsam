/*
 * Preconditioner.cpp
 *
 *  Created on: Feb 2, 2012
 *      Author: ydjian
 */


//#include <gtsam/linear/CombinatorialPreconditioner.h>
#include <gtsam/inference/FactorGraph-inst.h>
//#include <gtsam/linear/FactorGraphUtil-inl.h>
#include <gtsam/linear/GaussianFactorGraph.h>
#include <gtsam/linear/PCGSolver.h>
#include <gtsam/linear/Preconditioner.h>
//#include <gtsam/linear/JacobianFactorGraph.h>
//#include <gtsam/linear/JacobiPreconditioner.h>
//#include <gtsam/linear/MIQRPreconditioner.h>
#include <gtsam/linear/NoiseModel.h>
//#include <gtsam/linear/Subgraph.h>
//#include <gtsam/linear/SubgraphBuilder-inl.h>
//#include <gtsam/linear/SuiteSparseSolver.h>
//#include <gtsam/linear/SuiteSparseUtil.h>
//#include <ydjian/tool/ThreadSafeTimer.h>
//#include <ydjian/tool/Timer.h>
#include <boost/shared_ptr.hpp>
#include <boost/algorithm/string.hpp>
#include <iostream>
#include <vector>

using namespace std;

namespace gtsam {


/*****************************************************************************/
void PreconditionerParameters::print() const {
  print(cout);
}

/***************************************************************************************/
void PreconditionerParameters::print(ostream &os) const {
  os << "PreconditionerParameters" << endl
     << "kernel:        " << kernelTranslator(kernel_) << endl
     << "verbosity:     " << verbosityTranslator(verbosity_) << endl;
}

/*****************************************************************************/
 ostream& operator<<(ostream &os, const PreconditionerParameters &p) {
  p.print(os);
  return os;
}

/***************************************************************************************/
PreconditionerParameters::Kernel PreconditionerParameters::kernelTranslator(const std::string &src) {
  std::string s = src;  boost::algorithm::to_upper(s);
  if (s == "GTSAM") return PreconditionerParameters::GTSAM;
  else if (s == "CHOLMOD") return PreconditionerParameters::CHOLMOD;
  /* default is cholmod */
  else return PreconditionerParameters::CHOLMOD;
}

/***************************************************************************************/
PreconditionerParameters::Verbosity PreconditionerParameters::verbosityTranslator(const std::string &src)  {
  std::string s = src;  boost::algorithm::to_upper(s);
  if (s == "SILENT") return PreconditionerParameters::SILENT;
  else if (s == "COMPLEXITY") return PreconditionerParameters::COMPLEXITY;
  else if (s == "ERROR") return PreconditionerParameters::ERROR;
  /* default is default */
  else return PreconditionerParameters::SILENT;
}

/***************************************************************************************/
std::string PreconditionerParameters::kernelTranslator(PreconditionerParameters::Kernel k)  {
  if ( k == GTSAM ) return "GTSAM";
  if ( k == CHOLMOD ) return "CHOLMOD";
  else return "UNKNOWN";
}

/***************************************************************************************/
std::string PreconditionerParameters::verbosityTranslator(PreconditionerParameters::Verbosity verbosity)  {
  if (verbosity == SILENT)          return "SILENT";
  else if (verbosity == COMPLEXITY) return "COMPLEXITY";
  else if (verbosity == ERROR)      return "ERROR";
  else return "UNKNOWN";
}

/***************************************************************************************/
BlockJacobiPreconditioner::BlockJacobiPreconditioner()
  : Base(), buffer_(0), bufferSize_(0), nnz_(0) {}

/***************************************************************************************/
BlockJacobiPreconditioner::~BlockJacobiPreconditioner() { clean(); }

/***************************************************************************************/
void BlockJacobiPreconditioner::solve(const Vector& y, Vector &x) const {

  const size_t n = dims_.size();
  double *ptr = buffer_, *dst = x.data();

  std::copy(y.data(), y.data() + y.rows(), x.data());

  for ( size_t i = 0 ; i < n ; ++i ) {
    const size_t d = dims_[i];
    const size_t sz = d*d;

    const Eigen::Map<const Eigen::MatrixXd> R(ptr, d, d);
    Eigen::Map<Eigen::VectorXd> b(dst, d, 1);
    R.triangularView<Eigen::Upper>().solveInPlace(b);

    dst += d;
    ptr += sz;
  }
}

/***************************************************************************************/
void BlockJacobiPreconditioner::transposeSolve(const Vector& y, Vector& x) const {
  const size_t n = dims_.size();
  double *ptr = buffer_, *dst = x.data();

  std::copy(y.data(), y.data() + y.rows(), x.data());

  for ( size_t i = 0 ; i < n ; ++i ) {
    const size_t d = dims_[i];
    const size_t sz = d*d;

    const Eigen::Map<const Eigen::MatrixXd> R(ptr, d, d);
    Eigen::Map<Eigen::VectorXd> b(dst, d, 1);
    R.transpose().triangularView<Eigen::Upper>().solveInPlace(b);

    dst += d;
    ptr += sz;
  }
}

/***************************************************************************************/
void BlockJacobiPreconditioner::build(
  const GaussianFactorGraph &gfg, const KeyInfo &keyInfo, const std::map<Key,Vector> &lambda)
{
  const size_t n = keyInfo.size();
  dims_ = keyInfo.colSpec();

  /* prepare the buffer of block diagonals */
  std::vector<Matrix> blocks; blocks.reserve(n);

  /* allocate memory for the factorization of block diagonals */
  size_t nnz = 0;
  for ( size_t i = 0 ; i < n ; ++i ) {
    const size_t dim = dims_[i];
    blocks.push_back(Matrix::Zero(dim, dim));
    // nnz += (((dim)*(dim+1)) >> 1); // d*(d+1) / 2  ;
    nnz += dim*dim;
  }

  /* compute the block diagonal by scanning over the factors */
  BOOST_FOREACH ( const GaussianFactor::shared_ptr &gf, gfg ) {
    if ( JacobianFactor::shared_ptr jf = boost::dynamic_pointer_cast<JacobianFactor>(gf) ) {
      for ( JacobianFactor::const_iterator it = jf->begin() ; it != jf->end() ; ++it ) {
        const KeyInfoEntry &entry =  keyInfo.find(*it)->second;
        const Matrix &Ai = jf->getA(it);
        blocks[entry.index()] += (Ai.transpose() * Ai);
      }
    }
    else if ( HessianFactor::shared_ptr hf = boost::dynamic_pointer_cast<HessianFactor>(gf) ) {
      for ( HessianFactor::const_iterator it = hf->begin() ; it != hf->end() ; ++it ) {
        const KeyInfoEntry &entry =  keyInfo.find(*it)->second;
        const Matrix &Hii = hf->info(it, it).selfadjointView();
        blocks[entry.index()] += Hii;
      }
    }
    else {
      throw invalid_argument("BlockJacobiPreconditioner::build gfg contains a factor that is neither a JacobianFactor nor a HessianFactor.");
    }
  }

  /* if necessary, allocating the memory for cacheing the factorization results */
  if ( nnz > bufferSize_ ) {
    clean();
    buffer_ = new double [nnz];
    bufferSize_ = nnz;
  }
  nnz_ = nnz;

  /* factorizing the blocks respectively */
  double *ptr = buffer_;
  for ( size_t i = 0 ; i < n ; ++i ) {
    /* use eigen to decompose Di */
    const Matrix R = blocks[i].llt().matrixL().transpose();

    /* store the data in the buffer */
    size_t sz = dims_[i]*dims_[i] ;
    std::copy(R.data(), R.data() + sz, ptr);

    /* advance the pointer */
    ptr += sz;
  }
}

/*****************************************************************************/
void BlockJacobiPreconditioner::clean() {
  if ( buffer_ ) {
    delete [] buffer_;
    buffer_ = 0;
    bufferSize_ = 0;
    nnz_ = 0;
  }
}

/***************************************************************************************/
boost::shared_ptr<Preconditioner> createPreconditioner(const boost::shared_ptr<PreconditionerParameters> parameters) {

  if ( DummyPreconditionerParameters::shared_ptr dummy = boost::dynamic_pointer_cast<DummyPreconditionerParameters>(parameters) ) {
    return boost::make_shared<DummyPreconditioner>();
  }
  else if ( BlockJacobiPreconditionerParameters::shared_ptr blockJacobi = boost::dynamic_pointer_cast<BlockJacobiPreconditionerParameters>(parameters) ) {
    return boost::make_shared<BlockJacobiPreconditioner>();
  }


//  BlockJacobiPreconditioner::Parameters::shared_ptr jacobi = boost::dynamic_pointer_cast<BlockJacobiPreconditioner::Parameters>(parameters);
//  if ( jacobi ) {
//    BlockJacobiPreconditioner::shared_ptr p(new BlockJacobiPreconditioner(jacobi));
//    return p;
//  }
//
//  MIQRPreconditioner::Parameters::shared_ptr miqr = boost::dynamic_pointer_cast<MIQRPreconditioner::Parameters>(parameters);
//  if ( miqr ) {
//    MIQRPreconditioner::shared_ptr p(new MIQRPreconditioner(miqr));
//    return p;
//  }
//
//  CombinatorialPreconditioner::Parameters::shared_ptr combinatorial
//    = boost::dynamic_pointer_cast<CombinatorialPreconditioner::Parameters>(parameters);
//  if ( combinatorial ) {
//    return createCombinatorialPreconditioner(combinatorial);
//  }

  throw invalid_argument("createPreconditioner: unexpected preconditioner parameter type");
}
//
///***************************************************************************************/
//JacobianFactorGraph::shared_ptr
//buildFactorSubgraph(const JacobianFactorGraph &jfg, const KeyInfo &keyInfo,
//  const Subgraph &subgraph, const int hessian, const double lambda, const bool clone, const bool includeUnary){
//
//  const Subgraph::Edges &edges = subgraph.edges();
//  const size_t m = jfg.size(), n = keyInfo.size();
//  const bool augment = (lambda!=0.0) ? true : false ;
//
//  /* produce an edge weight table */
//  std::vector<double> weights(m, 0.0);
//  BOOST_FOREACH ( const Subgraph::Edge &edge, edges ) {
//    weights[edge.index()] = edge.weight();
//  }
//
//  /* collect the number of dangling unary factors */
//  /* upper bound of the factors */
//  size_t extraUnary = 0;
//  if ( includeUnary ) {
//     for ( Index i = 0 ; i < m ; ++i ) {
//       if ( weights[i] == 0.0 && jfg[i]->size() == 1 ) {
//         weights[i] = 1.0;
//         ++extraUnary;
//       }
//     }
//  }
//
//  const size_t e = edges.size() + extraUnary;
//  const size_t sz = e + (augment ? n : 0) + (hessian ? 2*(m-e) : 0);
//
//  JacobianFactorGraph::shared_ptr target(new JacobianFactorGraph());
//  target->reserve(sz);
//
//  /* auxiliary variable */
//  size_t r = jfg[0]->rows();
//  SharedDiagonal sigma(noiseModel::Unit::Create(r));
//
//  for ( Index i = 0 ; i < m ; ++i ) {
//
//    const double w = weights[i];
//
//    if ( w != 0.0 ) { /* subgraph edge */
//      if ( !clone && w == 1.0 ) {
//        target->push_back(jfg[i]);
//      }
//      else {
//        JacobianFactor::shared_ptr jf (new JacobianFactor(*jfg[i]));
//        scaleJacobianFactor(*jf, w);
//        target->push_back(jf);
//      }
//    }
//    else { /* non-subgraph edge */
//      if ( hessian ) {
//        const JacobianFactor &f = *jfg[i];
//        /* simple case: unary factor */
//        if ( f.size() == 1 ) {
//          if (!clone) target->push_back(jfg[i]);
//          else {
//            JacobianFactor::shared_ptr jf(new JacobianFactor(*jfg[i]));
//            target->push_back(jf);
//          }
//        }
//        else { /* general factor */
//          const size_t rj = f.rows();
//          if ( rj != r ) {
//            r = rj; sigma = noiseModel::Unit::Create(r);
//          }
//          for ( JacobianFactor::const_iterator j = f.begin() ; j != f.end() ; ++j ) {
//            JacobianFactor::shared_ptr jf(new JacobianFactor(*j, f.getA(j), Vector::Zero(r,1), sigma));
//            target->push_back(jf);
//          } /* for */
//        } /* factor arity */
//      } /* hessian */
//    } /* if w != 0.0 */
//  } /* for */
//
//  if ( !augment ) return target ;
//
//  return appendPriorFactors(*target, keyInfo, lambda, false /* clone */);
//}


}


