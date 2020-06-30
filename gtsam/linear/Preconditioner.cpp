/*
 * Preconditioner.cpp
 *
 *  Created on: Jun 2, 2014
 *      Author: Yong-Dian Jian
 *      Author: Sungtae An
 */

#include <gtsam/inference/FactorGraph-inst.h>
#include <gtsam/linear/GaussianFactorGraph.h>
#include <gtsam/linear/PCGSolver.h>
#include <gtsam/linear/Preconditioner.h>
#include <gtsam/linear/SubgraphPreconditioner.h>
#include <gtsam/linear/NoiseModel.h>
#include <boost/shared_ptr.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/range/adaptor/map.hpp>
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
    R.triangularView<Eigen::Lower>().solveInPlace(b);

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
  // n is the number of keys
  const size_t n = keyInfo.size();
  // dims_ is a vector that contains the dimension of keys
  dims_ = keyInfo.colSpec();

  /* prepare the buffer of block diagonals */
  std::vector<Matrix> blocks; blocks.reserve(n);

  /* allocate memory for the factorization of block diagonals */
  size_t nnz = 0;
  for ( size_t i = 0 ; i < n ; ++i ) {
    const size_t dim = dims_[i];
    // blocks.push_back(Matrix::Zero(dim, dim));
    // nnz += (((dim)*(dim+1)) >> 1); // d*(d+1) / 2  ;
    nnz += dim*dim;
  }

  /* getting the block diagonals over the factors */
  std::map<Key, Matrix> hessianMap =gfg.hessianBlockDiagonal();
  for ( const Matrix hessian: hessianMap | boost::adaptors::map_values)
    blocks.push_back(hessian);

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
    /* It is same as L = chol(M,'lower') in MATLAB where M is full preconditioner */
    const Matrix L = blocks[i].llt().matrixL();

    /* store the data in the buffer */
    size_t sz = dims_[i]*dims_[i] ;
    std::copy(L.data(), L.data() + sz, ptr);

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
boost::shared_ptr<Preconditioner> createPreconditioner(
    const boost::shared_ptr<PreconditionerParameters> params) {
  using boost::dynamic_pointer_cast;
  if (dynamic_pointer_cast<DummyPreconditionerParameters>(params)) {
    return boost::make_shared<DummyPreconditioner>();
  } else if (dynamic_pointer_cast<BlockJacobiPreconditionerParameters>(
                 params)) {
    return boost::make_shared<BlockJacobiPreconditioner>();
  } else if (auto subgraph =
                 dynamic_pointer_cast<SubgraphPreconditionerParameters>(
                     params)) {
    return boost::make_shared<SubgraphPreconditioner>(*subgraph);
  }

  throw invalid_argument(
      "createPreconditioner: unexpected preconditioner parameter type");
}
}  // namespace gtsam
