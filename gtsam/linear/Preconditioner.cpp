/**
 * @file Preconditioner.cpp
 *
 * Created on: Jun 2, 2014
 * @author Yong-Dian Jian
 * @author Sungtae An
 * @author Fan Jiang
 */

#include <gtsam/inference/FactorGraph-inst.h>
#include <gtsam/linear/GaussianFactorGraph.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/linear/PCGSolver.h>
#include <gtsam/linear/Preconditioner.h>
#include <gtsam/linear/SubgraphPreconditioner.h>

#include <Eigen/Cholesky>
#include <iostream>
#include <memory>
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
  std::string s = src;
  // convert string s to upper case
  std::transform(s.begin(), s.end(), s.begin(), ::toupper);
  if (s == "GTSAM") return PreconditionerParameters::GTSAM;
  else if (s == "CHOLMOD") return PreconditionerParameters::CHOLMOD;
  /* default is cholmod */
  else return PreconditionerParameters::CHOLMOD;
}

/***************************************************************************************/
PreconditionerParameters::Verbosity PreconditionerParameters::verbosityTranslator(const std::string &src)  {
  std::string s = src;
  // convert string to upper case
  std::transform(s.begin(), s.end(), s.begin(), ::toupper);
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
void BlockJacobiPreconditioner::solveInPlaceRange(Vector& x, size_t begin,
                                                  size_t end,
                                                  bool transpose) const {
  if (begin == end) return;
  const double* ptr = buffer_ + bufferOffsets_[begin];
  double* dst = x.data() + scalarOffsets_[begin];
  for (size_t i = begin; i < end; ++i) {
    const size_t d = dims_[i];
    const size_t size = d * d;

    const Eigen::Map<const Eigen::MatrixXd> R(ptr, d, d);
    Eigen::Map<Eigen::VectorXd> b(dst, d, 1);
    if (transpose) {
      R.transpose().triangularView<Eigen::Upper>().solveInPlace(b);
    } else {
      R.triangularView<Eigen::Lower>().solveInPlace(b);
    }

    dst += d;
    ptr += size;
  }
}

/***************************************************************************************/
void BlockJacobiPreconditioner::solve(const Vector& y, Vector &x) const {
  x = y;
  solveInPlaceRange(x, 0, dims_.size(), false);
}

/***************************************************************************************/
void BlockJacobiPreconditioner::transposeSolve(const Vector& y, Vector& x) const {
  x = y;
  solveInPlaceRange(x, 0, dims_.size(), true);
}

/***************************************************************************************/
void BlockJacobiPreconditioner::build(
  const GaussianFactorGraph &gfg, const KeyInfo &keyInfo, const std::map<Key,Vector> &lambda)
{
  /* getting the block diagonals over the factors */
  const std::map<Key, Matrix> hessianMap = gfg.hessianBlockDiagonal();
  std::vector<Matrix> blocks(keyInfo.size());
  for (const auto& [key, entry] : keyInfo) {
    blocks[entry.index] = hessianMap.at(key);
  }

  build(blocks, keyInfo);
}

/*****************************************************************************/
void BlockJacobiPreconditioner::build(const std::vector<Matrix>& blocks,
                                      const KeyInfo& keyInfo) {
  const size_t n = keyInfo.size();
  if (blocks.size() != n) {
    throw std::invalid_argument(
        "BlockJacobiPreconditioner::build: block count mismatch");
  }

  dims_ = keyInfo.colSpec();
  scalarOffsets_.resize(n + 1);
  bufferOffsets_.resize(n + 1);

  // Validate every block while constructing packed-buffer offsets.
  size_t nnz = 0;
  size_t scalarOffset = 0;
  for (size_t i = 0; i < n; ++i) {
    scalarOffsets_[i] = scalarOffset;
    bufferOffsets_[i] = nnz;
    const size_t dim = dims_[i];
    if (blocks[i].rows() != static_cast<DenseIndex>(dim) ||
        blocks[i].cols() != static_cast<DenseIndex>(dim)) {
      throw std::invalid_argument(
          "BlockJacobiPreconditioner::build: block dimension mismatch");
    }
    nnz += dim * dim;
    scalarOffset += dim;
  }
  scalarOffsets_[n] = scalarOffset;
  bufferOffsets_[n] = nnz;

  // Grow the reusable buffer when the packed Cholesky factors do not fit.
  if (nnz > bufferSize_) {
    clean();
    buffer_ = new double[nnz];
    bufferSize_ = nnz;
  }
  nnz_ = nnz;

  // Factorize each diagonal block into the contiguous solve buffer.
  double* ptr = buffer_;
  for (size_t i = 0; i < n; ++i) {
    // Equivalent to L = chol(M, 'lower') for the full preconditioner.
    const Matrix L = blocks[i].llt().matrixL();

    // Store and advance to the next packed block.
    size_t sz = dims_[i] * dims_[i];
    std::copy(L.data(), L.data() + sz, ptr);
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
std::shared_ptr<Preconditioner> createPreconditioner(
    const std::shared_ptr<PreconditionerParameters> params) {
  using std::dynamic_pointer_cast;
  if (dynamic_pointer_cast<DummyPreconditionerParameters>(params)) {
    return std::make_shared<DummyPreconditioner>();
  } else if (dynamic_pointer_cast<BlockJacobiPreconditionerParameters>(
                 params)) {
    return std::make_shared<BlockJacobiPreconditioner>();
  } else if (auto subgraph =
                 dynamic_pointer_cast<SubgraphPreconditionerParameters>(
                     params)) {
    return std::make_shared<SubgraphPreconditioner>(*subgraph);
  }

  throw invalid_argument(
      "createPreconditioner: unexpected preconditioner parameter type");
}
}  // namespace gtsam
