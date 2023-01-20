/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    HessianFactor.cpp
 * @author  Richard Roberts
 * @date    Dec 8, 2010
 */

#include <gtsam/linear/HessianFactor.h>

#include <gtsam/linear/GaussianConditional.h>
#include <gtsam/linear/GaussianFactor.h>
#include <gtsam/linear/GaussianFactorGraph.h>
#include <gtsam/linear/JacobianFactor.h>
#include <gtsam/linear/linearExceptions.h>
#include <gtsam/base/cholesky.h>
#include <gtsam/base/debug.h>
#include <gtsam/base/FastMap.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/base/ThreadsafeException.h>
#include <gtsam/base/timing.h>

#include <sstream>
#include <limits>
#include "gtsam/base/Vector.h"

using namespace std;

namespace gtsam {

// Typedefs used in constructors below.
using Dims = std::vector<Eigen::Index>;

/* ************************************************************************* */
void HessianFactor::Allocate(const Scatter& scatter) {
  gttic(HessianFactor_Allocate);

  // Allocate with dimensions for each variable plus 1 at the end for the information vector
  const size_t n = scatter.size();
  keys_.resize(n);
  FastVector<DenseIndex> dims(n + 1);
  DenseIndex slot = 0;
  for(const SlotEntry& slotentry: scatter) {
    keys_[slot] = slotentry.key;
    dims[slot] = slotentry.dimension;
    ++slot;
  }
  dims.back() = 1;
  info_ = SymmetricBlockMatrix(dims);
}

/* ************************************************************************* */
HessianFactor::HessianFactor(const Scatter& scatter) {
  Allocate(scatter);
}

/* ************************************************************************* */
HessianFactor::HessianFactor() :
    info_(Dims{1}) {
  assert(info_.rows() == 1);
  constantTerm() = 0.0;
}

/* ************************************************************************* */
HessianFactor::HessianFactor(Key j, const Matrix& G, const Vector& g, double f)
    : GaussianFactor(KeyVector{j}), info_(Dims{G.cols(), 1}) {
  if (G.rows() != G.cols() || G.rows() != g.size())
    throw invalid_argument(
        "Attempting to construct HessianFactor with inconsistent matrix and/or vector dimensions");
  info_.setDiagonalBlock(0, G);
  linearTerm() = g;
  constantTerm() = f;
}

/* ************************************************************************* */
// error is 0.5*(x-mu)'*inv(Sigma)*(x-mu) = 0.5*(x'*G*x - 2*x'*G*mu + mu'*G*mu)
// where G = inv(Sigma), g = G*mu, f = mu'*G*mu = mu'*g
HessianFactor::HessianFactor(Key j, const Vector& mu, const Matrix& Sigma)
    : GaussianFactor(KeyVector{j}), info_(Dims{Sigma.cols(), 1}) {
  if (Sigma.rows() != Sigma.cols() || Sigma.rows() != mu.size())
    throw invalid_argument(
        "Attempting to construct HessianFactor with inconsistent matrix and/or vector dimensions");
  info_.setDiagonalBlock(0, Sigma.inverse()); // G
  linearTerm() = info_.diagonalBlock(0) * mu; // g
  constantTerm() = mu.dot(linearTerm().col(0)); // f
}

/* ************************************************************************* */
HessianFactor::HessianFactor(Key j1, Key j2, const Matrix& G11,
    const Matrix& G12, const Vector& g1, const Matrix& G22, const Vector& g2,
    double f) :
    GaussianFactor(KeyVector{j1,j2}), info_(
        Dims{G11.cols(),G22.cols(),1}) {
  info_.setDiagonalBlock(0, G11);
  info_.setOffDiagonalBlock(0, 1, G12);
  info_.setDiagonalBlock(1, G22);
  linearTerm() << g1, g2;
  constantTerm() = f;
}

/* ************************************************************************* */
HessianFactor::HessianFactor(Key j1, Key j2, Key j3, const Matrix& G11,
    const Matrix& G12, const Matrix& G13, const Vector& g1, const Matrix& G22,
    const Matrix& G23, const Vector& g2, const Matrix& G33, const Vector& g3,
    double f) :
    GaussianFactor(KeyVector{j1,j2,j3}), info_(
        Dims{G11.cols(),G22.cols(),G33.cols(),1}) {
  if (G11.rows() != G11.cols() || G11.rows() != G12.rows()
      || G11.rows() != G13.rows() || G11.rows() != g1.size()
      || G22.cols() != G12.cols() || G33.cols() != G13.cols()
      || G22.cols() != g2.size() || G33.cols() != g3.size())
    throw invalid_argument(
        "Inconsistent matrix and/or vector dimensions in HessianFactor constructor");
  info_.setDiagonalBlock(0, G11);
  info_.setOffDiagonalBlock(0, 1, G12);
  info_.setOffDiagonalBlock(0, 2, G13);
  info_.setDiagonalBlock(1, G22);
  info_.setOffDiagonalBlock(1, 2, G23);
  info_.setDiagonalBlock(2, G33);
  linearTerm() << g1, g2, g3;
  constantTerm() = f;
}

/* ************************************************************************* */
namespace {
DenseIndex _getSizeHF(const Vector& m) {
  return m.size();
}

std::vector<DenseIndex> _getSizeHFVec(const std::vector<Vector>& m) {
  std::vector<DenseIndex> dims;
  for (const Vector& v : m) {
    dims.push_back(v.size());
  }
  return dims;
}
}

/* ************************************************************************* */
HessianFactor::HessianFactor(const KeyVector& js,
    const std::vector<Matrix>& Gs, const std::vector<Vector>& gs, double f) :
    GaussianFactor(js), info_(_getSizeHFVec(gs), true) {
  // Get the number of variables
  size_t variable_count = js.size();

  // Verify the provided number of entries in the vectors are consistent
  if (gs.size() != variable_count
      || Gs.size() != (variable_count * (variable_count + 1)) / 2)
    throw invalid_argument(
        "Inconsistent number of entries between js, Gs, and gs in HessianFactor constructor.\nThe number of keys provided \
        in js must match the number of linear vector pieces in gs. The number of upper-diagonal blocks in Gs must be n*(n+1)/2");

  // Verify the dimensions of each provided matrix are consistent
  // Note: equations for calculating the indices derived from the "sum of an arithmetic sequence" formula
  for (size_t i = 0; i < variable_count; ++i) {
    DenseIndex block_size = gs[i].size();
    // Check rows
    for (size_t j = 0; j < variable_count - i; ++j) {
      size_t index = i * (2 * variable_count - i + 1) / 2 + j;
      if (Gs[index].rows() != block_size) {
        throw invalid_argument(
            "Inconsistent matrix and/or vector dimensions in HessianFactor constructor");
      }
    }
    // Check cols
    for (size_t j = 0; j <= i; ++j) {
      size_t index = j * (2 * variable_count - j + 1) / 2 + (i - j);
      if (Gs[index].cols() != block_size) {
        throw invalid_argument(
            "Inconsistent matrix and/or vector dimensions in HessianFactor constructor");
      }
    }
  }

  // Fill in the blocks
  size_t index = 0;
  for (size_t i = 0; i < variable_count; ++i) {
    for (size_t j = i; j < variable_count; ++j) {
      if (i == j) {
        info_.setDiagonalBlock(i, Gs[index]);
      } else {
        info_.setOffDiagonalBlock(i, j, Gs[index]);
      }
      index++;
    }
    info_.setOffDiagonalBlock(i, variable_count, gs[i]);
  }
  constantTerm() = f;
}

/* ************************************************************************* */
namespace {
void _FromJacobianHelper(const JacobianFactor& jf, SymmetricBlockMatrix& info) {
  gttic(HessianFactor_fromJacobian);
  const SharedDiagonal& jfModel = jf.get_model();
  auto A = jf.matrixObject().full();
  if (jfModel) {
    if (jf.get_model()->isConstrained())
      throw invalid_argument(
          "Cannot construct HessianFactor from JacobianFactor with constrained noise model");

    auto D = (jfModel->invsigmas().array() * jfModel->invsigmas().array()).matrix().asDiagonal();

    info.setFullMatrix(A.transpose() * D * A);
  } else {
    info.setFullMatrix(A.transpose() * A);
  }
}
}

/* ************************************************************************* */
HessianFactor::HessianFactor(const JacobianFactor& jf) :
    GaussianFactor(jf), info_(
        SymmetricBlockMatrix::LikeActiveViewOf(jf.matrixObject())) {
  _FromJacobianHelper(jf, info_);
}

/* ************************************************************************* */
HessianFactor::HessianFactor(const GaussianFactor& gf) :
    GaussianFactor(gf) {
  // Copy the matrix data depending on what type of factor we're copying from
  if (const JacobianFactor* jf = dynamic_cast<const JacobianFactor*>(&gf)) {
    info_ = SymmetricBlockMatrix::LikeActiveViewOf(jf->matrixObject());
    _FromJacobianHelper(*jf, info_);
  } else if (const HessianFactor* hf = dynamic_cast<const HessianFactor*>(&gf)) {
    info_ = hf->info_;
  } else {
    throw std::invalid_argument(
        "In HessianFactor(const GaussianFactor& gf), gf is neither a JacobianFactor nor a HessianFactor");
  }
}

/* ************************************************************************* */
HessianFactor::HessianFactor(const GaussianFactorGraph& factors,
    const Scatter& scatter) {
  gttic(HessianFactor_MergeConstructor);

  Allocate(scatter);

  // Form A' * A
  gttic(update);
  info_.setZero();
  for(const auto& factor: factors)
    if (factor)
      factor->updateHessian(keys_, &info_);
  gttoc(update);
}

/* ************************************************************************* */
void HessianFactor::print(const std::string& s,
    const KeyFormatter& formatter) const {
  cout << s << "\n";
  cout << " keys: ";
  for (const_iterator key = begin(); key != end(); ++key)
    cout << formatter(*key) << "(" << getDim(key) << ") ";
  cout << "\n";
  gtsam::print(Matrix(info_.selfadjointView()),
      "Augmented information matrix: ");
}

/* ************************************************************************* */
bool HessianFactor::equals(const GaussianFactor& lf, double tol) const {
  const HessianFactor* rhs = dynamic_cast<const HessianFactor*>(&lf);
  if (!rhs || !Factor::equals(lf, tol))
    return false;
  return equal_with_abs_tol(augmentedInformation(), rhs->augmentedInformation(),
      tol);
}

/* ************************************************************************* */
Matrix HessianFactor::augmentedInformation() const {
  return info_.selfadjointView();
}

/* ************************************************************************* */
Eigen::SelfAdjointView<SymmetricBlockMatrix::constBlock, Eigen::Upper>
HessianFactor::informationView() const {
  return info_.selfadjointView(0, size());
}

/* ************************************************************************* */
Matrix HessianFactor::information() const {
  return informationView();
}

/* ************************************************************************* */
void HessianFactor::hessianDiagonalAdd(VectorValues &d) const {
  for (DenseIndex j = 0; j < (DenseIndex)size(); ++j) {
    auto result = d.emplace(keys_[j], info_.diagonal(j));
    if(!result.second) {
      // if emplace fails, it returns an iterator to the existing element, which we add to:
      result.first->second += info_.diagonal(j);
    }
  }
}

/* ************************************************************************* */
// Raw memory access version should be called in Regular Factors only currently
void HessianFactor::hessianDiagonal(double* d) const {
  throw std::runtime_error(
      "HessianFactor::hessianDiagonal raw memory access is allowed for Regular Factors only");
}

/* ************************************************************************* */
map<Key, Matrix> HessianFactor::hessianBlockDiagonal() const {
  map<Key, Matrix> blocks;
  // Loop over all variables
  for (DenseIndex j = 0; j < (DenseIndex) size(); ++j) {
    // Get the diagonal block, and insert it
    blocks.emplace(keys_[j], info_.diagonalBlock(j));
  }
  return blocks;
}

/* ************************************************************************* */
Matrix HessianFactor::augmentedJacobian() const {
  return JacobianFactor(*this).augmentedJacobian();
}

/* ************************************************************************* */
std::pair<Matrix, Vector> HessianFactor::jacobian() const {
  return JacobianFactor(*this).jacobian();
}

/* ************************************************************************* */
double HessianFactor::error(const VectorValues& c) const {
  // error 0.5*(f - 2*x'*g + x'*G*x)
  const double f = constantTerm();
  if (empty()) {
    return 0.5 * f;
  }
  double xtg = 0, xGx = 0;
  // extract the relevant subset of the VectorValues
  // NOTE may not be as efficient
  const Vector x = c.vector(keys());
  xtg = x.dot(linearTerm().col(0));
  auto AtA = informationView();
  xGx = x.transpose() * AtA * x;
  return 0.5 * (f - 2.0 * xtg + xGx);
}

/* ************************************************************************* */
void HessianFactor::updateHessian(const KeyVector& infoKeys,
                                  SymmetricBlockMatrix* info) const {
  gttic(updateHessian_HessianFactor);
  assert(info);
  // Apply updates to the upper triangle
  DenseIndex nrVariablesInThisFactor = size(), nrBlocksInInfo = info->nBlocks() - 1;
  vector<DenseIndex> slots(nrVariablesInThisFactor + 1);
  // Loop over this factor's blocks with indices (i,j)
  // For every block (i,j), we determine the block (I,J) in info.
  for (DenseIndex j = 0; j <= nrVariablesInThisFactor; ++j) {
    const bool rhs = (j == nrVariablesInThisFactor);
    const DenseIndex J = rhs ? nrBlocksInInfo : Slot(infoKeys, keys_[j]);
    slots[j] = J;
    for (DenseIndex i = 0; i <= j; ++i) {
      const DenseIndex I = slots[i];  // because i<=j, slots[i] is valid.

      if (i == j) {
        assert(I == J);
        info->updateDiagonalBlock(I, info_.diagonalBlock(i));
      } else {
        assert(i < j);
        assert(I != J);
        info->updateOffDiagonalBlock(I, J, info_.aboveDiagonalBlock(i, j));
      }
    }
  }
}

/* ************************************************************************* */
GaussianFactor::shared_ptr HessianFactor::negate() const {
  shared_ptr result = std::make_shared<This>(*this);
  // Negate the information matrix of the result
  result->info_.negate();
  return result;
}

/* ************************************************************************* */
void HessianFactor::multiplyHessianAdd(double alpha, const VectorValues& x,
    VectorValues& yvalues) const {

  // Create a vector of temporary y values, corresponding to rows i
  vector<Vector> y;
  y.reserve(size());
  for (const_iterator it = begin(); it != end(); it++)
    y.push_back(Vector::Zero(getDim(it)));

  // Accessing the VectorValues one by one is expensive
  // So we will loop over columns to access x only once per column
  // And fill the above temporary y values, to be added into yvalues after
  for (DenseIndex j = 0; j < (DenseIndex) size(); ++j) {
    // xj is the input vector
    Vector xj = x.at(keys_[j]);
    DenseIndex i = 0;
    for (; i < j; ++i)
      y[i] += info_.aboveDiagonalBlock(i, j) * xj;

    // blocks on the diagonal are only half
    y[i] += info_.diagonalBlock(j) * xj;
    // for below diagonal, we take transpose block from upper triangular part
    for (i = j + 1; i < (DenseIndex) size(); ++i)
      y[i] += info_.aboveDiagonalBlock(j, i).transpose() * xj;
  }

  // copy to yvalues
  for (DenseIndex i = 0; i < (DenseIndex) size(); ++i) {
    bool didNotExist;
    VectorValues::iterator it;
    std::tie(it, didNotExist) = yvalues.tryInsert(keys_[i], Vector());
    if (didNotExist)
      it->second = alpha * y[i]; // init
    else
      it->second += alpha * y[i]; // add
  }
}

/* ************************************************************************* */
VectorValues HessianFactor::gradientAtZero() const {
  VectorValues g;
  size_t n = size();
  for (size_t j = 0; j < n; ++j)
    g.emplace(keys_[j], -info_.aboveDiagonalBlock(j, n));
  return g;
}

/* ************************************************************************* */
// Raw memory access version should be called in Regular Factors only currently
void HessianFactor::gradientAtZero(double* d) const {
  throw std::runtime_error(
      "HessianFactor::gradientAtZero raw memory access is allowed for Regular Factors only");
}

/* ************************************************************************* */
Vector HessianFactor::gradient(Key key, const VectorValues& x) const {
  const Factor::const_iterator it_i = find(key);
  const DenseIndex I = std::distance(begin(), it_i);

  // Sum over G_ij*xj for all xj connecting to xi
  Vector b = Vector::Zero(x.at(key).size());
  for (Factor::const_iterator it_j = begin(); it_j != end(); ++it_j) {
    const DenseIndex J = std::distance(begin(), it_j);

    // Obtain Gij from the Hessian factor
    // Hessian factor only stores an upper triangular matrix, so be careful when i>j
    const Matrix Gij = info_.block(I, J);
    // Accumulate Gij*xj to gradf
    b += Gij * x.at(*it_j);
  }
  // Subtract the linear term gi
  b += -linearTerm(it_i);
  return b;
}

/* ************************************************************************* */
std::shared_ptr<GaussianConditional> HessianFactor::eliminateCholesky(const Ordering& keys) {
  gttic(HessianFactor_eliminateCholesky);

  GaussianConditional::shared_ptr conditional;

  try {
    // Do dense elimination
    size_t nFrontals = keys.size();
    assert(nFrontals <= size());
    info_.choleskyPartial(nFrontals);

    // TODO(frank): pre-allocate GaussianConditional and write into it
    const VerticalBlockMatrix Ab = info_.split(nFrontals);
    conditional = std::make_shared<GaussianConditional>(keys_, nFrontals, Ab);

    // Erase the eliminated keys in this factor
    keys_.erase(begin(), begin() + nFrontals);
  } catch (const CholeskyFailed&) {
#ifndef NDEBUG
    cout << "Partial Cholesky on HessianFactor failed." << endl;
    keys.print("Frontal keys ");
    print("HessianFactor:");
#endif
    throw IndeterminantLinearSystemException(keys.front());
  }

  // Return result
  return conditional;
}

/* ************************************************************************* */
VectorValues HessianFactor::solve() {
  gttic(HessianFactor_solve);

  // Do Cholesky Factorization
  const size_t n = size();
  assert(size_t(info_.nBlocks()) == n + 1);
  info_.choleskyPartial(n);
  auto R = info_.triangularView(0, n);
  auto eta = linearTerm();

  // Solve
  const Vector solution = R.solve(eta);

  // Parse into VectorValues
  VectorValues delta;
  std::size_t offset = 0;
  for (DenseIndex j = 0; j < (DenseIndex)n; ++j) {
    const DenseIndex dim = info_.getDim(j);
    delta.emplace(keys_[j], solution.segment(offset, dim));
    offset += dim;
  }

  return delta;
}

/* ************************************************************************* */
std::pair<std::shared_ptr<GaussianConditional>, std::shared_ptr<HessianFactor> >
EliminateCholesky(const GaussianFactorGraph& factors, const Ordering& keys) {
  gttic(EliminateCholesky);

  // Build joint factor
  HessianFactor::shared_ptr jointFactor;
  try {
    Scatter scatter(factors, keys);
    jointFactor = std::make_shared<HessianFactor>(factors, scatter);
  } catch (std::invalid_argument&) {
    throw InvalidDenseElimination(
        "EliminateCholesky was called with a request to eliminate variables that are not\n"
        "involved in the provided factors.");
  }

  // Do dense elimination
  auto conditional = jointFactor->eliminateCholesky(keys);

  // Return result
  return make_pair(conditional, jointFactor);
}

/* ************************************************************************* */
std::pair<std::shared_ptr<GaussianConditional>,
    std::shared_ptr<GaussianFactor> > EliminatePreferCholesky(
    const GaussianFactorGraph& factors, const Ordering& keys) {
  gttic(EliminatePreferCholesky);

  // If any JacobianFactors have constrained noise models, we have to convert
  // all factors to JacobianFactors.  Otherwise, we can convert all factors
  // to HessianFactors.  This is because QR can handle constrained noise
  // models but Cholesky cannot.
  if (hasConstraints(factors))
    return EliminateQR(factors, keys);
  else
    return EliminateCholesky(factors, keys);
}

} // gtsam
