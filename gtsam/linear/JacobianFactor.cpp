/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    JacobianFactor.cpp
 * @author  Richard Roberts
 * @author  Christian Potthast
 * @author  Frank Dellaert
 * @date    Dec 8, 2010
 */

#include <gtsam/linear/linearExceptions.h>
#include <gtsam/linear/GaussianConditional.h>
#include <gtsam/linear/JacobianFactor.h>
#include <gtsam/linear/Scatter.h>
#include <gtsam/linear/GaussianFactorGraph.h>
#include <gtsam/linear/VectorValues.h>
#include <gtsam/inference/VariableSlots.h>
#include <gtsam/inference/Ordering.h>
#include <gtsam/base/debug.h>
#include <gtsam/base/timing.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/base/FastMap.h>
#include <gtsam/base/cholesky.h>

#include <boost/assign/list_of.hpp>
#include <boost/format.hpp>
#include <boost/make_shared.hpp>
#include <boost/array.hpp>
#ifdef __GNUC__
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-variable"
#endif
#include <boost/bind.hpp>
#ifdef __GNUC__
#pragma GCC diagnostic pop
#endif
#include <boost/range/algorithm/copy.hpp>
#include <boost/range/adaptor/indirected.hpp>
#include <boost/range/adaptor/map.hpp>

#include <cmath>
#include <sstream>
#include <stdexcept>

using namespace std;
using namespace boost::assign;

namespace gtsam {

/* ************************************************************************* */
JacobianFactor::JacobianFactor() :
    Ab_(cref_list_of<1>(1), 0) {
  getb().setZero();
}

/* ************************************************************************* */
JacobianFactor::JacobianFactor(const GaussianFactor& gf) {
  // Copy the matrix data depending on what type of factor we're copying from
  if (const JacobianFactor* asJacobian = dynamic_cast<const JacobianFactor*>(&gf))
    *this = JacobianFactor(*asJacobian);
  else if (const HessianFactor* asHessian = dynamic_cast<const HessianFactor*>(&gf))
    *this = JacobianFactor(*asHessian);
  else
    throw std::invalid_argument(
        "In JacobianFactor(const GaussianFactor& rhs), rhs is neither a JacobianFactor nor a HessianFactor");
}

/* ************************************************************************* */
JacobianFactor::JacobianFactor(const Vector& b_in) :
    Ab_(cref_list_of<1>(1), b_in.size()) {
  getb() = b_in;
}

/* ************************************************************************* */
JacobianFactor::JacobianFactor(Key i1, const Matrix& A1, const Vector& b,
    const SharedDiagonal& model) {
  fillTerms(cref_list_of<1>(make_pair(i1, A1)), b, model);
}

/* ************************************************************************* */
JacobianFactor::JacobianFactor(const Key i1, const Matrix& A1, Key i2,
    const Matrix& A2, const Vector& b, const SharedDiagonal& model) {
  fillTerms(cref_list_of<2>(make_pair(i1, A1))(make_pair(i2, A2)), b, model);
}

/* ************************************************************************* */
JacobianFactor::JacobianFactor(const Key i1, const Matrix& A1, Key i2,
    const Matrix& A2, Key i3, const Matrix& A3, const Vector& b,
    const SharedDiagonal& model) {
  fillTerms(
      cref_list_of<3>(make_pair(i1, A1))(make_pair(i2, A2))(make_pair(i3, A3)),
      b, model);
}

/* ************************************************************************* */
JacobianFactor::JacobianFactor(const HessianFactor& factor) :
    Base(factor), Ab_(
        VerticalBlockMatrix::LikeActiveViewOf(factor.info(),
            factor.rows())) {
  // Copy Hessian into our matrix and then do in-place Cholesky
  Ab_.full() = factor.info().selfadjointView();

  // Do Cholesky to get a Jacobian
  size_t maxrank;
  bool success;
  boost::tie(maxrank, success) = choleskyCareful(Ab_.matrix());

  // Check for indefinite system
  if (!success)
    throw IndeterminantLinearSystemException(factor.keys().front());

  // Zero out lower triangle
  Ab_.matrix().topRows(maxrank).triangularView<Eigen::StrictlyLower>() =
      Matrix::Zero(maxrank, Ab_.matrix().cols());
  // FIXME: replace with triangular system
  Ab_.rowEnd() = maxrank;
  model_ = SharedDiagonal(); // should be same as Unit::Create(maxrank);
}

/* ************************************************************************* */
// Helper functions for combine constructor
namespace {
boost::tuple<FastVector<DenseIndex>, DenseIndex, DenseIndex> _countDims(
    const FastVector<JacobianFactor::shared_ptr>& factors,
    const FastVector<VariableSlots::const_iterator>& variableSlots) {
  gttic(countDims);
#ifdef GTSAM_EXTRA_CONSISTENCY_CHECKS
  FastVector<DenseIndex> varDims(variableSlots.size(), numeric_limits<DenseIndex>::max());
#else
  FastVector<DenseIndex> varDims(variableSlots.size(),
      numeric_limits<DenseIndex>::max());
#endif
  DenseIndex m = 0;
  DenseIndex n = 0;
  for (size_t jointVarpos = 0; jointVarpos < variableSlots.size();
      ++jointVarpos) {
    const VariableSlots::const_iterator& slots = variableSlots[jointVarpos];

    assert(slots->second.size() == factors.size());

    bool foundVariable = false;
    for (size_t sourceFactorI = 0; sourceFactorI < slots->second.size();
        ++sourceFactorI) {
      const size_t sourceVarpos = slots->second[sourceFactorI];
      if (sourceVarpos != VariableSlots::Empty) {
        const JacobianFactor& sourceFactor = *factors[sourceFactorI];
        if (sourceFactor.cols() > 1) {
          foundVariable = true;
          DenseIndex vardim = sourceFactor.getDim(
              sourceFactor.begin() + sourceVarpos);

#ifdef GTSAM_EXTRA_CONSISTENCY_CHECKS
          if(varDims[jointVarpos] == numeric_limits<DenseIndex>::max()) {
            varDims[jointVarpos] = vardim;
            n += vardim;
          } else {
            if(!(varDims[jointVarpos] == vardim)) {
              cout << "Factor " << sourceFactorI << " variable " << DefaultKeyFormatter(sourceFactor.keys()[sourceVarpos]) <<
              " has different dimensionality of " << vardim << " instead of " << varDims[jointVarpos] << endl;
              exit(1);
            }
          }
#else

          varDims[jointVarpos] = vardim;
          n += vardim;
          break;
#endif
        }
      }
    }

    if (!foundVariable)
      throw std::invalid_argument(
          "Unable to determine dimensionality for all variables");
  }

  for(const JacobianFactor::shared_ptr& factor: factors) {
    m += factor->rows();
  }

#ifdef GTSAM_EXTRA_CONSISTENCY_CHECKS
  for(DenseIndex d: varDims) {
    assert(d != numeric_limits<DenseIndex>::max());
  }
#endif

  return boost::make_tuple(varDims, m, n);
}

/* ************************************************************************* */
FastVector<JacobianFactor::shared_ptr> _convertOrCastToJacobians(
    const GaussianFactorGraph& factors) {
  gttic(Convert_to_Jacobians);
  FastVector<JacobianFactor::shared_ptr> jacobians;
  jacobians.reserve(factors.size());
  for(const GaussianFactor::shared_ptr& factor: factors) {
    if (factor) {
      if (JacobianFactor::shared_ptr jf = boost::dynamic_pointer_cast<
          JacobianFactor>(factor))
        jacobians.push_back(jf);
      else
        jacobians.push_back(boost::make_shared<JacobianFactor>(*factor));
    }
  }
  return jacobians;
}
}

/* ************************************************************************* */
JacobianFactor::JacobianFactor(const GaussianFactorGraph& graph,
    boost::optional<const Ordering&> ordering,
    boost::optional<const VariableSlots&> p_variableSlots) {
  gttic(JacobianFactor_combine_constructor);

  // Compute VariableSlots if one was not provided
  // Binds reference, does not copy VariableSlots
  const VariableSlots & variableSlots =
      p_variableSlots ? p_variableSlots.get() : VariableSlots(graph);

  // Cast or convert to Jacobians
  FastVector<JacobianFactor::shared_ptr> jacobians = _convertOrCastToJacobians(
      graph);

  gttic(Order_slots);
  // Order variable slots - we maintain the vector of ordered slots, as well as keep a list
  // 'unorderedSlots' of any variables discovered that are not in the ordering.  Those will then
  // be added after all of the ordered variables.
  FastVector<VariableSlots::const_iterator> orderedSlots;
  orderedSlots.reserve(variableSlots.size());
  if (ordering) {
    // If an ordering is provided, arrange the slots first that ordering
    FastList<VariableSlots::const_iterator> unorderedSlots;
    size_t nOrderingSlotsUsed = 0;
    orderedSlots.resize(ordering->size());
    FastMap<Key, size_t> inverseOrdering = ordering->invert();
    for (VariableSlots::const_iterator item = variableSlots.begin();
        item != variableSlots.end(); ++item) {
      FastMap<Key, size_t>::const_iterator orderingPosition =
          inverseOrdering.find(item->first);
      if (orderingPosition == inverseOrdering.end()) {
        unorderedSlots.push_back(item);
      } else {
        orderedSlots[orderingPosition->second] = item;
        ++nOrderingSlotsUsed;
      }
    }
    if (nOrderingSlotsUsed != ordering->size())
      throw std::invalid_argument(
          "The ordering provided to the JacobianFactor combine constructor\n"
              "contained extra variables that did not appear in the factors to combine.");
    // Add the remaining slots
    for(VariableSlots::const_iterator item: unorderedSlots) {
      orderedSlots.push_back(item);
    }
  } else {
    // If no ordering is provided, arrange the slots as they were, which will be sorted
    // numerically since VariableSlots uses a map sorting on Key.
    for (VariableSlots::const_iterator item = variableSlots.begin();
        item != variableSlots.end(); ++item)
      orderedSlots.push_back(item);
  }
  gttoc(Order_slots);

  // Count dimensions
  FastVector<DenseIndex> varDims;
  DenseIndex m, n;
  boost::tie(varDims, m, n) = _countDims(jacobians, orderedSlots);

  // Allocate matrix and copy keys in order
  gttic(allocate);
  Ab_ = VerticalBlockMatrix(varDims, m, true); // Allocate augmented matrix
  Base::keys_.resize(orderedSlots.size());
  boost::range::copy(
      // Get variable keys
      orderedSlots | boost::adaptors::indirected | boost::adaptors::map_keys,
      Base::keys_.begin());
  gttoc(allocate);

  // Loop over slots in combined factor and copy blocks from source factors
  gttic(copy_blocks);
  size_t combinedSlot = 0;
  for(VariableSlots::const_iterator varslot: orderedSlots) {
    JacobianFactor::ABlock destSlot(this->getA(this->begin() + combinedSlot));
    // Loop over source jacobians
    DenseIndex nextRow = 0;
    for (size_t factorI = 0; factorI < jacobians.size(); ++factorI) {
      // Slot in source factor
      const size_t sourceSlot = varslot->second[factorI];
      const DenseIndex sourceRows = jacobians[factorI]->rows();
      if (sourceRows > 0) {
        JacobianFactor::ABlock::RowsBlockXpr destBlock(
            destSlot.middleRows(nextRow, sourceRows));
        // Copy if exists in source factor, otherwise set zero
        if (sourceSlot != VariableSlots::Empty)
          destBlock = jacobians[factorI]->getA(
              jacobians[factorI]->begin() + sourceSlot);
        else
          destBlock.setZero();
        nextRow += sourceRows;
      }
    }
    ++combinedSlot;
  }
  gttoc(copy_blocks);

  // Copy the RHS vectors and sigmas
  gttic(copy_vectors);
  bool anyConstrained = false;
  boost::optional<Vector> sigmas;
  // Loop over source jacobians
  DenseIndex nextRow = 0;
  for (size_t factorI = 0; factorI < jacobians.size(); ++factorI) {
    const DenseIndex sourceRows = jacobians[factorI]->rows();
    if (sourceRows > 0) {
      this->getb().segment(nextRow, sourceRows) = jacobians[factorI]->getb();
      if (jacobians[factorI]->get_model()) {
        // If the factor has a noise model and we haven't yet allocated sigmas, allocate it.
        if (!sigmas)
          sigmas = Vector::Constant(m, 1.0);
        sigmas->segment(nextRow, sourceRows) =
            jacobians[factorI]->get_model()->sigmas();
        if (jacobians[factorI]->isConstrained())
          anyConstrained = true;
      }
      nextRow += sourceRows;
    }
  }
  gttoc(copy_vectors);

  if (sigmas)
    this->setModel(anyConstrained, *sigmas);
}

/* ************************************************************************* */
void JacobianFactor::print(const string& s,
    const KeyFormatter& formatter) const {
  static const Eigen::IOFormat matlab(
      Eigen::StreamPrecision, // precision
      0, // flags
      " ", // coeffSeparator
      ";\n", // rowSeparator
      "\t",  // rowPrefix
      "", // rowSuffix
      "[\n", // matPrefix
      "\n  ]" // matSuffix
      );
  if (!s.empty())
    cout << s << "\n";
  for (const_iterator key = begin(); key != end(); ++key) {
    cout << boost::format("  A[%1%] = ") % formatter(*key);
    cout << getA(key).format(matlab) << endl;
  }
  cout << formatMatrixIndented("  b = ", getb(), true) << "\n";
  if (model_)
    model_->print("  Noise model: ");
  else
    cout << "  No noise model" << endl;
}

/* ************************************************************************* */
// Check if two linear factors are equal
bool JacobianFactor::equals(const GaussianFactor& f_, double tol) const {
  static const bool verbose = false;
  if (!dynamic_cast<const JacobianFactor*>(&f_)) {
    if (verbose)
      cout << "JacobianFactor::equals: Incorrect type" << endl;
    return false;
  } else {
    const JacobianFactor& f(static_cast<const JacobianFactor&>(f_));

    // Check keys
    if (keys() != f.keys()) {
      if (verbose)
        cout << "JacobianFactor::equals: keys do not match" << endl;
      return false;
    }

    // Check noise model
    if ((model_ && !f.model_) || (!model_ && f.model_)) {
      if (verbose)
        cout << "JacobianFactor::equals: noise model mismatch" << endl;
      return false;
    }
    if (model_ && f.model_ && !model_->equals(*f.model_, tol)) {
      if (verbose)
        cout << "JacobianFactor::equals: noise modesl are not equal" << endl;
      return false;
    }

    // Check matrix sizes
    if (!(Ab_.rows() == f.Ab_.rows() && Ab_.cols() == f.Ab_.cols())) {
      if (verbose)
        cout << "JacobianFactor::equals: augmented size mismatch" << endl;
      return false;
    }

    // Check matrix contents
    constABlock Ab1(Ab_.range(0, Ab_.nBlocks()));
    constABlock Ab2(f.Ab_.range(0, f.Ab_.nBlocks()));
    for (size_t row = 0; row < (size_t) Ab1.rows(); ++row)
      if (!equal_with_abs_tol(Ab1.row(row), Ab2.row(row), tol)
          && !equal_with_abs_tol(-Ab1.row(row), Ab2.row(row), tol)) {
        if (verbose)
          cout << "JacobianFactor::equals: matrix mismatch at row " << row << endl;
        return false;
      }

    return true;
  }
}

/* ************************************************************************* */
Vector JacobianFactor::unweighted_error(const VectorValues& c) const {
  Vector e = -getb();
  for (size_t pos = 0; pos < size(); ++pos)
    e += Ab_(pos) * c[keys_[pos]];
  return e;
}

/* ************************************************************************* */
Vector JacobianFactor::error_vector(const VectorValues& c) const {
  if (model_)
    return model_->whiten(unweighted_error(c));
  else
    return unweighted_error(c);
}

/* ************************************************************************* */
double JacobianFactor::error(const VectorValues& c) const {
  Vector weighted = error_vector(c);
  return 0.5 * weighted.dot(weighted);
}

/* ************************************************************************* */
Matrix JacobianFactor::augmentedInformation() const {
  if (model_) {
    // is_model
    Matrix AbWhitened = Ab_.full();
    model_->WhitenInPlace(AbWhitened);
    return AbWhitened.transpose() * AbWhitened;
  } else {
    // not_model
    return Ab_.full().transpose() * Ab_.full();
  }
}

/* ************************************************************************* */
Matrix JacobianFactor::information() const {
  if (model_) {
    Matrix AWhitened = this->getA();
    model_->WhitenInPlace(AWhitened);
    return AWhitened.transpose() * AWhitened;
  } else {
    return this->getA().transpose() * this->getA();
  }
}

/* ************************************************************************* */
VectorValues JacobianFactor::hessianDiagonal() const {
  VectorValues d;
  for (size_t pos = 0; pos < size(); ++pos) {
    Key j = keys_[pos];
    size_t nj = Ab_(pos).cols();
    Vector dj(nj);
    for (size_t k = 0; k < nj; ++k) {
      Vector column_k = Ab_(pos).col(k);
      if (model_)
        column_k = model_->whiten(column_k);
      dj(k) = dot(column_k, column_k);
    }
    d.insert(j, dj);
  }
  return d;
}

/* ************************************************************************* */
// Raw memory access version should be called in Regular Factors only currently
void JacobianFactor::hessianDiagonal(double* d) const {
  throw std::runtime_error("JacobianFactor::hessianDiagonal raw memory access is allowed for Regular Factors only");
}

/* ************************************************************************* */
map<Key, Matrix> JacobianFactor::hessianBlockDiagonal() const {
  map<Key, Matrix> blocks;
  for (size_t pos = 0; pos < size(); ++pos) {
    Key j = keys_[pos];
    Matrix Aj = Ab_(pos);
    if (model_)
      Aj = model_->Whiten(Aj);
    blocks.insert(make_pair(j, Aj.transpose() * Aj));
  }
  return blocks;
}

/* ************************************************************************* */
void JacobianFactor::updateHessian(const FastVector<Key>& infoKeys,
                                   SymmetricBlockMatrix* info) const {
 
  gttic(updateHessian_JacobianFactor);

  if (rows() == 0) return;

  // Whiten the factor if it has a noise model
  const SharedDiagonal& model = get_model();
  if (model && !model->isUnit()) {
    if (model->isConstrained())
      throw invalid_argument(
          "JacobianFactor::updateHessian: cannot update information with "
          "constrained noise model");
    JacobianFactor whitenedFactor = whiten();
    whitenedFactor.updateHessian(infoKeys, info);
  } else {
    // Ab_ is the augmented Jacobian matrix A, and we perform I += A'*A below
    DenseIndex n = Ab_.nBlocks() - 1, N = info->nBlocks() - 1;

    // Apply updates to the upper triangle
    // Loop over blocks of A, including RHS with j==n
    vector<DenseIndex> slots(n+1);
    for (DenseIndex j = 0; j <= n; ++j) {
      Eigen::Block<const Matrix> Ab_j = Ab_(j);
      const DenseIndex J = (j == n) ? N : Slot(infoKeys, keys_[j]);
      slots[j] = J;
      // Fill off-diagonal blocks with Ai'*Aj
      for (DenseIndex i = 0; i < j; ++i) {
        const DenseIndex I = slots[i];  // because i<j, slots[i] is valid.
        info->updateOffDiagonalBlock(I, J, Ab_(i).transpose() * Ab_j);
      }
      // Fill diagonal block with Aj'*Aj
      info->diagonalBlock(J).rankUpdate(Ab_j.transpose());
    }
  
  }
}

/* ************************************************************************* */
Vector JacobianFactor::operator*(const VectorValues& x) const {
  Vector Ax = Vector::Zero(Ab_.rows());
  if (empty())
    return Ax;

  // Just iterate over all A matrices and multiply in correct config part
  for (size_t pos = 0; pos < size(); ++pos)
    Ax += Ab_(pos) * x[keys_[pos]];

  return model_ ? model_->whiten(Ax) : Ax;
}

/* ************************************************************************* */
void JacobianFactor::transposeMultiplyAdd(double alpha, const Vector& e,
    VectorValues& x) const {
  Vector E = alpha * (model_ ? model_->whiten(e) : e);
  // Just iterate over all A matrices and insert Ai^e into VectorValues
  for (size_t pos = 0; pos < size(); ++pos) {
    Key j = keys_[pos];
    // Create the value as a zero vector if it does not exist.
    pair<VectorValues::iterator, bool> xi = x.tryInsert(j, Vector());
    if (xi.second)
      xi.first->second = Vector::Zero(getDim(begin() + pos));
    xi.first->second += Ab_(pos).transpose()*E;
  }
}

/* ************************************************************************* */
void JacobianFactor::multiplyHessianAdd(double alpha, const VectorValues& x,
    VectorValues& y) const {
  Vector Ax = (*this) * x;
  transposeMultiplyAdd(alpha, Ax, y);
}

/* ************************************************************************* */
/** Raw memory access version of multiplyHessianAdd y += alpha * A'*A*x
 * Note: this is not assuming a fixed dimension for the variables,
 * but requires the vector accumulatedDims to tell the dimension of
 * each variable: e.g.: x0 has dim 3, x2 has dim 6, x3 has dim 2,
 * then accumulatedDims is [0 3 9 11 13]
 * NOTE: size of accumulatedDims is size of keys + 1!!
 * TODO Frank asks: why is this here if not regular ????
 */
void JacobianFactor::multiplyHessianAdd(double alpha, const double* x, double* y,
    const std::vector<size_t>& accumulatedDims) const {

  /// Use Eigen magic to access raw memory
  typedef Eigen::Map<Vector> VectorMap;
  typedef Eigen::Map<const Vector> ConstVectorMap;

  if (empty())
    return;
  Vector Ax = Vector::Zero(Ab_.rows());

  /// Just iterate over all A matrices and multiply in correct config part (looping over keys)
  /// E.g.: Jacobian A = [A0 A1 A2] multiplies x = [x0 x1 x2]'
  /// Hence: Ax = A0 x0 + A1 x1 + A2 x2 (hence we loop over the keys and accumulate)
  for (size_t pos = 0; pos < size(); ++pos) {
    size_t offset = accumulatedDims[keys_[pos]];
    size_t dim = accumulatedDims[keys_[pos] + 1] - offset;
    Ax += Ab_(pos) * ConstVectorMap(x + offset, dim);
  }
  /// Deal with noise properly, need to Double* whiten as we are dividing by variance
  if (model_) {
    model_->whitenInPlace(Ax);
    model_->whitenInPlace(Ax);
  }

  /// multiply with alpha
  Ax *= alpha;

  /// Again iterate over all A matrices and insert Ai^T into y
  for (size_t pos = 0; pos < size(); ++pos) {
    size_t offset = accumulatedDims[keys_[pos]];
    size_t dim = accumulatedDims[keys_[pos] + 1] - offset;
    VectorMap(y + offset, dim) += Ab_(pos).transpose() * Ax;
  }
}

/* ************************************************************************* */
VectorValues JacobianFactor::gradientAtZero() const {
  VectorValues g;
  Vector b = getb();
  // Gradient is really -A'*b / sigma^2
  // transposeMultiplyAdd will divide by sigma once, so we need one more
  Vector b_sigma = model_ ? model_->whiten(b) : b;
  this->transposeMultiplyAdd(-1.0, b_sigma, g); // g -= A'*b/sigma^2
  return g;
}

/* ************************************************************************* */
// Raw memory access version should be called in Regular Factors only currently
void JacobianFactor::gradientAtZero(double* d) const {
  throw std::runtime_error("JacobianFactor::gradientAtZero raw memory access is allowed for Regular Factors only");
}

/* ************************************************************************* */
Vector JacobianFactor::gradient(Key key, const VectorValues& x) const {
  // TODO: optimize it for JacobianFactor without converting to a HessianFactor
  HessianFactor hessian(*this);
  return hessian.gradient(key, x);
}

/* ************************************************************************* */
pair<Matrix, Vector> JacobianFactor::jacobian() const {
  pair<Matrix, Vector> result = jacobianUnweighted();
  // divide in sigma so error is indeed 0.5*|Ax-b|
  if (model_)
    model_->WhitenSystem(result.first, result.second);
  return result;
}

/* ************************************************************************* */
pair<Matrix, Vector> JacobianFactor::jacobianUnweighted() const {
  Matrix A(Ab_.range(0, size()));
  Vector b(getb());
  return make_pair(A, b);
}

/* ************************************************************************* */
Matrix JacobianFactor::augmentedJacobian() const {
  Matrix Ab = augmentedJacobianUnweighted();
  if (model_)
    model_->WhitenInPlace(Ab);
  return Ab;
}

/* ************************************************************************* */
Matrix JacobianFactor::augmentedJacobianUnweighted() const {
  return Ab_.range(0, Ab_.nBlocks());
}

/* ************************************************************************* */
JacobianFactor JacobianFactor::whiten() const {
  JacobianFactor result(*this);
  if (model_) {
    result.model_->WhitenInPlace(result.Ab_.full());
    result.model_ = SharedDiagonal();
  }
  return result;
}

/* ************************************************************************* */
GaussianFactor::shared_ptr JacobianFactor::negate() const {
  HessianFactor hessian(*this);
  return hessian.negate();
}

/* ************************************************************************* */
std::pair<GaussianConditional::shared_ptr,
    JacobianFactor::shared_ptr> JacobianFactor::eliminate(
    const Ordering& keys) {
  GaussianFactorGraph graph;
  graph.add(*this);
  return EliminateQR(graph, keys);
}

/* ************************************************************************* */
void JacobianFactor::setModel(bool anyConstrained, const Vector& sigmas) {
  if ((size_t) sigmas.size() != this->rows())
    throw InvalidNoiseModel(this->rows(), sigmas.size());
  if (anyConstrained)
    model_ = noiseModel::Constrained::MixedSigmas(sigmas);
  else
    model_ = noiseModel::Diagonal::Sigmas(sigmas);
}

/* ************************************************************************* */
std::pair<GaussianConditional::shared_ptr, JacobianFactor::shared_ptr> EliminateQR(
    const GaussianFactorGraph& factors, const Ordering& keys) {
  gttic(EliminateQR);
  // Combine and sort variable blocks in elimination order
  JacobianFactor::shared_ptr jointFactor;
  try {
    jointFactor = boost::make_shared<JacobianFactor>(factors, keys);
  } catch (std::invalid_argument&) {
    throw InvalidDenseElimination(
        "EliminateQR was called with a request to eliminate variables that are not\n"
        "involved in the provided factors.");
  }

  // Do dense elimination
  // The following QR variants eliminate to fully triangular or trapezoidal
  SharedDiagonal noiseModel;
  VerticalBlockMatrix& Ab = jointFactor->Ab_;
  if (jointFactor->model_) {
    // The noiseModel QR can, in the case of constraints, yield a "staggered" QR where
    // some rows have more leading zeros than in an upper triangular matrix.
    // In either case, QR will put zeros below the "diagonal".
    jointFactor->model_ = jointFactor->model_->QR(Ab.matrix());
  } else {
    // The inplace variant will have no valid rows anymore below m==n
    // and only entries above the diagonal are valid.
    inplace_QR(Ab.matrix());
    // We zero below the diagonal to agree with the result from noieModel QR
    Ab.matrix().triangularView<Eigen::StrictlyLower>().setZero();
    size_t m = Ab.rows(), n = Ab.cols() - 1;
    size_t maxRank = min(m, n);
    jointFactor->model_ = noiseModel::Unit::Create(maxRank);
  }

  // Split elimination result into conditional and remaining factor
  GaussianConditional::shared_ptr conditional =  //
      jointFactor->splitConditional(keys.size());

  return make_pair(conditional, jointFactor);
}

/* ************************************************************************* */
GaussianConditional::shared_ptr JacobianFactor::splitConditional(size_t nrFrontals) {
  gttic(JacobianFactor_splitConditional);

  if (!model_) {
    throw std::invalid_argument(
        "JacobianFactor::splitConditional cannot be  given a NULL noise model");
  }

  if (nrFrontals > size()) {
    throw std::invalid_argument(
        "JacobianFactor::splitConditional was requested to split off more variables than exist.");
  }

  // Convert nr of keys to number of scalar columns
  DenseIndex frontalDim = Ab_.range(0, nrFrontals).cols();

  // Check that the noise model has at least this dimension
  // If this is *not* the case, we do not have enough information on the frontal variables.
  if ((DenseIndex)model_->dim() < frontalDim)
    throw IndeterminantLinearSystemException(this->keys().front());

  // Restrict the matrix to be in the first nrFrontals variables and create the conditional
  const DenseIndex originalRowEnd = Ab_.rowEnd();
  Ab_.rowEnd() = Ab_.rowStart() + frontalDim;
  SharedDiagonal conditionalNoiseModel;
  conditionalNoiseModel =
      noiseModel::Diagonal::Sigmas(model_->sigmas().segment(Ab_.rowStart(), Ab_.rows()));
  GaussianConditional::shared_ptr conditional =
      boost::make_shared<GaussianConditional>(Base::keys_, nrFrontals, Ab_, conditionalNoiseModel);

  const DenseIndex maxRemainingRows =
      std::min(Ab_.cols(), originalRowEnd) - Ab_.rowStart() - frontalDim;
  const DenseIndex remainingRows = std::min(model_->sigmas().size() - frontalDim, maxRemainingRows);
  Ab_.rowStart() += frontalDim;
  Ab_.rowEnd() = Ab_.rowStart() + remainingRows;
  Ab_.firstBlock() += nrFrontals;

  // Take lower-right block of Ab to get the new factor
  keys_.erase(begin(), begin() + nrFrontals);
  // Set sigmas with the right model
  if (model_->isConstrained())
    model_ = noiseModel::Constrained::MixedSigmas(model_->sigmas().tail(remainingRows));
  else
    model_ = noiseModel::Diagonal::Sigmas(model_->sigmas().tail(remainingRows));
  assert(model_->dim() == (size_t)Ab_.rows());

  return conditional;
}

//============================= mhUpdateHessian() =============================
//[MH-A]:
void MHJacobianFactor::mhUpdateHessian(const FastVector<Key>& keys, HessList& hessian_list, const int& max_layer_idx) const {
  // max_layer_idx decides the recursive iter...
  const int this_layer_idx = resulting_layer_->getLayerIdx();
  const int layer_diff = max_layer_idx - this_layer_idx;

  //std::vector<int> descendant_num_arr(hypo_list_.size()); //[D]
  std::vector<int> descendant_num_arr(hypoSize());
  size_t dna_idx = 0;
  
  //for (HypoListCstIter it = hypo_list_.begin(); it != hypo_list_.end(); ++it) { //[D]
  const HypoList& hypo_list = getHypoList();
  for (HypoListCstIter it = hypo_list.begin(); it != hypo_list.end(); ++it) {
    //[MH-A]:
    descendant_num_arr[dna_idx] = (*it)->findDescendantNum(layer_diff);
    ++dna_idx;
  }

  HessListIter hit = hessian_list.begin();
  int dna_count = 0;
  for (JacobListCstIter jit = jacobian_list_.begin(); jit != jacobian_list_.end(); ++jit) { //should NOT ++hit here
    
    auto& num = descendant_num_arr[dna_count];
    for (int k = 0; k < num; ++k) {
      (*jit).updateHessian(keys, &((*hit).info())); 
      ++hit;
    }
    dna_count++;
  }

}
//============================= END mhUpdateHessian() =============================

}  // namespace gtsam
