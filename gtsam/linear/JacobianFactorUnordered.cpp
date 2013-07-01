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
#include <gtsam/linear/GaussianConditionalUnordered.h>
#include <gtsam/linear/JacobianFactorUnordered.h>
#include <gtsam/linear/HessianFactorUnordered.h>
#include <gtsam/linear/GaussianFactorGraphUnordered.h>
#include <gtsam/linear/VectorValuesUnordered.h>
#include <gtsam/inference/VariableSlots.h>
#include <gtsam/base/debug.h>
#include <gtsam/base/timing.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/base/FastMap.h>
#include <gtsam/base/cholesky.h>

#include <boost/assign/list_of.hpp>
#include <boost/foreach.hpp>
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

#include <cmath>
#include <sstream>
#include <stdexcept>

using namespace std;
using namespace boost::assign;

namespace gtsam {

  /* ************************************************************************* */
  JacobianFactorUnordered::JacobianFactorUnordered(const GaussianFactorUnordered& gf) {
    // Copy the matrix data depending on what type of factor we're copying from
    if(const JacobianFactorUnordered* rhs = dynamic_cast<const JacobianFactorUnordered*>(&gf))
      *this = JacobianFactorUnordered(*rhs);
    //else if(const HessianFactor* rhs = dynamic_cast<const HessianFactor*>(&gf))
    //  *this = JacobianFactorUnordered(*rhs);
    else
      throw std::invalid_argument("In JacobianFactor(const GaussianFactor& rhs), rhs is neither a JacobianFactor nor a HessianFactor");
    assertInvariants();
  }

  /* ************************************************************************* */
  JacobianFactorUnordered::JacobianFactorUnordered(const Vector& b_in) :
    Ab_(cref_list_of<1>(1), b_in.size())
  {
    getb() = b_in;
  }

  /* ************************************************************************* */
  JacobianFactorUnordered::JacobianFactorUnordered(Key i1, const Matrix& A1,
    const Vector& b, const SharedDiagonal& model)
  {
    fillTerms(cref_list_of<1>(make_pair(i1,A1)), b, model);
  }

  /* ************************************************************************* */
  JacobianFactorUnordered::JacobianFactorUnordered(Key i1, const Matrix& A1, Key i2, const Matrix& A2,
    const Vector& b, const SharedDiagonal& model)
  {
    fillTerms(cref_list_of<2>
      (make_pair(i1,A1))
      (make_pair(i2,A2)), b, model);
  }

  /* ************************************************************************* */
  JacobianFactorUnordered::JacobianFactorUnordered(Key i1, const Matrix& A1, Key i2, const Matrix& A2,
      Key i3, const Matrix& A3, const Vector& b, const SharedDiagonal& model)
  {
    fillTerms(cref_list_of<3>
      (make_pair(i1,A1))
      (make_pair(i2,A2))
      (make_pair(i3,A3)), b, model);
  }

  /* ************************************************************************* */
  //JacobianFactorUnordered::JacobianFactorUnordered(const HessianFactorUnordered& factor) {
  //  keys_ = factor.keys_;
  //  Ab_.assignNoalias(factor.info_);

  //  // Do Cholesky to get a Jacobian
  //  size_t maxrank;
  //  bool success;
  //  boost::tie(maxrank, success) = choleskyCareful(matrix_);

  //  // Check for indefinite system
  //  if(!success)
  //    throw IndeterminantLinearSystemException(factor.keys().front());

  //  // Zero out lower triangle
  //  matrix_.topRows(maxrank).triangularView<Eigen::StrictlyLower>() =
  //      Matrix::Zero(maxrank, matrix_.cols());
  //  // FIXME: replace with triangular system
  //  Ab_.rowEnd() = maxrank;
  //  model_ = noiseModel::Unit::Create(maxrank);

  //  assertInvariants();
  //}

  /* ************************************************************************* */
  // Helper functions for combine constructor
  namespace {
    boost::tuple<vector<size_t>, size_t, size_t> _countDims(
      const std::vector<JacobianFactorUnordered::shared_ptr>& factors, const VariableSlots& variableSlots)
    {
      gttic(countDims);
#ifdef GTSAM_EXTRA_CONSISTENCY_CHECKS
      vector<size_t> varDims(variableSlots.size(), numeric_limits<size_t>::max());
#else
      vector<size_t> varDims(variableSlots.size());
#endif
      size_t m = 0;
      size_t n = 0;
      {
        size_t jointVarpos = 0;
        BOOST_FOREACH(const VariableSlots::value_type& slots, variableSlots) {

          assert(slots.second.size() == factors.size());

          size_t sourceFactorI = 0;
          BOOST_FOREACH(const size_t sourceVarpos, slots.second) {
            if(sourceVarpos < numeric_limits<size_t>::max()) {
              const JacobianFactorUnordered& sourceFactor = *factors[sourceFactorI];
              size_t vardim = sourceFactor.getDim(sourceFactor.begin() + sourceVarpos);
#ifdef GTSAM_EXTRA_CONSISTENCY_CHECKS
              if(varDims[jointVarpos] == numeric_limits<size_t>::max()) {
                varDims[jointVarpos] = vardim;
                n += vardim;
              } else
                assert(varDims[jointVarpos] == vardim);
#else
              varDims[jointVarpos] = vardim;
              n += vardim;
              break;
#endif
            }
            ++ sourceFactorI;
          }
          ++ jointVarpos;
        }
        BOOST_FOREACH(const JacobianFactorUnordered::shared_ptr& factor, factors) {
          m += factor->rows();
        }
      }
      return boost::make_tuple(varDims, m, n);
    }

    /* ************************************************************************* */
    std::vector<JacobianFactorUnordered::shared_ptr>
      _convertOrCastToJacobians(const GaussianFactorGraphUnordered& factors)
    {
      gttic(Convert_to_Jacobians);
      std::vector<JacobianFactorUnordered::shared_ptr> jacobians;
      jacobians.reserve(factors.size());
      BOOST_FOREACH(const GaussianFactorUnordered::shared_ptr& factor, factors) {
        if(factor) {
          if(JacobianFactorUnordered::shared_ptr jf = boost::dynamic_pointer_cast<JacobianFactorUnordered>(factor))
            jacobians.push_back(jf);
          else
            jacobians.push_back(boost::make_shared<JacobianFactorUnordered>(*factor));
        }
      }
      return jacobians;
    }
  }

  /* ************************************************************************* */
  JacobianFactorUnordered::JacobianFactorUnordered(
    const GaussianFactorGraphUnordered& factors, boost::optional<const VariableSlots&> variableSlots)
  {
    gttic(JacobianFactorUnordered_combine_constructor);

    // Compute VariableSlots if one was not provided
    gttic(Compute_VariableSlots);
    boost::optional<VariableSlots> computedVariableSlots;
    if(!variableSlots) {
      computedVariableSlots = VariableSlots(factors);
      variableSlots = computedVariableSlots; // Binds reference, does not copy VariableSlots
    }
    gttoc(Compute_VariableSlots);

    // Cast or convert to Jacobians
    std::vector<JacobianFactorUnordered::shared_ptr> jacobians = _convertOrCastToJacobians(factors);

    // Count dimensions
    vector<size_t> varDims;
    size_t m, n;
    boost::tie(varDims, m, n) = _countDims(jacobians, *variableSlots);

    gttic(allocate);
    Ab_ = VerticalBlockMatrix(boost::join(varDims, cref_list_of<1>(1)), m); // Allocate augmented matrix
    Base::keys_.resize(variableSlots->size());
    boost::range::copy(*variableSlots | boost::adaptors::map_keys, Base::keys_.begin()); // Get variable keys from VariableSlots
    gttoc(allocate);

    gttic(copy_blocks);
    // Loop over slots in combined factor
    size_t combinedSlot = 0;
    BOOST_FOREACH(const VariableSlots::value_type& varslot, variableSlots) {
      JacobianFactorUnordered::ABlock destSlot(this->getA(this->begin()+combinedSlot));
      // Loop over source jacobians
      size_t nextRow = 0;
      for(size_t factorI = 0; factorI < jacobians.size(); ++factorI) {
        // Slot in source factor
        const size_t sourceSlot = varslot.second[factorI];
        const size_t sourceRows = jacobians[factorI]->rows();
        JacobianFactorUnordered::ABlock::RowsBlockXpr destBlock(destSlot.middleRows(nextRow, sourceRows));
        // Copy if exists in source factor, otherwise set zero
        if(sourceSlot != numeric_limits<size_t>::max())
          destBlock = jacobians[factorI]->getA(jacobians[factorI]->begin()+sourceSlot);
        else
          destBlock.setZero();
        nextRow += sourceRows;
      }
      ++combinedSlot;
    }
    gttoc(copy_blocks);

    gttic(copy_vectors);
    bool anyConstrained = false;
    boost::optional<Vector> sigmas;
    // Loop over source jacobians
    size_t nextRow = 0;
    for(size_t factorI = 0; factorI < jacobians.size(); ++factorI) {
      const size_t sourceRows = jacobians[factorI]->rows();
      this->getb().segment(nextRow, sourceRows) = jacobians[factorI]->getb();
      if(jacobians[factorI]->get_model()) {
        // If the factor has a noise model and we haven't yet allocated sigmas, allocate it.
        if(!sigmas)
          sigmas = Vector::Constant(m, 1.0);
        sigmas->segment(nextRow, sourceRows) = jacobians[factorI]->get_model()->sigmas();
        if (jacobians[factorI]->isConstrained())
          anyConstrained = true;
      }
      nextRow += sourceRows;
    }
    gttoc(copy_vectors);

    if(sigmas)
      this->setModel(anyConstrained, *sigmas);
  }

  /* ************************************************************************* */
  void JacobianFactorUnordered::print(const string& s, const KeyFormatter& formatter) const {
    cout << s << "\n";
    if (empty()) {
      cout << " empty, keys: ";
      BOOST_FOREACH(const Key& key, keys()) { cout << formatter(key) << " "; }
      cout << endl;
    } else {
      for(const_iterator key=begin(); key!=end(); ++key)
        cout << boost::format("A[%1%]=\n")%formatter(*key) << getA(key) << endl;
      cout << "b=" << getb() << endl;
      model_->print("model");
    }
  }

  /* ************************************************************************* */
  // Check if two linear factors are equal
  bool JacobianFactorUnordered::equals(const GaussianFactorUnordered& f_, double tol) const {
    if(!dynamic_cast<const JacobianFactorUnordered*>(&f_))
      return false;
    else {
      const JacobianFactorUnordered& f(static_cast<const JacobianFactorUnordered&>(f_));
      if (empty()) return (f.empty());
      if(keys()!=f.keys() /*|| !model_->equals(lf->model_, tol)*/)
        return false;

      if (!(Ab_.rows() == f.Ab_.rows() && Ab_.cols() == f.Ab_.cols()))
        return false;

      constABlock Ab1(Ab_.range(0, Ab_.nBlocks()));
      constABlock Ab2(f.Ab_.range(0, f.Ab_.nBlocks()));
      for(size_t row=0; row< (size_t) Ab1.rows(); ++row)
        if(!equal_with_abs_tol(Ab1.row(row), Ab2.row(row), tol) &&
            !equal_with_abs_tol(-Ab1.row(row), Ab2.row(row), tol))
          return false;

      return true;
    }
  }

  /* ************************************************************************* */
  Vector JacobianFactorUnordered::unweighted_error(const VectorValuesUnordered& c) const {
    Vector e = -getb();
    if (empty()) return e;
    for(size_t pos=0; pos<size(); ++pos)
      e += Ab_(pos) * c[keys_[pos]];
    return e;
  }

  /* ************************************************************************* */
  Vector JacobianFactorUnordered::error_vector(const VectorValuesUnordered& c) const {
    if (empty()) return model_->whiten(-getb());
    return model_->whiten(unweighted_error(c));
  }

  /* ************************************************************************* */
  double JacobianFactorUnordered::error(const VectorValuesUnordered& c) const {
    if (empty()) return 0;
    Vector weighted = error_vector(c);
    return 0.5 * weighted.dot(weighted);
  }

  /* ************************************************************************* */
  Matrix JacobianFactorUnordered::augmentedInformation() const {
    Matrix AbWhitened = Ab_.full();
    model_->WhitenInPlace(AbWhitened);
    return AbWhitened.transpose() * AbWhitened;
  }

  /* ************************************************************************* */
  Matrix JacobianFactorUnordered::information() const {
    Matrix AWhitened = this->getA();
    model_->WhitenInPlace(AWhitened);
    return AWhitened.transpose() * AWhitened;
  }

  /* ************************************************************************* */
  Vector JacobianFactorUnordered::operator*(const VectorValuesUnordered& x) const {
    Vector Ax = zero(Ab_.rows());
    if (empty()) return Ax;

    // Just iterate over all A matrices and multiply in correct config part
    for(size_t pos=0; pos<size(); ++pos)
      Ax += Ab_(pos) * x[keys_[pos]];

    return model_->whiten(Ax);
  }

  /* ************************************************************************* */
  void JacobianFactorUnordered::transposeMultiplyAdd(double alpha, const Vector& e,
      VectorValuesUnordered& x) const {
    Vector E = alpha * model_->whiten(e);
    // Just iterate over all A matrices and insert Ai^e into VectorValues
    for(size_t pos=0; pos<size(); ++pos)
      gtsam::transposeMultiplyAdd(1.0, Ab_(pos), E, x[keys_[pos]]);
  }

  /* ************************************************************************* */
  pair<Matrix,Vector> JacobianFactorUnordered::matrix(bool weight) const {
    Matrix A(Ab_.range(0, size()));
    Vector b(getb());
    // divide in sigma so error is indeed 0.5*|Ax-b|
    if (weight) model_->WhitenSystem(A,b);
    return make_pair(A, b);
  }

  /* ************************************************************************* */
  Matrix JacobianFactorUnordered::matrix_augmented(bool weight) const {
    if (weight) { Matrix Ab(Ab_.range(0,Ab_.nBlocks())); model_->WhitenInPlace(Ab); return Ab; }
    else return Ab_.range(0, Ab_.nBlocks());
  }

  /* ************************************************************************* */
  std::vector<boost::tuple<size_t, size_t, double> >
  JacobianFactorUnordered::sparse(const std::vector<size_t>& columnIndices) const {

    std::vector<boost::tuple<size_t, size_t, double> > entries;

    // iterate over all variables in the factor
    for(const_iterator var=begin(); var<end(); ++var) {
      Matrix whitenedA(model_->Whiten(getA(var)));
      // find first column index for this key
      size_t column_start = columnIndices[*var];
      for (size_t i = 0; i < (size_t) whitenedA.rows(); i++)
        for (size_t j = 0; j < (size_t) whitenedA.cols(); j++) {
          double s = whitenedA(i,j);
          if (std::abs(s) > 1e-12) entries.push_back(
              boost::make_tuple(i, column_start + j, s));
        }
    }

    Vector whitenedb(model_->whiten(getb()));
    size_t bcolumn = columnIndices.back();
    for (size_t i = 0; i < (size_t) whitenedb.size(); i++)
      entries.push_back(boost::make_tuple(i, bcolumn, whitenedb(i)));

    // return the result
    return entries;
  }

  /* ************************************************************************* */
  JacobianFactorUnordered JacobianFactorUnordered::whiten() const {
    JacobianFactorUnordered result(*this);
    result.model_->WhitenInPlace(result.Ab_.matrix());
    result.model_ = noiseModel::Unit::Create(result.model_->dim());
    return result;
  }

  /* ************************************************************************* */
  //GaussianFactorUnordered::shared_ptr JacobianFactorUnordered::negate() const {
  //  HessianFactor hessian(*this);
  //  return hessian.negate();
  //}

  /* ************************************************************************* */
  GaussianConditionalUnordered::shared_ptr JacobianFactorUnordered::eliminateFirst() {
    return this->eliminate(1);
  }

  /* ************************************************************************* */
  GaussianConditionalUnordered::shared_ptr JacobianFactorUnordered::splitConditional(size_t nrFrontals)
  {
    if(nrFrontals > size())
      throw std::invalid_argument("Requesting to split more variables than exist using JacobianFactor::splitConditional");

    size_t frontalDim = Ab_.range(0, nrFrontals).cols();

    // Check for singular factor
    if(model_->dim() < frontalDim)
      throw IndeterminantLinearSystemException(this->keys().front());

    // Restrict the matrix to be in the first nrFrontals variables and create the conditional
    gttic(cond_Rd);
    const DenseIndex originalRowEnd = Ab_.rowEnd();
    Ab_.rowEnd() = Ab_.rowStart() + frontalDim;
    const Eigen::VectorBlock<const Vector> sigmas = model_->sigmas().segment(Ab_.rowStart(), Ab_.rowEnd()-Ab_.rowStart());
    GaussianConditionalUnordered::shared_ptr conditional(boost::make_shared<GaussianConditionalUnordered>(
      Base::keys_, nrFrontals, Ab_, sigmas));
    Ab_.rowStart() += frontalDim;
    Ab_.firstBlock() += nrFrontals;
    Ab_.rowEnd() = originalRowEnd;
    gttoc(cond_Rd);

    // Take lower-right block of Ab to get the new factor
    gttic(remaining_factor);
    keys_.erase(begin(), begin() + nrFrontals);
    // Set sigmas with the right model
    if(model_) {
      if (model_->isConstrained())
        model_ = noiseModel::Constrained::MixedSigmas(model_->sigmas().tail(model_->sigmas().size() - frontalDim));
      else
        model_ = noiseModel::Diagonal::Sigmas(model_->sigmas().tail(model_->sigmas().size() - frontalDim));
    }
    gttoc(remaining_factor);

    return conditional;
  }

  /* ************************************************************************* */
  GaussianConditionalUnordered::shared_ptr JacobianFactorUnordered::eliminate(size_t nrFrontals) {

    if(Ab_.rowStart() != 0 || Ab_.rowEnd() != (size_t) Ab_.matrix().rows() && Ab_.firstBlock() == 0);
    if(nrFrontals > size())
      throw std::invalid_argument("Requesting to eliminate more variables than exist using JacobianFactor::splitConditional");
    assertInvariants();

    const bool debug = ISDEBUG("JacobianFactor::eliminate");

    if(debug) cout << "Eliminating " << nrFrontals << " frontal variables" << endl;
    if(debug) this->print("Eliminating JacobianFactor: ");
    if(debug) gtsam::print(matrix_, "Augmented Ab: ");

    size_t frontalDim = Ab_.range(0,nrFrontals).cols();

    if(debug) cout << "frontalDim = " << frontalDim << endl;

    // Use in-place QR dense Ab appropriate to NoiseModel
    gttic(QR);
    SharedDiagonal noiseModel = model_->QR(matrix_);
    gttoc(QR);

    // Zero the lower-left triangle.  todo: not all of these entries actually
    // need to be zeroed if we are careful to start copying rows after the last
    // structural zero.
    if(matrix_.rows() > 0)
      for(size_t j=0; j<(size_t) matrix_.cols(); ++j)
        for(size_t i=j+1; i<noiseModel->dim(); ++i)
          matrix_(i,j) = 0.0;

    if(debug) gtsam::print(matrix_, "QR result: ");
    if(debug) noiseModel->print("QR result noise model: ");

    // Start of next part
    model_ = noiseModel;
    return splitConditional(nrFrontals);
  }

  /* ************************************************************************* */
  //void JacobianFactorUnordered::allocate(const VariableSlots& variableSlots, vector<
  //    size_t>& varDims, size_t m) {
  //  keys_.resize(variableSlots.size());
  //  std::transform(variableSlots.begin(), variableSlots.end(), begin(),
  //      boost::bind(&VariableSlots::const_iterator::value_type::first, _1));
  //  varDims.push_back(1);
  //  Ab_.copyStructureFrom(BlockAb(matrix_, varDims.begin(), varDims.end(), m));
  //}

  /* ************************************************************************* */
  void JacobianFactorUnordered::setModel(bool anyConstrained, const Vector& sigmas) {
    if((size_t) sigmas.size() != this->rows())
      throw InvalidNoiseModel(this->rows(), sigmas.size());
    if (anyConstrained)
      model_ = noiseModel::Constrained::MixedSigmas(sigmas);
    else
      model_ = noiseModel::Diagonal::Sigmas(sigmas);
  }

}
