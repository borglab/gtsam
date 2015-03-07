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
 * @date    Dec 8, 2010
 */

#include <gtsam/linear/linearExceptions.h>
#include <gtsam/linear/GaussianConditional.h>
#include <gtsam/linear/JacobianFactor.h>
#include <gtsam/linear/HessianFactor.h>
#include <gtsam/linear/GaussianFactorGraph.h>
#include <gtsam/inference/VariableSlots.h>
#include <gtsam/base/debug.h>
#include <gtsam/base/timing.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/base/FastMap.h>
#include <gtsam/base/cholesky.h>

#include <boost/foreach.hpp>
#include <boost/format.hpp>
#include <boost/make_shared.hpp>
#include <boost/bind.hpp>
//#include <boost/lambda/bind.hpp>
//#include <boost/lambda/lambda.hpp>

#include <cmath>
#include <sstream>
#include <stdexcept>

using namespace std;
//using namespace boost::lambda;

namespace gtsam {

  /* ************************************************************************* */
  void JacobianFactor::assertInvariants() const {
#ifndef NDEBUG
    GaussianFactor::assertInvariants(); // The base class checks for unique keys
    assert((size() == 0 && Ab_.rows() == 0 && Ab_.nBlocks() == 0) || size()+1 == Ab_.nBlocks());
#endif
  }

  /* ************************************************************************* */
  JacobianFactor::JacobianFactor(const JacobianFactor& gf) :
      GaussianFactor(gf), model_(gf.model_), Ab_(matrix_) {
    Ab_.assignNoalias(gf.Ab_);
    assertInvariants();
  }

  /* ************************************************************************* */
  JacobianFactor::JacobianFactor(const GaussianFactor& gf) : Ab_(matrix_) {
    // Copy the matrix data depending on what type of factor we're copying from
    if(const JacobianFactor* rhs = dynamic_cast<const JacobianFactor*>(&gf))
      *this = JacobianFactor(*rhs);
    else if(const HessianFactor* rhs = dynamic_cast<const HessianFactor*>(&gf))
      *this = JacobianFactor(*rhs);
    else
      throw std::invalid_argument("In JacobianFactor(const GaussianFactor& rhs), rhs is neither a JacobianFactor nor a HessianFactor");
    assertInvariants();
  }

  /* ************************************************************************* */
  JacobianFactor::JacobianFactor() : Ab_(matrix_) { assertInvariants(); }

  /* ************************************************************************* */
  JacobianFactor::JacobianFactor(const Vector& b_in) : Ab_(matrix_) {
    size_t dims[] = { 1 };
    Ab_.copyStructureFrom(BlockAb(matrix_, dims, dims+1, b_in.size()));
    getb() = b_in;
    model_ = noiseModel::Unit::Create(this->rows());
    assertInvariants();
  }

  /* ************************************************************************* */
  JacobianFactor::JacobianFactor(Index i1, const Matrix& A1,
      const Vector& b, const SharedDiagonal& model) :
      GaussianFactor(i1), model_(model), Ab_(matrix_) {

    if(model->dim() != (size_t) b.size())
      throw InvalidNoiseModel(b.size(), model->dim());

    size_t dims[] = { A1.cols(), 1};
    Ab_.copyStructureFrom(BlockAb(matrix_, dims, dims+2, b.size()));
    Ab_(0) = A1;
    getb() = b;
    assertInvariants();
  }

  /* ************************************************************************* */
  JacobianFactor::JacobianFactor(Index i1, const Matrix& A1, Index i2, const Matrix& A2,
      const Vector& b, const SharedDiagonal& model) :
      GaussianFactor(i1,i2), model_(model), Ab_(matrix_) {

    if(model->dim() != (size_t) b.size())
      throw InvalidNoiseModel(b.size(), model->dim());

    size_t dims[] = { A1.cols(), A2.cols(), 1};
    Ab_.copyStructureFrom(BlockAb(matrix_, dims, dims+3, b.size()));
    Ab_(0) = A1;
    Ab_(1) = A2;
    getb() = b;
    assertInvariants();
  }

  /* ************************************************************************* */
  JacobianFactor::JacobianFactor(Index i1, const Matrix& A1, Index i2, const Matrix& A2,
      Index i3, const Matrix& A3, const Vector& b, const SharedDiagonal& model) :
      GaussianFactor(i1,i2,i3), model_(model), Ab_(matrix_) {

    if(model->dim() != (size_t) b.size())
      throw InvalidNoiseModel(b.size(), model->dim());

    size_t dims[] = { A1.cols(), A2.cols(), A3.cols(), 1};
    Ab_.copyStructureFrom(BlockAb(matrix_, dims, dims+4, b.size()));
    Ab_(0) = A1;
    Ab_(1) = A2;
    Ab_(2) = A3;
    getb() = b;
    assertInvariants();
  }

  /* ************************************************************************* */
  JacobianFactor::JacobianFactor(const std::vector<std::pair<Index, Matrix> > &terms,
  		const Vector &b, const SharedDiagonal& model) :
  	GaussianFactor(GetKeys(terms.size(), terms.begin(), terms.end())),
		model_(model), Ab_(matrix_)
  {

    if(model->dim() != (size_t) b.size())
      throw InvalidNoiseModel(b.size(), model->dim());

    size_t* dims = (size_t*)alloca(sizeof(size_t)*(terms.size()+1)); // FIXME: alloca is bad, just ask Google.
    for(size_t j=0; j<terms.size(); ++j)
      dims[j] = terms[j].second.cols();
    dims[terms.size()] = 1;
    Ab_.copyStructureFrom(BlockAb(matrix_, dims, dims+terms.size()+1, b.size()));
    for(size_t j=0; j<terms.size(); ++j)
      Ab_(j) = terms[j].second;
    getb() = b;
    assertInvariants();
  }

  /* ************************************************************************* */
  JacobianFactor::JacobianFactor(const std::list<std::pair<Index, Matrix> > &terms,
      const Vector &b, const SharedDiagonal& model) :
      GaussianFactor(GetKeys(terms.size(), terms.begin(), terms.end())),
    model_(model), Ab_(matrix_)
  {

    if(model->dim() != (size_t) b.size())
      throw InvalidNoiseModel(b.size(), model->dim());

    size_t* dims=(size_t*)alloca(sizeof(size_t)*(terms.size()+1)); // FIXME: alloca is bad, just ask Google.
    size_t j=0;
    std::list<std::pair<Index, Matrix> >::const_iterator term=terms.begin();
    for(; term!=terms.end(); ++term,++j)
      dims[j] = term->second.cols();
    dims[j] = 1;
    Ab_.copyStructureFrom(BlockAb(matrix_, dims, dims+terms.size()+1, b.size()));
    j = 0;
    for(term=terms.begin(); term!=terms.end(); ++term,++j)
      Ab_(j) = term->second;
    getb() = b;
    assertInvariants();
  }

  /* ************************************************************************* */
  JacobianFactor::JacobianFactor(const GaussianConditional& cg) :
		GaussianFactor(cg),
		model_(noiseModel::Diagonal::Sigmas(cg.get_sigmas(), true)),
		Ab_(matrix_) {
    Ab_.assignNoalias(cg.rsd_);
    assertInvariants();
  }

  /* ************************************************************************* */
  JacobianFactor::JacobianFactor(const HessianFactor& factor) : Ab_(matrix_) {
    keys_ = factor.keys_;
    Ab_.assignNoalias(factor.info_);

		// Do Cholesky to get a Jacobian
    size_t maxrank;
		bool success;
		boost::tie(maxrank, success) = choleskyCareful(matrix_);

		// Check for indefinite system
		if(!success)
			throw IndeterminantLinearSystemException(factor.keys().front());

    // Zero out lower triangle
    matrix_.topRows(maxrank).triangularView<Eigen::StrictlyLower>() =
        Matrix::Zero(maxrank, matrix_.cols());
    // FIXME: replace with triangular system
    Ab_.rowEnd() = maxrank;
    model_ = noiseModel::Unit::Create(maxrank);

    assertInvariants();
  }

  /* ************************************************************************* */
  JacobianFactor& JacobianFactor::operator=(const JacobianFactor& rhs) {
    this->Base::operator=(rhs); // Copy keys
    model_ = rhs.model_;        // Copy noise model
    Ab_.assignNoalias(rhs.Ab_); // Copy matrix and block structure
    assertInvariants();
    return *this;
  }

  /* ************************************************************************* */
  void JacobianFactor::print(const string& s, const IndexFormatter& formatter) const {
    cout << s << "\n";
    if (empty()) {
      cout << " empty, keys: ";
      BOOST_FOREACH(const Index& key, keys()) { cout << formatter(key) << " "; }
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
  bool JacobianFactor::equals(const GaussianFactor& f_, double tol) const {
    if(!dynamic_cast<const JacobianFactor*>(&f_))
      return false;
    else {
      const JacobianFactor& f(static_cast<const JacobianFactor&>(f_));
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
  Vector JacobianFactor::unweighted_error(const VectorValues& c) const {
    Vector e = -getb();
    if (empty()) return e;
    for(size_t pos=0; pos<size(); ++pos)
      e += Ab_(pos) * c[keys_[pos]];
    return e;
  }

  /* ************************************************************************* */
  Vector JacobianFactor::error_vector(const VectorValues& c) const {
    if (empty()) return model_->whiten(-getb());
    return model_->whiten(unweighted_error(c));
  }

  /* ************************************************************************* */
  double JacobianFactor::error(const VectorValues& c) const {
    if (empty()) return 0;
    Vector weighted = error_vector(c);
    return 0.5 * weighted.dot(weighted);
  }

  /* ************************************************************************* */
  Matrix JacobianFactor::computeInformation() const {
    Matrix AbWhitened = Ab_.full();
    model_->WhitenInPlace(AbWhitened);
    return AbWhitened.transpose() * AbWhitened;
  }

  /* ************************************************************************* */
  Vector JacobianFactor::operator*(const VectorValues& x) const {
    Vector Ax = zero(Ab_.rows());
    if (empty()) return Ax;

    // Just iterate over all A matrices and multiply in correct config part
    for(size_t pos=0; pos<size(); ++pos)
      Ax += Ab_(pos) * x[keys_[pos]];

    return model_->whiten(Ax);
  }

  /* ************************************************************************* */
  void JacobianFactor::transposeMultiplyAdd(double alpha, const Vector& e,
      VectorValues& x) const {
    Vector E = alpha * model_->whiten(e);
    // Just iterate over all A matrices and insert Ai^e into VectorValues
    for(size_t pos=0; pos<size(); ++pos)
      gtsam::transposeMultiplyAdd(1.0, Ab_(pos), E, x[keys_[pos]]);
  }

  /* ************************************************************************* */
  pair<Matrix,Vector> JacobianFactor::matrix(bool weight) const {
    Matrix A(Ab_.range(0, size()));
    Vector b(getb());
    // divide in sigma so error is indeed 0.5*|Ax-b|
    if (weight) model_->WhitenSystem(A,b);
    return make_pair(A, b);
  }

  /* ************************************************************************* */
  Matrix JacobianFactor::matrix_augmented(bool weight) const {
    if (weight) { Matrix Ab(Ab_.range(0,Ab_.nBlocks())); model_->WhitenInPlace(Ab); return Ab; }
    else return Ab_.range(0, Ab_.nBlocks());
  }

  /* ************************************************************************* */
  std::vector<boost::tuple<size_t, size_t, double> >
  JacobianFactor::sparse(const std::vector<size_t>& columnIndices) const {

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
  JacobianFactor JacobianFactor::whiten() const {
    JacobianFactor result(*this);
    result.model_->WhitenInPlace(result.matrix_);
    result.model_ = noiseModel::Unit::Create(result.model_->dim());
    return result;
  }

  /* ************************************************************************* */
  GaussianFactor::shared_ptr JacobianFactor::negate() const {
  	HessianFactor hessian(*this);
  	return hessian.negate();
  }

  /* ************************************************************************* */
  GaussianConditional::shared_ptr JacobianFactor::eliminateFirst() {
    return this->eliminate(1);
  }

  /* ************************************************************************* */
  GaussianConditional::shared_ptr JacobianFactor::splitConditional(size_t nrFrontals) {
  	assert(Ab_.rowStart() == 0 && Ab_.rowEnd() == (size_t) matrix_.rows() && Ab_.firstBlock() == 0);
  	assert(size() >= nrFrontals);
  	assertInvariants();

  	const bool debug = ISDEBUG("JacobianFactor::splitConditional");

  	if(debug) cout << "Eliminating " << nrFrontals << " frontal variables" << endl;
  	if(debug) this->print("Splitting JacobianFactor: ");

  	size_t frontalDim = Ab_.range(0,nrFrontals).cols();

  	// Check for singular factor
  	if(model_->dim() < frontalDim)
			throw IndeterminantLinearSystemException(this->keys().front());

  	// Extract conditional
  	tic(3, "cond Rd");

  	// Restrict the matrix to be in the first nrFrontals variables
  	Ab_.rowEnd() = Ab_.rowStart() + frontalDim;
  	const Eigen::VectorBlock<const Vector> sigmas = model_->sigmas().segment(Ab_.rowStart(), Ab_.rowEnd()-Ab_.rowStart());
  	GaussianConditional::shared_ptr conditional(new GaussianConditional(begin(), end(), nrFrontals, Ab_, sigmas));
  	if(debug) conditional->print("Extracted conditional: ");
  	Ab_.rowStart() += frontalDim;
  	Ab_.firstBlock() += nrFrontals;
  	toc(3, "cond Rd");

  	if(debug) conditional->print("Extracted conditional: ");

  	tic(4, "remaining factor");
  	// Take lower-right block of Ab to get the new factor
  	Ab_.rowEnd() = model_->dim();
  	keys_.erase(begin(), begin() + nrFrontals);
  	// Set sigmas with the right model
  	if (model_->isConstrained())
  		model_ = noiseModel::Constrained::MixedSigmas(sub(model_->sigmas(), frontalDim, model_->dim()));
  	else
  		model_ = noiseModel::Diagonal::Sigmas(sub(model_->sigmas(), frontalDim, model_->dim()));
  	if(debug) this->print("Eliminated factor: ");
  	assert(Ab_.rows() <= Ab_.cols()-1);
  	toc(4, "remaining factor");

  	if(debug) print("Eliminated factor: ");

  	assertInvariants();

  	return conditional;
  }

  /* ************************************************************************* */
  GaussianConditional::shared_ptr JacobianFactor::eliminate(size_t nrFrontals) {

    assert(Ab_.rowStart() == 0 && Ab_.rowEnd() == (size_t) matrix_.rows() && Ab_.firstBlock() == 0);
    assert(size() >= nrFrontals);
    assertInvariants();

    const bool debug = ISDEBUG("JacobianFactor::eliminate");

    if(debug) cout << "Eliminating " << nrFrontals << " frontal variables" << endl;
    if(debug) this->print("Eliminating JacobianFactor: ");
    if(debug) gtsam::print(matrix_, "Augmented Ab: ");

    size_t frontalDim = Ab_.range(0,nrFrontals).cols();

    if(debug) cout << "frontalDim = " << frontalDim << endl;

    // Use in-place QR dense Ab appropriate to NoiseModel
    tic(2, "QR");
    SharedDiagonal noiseModel = model_->QR(matrix_);
    toc(2, "QR");

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
  void JacobianFactor::allocate(const VariableSlots& variableSlots, vector<
			size_t>& varDims, size_t m) {
		keys_.resize(variableSlots.size());
		std::transform(variableSlots.begin(), variableSlots.end(), begin(),
				boost::bind(&VariableSlots::const_iterator::value_type::first, _1));
		varDims.push_back(1);
		Ab_.copyStructureFrom(BlockAb(matrix_, varDims.begin(), varDims.end(), m));
	}

  /* ************************************************************************* */
	void JacobianFactor::setModel(bool anyConstrained, const Vector& sigmas) {
    if((size_t) sigmas.size() != this->rows())
      throw InvalidNoiseModel(this->rows(), sigmas.size());
		if (anyConstrained)
			model_ = noiseModel::Constrained::MixedSigmas(sigmas);
		else
			model_ = noiseModel::Diagonal::Sigmas(sigmas);
	}

  /* ************************************************************************* */
  const char* JacobianFactor::InvalidNoiseModel::what() const throw() {
    if(description_.empty())
      description_ = (boost::format(
        "A JacobianFactor was attempted to be constructed or modified to use a\n"
        "noise model of incompatible dimension.  The JacobianFactor has\n"
        "dimensionality (i.e. length of error vector) %d but the provided noise\n"
        "model has dimensionality %d.") % factorDims % noiseModelDims).str();
    return description_.c_str();
  }

}
