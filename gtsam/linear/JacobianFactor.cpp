/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    JacobianFactor.cpp
 * @brief   
 * @author  Richard Roberts
 * @created Dec 8, 2010
 */

#include <gtsam/base/debug.h>
#include <gtsam/base/timing.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/base/FastMap.h>
#include <gtsam/base/cholesky.h>
#include <gtsam/inference/VariableSlots.h>
#include <gtsam/inference/FactorGraph-inl.h>
#include <gtsam/linear/GaussianConditional.h>
#include <gtsam/linear/JacobianFactor.h>
#include <gtsam/linear/HessianFactor.h>
#include <gtsam/linear/GaussianFactorGraph.h>

#include <boost/foreach.hpp>
#include <boost/format.hpp>
#include <boost/make_shared.hpp>
#include <boost/lambda/bind.hpp>
#include <boost/lambda/lambda.hpp>

#include <sstream>
#include <stdexcept>

using namespace std;
using namespace boost::lambda;

namespace gtsam {

  /* ************************************************************************* */
  inline void JacobianFactor::assertInvariants() const {
  #ifndef NDEBUG
    IndexFactor::assertInvariants(); // The base class checks for sorted keys
    assert((size() == 0 && Ab_.rows() == 0 && Ab_.nBlocks() == 0) || size()+1 == Ab_.nBlocks());
    assert(firstNonzeroBlocks_.size() == Ab_.rows());
    for(size_t i=0; i<firstNonzeroBlocks_.size(); ++i)
      assert(firstNonzeroBlocks_[i] < Ab_.nBlocks());
  #endif

    // Check for non-finite values
    for(size_t i=0; i<Ab_.rows(); ++i)
      for(size_t j=0; j<Ab_.cols(); ++j)
        if(isnan(matrix_(i,j)))
          throw invalid_argument("JacobianFactor contains NaN matrix entries.");
  }

  /* ************************************************************************* */
  JacobianFactor::JacobianFactor(const JacobianFactor& gf) :
      GaussianFactor(gf), model_(gf.model_), firstNonzeroBlocks_(gf.firstNonzeroBlocks_), Ab_(matrix_) {
    Ab_.assignNoalias(gf.Ab_);
    assertInvariants();
  }

  /* ************************************************************************* */
  JacobianFactor::JacobianFactor() : Ab_(matrix_) { assertInvariants(); }

  /* ************************************************************************* */
  JacobianFactor::JacobianFactor(const Vector& b_in) : firstNonzeroBlocks_(b_in.size(), 0), Ab_(matrix_) {
    size_t dims[] = { 1 };
    Ab_.copyStructureFrom(BlockAb(matrix_, dims, dims+1, b_in.size()));
    getb() = b_in;
    assertInvariants();
  }

  /* ************************************************************************* */
  JacobianFactor::JacobianFactor(Index i1, const Matrix& A1,
      const Vector& b, const SharedDiagonal& model) :
      GaussianFactor(i1), model_(model), firstNonzeroBlocks_(b.size(), 0), Ab_(matrix_) {
    size_t dims[] = { A1.cols(), 1};
    Ab_.copyStructureFrom(BlockAb(matrix_, dims, dims+2, b.size()));
    Ab_(0) = A1;
    getb() = b;
    assertInvariants();
  }

  /* ************************************************************************* */
  JacobianFactor::JacobianFactor(Index i1, const Matrix& A1, Index i2, const Matrix& A2,
      const Vector& b, const SharedDiagonal& model) :
      GaussianFactor(i1,i2), model_(model), firstNonzeroBlocks_(b.size(), 0), Ab_(matrix_) {
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
      GaussianFactor(i1,i2,i3), model_(model), firstNonzeroBlocks_(b.size(), 0), Ab_(matrix_) {
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
		model_(model), firstNonzeroBlocks_(b.size(), 0), Ab_(matrix_)
  {
    size_t dims[terms.size()+1];
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
    model_(model), firstNonzeroBlocks_(b.size(), 0), Ab_(matrix_)
  {
    size_t dims[terms.size()+1];
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
  JacobianFactor::JacobianFactor(const GaussianConditional& cg) : GaussianFactor(cg), model_(noiseModel::Diagonal::Sigmas(cg.get_sigmas(), true)), Ab_(matrix_) {
    Ab_.assignNoalias(cg.rsd_);
    // todo SL: make firstNonzeroCols triangular?
    firstNonzeroBlocks_.resize(cg.get_d().size(), 0); // set sigmas from precisions
    assertInvariants();
  }

  /* ************************************************************************* */
  JacobianFactor::JacobianFactor(const HessianFactor& factor) : Ab_(matrix_) {
    keys_ = factor.keys_;
    Ab_.assignNoalias(factor.info_);
    size_t maxrank = choleskyCareful(matrix_).first;
    // FIXME: replace with triangular system
//    matrix_ = ublas::triangular_adaptor<AbMatrix, ublas::upper>(matrix_);
    Ab_.rowEnd() = maxrank;
    model_ = noiseModel::Unit::Create(maxrank);

    firstNonzeroBlocks_.resize(this->rows(), 0);

    // Sort keys
    set<Index> vars;
    for(size_t j=0; j<size(); ++j)
      vars.insert(keys_[j]);
    Permutation permutation(Permutation::Identity(*vars.rbegin() + 1));
    size_t jNew = 0;
    BOOST_FOREACH(const Index& var, vars) {
      permutation[var] = jNew++;
    }
    permuteWithInverse(permutation);
    jNew = 0;
    BOOST_FOREACH(const Index& var, vars) {
      keys_[jNew++] = var;
    }

    assertInvariants();
  }

  /* ************************************************************************* */
  void JacobianFactor::print(const string& s) const {
    cout << s << "\n";
    if (empty()) {
      cout << " empty, keys: ";
      BOOST_FOREACH(const Index& key, keys()) { cout << key << " "; }
      cout << endl;
    } else {
      for(const_iterator key=begin(); key!=end(); ++key)
      	cout << boost::format("A[%1%]=\n")%*key << getA(key) << endl;
//        gtsam::print(getA(key), (boost::format("A[%1%]=\n")%*key).str());
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
  void JacobianFactor::permuteWithInverse(const Permutation& inversePermutation) {

    // Build a map from the new variable indices to the old slot positions.
    typedef FastMap<size_t, size_t> SourceSlots;
    SourceSlots sourceSlots;
    for(size_t j=0; j<size(); ++j)
      sourceSlots.insert(make_pair(inversePermutation[keys_[j]], j));

    // Build a vector of variable dimensions in the new order
    vector<size_t> dimensions(size() + 1);
    size_t j = 0;
    BOOST_FOREACH(const SourceSlots::value_type& sourceSlot, sourceSlots) {
      dimensions[j++] = Ab_(sourceSlot.second).cols();
    }
    assert(j == size());
    dimensions.back() = 1;

    // Copy the variables and matrix into the new order
    vector<Index> oldKeys(size());
    keys_.swap(oldKeys);
    AbMatrix oldMatrix;
    BlockAb oldAb(oldMatrix, dimensions.begin(), dimensions.end(), Ab_.rows());
    Ab_.swap(oldAb);
    j = 0;
    BOOST_FOREACH(const SourceSlots::value_type& sourceSlot, sourceSlots) {
      keys_[j] = sourceSlot.first;
      Ab_(j++).noalias() = oldAb(sourceSlot.second);
    }
    Ab_(j).noalias() = oldAb(j);

    // Since we're permuting the variables, ensure that entire rows from this
    // factor are copied when Combine is called
    BOOST_FOREACH(size_t& varpos, firstNonzeroBlocks_) { varpos = 0; }
    assertInvariants();
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
        for (size_t j = 0; j < (size_t) whitenedA.cols(); j++)
          entries.push_back(boost::make_tuple(i, column_start+j, whitenedA(i,j)));
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
  GaussianConditional::shared_ptr JacobianFactor::eliminateFirst() {
    return this->eliminate(1)->front();
  }

  /* ************************************************************************* */
  GaussianBayesNet::shared_ptr JacobianFactor::eliminate(size_t nrFrontals) {

    assert(Ab_.rowStart() == 0 && Ab_.rowEnd() == (size_t) matrix_.rows() && Ab_.firstBlock() == 0);
    assert(size() >= nrFrontals);
    assertInvariants();

    const bool debug = ISDEBUG("JacobianFactor::eliminate");

    if(debug) cout << "Eliminating " << nrFrontals << " frontal variables" << endl;
    if(debug) this->print("Eliminating JacobianFactor: ");

    // NOTE: stairs are not currently used in the Eigen QR implementation
    // add this back if DenseQR is ever reimplemented
//    tic(1, "stairs");
//    // Translate the left-most nonzero column indices into top-most zero row indices
//    vector<int> firstZeroRows(Ab_.cols());
//    {
//      size_t lastNonzeroRow = 0;
//      vector<int>::iterator firstZeroRowsIt = firstZeroRows.begin();
//      for(size_t var=0; var<keys().size(); ++var) {
//        while(lastNonzeroRow < this->rows() && firstNonzeroBlocks_[lastNonzeroRow] <= var)
//          ++ lastNonzeroRow;
//        fill(firstZeroRowsIt, firstZeroRowsIt+Ab_(var).cols(), lastNonzeroRow);
//        firstZeroRowsIt += Ab_(var).cols();
//      }
//      assert(firstZeroRowsIt+1 == firstZeroRows.end());
//      *firstZeroRowsIt = this->rows();
//    }
//    toc(1, "stairs");

//  #ifndef NDEBUG
//    for(size_t col=0; col<Ab_.cols(); ++col) {
//      if(debug) cout << "Staircase[" << col << "] = " << firstZeroRows[col] << endl;
//      if(col != 0) assert(firstZeroRows[col] >= firstZeroRows[col-1]);
//      assert(firstZeroRows[col] <= (long)this->rows());
//    }
//  #endif

    if(debug) gtsam::print(matrix_, "Augmented Ab: ");

    size_t frontalDim = Ab_.range(0,nrFrontals).cols();

    if(debug) cout << "frontalDim = " << frontalDim << endl;

    // Use in-place QR or Cholesky on dense Ab appropriate to NoiseModel
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

    // Check for singular factor
    if(noiseModel->dim() < frontalDim) {
      throw domain_error((boost::format(
          "JacobianFactor is singular in variable %1%, discovered while attempting\n"
          "to eliminate this variable.") % front()).str());
    }

    // Extract conditionals
    tic(3, "cond Rd");
    GaussianBayesNet::shared_ptr conditionals(new GaussianBayesNet());
    for(size_t j=0; j<nrFrontals; ++j) {
      // Temporarily restrict the matrix view to the conditional blocks of the
      // eliminated Ab matrix to create the GaussianConditional from it.
      size_t varDim = Ab_(0).cols();
      Ab_.rowEnd() = Ab_.rowStart() + varDim;
      const Eigen::VectorBlock<const Vector> sigmas = noiseModel->sigmas().segment(Ab_.rowStart(), Ab_.rowEnd()-Ab_.rowStart());
      conditionals->push_back(boost::make_shared<ConditionalType>(begin()+j, end(), 1, Ab_, sigmas));
      if(debug) conditionals->back()->print("Extracted conditional: ");
      Ab_.rowStart() += varDim;
      Ab_.firstBlock() += 1;
    }
    toc(3, "cond Rd");

    if(debug) conditionals->print("Extracted conditionals: ");

    tic(4, "remaining factor");
    // Take lower-right block of Ab to get the new factor
    Ab_.rowEnd() = noiseModel->dim();
    keys_.assign(begin() + nrFrontals, end());
    // Set sigmas with the right model
    if (noiseModel->isConstrained())
      model_ = noiseModel::Constrained::MixedSigmas(sub(noiseModel->sigmas(), frontalDim, noiseModel->dim()));
    else
      model_ = noiseModel::Diagonal::Sigmas(sub(noiseModel->sigmas(), frontalDim, noiseModel->dim()));
    if(debug) this->print("Eliminated factor: ");
    assert(Ab_.rows() <= Ab_.cols()-1);
    toc(4, "remaining factor");

    // todo SL: deal with "dead" pivot columns!!!
    tic(5, "rowstarts");
    size_t varpos = 0;
    firstNonzeroBlocks_.resize(this->rows());
    for(size_t row=0; row<rows(); ++row) {
      if(debug) cout << "row " << row << " varpos " << varpos << " Ab_.offset(varpos)=" << Ab_.offset(varpos) << " Ab_.offset(varpos+1)=" << Ab_.offset(varpos+1) << endl;
      while(varpos < this->size() && Ab_.offset(varpos+1)-Ab_.offset(0) <= row)
        ++ varpos;
      firstNonzeroBlocks_[row] = varpos;
      if(debug) cout << "firstNonzeroVars_[" << row << "] = " << firstNonzeroBlocks_[row] << endl;
    }
    toc(5, "rowstarts");

    if(debug) print("Eliminated factor: ");

    assertInvariants();

    return conditionals;

  }

  /* ************************************************************************* */
  void JacobianFactor::collectInfo(size_t index, vector<_RowSource>& rowSources) const {
		assertInvariants();
		for (size_t row = 0; row < rows(); ++row) {
			Index firstNonzeroVar;
			if (firstNonzeroBlocks_[row] < size()) {
				firstNonzeroVar = keys_[firstNonzeroBlocks_[row]];
			} else if (firstNonzeroBlocks_[row] == size()) {
				firstNonzeroVar = back() + 1;
			} else {
				assert(false);
			}
			rowSources.push_back(_RowSource(firstNonzeroVar, index, row));
		}
	}

  /* ************************************************************************* */
  void JacobianFactor::allocate(const VariableSlots& variableSlots, vector<
			size_t>& varDims, size_t m) {
		keys_.resize(variableSlots.size());
		std::transform(variableSlots.begin(), variableSlots.end(), begin(),
				bind(&VariableSlots::const_iterator::value_type::first,
						boost::lambda::_1));
		varDims.push_back(1);
		Ab_.copyStructureFrom(BlockAb(matrix_, varDims.begin(), varDims.end(), m));
		firstNonzeroBlocks_.resize(m);
	}

  /* ************************************************************************* */
  void JacobianFactor::copyRow(const JacobianFactor& source, Index sourceRow,
			size_t sourceSlot, size_t row, Index slot) {
		ABlock combinedBlock = Ab_(slot);
		if (sourceSlot != numeric_limits<Index>::max()) {
			if (source.firstNonzeroBlocks_[sourceRow] <= sourceSlot) {
				const constABlock sourceBlock(source.Ab_(sourceSlot));
				combinedBlock.row(row).noalias() = sourceBlock.row(sourceRow);
			} else {
				combinedBlock.row(row).setZero();
			}
		} else {
			combinedBlock.row(row).setZero();
		}
	}

  /* ************************************************************************* */
  void JacobianFactor::copyFNZ(size_t m, size_t n,
			vector<_RowSource>& rowSources) {
		Index i = 0;
		for (size_t row = 0; row < m; ++row) {
			while (i < n && rowSources[row].firstNonzeroVar > keys_[i])
				++i;
			firstNonzeroBlocks_[row] = i;
		}
	}

  /* ************************************************************************* */
	void JacobianFactor::setModel(bool anyConstrained, const Vector& sigmas) {
		if (anyConstrained)
			model_ = noiseModel::Constrained::MixedSigmas(sigmas);
		else
			model_ = noiseModel::Diagonal::Sigmas(sigmas);
	}

  /* ************************************************************************* */
  Errors operator*(const FactorGraph<JacobianFactor>& fg, const VectorValues& x) {
    Errors e;
    BOOST_FOREACH(const JacobianFactor::shared_ptr& Ai, fg) {
      e.push_back((*Ai)*x);
    }
    return e;
  }

  /* ************************************************************************* */
  void multiplyInPlace(const FactorGraph<JacobianFactor>& fg, const VectorValues& x, Errors& e) {
    multiplyInPlace(fg,x,e.begin());
  }

  /* ************************************************************************* */
  void multiplyInPlace(const FactorGraph<JacobianFactor>& fg, const VectorValues& x, const Errors::iterator& e) {
    Errors::iterator ei = e;
    BOOST_FOREACH(const JacobianFactor::shared_ptr& Ai, fg) {
      *ei = (*Ai)*x;
      ei++;
    }
  }


  /* ************************************************************************* */
  // x += alpha*A'*e
  void transposeMultiplyAdd(const FactorGraph<JacobianFactor>& fg, double alpha, const Errors& e, VectorValues& x) {
    // For each factor add the gradient contribution
    Errors::const_iterator ei = e.begin();
    BOOST_FOREACH(const JacobianFactor::shared_ptr& Ai, fg) {
      Ai->transposeMultiplyAdd(alpha,*(ei++),x);
    }
  }

  /* ************************************************************************* */
  VectorValues gradient(const FactorGraph<JacobianFactor>& fg, const VectorValues& x) {
    // It is crucial for performance to make a zero-valued clone of x
    VectorValues g = VectorValues::zero(x);
    Errors e;
    BOOST_FOREACH(const JacobianFactor::shared_ptr& factor, fg) {
      e.push_back(factor->error_vector(x));
    }
    transposeMultiplyAdd(fg, 1.0, e, g);
    return g;
  }

  /* ************************************************************************* */
  void residual(const FactorGraph<JacobianFactor>& fg, const VectorValues &x, VectorValues &r) {
    Index i = 0 ;
    BOOST_FOREACH(const JacobianFactor::shared_ptr& factor, fg) {
      r[i] = factor->getb();
      i++;
    }
    VectorValues Ax = VectorValues::SameStructure(r);
    multiply(fg,x,Ax);
    axpy(-1.0,Ax,r);
  }

  /* ************************************************************************* */
  void multiply(const FactorGraph<JacobianFactor>& fg, const VectorValues &x, VectorValues &r) {
    r.makeZero();
    Index i = 0;
    BOOST_FOREACH(const JacobianFactor::shared_ptr& factor, fg) {
      for(JacobianFactor::const_iterator j = factor->begin(); j != factor->end(); ++j) {
        r[i] += factor->getA(j) * x[*j];
      }
      ++i;
    }
  }

  /* ************************************************************************* */
  void transposeMultiply(const FactorGraph<JacobianFactor>& fg, const VectorValues &r, VectorValues &x) {
    x.makeZero();
    Index i = 0;
    BOOST_FOREACH(const JacobianFactor::shared_ptr& factor, fg) {
      for(JacobianFactor::const_iterator j = factor->begin(); j != factor->end(); ++j) {
        x[*j] += factor->getA(j).transpose() * r[i];
      }
      ++i;
    }
  }

}
