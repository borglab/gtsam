/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    GaussianFactor.cpp
 * @brief   Linear Factor....A Gaussian
 * @brief   linearFactor
 * @author  Christian Potthast
 */

#include <gtsam/base/timing.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/base/FastMap.h>
#include <gtsam/base/cholesky.h>
#include <gtsam/linear/GaussianConditional.h>
#include <gtsam/linear/GaussianFactor.h>
#include <gtsam/linear/GaussianFactorGraph.h>

#include <boost/foreach.hpp>
#include <boost/format.hpp>
#include <boost/make_shared.hpp>
#include <boost/pool/pool_alloc.hpp>
#include <boost/lambda/bind.hpp>
#include <boost/lambda/lambda.hpp>
#include <boost/tuple/tuple.hpp>

#include <boost/numeric/ublas/triangular.hpp>
#include <boost/numeric/ublas/io.hpp>
#include <boost/numeric/ublas/matrix_proxy.hpp>
#include <boost/numeric/ublas/vector_proxy.hpp>
#include <boost/numeric/ublas/blas.hpp>

#include <sstream>

using namespace std;

namespace ublas = boost::numeric::ublas;
using namespace boost::lambda;

namespace gtsam {

/* ************************************************************************* */
inline void GaussianFactor::assertInvariants() const {
#ifndef NDEBUG
  IndexFactor::assertInvariants();
  assert((keys_.size() == 0 && Ab_.size1() == 0 && Ab_.nBlocks() == 0) || keys_.size()+1 == Ab_.nBlocks());
#endif
}

/* ************************************************************************* */
GaussianFactor::GaussianFactor(const GaussianFactor& gf) :
    IndexFactor(gf), model_(gf.model_), firstNonzeroBlocks_(gf.firstNonzeroBlocks_), Ab_(matrix_) {
  Ab_.assignNoalias(gf.Ab_);
  assertInvariants();
}

/* ************************************************************************* */
GaussianFactor::GaussianFactor() : Ab_(matrix_) { assertInvariants(); }

/* ************************************************************************* */
GaussianFactor::GaussianFactor(const Vector& b_in) : firstNonzeroBlocks_(b_in.size(), 0), Ab_(matrix_) {
  size_t dims[] = { 1 };
  Ab_.copyStructureFrom(BlockAb(matrix_, dims, dims+1, b_in.size()));
  getb() = b_in;
  assertInvariants();
}

/* ************************************************************************* */
GaussianFactor::GaussianFactor(Index i1, const Matrix& A1,
    const Vector& b, const SharedDiagonal& model) :
		IndexFactor(i1), model_(model), firstNonzeroBlocks_(b.size(), 0), Ab_(matrix_) {
  size_t dims[] = { A1.size2(), 1};
  Ab_.copyStructureFrom(BlockAb(matrix_, dims, dims+2, b.size()));
	Ab_(0) = A1;
	getb() = b;
	assertInvariants();
}

/* ************************************************************************* */
GaussianFactor::GaussianFactor(Index i1, const Matrix& A1, Index i2, const Matrix& A2,
		const Vector& b, const SharedDiagonal& model) :
		IndexFactor(i1,i2), model_(model), firstNonzeroBlocks_(b.size(), 0), Ab_(matrix_) {
  size_t dims[] = { A1.size2(), A2.size2(), 1};
  Ab_.copyStructureFrom(BlockAb(matrix_, dims, dims+3, b.size()));
  Ab_(0) = A1;
  Ab_(1) = A2;
  getb() = b;
  assertInvariants();
}

/* ************************************************************************* */
GaussianFactor::GaussianFactor(Index i1, const Matrix& A1, Index i2, const Matrix& A2,
    Index i3, const Matrix& A3,	const Vector& b, const SharedDiagonal& model) :
    IndexFactor(i1,i2,i3), model_(model), firstNonzeroBlocks_(b.size(), 0), Ab_(matrix_) {
  size_t dims[] = { A1.size2(), A2.size2(), A3.size2(), 1};
  Ab_.copyStructureFrom(BlockAb(matrix_, dims, dims+4, b.size()));
  Ab_(0) = A1;
  Ab_(1) = A2;
  Ab_(2) = A3;
  getb() = b;
  assertInvariants();
}

/* ************************************************************************* */
GaussianFactor::GaussianFactor(const std::vector<std::pair<Index, Matrix> > &terms,
    const Vector &b, const SharedDiagonal& model) :
    model_(model), firstNonzeroBlocks_(b.size(), 0), Ab_(matrix_) {
  keys_.resize(terms.size());
  size_t dims[terms.size()+1];
  for(size_t j=0; j<terms.size(); ++j) {
    keys_[j] = terms[j].first;
    dims[j] = terms[j].second.size2();
	}
  dims[terms.size()] = 1;
  Ab_.copyStructureFrom(BlockAb(matrix_, dims, dims+terms.size()+1, b.size()));
  for(size_t j=0; j<terms.size(); ++j)
    Ab_(j) = terms[j].second;
  getb() = b;
  assertInvariants();
}

/* ************************************************************************* */
GaussianFactor::GaussianFactor(const std::list<std::pair<Index, Matrix> > &terms,
    const Vector &b, const SharedDiagonal& model) :
    model_(model), firstNonzeroBlocks_(b.size(), 0), Ab_(matrix_) {
  keys_.resize(terms.size());
  size_t dims[terms.size()+1];
  size_t j=0;
  for(std::list<std::pair<Index, Matrix> >::const_iterator term=terms.begin(); term!=terms.end(); ++term) {
    keys_[j] = term->first;
    dims[j] = term->second.size2();
    ++ j;
  }
  dims[j] = 1;
  firstNonzeroBlocks_.resize(b.size(), 0);
  Ab_.copyStructureFrom(BlockAb(matrix_, dims, dims+terms.size()+1, b.size()));
  j = 0;
  for(std::list<std::pair<Index, Matrix> >::const_iterator term=terms.begin(); term!=terms.end(); ++term) {
    Ab_(j) = term->second;
    ++ j;
  }
  getb() = b;
  assertInvariants();
}

/* ************************************************************************* */
GaussianFactor::GaussianFactor(const GaussianConditional& cg) : IndexFactor(cg), model_(noiseModel::Diagonal::Sigmas(cg.get_sigmas(), true)), Ab_(matrix_) {
  Ab_.assignNoalias(cg.rsd_);
  // todo SL: make firstNonzeroCols triangular?
  firstNonzeroBlocks_.resize(cg.get_d().size(), 0);	// set sigmas from precisions
  assertInvariants();
}

/* ************************************************************************* */
void GaussianFactor::print(const string& s) const {
  cout << s << "\n";
  if (empty()) {
    cout << " empty, keys: ";
    BOOST_FOREACH(const Index key, keys_) { cout << key << " "; }
    cout << endl;
  } else {
  	for(const_iterator key=begin(); key!=end(); ++key)
  		gtsam::print(getA(key), (boost::format("A[%1%]=\n")%*key).str());
    gtsam::print(getb(),"b=");
    model_->print("model");
  }
}

/* ************************************************************************* */
// Check if two linear factors are equal
bool GaussianFactor::equals(const GaussianFactor& f, double tol) const {
  if (empty()) return (f.empty());
  if(keys_!=f.keys_ /*|| !model_->equals(lf->model_, tol)*/)
    return false;

  assert(Ab_.size1() == f.Ab_.size1() && Ab_.size2() == f.Ab_.size2());

  constABlock Ab1(Ab_.range(0, Ab_.nBlocks()));
  constABlock Ab2(f.Ab_.range(0, f.Ab_.nBlocks()));
  for(size_t row=0; row<Ab1.size1(); ++row)
    if(!equal_with_abs_tol(ublas::row(Ab1, row), ublas::row(Ab2, row), tol) &&
        !equal_with_abs_tol(-ublas::row(Ab1, row), ublas::row(Ab2, row), tol))
      return false;

  return true;
}

/* ************************************************************************* */
void GaussianFactor::permuteWithInverse(const Permutation& inversePermutation) {

  // Build a map from the new variable indices to the old slot positions.
  typedef map<size_t, size_t, std::less<size_t>, boost::fast_pool_allocator<std::pair<const size_t, size_t> > > SourceSlots;
  SourceSlots sourceSlots;
  for(size_t j=0; j<keys_.size(); ++j)
    sourceSlots.insert(make_pair(inversePermutation[keys_[j]], j));

  // Build a vector of variable dimensions in the new order
  vector<size_t> dimensions(keys_.size() + 1);
  size_t j = 0;
  BOOST_FOREACH(const SourceSlots::value_type& sourceSlot, sourceSlots) {
    dimensions[j++] = Ab_(sourceSlot.second).size2();
  }
  assert(j == keys_.size());
  dimensions.back() = 1;

  // Copy the variables and matrix into the new order
  vector<Index> oldKeys(keys_.size());
  keys_.swap(oldKeys);
  AbMatrix oldMatrix;
  BlockAb oldAb(oldMatrix, dimensions.begin(), dimensions.end(), Ab_.size1());
  Ab_.swap(oldAb);
  j = 0;
  BOOST_FOREACH(const SourceSlots::value_type& sourceSlot, sourceSlots) {
    keys_[j] = sourceSlot.first;
    ublas::noalias(Ab_(j++)) = oldAb(sourceSlot.second);
  }
  ublas::noalias(Ab_(j)) = oldAb(j);

  // Since we're permuting the variables, ensure that entire rows from this
  // factor are copied when Combine is called
  BOOST_FOREACH(size_t& varpos, firstNonzeroBlocks_) { varpos = 0; }
  assertInvariants();
}

/* ************************************************************************* */
Vector GaussianFactor::unweighted_error(const VectorValues& c) const {
  Vector e = -getb();
  if (empty()) return e;
  for(size_t pos=0; pos<keys_.size(); ++pos)
    e += ublas::prod(Ab_(pos), c[keys_[pos]]);
  return e;
}

/* ************************************************************************* */
Vector GaussianFactor::error_vector(const VectorValues& c) const {
	if (empty()) return model_->whiten(-getb());
	return model_->whiten(unweighted_error(c));
}

/* ************************************************************************* */
double GaussianFactor::error(const VectorValues& c) const {
  if (empty()) return 0;
  Vector weighted = error_vector(c);
  return 0.5 * inner_prod(weighted,weighted);
}


/* ************************************************************************* */
Vector GaussianFactor::operator*(const VectorValues& x) const {
	Vector Ax = zero(Ab_.size1());
  if (empty()) return Ax;

  // Just iterate over all A matrices and multiply in correct config part
  for(size_t pos=0; pos<keys_.size(); ++pos)
    Ax += ublas::prod(Ab_(pos), x[keys_[pos]]);

  return model_->whiten(Ax);
}


/* ************************************************************************* */
void GaussianFactor::transposeMultiplyAdd(double alpha, const Vector& e,
		VectorValues& x) const {
	Vector E = alpha * model_->whiten(e);
	// Just iterate over all A matrices and insert Ai^e into VectorValues
  for(size_t pos=0; pos<keys_.size(); ++pos)
    gtsam::transposeMultiplyAdd(1.0, Ab_(pos), E, x[keys_[pos]]);
}

/* ************************************************************************* */  
pair<Matrix,Vector> GaussianFactor::matrix(bool weight) const {
  Matrix A(Ab_.range(0, keys_.size()));
  Vector b(getb());
	// divide in sigma so error is indeed 0.5*|Ax-b|
	if (weight) model_->WhitenSystem(A,b);
	return make_pair(A, b);
}

/* ************************************************************************* */
Matrix GaussianFactor::matrix_augmented(bool weight) const {
	if (weight) { Matrix Ab(Ab_.range(0,Ab_.nBlocks())); model_->WhitenInPlace(Ab); return Ab; }
	else return Ab_.range(0, Ab_.nBlocks());
}

/* ************************************************************************* */
boost::tuple<list<int>, list<int>, list<double> >
GaussianFactor::sparse(const Dimensions& columnIndices) const {

	// declare return values
	list<int> I,J;
	list<double> S;

	// iterate over all matrices in the factor
  for(size_t pos=0; pos<keys_.size(); ++pos) {
    constABlock A(Ab_(pos));
		// find first column index for this key
		int column_start = columnIndices.at(keys_[pos]);
		for (size_t i = 0; i < A.size1(); i++) {
			double sigma_i = model_->sigma(i);
			for (size_t j = 0; j < A.size2(); j++)
				if (A(i, j) != 0.0) {
					I.push_back(i + 1);
					J.push_back(j + column_start);
					S.push_back(A(i, j) / sigma_i);
				}
		}
	}

	// return the result
	return boost::tuple<list<int>, list<int>, list<double> >(I,J,S);
}

/* ************************************************************************* */
GaussianFactor GaussianFactor::whiten() const {
  GaussianFactor result(*this);
  result.model_->WhitenInPlace(result.matrix_);
  result.model_ = noiseModel::Unit::Create(result.model_->dim());
  return result;
}

/* ************************************************************************* */
struct SlotEntry {
  size_t slot;
  size_t dimension;
  SlotEntry(size_t _slot, size_t _dimension) : slot(_slot), dimension(_dimension) {}
};

typedef FastMap<Index, SlotEntry> Scatter;

/* ************************************************************************* */
static FastMap<Index, SlotEntry> findScatterAndDims(const FactorGraph<GaussianFactor>& factors) {

  static const bool debug = false;

  // The "scatter" is a map from global variable indices to slot indices in the
  // union of involved variables.  We also include the dimensionality of the
  // variable.

  Scatter scatter;

  // First do the set union.
  BOOST_FOREACH(const GaussianFactor::shared_ptr& factor, factors) {
    for(GaussianFactor::const_iterator variable = factor->begin(); variable != factor->end(); ++variable) {
      scatter.insert(make_pair(*variable, SlotEntry(0, factor->getDim(variable))));
    }
  }

  // Next fill in the slot indices (we can only get these after doing the set
  // union.
  size_t slot = 0;
  BOOST_FOREACH(Scatter::value_type& var_slot, scatter) {
    var_slot.second.slot = (slot ++);
    if(debug)
      cout << "scatter[" << var_slot.first << "] = (slot " << var_slot.second.slot << ", dim " << var_slot.second.dimension << ")" << endl;
  }

  return scatter;
}

/* ************************************************************************* */
static MatrixColMajor formAbTAb(const FactorGraph<GaussianFactor>& factors, const Scatter& scatter) {

  static const bool debug = false;

  tic("CombineAndEliminate: 3.1 varStarts");
  // Determine scalar indices of each variable
  vector<size_t> varStarts;
  varStarts.reserve(scatter.size() + 2);
  varStarts.push_back(0);
  BOOST_FOREACH(const Scatter::value_type& var_slot, scatter) {
    varStarts.push_back(varStarts.back() + var_slot.second.dimension);
  }
  // This is for the r.h.s. vector
  varStarts.push_back(varStarts.back() + 1);
  toc("CombineAndEliminate: 3.1 varStarts");

  // Allocate and zero matrix for Ab' * Ab
  MatrixColMajor ATA(ublas::zero_matrix<double>(varStarts.back(), varStarts.back()));

  tic("CombineAndEliminate: 3.2 updates");
  // Do blockwise low-rank updates to Ab' * Ab for each factor.  Here, we
  // only update the upper triangle because this is all that Cholesky uses.
  BOOST_FOREACH(const GaussianFactor::shared_ptr& factor, factors) {

    // Whiten the factor first so it has a unit diagonal noise model
    GaussianFactor whitenedFactor(factor->whiten());

    if(debug) whitenedFactor.print("whitened factor: ");

    for(GaussianFactor::const_iterator var2 = whitenedFactor.begin(); var2 != whitenedFactor.end(); ++var2) {
      assert(scatter.find(*var2) != scatter.end());
      size_t vj = scatter.find(*var2)->second.slot;
      for(GaussianFactor::const_iterator var1 = whitenedFactor.begin(); var1 <= var2; ++var1) {
        assert(scatter.find(*var1) != scatter.end());
        size_t vi = scatter.find(*var1)->second.slot;
        if(debug) cout << "Updating block " << vi << ", " << vj << endl;
        if(debug) cout << "Updating (" << varStarts[vi] << ":" << varStarts[vi+1] << ", " <<
            varStarts[vj] << ":" << varStarts[vj+1] << ") from A" << *var1 << "' * A" << *var2 << endl;
        ublas::project(ATA,
            ublas::range(varStarts[vi], varStarts[vi+1]), ublas::range(varStarts[vj], varStarts[vj+1])) +=
                ublas::prod(ublas::trans(whitenedFactor.getA(var1)), whitenedFactor.getA(var2));
      }
    }

    // Update r.h.s. vector
    size_t vj = scatter.size();
    for(GaussianFactor::const_iterator var1 = whitenedFactor.begin(); var1 < whitenedFactor.end(); ++var1) {
      assert(scatter.find(*var1) != scatter.end());
      size_t vi = scatter.find(*var1)->second.slot;
      if(debug) cout << "Updating block " << vi << ", " << vj << endl;
      if(debug) cout << "Updating (" << varStarts[vi] << ":" << varStarts[vi+1] << ", " <<
          varStarts[vj] << ":" << varStarts[vj+1] << ") from A" << *var1 << "' * b" << endl;
      ublas::matrix_column<MatrixColMajor> col(ATA, varStarts[vj]);
      ublas::subrange(col, varStarts[vi], varStarts[vi+1]) +=
          ublas::prod(ublas::trans(whitenedFactor.getA(var1)), whitenedFactor.getb());
    }

    size_t vi = scatter.size();
    if(debug) cout << "Updating block " << vi << ", " << vj << endl;
    if(debug) cout << "Updating (" << varStarts[vi] << ":" << varStarts[vi+1] << ", " <<
        varStarts[vj] << ":" << varStarts[vj+1] << ") from b" << "' * b" << endl;
    ATA(varStarts[vi], varStarts[vj]) += ublas::inner_prod(whitenedFactor.getb(), whitenedFactor.getb());
  }
  toc("CombineAndEliminate: 3.2 updates");

  return ATA;
}

GaussianBayesNet::shared_ptr GaussianFactor::splitEliminatedFactor(size_t nrFrontals, const vector<Index>& keys) {

  static const bool debug = false;

  const size_t maxrank = Ab_.size1();

  // Check for rank-deficiency that would prevent back-substitution
  if(maxrank < Ab_.range(0, nrFrontals).size2()) {
    stringstream ss;
    ss << "Problem is rank-deficient, discovered while eliminating frontal variables";
    for(size_t i=0; i<nrFrontals; ++i)
      ss << " " << keys[i];
    throw invalid_argument(ss.str());
  }

  if(debug) gtsam::print(Matrix(Ab_.range(0, Ab_.nBlocks())), "remaining Ab: ");

  // Extract conditionals
  tic("CombineAndEliminate: 5.1 cond Rd");
  GaussianBayesNet::shared_ptr conditionals(new GaussianBayesNet());
  for(size_t j=0; j<nrFrontals; ++j) {
    // Temporarily restrict the matrix view to the conditional blocks of the
    // eliminated Ab_ matrix to create the GaussianConditional from it.
    size_t varDim = Ab_(0).size2();
    Ab_.rowEnd() = Ab_.rowStart() + varDim;

    // Zero the entries below the diagonal (this relies on the matrix being
    // column-major).
    {
      ABlock remainingMatrix(Ab_.range(0, Ab_.nBlocks()));
      if(remainingMatrix.size1() > 1)
        for(size_t j = 0; j < remainingMatrix.size1() - 1; ++j)
          memset(&remainingMatrix(j+1, j), 0, sizeof(remainingMatrix(0,0)) * (remainingMatrix.size1() - j - 1));
    }

    const ublas::scalar_vector<double> sigmas(varDim, 1.0);
    conditionals->push_back(boost::make_shared<Conditional>(keys.begin()+j, keys.end(), 1, Ab_, sigmas));
    if(debug) conditionals->back()->print("Extracted conditional: ");
    Ab_.rowStart() += varDim;
    Ab_.firstBlock() += 1;
    if(debug) cout << "rowStart = " << Ab_.rowStart() << ", rowEnd = " << Ab_.rowEnd() << endl;
  }
  toc("CombineAndEliminate: 5.1 cond Rd");

  // Take lower-right block of Ab_ to get the new factor
  tic("CombineAndEliminate: 5.2 remaining factor");
  Ab_.rowEnd() = maxrank;

  // Assign the keys
  keys_.assign(keys.begin() + nrFrontals, keys.end());

  // Zero the entries below the diagonal (this relies on the matrix being
  // column-major).
  {
    ABlock remainingMatrix(Ab_.range(0, Ab_.nBlocks()));
    if(remainingMatrix.size1() > 1)
      for(size_t j = 0; j < remainingMatrix.size1() - 1; ++j)
        memset(&remainingMatrix(j+1, j), 0, sizeof(remainingMatrix(0,0)) * (remainingMatrix.size1() - j - 1));
  }

  // Make a unit diagonal noise model
  model_ = noiseModel::Unit::Create(Ab_.size1());
  if(debug) this->print("Eliminated factor: ");
  toc("CombineAndEliminate: 5.2 remaining factor");

  // todo SL: deal with "dead" pivot columns!!!
  tic("CombineAndEliminate: 5.3 rowstarts");
  size_t varpos = 0;
  firstNonzeroBlocks_.resize(numberOfRows());
  for(size_t row=0; row<numberOfRows(); ++row) {
    if(debug) cout << "row " << row << " varpos " << varpos << " Ab_.offset(varpos)=" << Ab_.offset(varpos) << " Ab_.offset(varpos+1)=" << Ab_.offset(varpos+1) << endl;
    while(varpos < keys_.size() && Ab_.offset(varpos+1) <= row)
      ++ varpos;
    firstNonzeroBlocks_[row] = varpos;
    if(debug) cout << "firstNonzeroVars_[" << row << "] = " << firstNonzeroBlocks_[row] << endl;
  }
  toc("CombineAndEliminate: 5.3 rowstarts");

  return conditionals;
}

/* ************************************************************************* */
pair<GaussianBayesNet::shared_ptr, GaussianFactor::shared_ptr> GaussianFactor::CombineAndEliminate(
        const FactorGraph<GaussianFactor>& factors, size_t nrFrontals, SolveMethod solveMethod) {

  static const bool debug = false;

  SolveMethod correctedSolveMethod = solveMethod;

  // Check for constrained noise models
  if(correctedSolveMethod != SOLVE_QR) {
    BOOST_FOREACH(const shared_ptr& factor, factors) {
      if(factor->model_->isConstrained()) {
        correctedSolveMethod = SOLVE_QR;
        break;
      }
    }
  }

  if(correctedSolveMethod == SOLVE_QR) {
    shared_ptr jointFactor(Combine(factors, VariableSlots(factors)));
    GaussianBayesNet::shared_ptr gbn(jointFactor->eliminate(nrFrontals, SOLVE_QR));
    return make_pair(gbn, jointFactor);
  } else if(correctedSolveMethod == SOLVE_CHOLESKY) {

    // Find the scatter and variable dimensions
    tic("CombineAndEliminate: 1 find scatter");
    Scatter scatter(findScatterAndDims(factors));
    toc("CombineAndEliminate: 1 find scatter");

    // Pull out keys and dimensions
    tic("CombineAndEliminate: 2 keys");
    vector<Index> keys(scatter.size());
    vector<size_t> dimensions(scatter.size() + 1);
    BOOST_FOREACH(const Scatter::value_type& var_slot, scatter) {
      keys[var_slot.second.slot] = var_slot.first;
      dimensions[var_slot.second.slot] = var_slot.second.dimension;
    }
    // This is for the r.h.s. vector
    dimensions.back() = 1;
    toc("CombineAndEliminate: 2 keys");

    // Form Ab' * Ab
    tic("CombineAndEliminate: 3 Ab'*Ab");
    MatrixColMajor ATA(formAbTAb(factors, scatter));
    if(debug) gtsam::print(ATA, "Ab' * Ab: ");
    toc("CombineAndEliminate: 3 Ab'*Ab");

    // Do Cholesky, note that after this, the lower triangle still contains
    // some untouched non-zeros that should be zero.  We zero them while
    // extracting submatrices next.
    tic("CombineAndEliminate: 4 Cholesky careful");
    size_t maxrank = choleskyCareful(ATA);
    if(maxrank > ATA.size2() - 1)
      maxrank = ATA.size2() - 1;
    if(debug) {
      gtsam::print(ATA, "chol(Ab' * Ab): ");
      cout << "maxrank = " << maxrank << endl;
    }
    toc("CombineAndEliminate: 4 Cholesky careful");

    // Create the remaining factor and swap in the matrix and block structure.
    // We declare a reference Ab to the block matrix in the remaining factor to
    // refer to below.
    GaussianFactor::shared_ptr remainingFactor(new GaussianFactor());
    BlockAb& Ab(remainingFactor->Ab_);
    {
      BlockAb newAb(ATA, dimensions.begin(), dimensions.end());
      newAb.rowEnd() = maxrank;
      newAb.swap(Ab);
    }

    // Extract conditionals and fill in details of the remaining factor
    tic("CombineAndEliminate: 5 Split");
    GaussianBayesNet::shared_ptr conditionals(remainingFactor->splitEliminatedFactor(nrFrontals, keys));
    if(debug) {
      conditionals->print("Extracted conditionals: ");
      remainingFactor->print("Eliminated factor: ");
    }
    toc("CombineAndEliminate: 5 Split");

    return make_pair(conditionals, remainingFactor);

  } else {
    assert(false);
    return make_pair(GaussianBayesNet::shared_ptr(), shared_ptr());
  }
}

/* ************************************************************************* */
GaussianConditional::shared_ptr GaussianFactor::eliminateFirst(SolveMethod solveMethod) {

  assert(Ab_.rowStart() == 0 && Ab_.rowEnd() == matrix_.size1() && Ab_.firstBlock() == 0);
  assert(!keys_.empty());
  assertInvariants();

  static const bool debug = false;

  tic("eliminateFirst");

  if(debug) print("Eliminating GaussianFactor: ");

  tic("eliminateFirst: stairs");
  // Translate the left-most nonzero column indices into top-most zero row indices
  vector<int> firstZeroRows(Ab_.size2());
  {
    size_t lastNonzeroRow = 0;
    vector<int>::iterator firstZeroRowsIt = firstZeroRows.begin();
    for(size_t var=0; var<keys().size(); ++var) {
      while(lastNonzeroRow < this->numberOfRows() && firstNonzeroBlocks_[lastNonzeroRow] <= var)
        ++ lastNonzeroRow;
      fill(firstZeroRowsIt, firstZeroRowsIt+Ab_(var).size2(), lastNonzeroRow);
      firstZeroRowsIt += Ab_(var).size2();
    }
    assert(firstZeroRowsIt+1 == firstZeroRows.end());
    *firstZeroRowsIt = this->numberOfRows();
  }
  toc("eliminateFirst: stairs");

#ifndef NDEBUG
  for(size_t col=0; col<Ab_.size2(); ++col) {
    if(debug) cout << "Staircase[" << col << "] = " << firstZeroRows[col] << endl;
    if(col != 0) assert(firstZeroRows[col] >= firstZeroRows[col-1]);
    assert(firstZeroRows[col] <= (long)this->numberOfRows());
  }
#endif

  if(debug) gtsam::print(matrix_, "Augmented Ab: ");

  size_t firstVarDim = Ab_(0).size2();

  // Cholesky currently does not work with a constrained noise model
  SolveMethod correctedSolveMethod;
  if(model_->isConstrained())
    correctedSolveMethod = SOLVE_QR;
  else
    correctedSolveMethod = solveMethod;

  // Use in-place QR or Cholesky on dense Ab appropriate to NoiseModel
  SharedDiagonal noiseModel;
  if(correctedSolveMethod == SOLVE_QR) {
    tic("eliminateFirst: QR");
    noiseModel = model_->QRColumnWise(matrix_, firstZeroRows);
    toc("eliminateFirst: QR");
  } else if(correctedSolveMethod == SOLVE_CHOLESKY) {
    tic("eliminateFirst: Cholesky");
    noiseModel = model_->Cholesky(matrix_, firstVarDim);
    Ab_.rowEnd() = noiseModel->dim();
    toc("eliminateFirst: Cholesky");
  } else
    assert(false);

  if(matrix_.size1() > 0) {
    for(size_t j=0; j<matrix_.size2(); ++j)
      for(size_t i=j+1; i<noiseModel->dim(); ++i)
        matrix_(i,j) = 0.0;
  }

  if(debug) gtsam::print(matrix_, "QR result: ");

  // Check for singular factor
  if(noiseModel->dim() < firstVarDim) {
    throw domain_error((boost::format(
        "GaussianFactor is singular in variable %1%, discovered while attempting\n"
        "to eliminate this variable.") % keys_.front()).str());
  }

  // Extract conditional
  // todo SL: are we pulling Householder vectors into the conditional and does it matter?
  tic("eliminateFirst: cond Rd");
  // Temporarily restrict the matrix view to the conditional blocks of the
  // eliminated Ab matrix to create the GaussianConditional from it.
  Ab_.rowStart() = 0;
  Ab_.rowEnd() = firstVarDim;
  Ab_.firstBlock() = 0;
  const ublas::vector_range<const Vector> sigmas(noiseModel->sigmas(), ublas::range(0, firstVarDim));
  GaussianConditional::shared_ptr conditional(new GaussianConditional(
      keys_.begin(), keys_.end(), 1, Ab_, sigmas));
  toc("eliminateFirst: cond Rd");

  if(debug) conditional->print("Extracted conditional: ");

  tic("eliminateFirst: remaining factor");
  // Take lower-right block of Ab to get the new factor
  Ab_.rowStart() = firstVarDim;
  Ab_.rowEnd() = noiseModel->dim();
  Ab_.firstBlock() = 1;
  keys_.erase(keys_.begin());
  // Set sigmas with the right model
  if (noiseModel->isConstrained())
    model_ = noiseModel::Constrained::MixedSigmas(sub(noiseModel->sigmas(), firstVarDim, noiseModel->dim()));
  else
    model_ = noiseModel::Diagonal::Sigmas(sub(noiseModel->sigmas(), firstVarDim, noiseModel->dim()));
  if(correctedSolveMethod == SOLVE_QR) assert(Ab_.size1() <= Ab_.size2()-1);
  toc("eliminateFirst: remaining factor");

  // todo SL: deal with "dead" pivot columns!!!
  tic("eliminateFirst: rowstarts");
  size_t varpos = 0;
  firstNonzeroBlocks_.resize(this->numberOfRows());
  for(size_t row=0; row<numberOfRows(); ++row) {
    while(varpos < this->keys_.size() && Ab_.offset(varpos+1) <= row)
      ++ varpos;
    firstNonzeroBlocks_[row] = varpos;
    if(debug) cout << "firstNonzeroVars_[" << row << "] = " << firstNonzeroBlocks_[row] << endl;
    //    if(debug && varpos < size()) {
    //      ABlock block(Ab_(varpos));
    //      assert(!gtsam::equal_with_abs_tol(ublas::row(block, row), zero(block.size2()), 1e-5));
    //    }
  }
  toc("eliminateFirst: rowstarts");

  if(debug) print("Eliminated factor: ");

  toc("eliminateFirst");

  assertInvariants();

  return conditional;
}

/* ************************************************************************* */
GaussianBayesNet::shared_ptr GaussianFactor::eliminate(size_t nrFrontals, SolveMethod solveMethod) {

  assert(Ab_.rowStart() == 0 && Ab_.rowEnd() == matrix_.size1() && Ab_.firstBlock() == 0);
  assert(keys_.size() >= nrFrontals);
  assertInvariants();

  static const bool debug = false;

  tic("eliminate");

  if(debug) cout << "Eliminating " << nrFrontals << " frontal variables" << endl;
  if(debug) this->print("Eliminating GaussianFactor: ");

  tic("eliminate: stairs");
  // Translate the left-most nonzero column indices into top-most zero row indices
  vector<int> firstZeroRows(Ab_.size2());
  {
    size_t lastNonzeroRow = 0;
    vector<int>::iterator firstZeroRowsIt = firstZeroRows.begin();
    for(size_t var=0; var<keys().size(); ++var) {
      while(lastNonzeroRow < this->numberOfRows() && firstNonzeroBlocks_[lastNonzeroRow] <= var)
        ++ lastNonzeroRow;
      fill(firstZeroRowsIt, firstZeroRowsIt+Ab_(var).size2(), lastNonzeroRow);
      firstZeroRowsIt += Ab_(var).size2();
    }
    assert(firstZeroRowsIt+1 == firstZeroRows.end());
    *firstZeroRowsIt = this->numberOfRows();
  }
  toc("eliminate: stairs");

#ifndef NDEBUG
  for(size_t col=0; col<Ab_.size2(); ++col) {
    if(debug) cout << "Staircase[" << col << "] = " << firstZeroRows[col] << endl;
    if(col != 0) assert(firstZeroRows[col] >= firstZeroRows[col-1]);
    assert(firstZeroRows[col] <= (long)this->numberOfRows());
  }
#endif

  if(debug) gtsam::print(matrix_, "Augmented Ab: ");

  size_t frontalDim = Ab_.range(0,nrFrontals).size2();

  if(debug) cout << "frontalDim = " << frontalDim << endl;

  // Cholesky currently does not work with a constrained noise model
  SolveMethod correctedSolveMethod;
  if(model_->isConstrained())
    correctedSolveMethod = SOLVE_QR;
  else
    correctedSolveMethod = solveMethod;

  // Use in-place QR or Cholesky on dense Ab appropriate to NoiseModel
  SharedDiagonal noiseModel;
  if(correctedSolveMethod == SOLVE_QR) {
    tic("eliminateFirst: QR");
    noiseModel = model_->QRColumnWise(matrix_, firstZeroRows);
    toc("eliminateFirst: QR");
  } else if(correctedSolveMethod == SOLVE_CHOLESKY) {
    tic("eliminateFirst: Cholesky");
    noiseModel = model_->Cholesky(matrix_, frontalDim);
    Ab_.rowEnd() = noiseModel->dim();
    toc("eliminateFirst: Cholesky");
  } else
    assert(false);

  // Zero the lower-left triangle.  todo: not all of these entries actually
  // need to be zeroed if we are careful to start copying rows after the last
  // structural zero.
  if(matrix_.size1() > 0) {
    for(size_t j=0; j<matrix_.size2(); ++j)
      for(size_t i=j+1; i<noiseModel->dim(); ++i)
        matrix_(i,j) = 0.0;
  }

  if(debug) gtsam::print(matrix_, "QR result: ");

  // Check for singular factor
  if(noiseModel->dim() < frontalDim) {
    throw domain_error((boost::format(
        "GaussianFactor is singular in variable %1%, discovered while attempting\n"
        "to eliminate this variable.") % keys_.front()).str());
  }

  // Extract conditionals
  tic("eliminate: cond Rd");
  GaussianBayesNet::shared_ptr conditionals(new GaussianBayesNet());
  for(size_t j=0; j<nrFrontals; ++j) {
    // Temporarily restrict the matrix view to the conditional blocks of the
    // eliminated Ab matrix to create the GaussianConditional from it.
    size_t varDim = Ab_(0).size2();
    Ab_.rowEnd() = Ab_.rowStart() + varDim;
    const ublas::vector_range<const Vector> sigmas(noiseModel->sigmas(), ublas::range(Ab_.rowStart(), Ab_.rowEnd()));
    conditionals->push_back(boost::make_shared<Conditional>(keys_.begin()+j, keys_.end(), 1, Ab_, sigmas));
    if(debug) conditionals->back()->print("Extracted conditional: ");
    Ab_.rowStart() += varDim;
    Ab_.firstBlock() += 1;
  }
  toc("eliminate: cond Rd");

  if(debug) conditionals->print("Extracted conditionals: ");

  tic("eliminate: remaining factor");
  // Take lower-right block of Ab to get the new factor
  Ab_.rowEnd() = noiseModel->dim();
  keys_.assign(keys_.begin() + nrFrontals, keys_.end());
  // Set sigmas with the right model
  if (noiseModel->isConstrained())
    model_ = noiseModel::Constrained::MixedSigmas(sub(noiseModel->sigmas(), frontalDim, noiseModel->dim()));
  else
    model_ = noiseModel::Diagonal::Sigmas(sub(noiseModel->sigmas(), frontalDim, noiseModel->dim()));
  if(debug) this->print("Eliminated factor: ");
  if(correctedSolveMethod == SOLVE_QR) assert(Ab_.size1() <= Ab_.size2()-1);
  toc("eliminate: remaining factor");

  // todo SL: deal with "dead" pivot columns!!!
  tic("eliminate: rowstarts");
  size_t varpos = 0;
  firstNonzeroBlocks_.resize(this->numberOfRows());
  for(size_t row=0; row<numberOfRows(); ++row) {
    if(debug) cout << "row " << row << " varpos " << varpos << " Ab_.offset(varpos)=" << Ab_.offset(varpos) << " Ab_.offset(varpos+1)=" << Ab_.offset(varpos+1) << endl;
    while(varpos < this->keys_.size() && Ab_.offset(varpos+1) <= row)
      ++ varpos;
    firstNonzeroBlocks_[row] = varpos;
    if(debug) cout << "firstNonzeroVars_[" << row << "] = " << firstNonzeroBlocks_[row] << endl;
  }
  toc("eliminate: rowstarts");

  if(debug) print("Eliminated factor: ");

  toc("eliminate");

  assertInvariants();

  return conditionals;

}

/* ************************************************************************* */
/* Used internally by GaussianFactor::Combine for sorting */
struct _RowSource {
  size_t firstNonzeroVar;
  size_t factorI;
  size_t factorRowI;
  _RowSource(size_t _firstNonzeroVar, size_t _factorI, size_t _factorRowI) :
    firstNonzeroVar(_firstNonzeroVar), factorI(_factorI), factorRowI(_factorRowI) {}
  bool operator<(const _RowSource& o) const { return firstNonzeroVar < o.firstNonzeroVar; }
};

/* ************************************************************************* */
// Helper functions for Combine
static boost::tuple<vector<size_t>, size_t, size_t> countDims(const std::vector<GaussianFactor::shared_ptr>& factors, const VariableSlots& variableSlots) {
#ifndef NDEBUG
  vector<size_t> varDims(variableSlots.size(), numeric_limits<size_t>::max());
#else
  vector<size_t> varDims(variableSlots.size());
#endif
  size_t m = 0;
  size_t n = 0;
  {
    Index jointVarpos = 0;
    BOOST_FOREACH(const VariableSlots::value_type& slots, variableSlots) {

      assert(slots.second.size() == factors.size());

      Index sourceFactorI = 0;
      BOOST_FOREACH(const Index sourceVarpos, slots.second) {
        if(sourceVarpos < numeric_limits<Index>::max()) {
          const GaussianFactor& sourceFactor = *factors[sourceFactorI];
          size_t vardim = sourceFactor.getDim(sourceFactor.begin() + sourceVarpos);
#ifndef NDEBUG
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
    BOOST_FOREACH(const GaussianFactor::shared_ptr& factor, factors) {
      m += factor->numberOfRows();
    }
  }
  return boost::make_tuple(varDims, m, n);
}

/* ************************************************************************* */
GaussianFactor::shared_ptr GaussianFactor::Combine(const FactorGraph<GaussianFactor>& factors, const VariableSlots& variableSlots) {

  static const bool verbose = false;
  static const bool debug = false;
  if (verbose) std::cout << "GaussianFactor::GaussianFactor (factors)" << std::endl;

  if(debug) factors.print("Combining factors: ");

  if(debug) variableSlots.print();

  // Determine dimensions
  tic("Combine 1: countDims");
  vector<size_t> varDims;
  size_t m;
  size_t n;
  boost::tie(varDims, m, n) = countDims(factors, variableSlots);
  if(debug) {
    cout << "Dims: " << m << " x " << n << "\n";
    BOOST_FOREACH(const size_t dim, varDims) { cout << dim << " "; }
    cout << endl;
  }
  toc("Combine 1: countDims");

  // Sort rows
  tic("Combine 2: sort rows");
  vector<_RowSource> rowSources; rowSources.reserve(m);
  bool anyConstrained = false;
  for(size_t sourceFactorI = 0; sourceFactorI < factors.size(); ++sourceFactorI) {
    const GaussianFactor& sourceFactor(*factors[sourceFactorI]);
    for(size_t sourceFactorRow = 0; sourceFactorRow < sourceFactor.numberOfRows(); ++sourceFactorRow) {
      Index firstNonzeroVar;
      firstNonzeroVar = sourceFactor.keys_[sourceFactor.firstNonzeroBlocks_[sourceFactorRow]];
      rowSources.push_back(_RowSource(firstNonzeroVar, sourceFactorI, sourceFactorRow));
    }
    if(sourceFactor.model_->isConstrained()) anyConstrained = true;
  }
  assert(rowSources.size() == m);
  std::sort(rowSources.begin(), rowSources.end());
  toc("Combine 2: sort rows");

  // Allocate new factor
  tic("Combine 3: allocate");
  shared_ptr combined(new GaussianFactor());
  combined->keys_.resize(variableSlots.size());
  std::transform(variableSlots.begin(), variableSlots.end(), combined->keys_.begin(), bind(&VariableSlots::const_iterator::value_type::first, boost::lambda::_1));
  varDims.push_back(1);
  combined->Ab_.copyStructureFrom(BlockAb(combined->matrix_, varDims.begin(), varDims.end(), m));
  combined->firstNonzeroBlocks_.resize(m);
  Vector sigmas(m);
  toc("Combine 3: allocate");

  // Copy rows
  tic("Combine 4: copy rows");
  Index combinedSlot = 0;
  BOOST_FOREACH(const VariableSlots::value_type& varslot, variableSlots) {
    for(size_t row = 0; row < m; ++row) {
      const Index sourceSlot = varslot.second[rowSources[row].factorI];
      ABlock combinedBlock(combined->Ab_(combinedSlot));
      if(sourceSlot != numeric_limits<Index>::max()) {
        const GaussianFactor& source(*factors[rowSources[row].factorI]);
        const size_t sourceRow = rowSources[row].factorRowI;
        if(source.firstNonzeroBlocks_[sourceRow] <= sourceSlot) {
          const constABlock sourceBlock(source.Ab_(sourceSlot));
          ublas::noalias(ublas::row(combinedBlock, row)) = ublas::row(sourceBlock, sourceRow);
        } else
          ublas::noalias(ublas::row(combinedBlock, row)) = ublas::zero_vector<double>(combinedBlock.size2());
      } else
        ublas::noalias(ublas::row(combinedBlock, row)) = ublas::zero_vector<double>(combinedBlock.size2());
    }
    ++ combinedSlot;
  }
  toc("Combine 4: copy rows");

  // Copy rhs (b), sigma, and firstNonzeroBlocks
  tic("Combine 5: copy vectors");
  Index firstNonzeroSlot = 0;
  for(size_t row = 0; row < m; ++row) {
    const GaussianFactor& source(*factors[rowSources[row].factorI]);
    const size_t sourceRow = rowSources[row].factorRowI;
    combined->getb()(row) = source.getb()(sourceRow);
    sigmas(row) = source.get_sigmas()(sourceRow);
    while(firstNonzeroSlot < variableSlots.size() && rowSources[row].firstNonzeroVar > combined->keys_[firstNonzeroSlot])
      ++ firstNonzeroSlot;
    combined->firstNonzeroBlocks_[row] = firstNonzeroSlot;
  }
  toc("Combine 5: copy vectors");

  // Create noise model from sigmas
  tic("Combine 6: noise model");
  if(anyConstrained) combined->model_ = noiseModel::Constrained::MixedSigmas(sigmas);
  else combined->model_ = noiseModel::Diagonal::Sigmas(sigmas);
  toc("Combine 6: noise model");

  combined->assertInvariants();

  return combined;
}

}
