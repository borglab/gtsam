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

#include <gtsam/base/timing.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/linear/GaussianConditional.h>
#include <gtsam/linear/GaussianFactor.h>
#include <gtsam/linear/GaussianFactorGraph.h>

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
  Ab_.copyStructureFrom(ab_type(matrix_, dims, dims+1, b_in.size()));
  getb() = b_in;
  assertInvariants();
}

/* ************************************************************************* */
GaussianFactor::GaussianFactor(Index i1, const Matrix& A1,
    const Vector& b, const SharedDiagonal& model) :
		IndexFactor(i1), model_(model), firstNonzeroBlocks_(b.size(), 0), Ab_(matrix_) {
  size_t dims[] = { A1.size2(), 1};
  Ab_.copyStructureFrom(ab_type(matrix_, dims, dims+2, b.size()));
	Ab_(0) = A1;
	getb() = b;
	assertInvariants();
}

/* ************************************************************************* */
GaussianFactor::GaussianFactor(Index i1, const Matrix& A1, Index i2, const Matrix& A2,
		const Vector& b, const SharedDiagonal& model) :
		IndexFactor(i1,i2), model_(model), firstNonzeroBlocks_(b.size(), 0), Ab_(matrix_) {
  size_t dims[] = { A1.size2(), A2.size2(), 1};
  Ab_.copyStructureFrom(ab_type(matrix_, dims, dims+3, b.size()));
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
  Ab_.copyStructureFrom(ab_type(matrix_, dims, dims+4, b.size()));
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
  Ab_.copyStructureFrom(ab_type(matrix_, dims, dims+terms.size()+1, b.size()));
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
  Ab_.copyStructureFrom(ab_type(matrix_, dims, dims+terms.size()+1, b.size()));
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

  ab_type::const_block_type Ab1(Ab_.range(0, Ab_.nBlocks()));
  ab_type::const_block_type Ab2(f.Ab_.range(0, f.Ab_.nBlocks()));
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
  matrix_type oldMatrix;
  ab_type oldAb(oldMatrix, dimensions.begin(), dimensions.end(), Ab_.size1());
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
    ab_type::const_block_type A(Ab_(pos));
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
GaussianConditional::shared_ptr GaussianFactor::eliminateFirst() {

  assert(Ab_.rowStart() == 0 && Ab_.rowEnd() == matrix_.size1() && Ab_.firstBlock() == 0);
  assert(!keys_.empty());
  assertInvariants();

  static const bool debug = false;

  tic("eliminateFirst");

  if(debug) print("Eliminating GaussianFactor: ");

  tic("eliminateFirst: stairs");
  // Translate the left-most nonzero column indices into top-most zero row indices
  vector<long> firstZeroRows(Ab_.size2());
  {
    size_t lastNonzeroRow = 0;
    vector<long>::iterator firstZeroRowsIt = firstZeroRows.begin();
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

  // Use in-place QR on dense Ab appropriate to NoiseModel
  tic("eliminateFirst: QR");
  SharedDiagonal noiseModel = model_->QRColumnWise(matrix_, firstZeroRows);
  toc("eliminateFirst: QR");

  if(matrix_.size1() > 0) {
    for(size_t j=0; j<matrix_.size2(); ++j)
      for(size_t i=j+1; i<noiseModel->dim(); ++i)
        matrix_(i,j) = 0.0;

  }

  if(debug) gtsam::print(matrix_, "QR result: ");

  size_t firstVarDim = Ab_(0).size2();

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
  assert(Ab_.size1() <= Ab_.size2()-1);
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
  }
  toc("eliminateFirst: rowstarts");

  if(debug) print("Eliminated factor: ");

  toc("eliminateFirst");

  assertInvariants();

  return conditional;
}

/* ************************************************************************* */
GaussianBayesNet::shared_ptr GaussianFactor::eliminate(size_t nrFrontals) {

  assert(Ab_.rowStart() == 0 && Ab_.rowEnd() == matrix_.size1() && Ab_.firstBlock() == 0);
  assert(keys_.size() >= nrFrontals);
  assertInvariants();

  static const bool debug = false;

  tic("eliminate");

  if(debug) this->print("Eliminating GaussianFactor: ");

  tic("eliminate: stairs");
  // Translate the left-most nonzero column indices into top-most zero row indices
  vector<long> firstZeroRows(Ab_.size2());
  {
    size_t lastNonzeroRow = 0;
    vector<long>::iterator firstZeroRowsIt = firstZeroRows.begin();
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

  // Use in-place QR on dense Ab appropriate to NoiseModel
  tic("eliminate: QR");
  SharedDiagonal noiseModel = model_->QRColumnWise(matrix_, firstZeroRows);
  toc("eliminate: QR");

  // Zero the lower-left triangle.  todo: not all of these entries actually
  // need to be zeroed if we are careful to start copying rows after the last
  // structural zero.
  if(matrix_.size1() > 0) {
    for(size_t j=0; j<matrix_.size2(); ++j)
      for(size_t i=j+1; i<noiseModel->dim(); ++i)
        matrix_(i,j) = 0.0;
  }

  if(debug) gtsam::print(matrix_, "QR result: ");

  size_t frontalDim = Ab_.range(0,nrFrontals).size2();

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
  assert(Ab_.size1() <= Ab_.size2()-1);
  if(debug) this->print("Eliminated factor: ");
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

/* Explicit instantiations for storage types */
template GaussianFactor::shared_ptr GaussianFactor::Combine(const FactorGraph<GaussianFactor>&, const GaussianVariableIndex<VariableIndexStorage_vector>&, const vector<size_t>&, const vector<Index>&, const std::vector<std::vector<size_t> >&);
template GaussianFactor::shared_ptr GaussianFactor::Combine(const FactorGraph<GaussianFactor>&, const GaussianVariableIndex<VariableIndexStorage_deque>&, const vector<size_t>&, const vector<Index>&, const std::vector<std::vector<size_t> >&);

// Utility function to determine row count and check if any noise models are constrained
// TODO: would be nicer if this function was split and are factorgraph methods
static std::pair<size_t,bool> rowCount(const FactorGraph<GaussianFactor>& factorGraph,
		const vector<size_t>& factorIndices)
{
	static const bool debug = false;
	tic("Combine: count sizes");
	size_t m = 0;
	bool constrained = false;
	BOOST_FOREACH(const size_t i, factorIndices)
	{
		assert(factorGraph[i] != NULL);
		assert(!factorGraph[i]->keys().empty());
		m += factorGraph[i]->numberOfRows();
		if (debug) cout << "Combining factor " << i << endl;
		if (debug) factorGraph[i]->print("  :");
		if (!constrained && factorGraph[i]->isConstrained()) {
			constrained = true;
			if (debug) std::cout << "Found a constraint!" << std::endl;
		}
	}
	toc("Combine: count sizes");
	return make_pair(m,constrained);
}

// Determine row and column counts and check if any noise models are constrained
template<class STORAGE>
static vector<size_t> columnDimensions(
		const GaussianVariableIndex<STORAGE>& variableIndex,
		const vector<Index>& variables)
{
	tic("Combine: count dims");
	static const bool debug = false;
	vector<size_t> dims(variables.size() + 1);
	size_t n = 0;
	{
		size_t j = 0;
		BOOST_FOREACH(const Index& var, variables)
		{
			if (debug) cout << "Have variable " << var << endl;
			dims[j] = variableIndex.dim(var);
			n += dims[j];
			++j;
		}
		dims[j] = 1;
	}
	toc("Combine: count dims");
	return dims;
}

// To do this, we merge-sort the rows so that the column indices of the first structural
// non-zero in each row increase monotonically.
vector<_RowSource> computeRowPermutation(size_t m, const vector<size_t>& factorIndices,
		const FactorGraph<GaussianFactor>& factorGraph) {
	vector<_RowSource> rowSources;
	rowSources.reserve(m);
	size_t i1 = 0;
	BOOST_FOREACH(const size_t i2, factorIndices) {
		const GaussianFactor& factor(*factorGraph[i2]);
		size_t factorRowI = 0;
		assert(factor.get_firstNonzeroBlocks().size() == factor.numberOfRows());
		BOOST_FOREACH(const size_t factorFirstNonzeroVarpos, factor.get_firstNonzeroBlocks()) {
			Index firstNonzeroVar;
			firstNonzeroVar = factor.keys()[factorFirstNonzeroVarpos];
			rowSources.push_back(_RowSource(firstNonzeroVar, i1, factorRowI));
			++ factorRowI;
		}
		assert(factorRowI == factor.numberOfRows());
		++ i1;
	}
	assert(rowSources.size() == m);
	assert(i1 == factorIndices.size());
	sort(rowSources.begin(), rowSources.end());
	return rowSources;
}

void copyMatrices(boost::shared_ptr<GaussianFactor> combinedFactor, size_t row,
		const GaussianFactor& factor, const std::vector<std::vector<size_t> >& variablePositions,
		const size_t factorRow, const size_t factorI, const vector<Index>& variables) {
	const static bool debug = false;
    const size_t factorFirstNonzeroVarpos = factor.get_firstNonzeroBlocks()[factorRow];
	std::vector<Index>::const_iterator keyit = factor.keys().begin() + factorFirstNonzeroVarpos;
	std::vector<size_t>::const_iterator varposIt = variablePositions[factorI].begin() + factorFirstNonzeroVarpos;
	combinedFactor->set_firstNonzeroBlocks(row, *varposIt);
	if(debug) cout << "  copying starting at varpos " << *varposIt << " (variable " << variables[*varposIt] << ")" << endl;
	std::vector<Index>::const_iterator keyitend = factor.keys().end();
	while(keyit != keyitend) {
		const size_t varpos = *varposIt;
		assert(variables[varpos] == *keyit);
		GaussianFactor::ab_type::block_type retBlock(combinedFactor->getAb(varpos));
		const GaussianFactor::ab_type::const_block_type factorBlock(factor.getA(keyit));
		ublas::noalias(ublas::row(retBlock, row)) = ublas::row(factorBlock, factorRow);
		++ keyit;
		++ varposIt;
	}
}

template<class STORAGE>
GaussianFactor::shared_ptr GaussianFactor::Combine(const FactorGraph<GaussianFactor>& factorGraph,
    const GaussianVariableIndex<STORAGE>& variableIndex, const vector<size_t>& factorIndices,
    const vector<Index>& variables, const std::vector<std::vector<size_t> >& variablePositions) {

  // Debugging flags
  static const bool verbose = false;
  static const bool debug = false;
  if (verbose) std::cout << "GaussianFactor::GaussianFactor (factors)" << std::endl;
  assert(factorIndices.size() == variablePositions.size());

  // Determine row count and check if any noise models are constrained
  size_t m; bool constrained;
  boost::tie(m,constrained) = rowCount(factorGraph,factorIndices);

  // Determine column dimensions
  vector<size_t> dims = columnDimensions(variableIndex,variables);

	// Allocate return value, the combined factor, the augmented Ab matrix and other arrays
  tic("Combine: set up empty");
  shared_ptr combinedFactor(boost::make_shared<GaussianFactor>());
  combinedFactor->Ab_.copyStructureFrom(ab_type(combinedFactor->matrix_, dims.begin(), dims.end(), m));
  ublas::noalias(combinedFactor->matrix_) = ublas::zero_matrix<double>(combinedFactor->matrix_.size1(), combinedFactor->matrix_.size2());
  combinedFactor->firstNonzeroBlocks_.resize(m);
  Vector sigmas(m);

  // Copy keys
  combinedFactor->keys_.reserve(variables.size());
  combinedFactor->keys_.insert(combinedFactor->keys_.end(), variables.begin(), variables.end());
  toc("Combine: set up empty");

  // Compute a row permutation that maintains a staircase pattern in the new combined factor.
  tic("Combine: sort rows");
  vector<_RowSource> rowSources = computeRowPermutation(m, factorIndices, factorGraph);
  toc("Combine: sort rows");

  // Fill in the rows of the new factor in sorted order.  Fill in the array of
  // the left-most nonzero for each row and the first structural zero in each column.
  // todo SL: smarter ignoring of zero factor variables (store first possible like above)

  if(debug) gtsam::print(combinedFactor->matrix_, "matrix_ before copying rows: ");

  tic("Combine: copy rows");
#ifndef NDEBUG
  size_t lastRowFirstVarpos = 0;
#endif
  for(size_t row=0; row<m; ++row) {

    const _RowSource& rowSource = rowSources[row];
    assert(rowSource.factorI < factorGraph.size());
    const size_t factorI = rowSource.factorI;
    const GaussianFactor& factor(*factorGraph[factorIndices[factorI]]);
    const size_t factorRow = rowSource.factorRowI;

    if(debug)
      cout << "Combined row " << row << " is from row " << factorRow << " of factor " << factorIndices[factorI] << endl;

    // Copy rhs b and sigma
    combinedFactor->getb()(row) = factor.getb()(factorRow);
    sigmas(row) = factor.get_sigmas()(factorRow);

    // Copy the row of A variable by variable, starting at the first nonzero variable.
    copyMatrices(combinedFactor, row, factor, variablePositions, factorRow, factorI, variables);

#ifndef NDEBUG
    // Debug check, make sure the first column of nonzeros increases monotonically
    if(row != 0)
      assert(combinedFactor->firstNonzeroBlocks_[row] >= lastRowFirstVarpos);
    lastRowFirstVarpos = combinedFactor->firstNonzeroBlocks_[row];
#endif
  }
  toc("Combine: copy rows");

  if (verbose) std::cout << "GaussianFactor::GaussianFactor done" << std::endl;

  if (constrained) {
    combinedFactor->model_ = noiseModel::Constrained::MixedSigmas(sigmas);
    if (verbose) combinedFactor->model_->print("Just created Constraint ^");
  } else {
    combinedFactor->model_ = noiseModel::Diagonal::Sigmas(sigmas);
    if (verbose) combinedFactor->model_->print("Just created Diagonal");
  }

  if(debug) combinedFactor->print("Combined factor: ");

  combinedFactor->assertInvariants();

  return combinedFactor;
}

/* ************************************************************************* */
// Helper functions for Combine
boost::tuple<vector<size_t>, size_t, size_t> countDims(const std::vector<GaussianFactor::shared_ptr>& factors, const VariableSlots& variableSlots) {
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

GaussianFactor::shared_ptr GaussianFactor::Combine(const FactorGraph<GaussianFactor>& factors, const VariableSlots& variableSlots) {

  static const bool verbose = false;
  static const bool debug = false;
  if (verbose) std::cout << "GaussianFactor::GaussianFactor (factors)" << std::endl;

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
  combined->Ab_.copyStructureFrom(ab_type(combined->matrix_, varDims.begin(), varDims.end(), m));
  combined->firstNonzeroBlocks_.resize(m);
  Vector sigmas(m);
  toc("Combine 3: allocate");

  // Copy rows
  tic("Combine 4: copy rows");
  Index combinedSlot = 0;
  BOOST_FOREACH(const VariableSlots::value_type& varslot, variableSlots) {
    for(size_t row = 0; row < m; ++row) {
      const Index sourceSlot = varslot.second[rowSources[row].factorI];
      ab_type::block_type combinedBlock(combined->Ab_(combinedSlot));
      if(sourceSlot != numeric_limits<Index>::max()) {
        const GaussianFactor& source(*factors[rowSources[row].factorI]);
        const size_t sourceRow = rowSources[row].factorRowI;
        if(source.firstNonzeroBlocks_[sourceRow] <= sourceSlot) {
          const ab_type::const_block_type sourceBlock(source.Ab_(sourceSlot));
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
