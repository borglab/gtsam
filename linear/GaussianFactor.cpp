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
//using namespace boost::assign;
namespace ublas = boost::numeric::ublas;
using namespace boost::lambda;

namespace gtsam {

/* ************************************************************************* */
inline void GaussianFactor::checkSorted() const {
#ifndef NDEBUG
  // Make sure variables are sorted
  assert(keys_.size()+1 == Ab_.nBlocks());
  for(size_t varpos=0; varpos<keys_.size(); ++varpos) {
    if(varpos > 0) {
      assert(keys_[varpos] > keys_[varpos-1]);
    }
  }
#endif
}

/* ************************************************************************* */
GaussianFactor::GaussianFactor(const GaussianFactor& gf) :
    Factor(gf), model_(gf.model_), firstNonzeroBlocks_(gf.firstNonzeroBlocks_), Ab_(matrix_) {
  Ab_.assignNoalias(gf.Ab_);
}

/* ************************************************************************* */
GaussianFactor::GaussianFactor() : Ab_(matrix_) {}

/* ************************************************************************* */
GaussianFactor::GaussianFactor(const Vector& b_in) : firstNonzeroBlocks_(b_in.size(), 0), Ab_(matrix_) {
  size_t dims[] = { 1 };
  Ab_.copyStructureFrom(ab_type(matrix_, dims, dims+1, b_in.size()));
  getb() = b_in;
}

/* ************************************************************************* */
GaussianFactor::GaussianFactor(Index i1, const Matrix& A1,
    const Vector& b, const SharedDiagonal& model) :
		Factor(i1), model_(model), firstNonzeroBlocks_(b.size(), 0), Ab_(matrix_) {
  size_t dims[] = { A1.size2(), 1};
  Ab_.copyStructureFrom(ab_type(matrix_, dims, dims+2, b.size()));
	Ab_(0) = A1;
	getb() = b;
}

/* ************************************************************************* */
GaussianFactor::GaussianFactor(Index i1, const Matrix& A1, Index i2, const Matrix& A2,
		const Vector& b, const SharedDiagonal& model) :
		Factor(i1,i2), model_(model), firstNonzeroBlocks_(b.size(), 0), Ab_(matrix_) {
  size_t dims[] = { A1.size2(), A2.size2(), 1};
  Ab_.copyStructureFrom(ab_type(matrix_, dims, dims+3, b.size()));
  Ab_(0) = A1;
  Ab_(1) = A2;
  getb() = b;
}

/* ************************************************************************* */
GaussianFactor::GaussianFactor(Index i1, const Matrix& A1, Index i2, const Matrix& A2,
    Index i3, const Matrix& A3,	const Vector& b, const SharedDiagonal& model) :
    Factor(i1,i2,i3), model_(model), firstNonzeroBlocks_(b.size(), 0), Ab_(matrix_) {
  size_t dims[] = { A1.size2(), A2.size2(), A3.size2(), 1};
  Ab_.copyStructureFrom(ab_type(matrix_, dims, dims+4, b.size()));
  Ab_(0) = A1;
  Ab_(1) = A2;
  Ab_(2) = A3;
  getb() = b;
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
  Factor::permuted_ = false;
  checkSorted();
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
  Factor::permuted_ = false;
  checkSorted();
}

/* ************************************************************************* */
GaussianFactor::GaussianFactor(const GaussianConditional& cg) : Factor(cg), model_(noiseModel::Diagonal::Sigmas(cg.get_sigmas(), true)), Ab_(matrix_) {
  Ab_.assignNoalias(cg.rsd_);
  // todo SL: make firstNonzeroCols triangular?
  firstNonzeroBlocks_.resize(cg.get_d().size(), 0);	// set sigmas from precisions
}

///* ************************************************************************* */
//GaussianFactor::GaussianFactor(const vector<shared_ptr> & factors)
//{
//	bool verbose = false;
//	if (verbose) cout << "GaussianFactor::GaussianFactor (factors)" << endl;
//
//	// Create RHS and sigmas of right size by adding together row counts
//  size_t m = 0;
//  BOOST_FOREACH(const shared_ptr& factor, factors) m += factor->numberOfRows();
//  b_ = Vector(m);
//  Vector sigmas(m);
//
//  size_t pos = 0; // save last position inserted into the new rhs vector
//
//  // iterate over all factors
//  bool constrained = false;
//  BOOST_FOREACH(const shared_ptr& factor, factors){
//  	if (verbose) factor->print();
//    // number of rows for factor f
//    const size_t mf = factor->numberOfRows();
//
//    // copy the rhs vector from factor to b
//    const Vector bf = factor->get_b();
//    for (size_t i=0; i<mf; i++) b_(pos+i) = bf(i);
//
//    // copy the model_
//    for (size_t i=0; i<mf; i++) sigmas(pos+i) = factor->model_->sigma(i);
//
//    // update the matrices
//    append_factor(factor,m,pos);
//
//    // check if there are constraints
//    if (verbose) factor->model_->print("Checking for zeros");
//    if (!constrained && factor->model_->isConstrained()) {
//    	constrained = true;
//    	if (verbose) cout << "Found a constraint!" << endl;
//    }
//
//    pos += mf;
//  }
//
//  if (verbose) cout << "GaussianFactor::GaussianFactor done" << endl;
//
//  if (constrained) {
//	  model_ = noiseModel::Constrained::MixedSigmas(sigmas);
//	  if (verbose) model_->print("Just created Constraint ^");
//  } else {
//	  model_ = noiseModel::Diagonal::Sigmas(sigmas);
//	  if (verbose) model_->print("Just created Diagonal");
//  }
//}

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

///* ************************************************************************* */
//size_t GaussianFactor::getDim(Index key) const {
//	const_iterator it = findA(key);
//	if (it != end())
//		return it->second.size2();
//	else
//		return 0;
//}

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
  this->permuted_.value = true;
  BOOST_FOREACH(Index& key, keys_) { key = inversePermutation[key]; }
  // Since we're permuting the variables, ensure that entire rows from this
  // factor are copied when Combine is called
  BOOST_FOREACH(size_t& varpos, firstNonzeroBlocks_) { varpos = 0; }
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

///* ************************************************************************* */
//Dimensions GaussianFactor::dimensions() const {
//  Dimensions result;
//  BOOST_FOREACH(const NamedMatrix& jA, As_)
//		result.insert(std::pair<Index,size_t>(jA.first,jA.second.size2()));
//  return result;
//}
//
///* ************************************************************************* */
//void GaussianFactor::tally_separator(Index key, set<Index>& separator) const {
//  if(involves(key)) {
//    BOOST_FOREACH(const NamedMatrix& jA, As_)
//      if(jA.first != key) separator.insert(jA.first);
//  }
//}

/* ************************************************************************* */
Vector GaussianFactor::operator*(const VectorValues& x) const {
	Vector Ax = zero(Ab_.size1());
  if (empty()) return Ax;

  // Just iterate over all A matrices and multiply in correct config part
  for(size_t pos=0; pos<keys_.size(); ++pos)
    Ax += ublas::prod(Ab_(pos), x[keys_[pos]]);
//  BOOST_FOREACH(const NamedMatrix& jA, As_)
//    Ax += (jA.second * x[jA.first]);

  return model_->whiten(Ax);
}

///* ************************************************************************* */
//VectorValues GaussianFactor::operator^(const Vector& e) const {
//  Vector E = model_->whiten(e);
//	VectorValues x;
//  // Just iterate over all A matrices and insert Ai^e into VectorValues
//  for(size_t pos=0; pos<keys_.size(); ++pos)
//    x.insert(keys_[pos], ARange(pos)^E);
////  BOOST_FOREACH(const NamedMatrix& jA, As_)
////    x.insert(jA.first,jA.second^E);
//	return x;
//}

/* ************************************************************************* */
void GaussianFactor::transposeMultiplyAdd(double alpha, const Vector& e,
		VectorValues& x) const {
	Vector E = alpha * model_->whiten(e);
	// Just iterate over all A matrices and insert Ai^e into VectorValues
  for(size_t pos=0; pos<keys_.size(); ++pos)
    gtsam::transposeMultiplyAdd(1.0, Ab_(pos), E, x[keys_[pos]]);
//	BOOST_FOREACH(const NamedMatrix& jA, As_)
//		gtsam::transposeMultiplyAdd(1.0, jA.second, E, x[jA.first]);
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
//	// get pointers to the matrices
//	vector<const Matrix *> matrices;
//	BOOST_FOREACH(Index j, ordering) {
//		const Matrix& Aj = get_A(j);
//		matrices.push_back(&Aj);
//	}
//
//	// load b into a matrix
//	size_t rows = b_.size();
//	Matrix B_mat(rows, 1);
//	memcpy(B_mat.data().begin(), b_.data().begin(), rows*sizeof(double));
//	matrices.push_back(&B_mat);

	// divide in sigma so error is indeed 0.5*|Ax-b|
//	Matrix Ab = collect(matrices);
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

///* ************************************************************************* */
//void GaussianFactor::append_factor(GaussianFactor::shared_ptr f, size_t m, size_t pos) {
//
//	// iterate over all matrices from the factor f
//	BOOST_FOREACH(const NamedMatrix& p, f->As_) {
//		Index key = p.first;
//		const Matrix& Aj = p.second;
//
//		// find the corresponding matrix among As
//		iterator mine = findA(key);
//		const bool exists = (mine != end());
//
//		// find rows and columns
//		const size_t n = Aj.size2();
//
//		// use existing or create new matrix
//		if (exists)
//		  copy(Aj.data().begin(), Aj.data().end(), (mine->second).data().begin()+pos*n);
//		else {
//			Matrix Z = zeros(m, n);
//			copy(Aj.data().begin(), Aj.data().end(), Z.data().begin()+pos*n);
//			insert(key, Z);
//		}
//
//	} // FOREACH
//}

/* ************************************************************************* */
GaussianConditional::shared_ptr GaussianFactor::eliminateFirst() {

  assert(Ab_.rowStart() == 0 && Ab_.rowEnd() == matrix_.size1() && Ab_.firstBlock() == 0);
  assert(!permuted_.value);
  assert(!keys_.empty());
  checkSorted();

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
//    ublas::triangular_adaptor<matrix_type, ublas::strict_lower> AbLower(Ab_);
//    fill(AbLower.begin1(), AbLower.end1(), 0.0);
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

  checkSorted();

  if(debug) print("Eliminated factor: ");

  toc("eliminateFirst");

  return conditional;
}

/* ************************************************************************* */
GaussianBayesNet::shared_ptr GaussianFactor::eliminate(size_t nrFrontals) {

  assert(Ab_.rowStart() == 0 && Ab_.rowEnd() == matrix_.size1() && Ab_.firstBlock() == 0);
  assert(!permuted_.value);
  assert(keys_.size() >= nrFrontals);
  checkSorted();

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
//    ublas::triangular_adaptor<matrix_type, ublas::strict_lower> AbLower(Ab_);
//    fill(AbLower.begin1(), AbLower.end1(), 0.0);
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

  checkSorted();

  if(debug) print("Eliminated factor: ");

  toc("eliminate");

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
template GaussianFactor::shared_ptr GaussianFactor::Combine(const GaussianFactorGraph&, const GaussianVariableIndex<VariableIndexStorage_vector>&, const vector<size_t>&, const vector<Index>&, const std::vector<std::vector<size_t> >&);
template GaussianFactor::shared_ptr GaussianFactor::Combine(const GaussianFactorGraph&, const GaussianVariableIndex<VariableIndexStorage_deque>&, const vector<size_t>&, const vector<Index>&, const std::vector<std::vector<size_t> >&);

template<class Storage>
GaussianFactor::shared_ptr GaussianFactor::Combine(const GaussianFactorGraph& factorGraph,
    const GaussianVariableIndex<Storage>& variableIndex, const vector<size_t>& factors,
    const vector<Index>& variables, const std::vector<std::vector<size_t> >& variablePositions) {

  shared_ptr ret(boost::make_shared<GaussianFactor>());
  static const bool verbose = false;
  static const bool debug = false;
  if (verbose) std::cout << "GaussianFactor::GaussianFactor (factors)" << std::endl;

  assert(factors.size() == variablePositions.size());

  // Determine row and column counts and check if any noise models are constrained
  tic("Combine: count sizes");
  size_t m = 0;
  bool constrained = false;
  BOOST_FOREACH(const size_t factor, factors) {
    assert(factorGraph[factor] != NULL);
    assert(!factorGraph[factor]->keys_.empty());
    m += factorGraph[factor]->numberOfRows();
    if(debug) cout << "Combining factor " << factor << endl;
    if(debug) factorGraph[factor]->print("  :");
    if (!constrained && factorGraph[factor]->model_->isConstrained()) {
      constrained = true;
      if (debug) std::cout << "Found a constraint!" << std::endl;
    }
  }
  size_t dims[variables.size()+1];
  size_t n = 0;
  {
    size_t j=0;
    BOOST_FOREACH(const Index& var, variables) {
      if(debug) cout << "Have variable " << var << endl;
      dims[j] = variableIndex.dim(var);
      n += dims[j];
      ++ j;
    }
    dims[j] = 1;
  }
  toc("Combine: count sizes");

  tic("Combine: set up empty");
  // Allocate augmented Ab matrix and other arrays
  ret->Ab_.copyStructureFrom(ab_type(ret->matrix_, dims, dims+variables.size()+1, m));
  ublas::noalias(ret->matrix_) = ublas::zero_matrix<double>(ret->matrix_.size1(), ret->matrix_.size2());
  ret->firstNonzeroBlocks_.resize(m);
  Vector sigmas(m);

  // Copy keys
  ret->keys_.reserve(variables.size());
  ret->keys_.insert(ret->keys_.end(), variables.begin(), variables.end());
  toc("Combine: set up empty");

  // Compute a row permutation that maintains a staircase pattern in the new
  // combined factor.  To do this, we merge-sort the rows so that the column
  // indices of the first structural non-zero in each row increases
  // monotonically.
  tic("Combine: sort rows");
  vector<_RowSource> rowSources;
  rowSources.reserve(m);
  size_t factorI = 0;
  BOOST_FOREACH(const size_t factorII, factors) {
    const GaussianFactor& factor(*factorGraph[factorII]);
    size_t factorRowI = 0;
    assert(factor.firstNonzeroBlocks_.size() == factor.numberOfRows());
    BOOST_FOREACH(const size_t factorFirstNonzeroVarpos, factor.firstNonzeroBlocks_) {
      Index firstNonzeroVar;
      if(factor.permuted_.value == true)
        firstNonzeroVar = *std::min_element(factor.keys_.begin(), factor.keys_.end());
      else
        firstNonzeroVar = factor.keys_[factorFirstNonzeroVarpos];
      rowSources.push_back(_RowSource(firstNonzeroVar, factorI, factorRowI));
      ++ factorRowI;
    }
    assert(factorRowI == factor.numberOfRows());
    ++ factorI;
  }
  assert(rowSources.size() == m);
  assert(factorI == factors.size());
  sort(rowSources.begin(), rowSources.end());
  toc("Combine: sort rows");

  // Fill in the rows of the new factor in sorted order.  Fill in the array of
  // the left-most nonzero for each row and the first structural zero in each
  // column.
  // todo SL: smarter ignoring of zero factor variables (store first possible like above)

  if(debug) gtsam::print(ret->matrix_, "matrix_ before copying rows: ");

  tic("Combine: copy rows");
#ifndef NDEBUG
  size_t lastRowFirstVarpos;
#endif
  for(size_t row=0; row<m; ++row) {

    const _RowSource& rowSource = rowSources[row];
    assert(rowSource.factorI < factorGraph.size());
    const size_t factorI = rowSource.factorI;
    const GaussianFactor& factor(*factorGraph[factors[factorI]]);
    const size_t factorRow = rowSource.factorRowI;
    const size_t factorFirstNonzeroVarpos = factor.firstNonzeroBlocks_[factorRow];

    if(debug) {
      cout << "Combined row " << row << " is from row " << factorRow << " of factor " << factors[factorI] << endl;
    }

    // Copy rhs b and sigma
    ret->getb()(row) = factor.getb()(factorRow);
    sigmas(row) = factor.get_sigmas()(factorRow);

    // Copy the row of A variable by variable, starting at the first nonzero
    // variable.
    std::vector<Index>::const_iterator keyit = factor.keys_.begin() + factorFirstNonzeroVarpos;
    std::vector<size_t>::const_iterator varposIt = variablePositions[factorI].begin() + factorFirstNonzeroVarpos;
    ret->firstNonzeroBlocks_[row] = *varposIt;
    if(debug) cout << "  copying starting at varpos " << *varposIt << " (variable " << variables[*varposIt] << ")" << endl;
    std::vector<Index>::const_iterator keyitend = factor.keys_.end();
    while(keyit != keyitend) {
      const size_t varpos = *varposIt;
      // If the factor is permuted, the varpos's in the joint factor could be
      // out of order.
      if(factor.permuted_.value == true && varpos < ret->firstNonzeroBlocks_[row])
        ret->firstNonzeroBlocks_[row] = varpos;
      assert(variables[varpos] == *keyit);
      ab_type::block_type retBlock(ret->Ab_(varpos));
      const ab_type::const_block_type factorBlock(factor.getA(keyit));
      ublas::noalias(ublas::row(retBlock, row)) = ublas::row(factorBlock, factorRow);
      ++ keyit;
      ++ varposIt;
    }
#ifndef NDEBUG
    // Debug check, make sure the first column of nonzeros increases monotonically
    if(row != 0)
      assert(ret->firstNonzeroBlocks_[row] >= lastRowFirstVarpos);
    lastRowFirstVarpos = ret->firstNonzeroBlocks_[row];
#endif
  }
  toc("Combine: copy rows");

  if (verbose) std::cout << "GaussianFactor::GaussianFactor done" << std::endl;

  if (constrained) {
    ret->model_ = noiseModel::Constrained::MixedSigmas(sigmas);
    if (verbose) ret->model_->print("Just created Constraint ^");
  } else {
    ret->model_ = noiseModel::Diagonal::Sigmas(sigmas);
    if (verbose) ret->model_->print("Just created Diagonal");
  }

  if(debug) ret->print("Combined factor: ");

  ret->checkSorted();

  return ret;
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

GaussianFactor::shared_ptr GaussianFactor::Combine(const GaussianFactorGraph& factors, const VariableSlots& variableSlots) {

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
      if(sourceFactor.permuted_.value)
        firstNonzeroVar = *std::min_element(sourceFactor.begin(), sourceFactor.end());
      else
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
        assert(!source.permuted_.value || source.firstNonzeroBlocks_[sourceRow] == 0);
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

  combined->checkSorted();

  return combined;
}


///* ************************************************************************* */
//static GaussianFactor::shared_ptr
//GaussianFactor::Combine(const GaussianFactorGraph& factorGraph, const std::vector<size_t>& factors) {
//
//  // Determine row count
//  size_t m = 0;
//  BOOST_FOREACH(const size_t& factor, factors) {
//    m += factorGraph[factor]->numberOfRows();
//  }
//
//}

///* ************************************************************************* */
///* Note, in place !!!!
// * Do incomplete QR factorization for the first n columns
// * We will do QR on all matrices and on RHS
// * Then take first n rows and make a GaussianConditional,
// * and last rows to make a new joint linear factor on separator
// */
///* ************************************************************************* */
//
//pair<GaussianBayesNet, GaussianFactor::shared_ptr>
//GaussianFactor::eliminateMatrix(Matrix& Ab, SharedDiagonal model,
//		        const Ordering& frontals, const Ordering& separators,
//		        const Dimensions& dimensions) {
//	bool verbose = false;
//
//	// Use in-place QR on dense Ab appropriate to NoiseModel
//	if (verbose) model->print("Before QR");
//	SharedDiagonal noiseModel = model->QR(Ab);
//	if (verbose) model->print("After QR");
//
//	// get dimensions of the eliminated variable
//	// TODO: this is another map find that should be avoided !
//	size_t n1 = dimensions.at(frontals.front()), n = Ab.size2() - 1;
//
//	// Get alias to augmented RHS d
//	ublas::matrix_column<Matrix> d(Ab,n);
//
//	// extract the conditionals
//	GaussianBayesNet bn;
//	size_t n0 = 0;
//	Ordering::const_iterator itFrontal1 = frontals.begin(), itFrontal2;
//	for(; itFrontal1!=frontals.end(); itFrontal1++) {
//		n1 = n0 + dimensions.at(*itFrontal1);
//		// create base conditional Gaussian
//		GaussianConditional::shared_ptr conditional(new GaussianConditional(*itFrontal1,
//				sub(d,  n0, n1),                   // form d vector
//				sub(Ab, n0, n1, n0, n1),           // form R matrix
//				sub(noiseModel->sigmas(),n0,n1))); // get standard deviations
//
//		// add parents to the conditional
//		itFrontal2 = itFrontal1;
//		itFrontal2 ++;
//		size_t j = n1;
//		for (; itFrontal2!=frontals.end(); itFrontal2++) {
//			size_t dim = dimensions.at(*itFrontal2);
//			conditional->add(*itFrontal2, sub(Ab, n0, n1, j, j+dim));
//			j+=dim;
//		}
//		BOOST_FOREACH(Index cur_key, separators) {
//			size_t dim = dimensions.at(cur_key);
//			conditional->add(cur_key, sub(Ab, n0, n1, j, j+dim));
//			j+=dim;
//		}
//		n0 = n1;
//		bn.push_back(conditional);
//	}
//
//	// if m<n1, this factor cannot be eliminated
//	size_t maxRank = noiseModel->dim();
//	if (maxRank<n1) {
//		cout << "Perhaps your factor graph is singular." << endl;
//		cout << "Here are the keys involved in the factor now being eliminated:" << endl;
//		separators.print("Keys");
//		cout << "The first key, '" << frontals.front() << "', corresponds to the variable being eliminated" << endl;
//		throw(domain_error("GaussianFactor::eliminate: fewer constraints than unknowns"));
//	}
//
//	// extract the new factor
//	GaussianFactor::shared_ptr factor(new GaussianFactor);
//	size_t j = n1;
//	BOOST_FOREACH(Index cur_key, separators) {
//		size_t dim = dimensions.at(cur_key); // TODO avoid find !
//		factor->insert(cur_key, sub(Ab, n1, maxRank, j, j+dim)); // TODO: handle zeros properly
//		j+=dim;
//	}
//
//	// Set sigmas
//	// set the right model here
//	if (noiseModel->isConstrained())
//		factor->model_ = noiseModel::Constrained::MixedSigmas(sub(noiseModel->sigmas(),n1,maxRank));
//	else
//		factor->model_ = noiseModel::Diagonal::Sigmas(sub(noiseModel->sigmas(),n1,maxRank));
//
//	// extract ds vector for the new b
//	factor->set_b(sub(d, n1, maxRank));
//
//	return make_pair(bn, factor);
//
//}
//
///* ************************************************************************* */
//pair<GaussianConditional::shared_ptr, GaussianFactor::shared_ptr>
//GaussianFactor::eliminateMatrix(Matrix& Ab, SharedDiagonal model,
//		        Index frontal, const Ordering& separator,
//		        const Dimensions& dimensions) {
//	Ordering frontals; frontals += frontal;
//	pair<GaussianBayesNet, shared_ptr> ret =
//			eliminateMatrix(Ab, model, frontals, separator, dimensions);
//	return make_pair(*ret.first.begin(), ret.second);
//}
///* ************************************************************************* */
//pair<GaussianConditional::shared_ptr, GaussianFactor::shared_ptr>
//GaussianFactor::eliminate(Index key) const
//{
//	// if this factor does not involve key, we exit with empty CG and LF
//	const_iterator it = findA(key);
//	if (it==end()) {
//		// Conditional Gaussian is just a parent-less node with P(x)=1
//		GaussianFactor::shared_ptr lf(new GaussianFactor);
//		GaussianConditional::shared_ptr cg(new GaussianConditional(key));
//		return make_pair(cg,lf);
//	}
//
//	// create an internal ordering that eliminates key first
//	Ordering ordering;
//	ordering += key;
//	BOOST_FOREACH(Index k, keys())
//		if (k != key) ordering += k;
//
//	// extract [A b] from the combined linear factor (ensure that x is leading)
//	Matrix Ab = matrix_augmented(ordering,false);
//
//	// TODO: this is where to split
//	ordering.pop_front();
//	return eliminateMatrix(Ab, model_, key, ordering, dimensions());
//}

/* ************************************************************************* */

	string symbol(char c, int index) {
		stringstream ss;
		ss << c << index;
		return ss.str();
	}

}
/* ************************************************************************* */
;
