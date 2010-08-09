/**
 * @file    GaussianFactor.cpp
 * @brief   Linear Factor....A Gaussian
 * @brief   linearFactor
 * @author  Christian Potthast
 */

#include <boost/foreach.hpp>
#include <boost/assign/list_inserter.hpp> // for 'insert()'
#include <boost/assign/std/list.hpp> // for operator += in Ordering

#include "Matrix.h"
#include "Ordering.h"
#include "GaussianConditional.h"
#include "GaussianFactor.h"

using namespace std;
using namespace boost::assign;
namespace ublas = boost::numeric::ublas;
using namespace gtsam;

typedef pair<Symbol,Matrix> NamedMatrix;

/* ************************************************************************* */
GaussianFactor::GaussianFactor(const Vector& b_in) :
	b_(b_in) {
}

/* ************************************************************************* */
GaussianFactor::GaussianFactor(const Symbol& key1, const Matrix& A1,
		const Vector& b, const SharedDiagonal& model) :
	model_(model),b_(b) {
	As_[key1] = A1;
}

/* ************************************************************************* */
GaussianFactor::GaussianFactor(const Symbol& key1, const Matrix& A1,
		const Symbol& key2, const Matrix& A2,
		const Vector& b, const SharedDiagonal& model) :
	model_(model), b_(b)  {
	As_[key1] = A1;
	As_[key2] = A2;
}

/* ************************************************************************* */
GaussianFactor::GaussianFactor(const Symbol& key1, const Matrix& A1,
		const Symbol& key2, const Matrix& A2,
		const Symbol& key3, const Matrix& A3,
		const Vector& b, const SharedDiagonal& model) :
        model_(model),b_(b)  {
	As_[key1] = A1;
	As_[key2] = A2;
	As_[key3] = A3;
}

/* ************************************************************************* */
GaussianFactor::GaussianFactor(const std::vector<std::pair<Symbol, Matrix> > &terms,
    const Vector &b, const SharedDiagonal& model) :
	model_(model), b_(b)  {
	BOOST_FOREACH(const NamedMatrix& pair, terms)
    As_.insert(pair);
}

/* ************************************************************************* */
GaussianFactor::GaussianFactor(const std::list<std::pair<Symbol, Matrix> > &terms,
    const Vector &b, const SharedDiagonal& model) :
	model_(model), b_(b)  {
	BOOST_FOREACH(const NamedMatrix& pair, terms)
		As_.insert(pair);
}

/* ************************************************************************* */
GaussianFactor::GaussianFactor(const boost::shared_ptr<GaussianConditional>& cg) :
	b_(cg->get_d()) {
	As_.insert(NamedMatrix(cg->key(), cg->get_R()));
	SymbolMap<Matrix>::const_iterator it = cg->parentsBegin();
	for (; it != cg->parentsEnd(); it++)
		As_.insert(*it);
	// set sigmas from precisions
	model_ = noiseModel::Diagonal::Sigmas(cg->get_sigmas(), true);
}

/* ************************************************************************* */
GaussianFactor::GaussianFactor(const vector<shared_ptr> & factors)
{
	bool verbose = false;
	if (verbose) cout << "GaussianFactor::GaussianFactor (factors)" << endl;

	// Create RHS and sigmas of right size by adding together row counts
  size_t m = 0;
  BOOST_FOREACH(const shared_ptr& factor, factors) m += factor->numberOfRows();
  b_ = Vector(m);
  Vector sigmas(m);

  size_t pos = 0; // save last position inserted into the new rhs vector

  // iterate over all factors
  bool constrained = false;
  BOOST_FOREACH(const shared_ptr& factor, factors){
  	if (verbose) factor->print();
    // number of rows for factor f
    const size_t mf = factor->numberOfRows();

    // copy the rhs vector from factor to b
    const Vector bf = factor->get_b();
    for (size_t i=0; i<mf; i++) b_(pos+i) = bf(i);

    // copy the model_
    for (size_t i=0; i<mf; i++) sigmas(pos+i) = factor->model_->sigma(i);

    // update the matrices
    append_factor(factor,m,pos);

    // check if there are constraints
    if (verbose) factor->model_->print("Checking for zeros");
    if (!constrained && factor->model_->isConstrained()) {
    	constrained = true;
    	if (verbose) cout << "Found a constraint!" << endl;
    }

    pos += mf;
  }

  if (verbose) cout << "GaussianFactor::GaussianFactor done" << endl;

  if (constrained) {
	  model_ = noiseModel::Constrained::MixedSigmas(sigmas);
	  if (verbose) model_->print("Just created Constraint ^");
  } else {
	  model_ = noiseModel::Diagonal::Sigmas(sigmas);
	  if (verbose) model_->print("Just created Diagonal");
  }
}

/* ************************************************************************* */
void GaussianFactor::print(const string& s) const {
  cout << s << endl;
  if (empty()) cout << " empty" << endl; 
  else {
  	BOOST_FOREACH(const NamedMatrix& jA, As_)
  		gtsam::print(jA.second, "A["+(string)jA.first+"]=\n");
    gtsam::print(b_,"b=");
    model_->print("model");
  }
}

/* ************************************************************************* */
size_t GaussianFactor::getDim(const Symbol& key) const {
	const_iterator it = As_.find(key);
	if (it != As_.end())
		return it->second.size2();
	else
		return 0;
}

/* ************************************************************************* */
// Check if two linear factors are equal
bool GaussianFactor::equals(const Factor<VectorConfig>& f, double tol) const {
    
  const GaussianFactor* lf = dynamic_cast<const GaussianFactor*>(&f);
  if (lf == NULL) return false;

  if (empty()) return (lf->empty());

  const_iterator it1 = As_.begin(), it2 = lf->As_.begin();
  if(As_.size() != lf->As_.size()) return false;

  // check whether each row is up to a sign
  for (size_t i=0; i<b_.size(); i++) {
  	list<Vector> row1;
  	list<Vector> row2;
  	row1.push_back(Vector_(1,     b_(i)));
  	row2.push_back(Vector_(1, lf->b_(i)));

  	for(; it1 != As_.end(); it1++, it2++) {
  		const Symbol& j1 = it1->first, j2 = it2->first;
  		const Matrix A1 = it1->second, A2 = it2->second;
  		if (j1 != j2) return false;

  		row1.push_back(row_(A1,i));
  		row2.push_back(row_(A2,i));
  	}

  	Vector r1 = concatVectors(row1);
  	Vector r2 = concatVectors(row2);
  	if( !::equal_with_abs_tol(r1,      r2, tol) &&
  			!::equal_with_abs_tol(r1*(-1), r2, tol)) {
  		return false;
  	}
  }

  return model_->equals(*(lf->model_),tol);
}

/* ************************************************************************* */
Vector GaussianFactor::unweighted_error(const VectorConfig& c) const {
  Vector e = -b_;
  if (empty()) return e;
	BOOST_FOREACH(const NamedMatrix& jA, As_)
    e += (jA.second * c[jA.first]);
  return e;
}

/* ************************************************************************* */
Vector GaussianFactor::error_vector(const VectorConfig& c) const {
	if (empty()) return model_->whiten(-b_);
	return model_->whiten(unweighted_error(c));
}

/* ************************************************************************* */
double GaussianFactor::error(const VectorConfig& c) const {
  if (empty()) return 0;
  Vector weighted = error_vector(c); // rtodo: copying vector here?
  return 0.5 * inner_prod(weighted,weighted);
}

/* ************************************************************************* */
list<Symbol> GaussianFactor::keys() const {
	list<Symbol> result;
	typedef pair<Symbol,Matrix> NamedMatrix;
	BOOST_FOREACH(const NamedMatrix& jA, As_)
    result.push_back(jA.first);
  return result;
}

/* ************************************************************************* */
Dimensions GaussianFactor::dimensions() const {
  Dimensions result;
  BOOST_FOREACH(const NamedMatrix& jA, As_)
		result.insert(std::pair<Symbol,size_t>(jA.first,jA.second.size2()));
  return result;
}

/* ************************************************************************* */
void GaussianFactor::tally_separator(const Symbol& key, set<Symbol>& separator) const {
  if(involves(key)) {
    BOOST_FOREACH(const NamedMatrix& jA, As_)
      if(jA.first != key) separator.insert(jA.first);
  }
}

/* ************************************************************************* */
Vector GaussianFactor::operator*(const VectorConfig& x) const {
	Vector Ax = zero(b_.size());
  if (empty()) return Ax;

  // Just iterate over all A matrices and multiply in correct config part
  BOOST_FOREACH(const NamedMatrix& jA, As_)
    Ax += (jA.second * x[jA.first]);

  return model_->whiten(Ax);
}

/* ************************************************************************* */
VectorConfig GaussianFactor::operator^(const Vector& e) const {
  Vector E = model_->whiten(e);
	VectorConfig x;
  // Just iterate over all A matrices and insert Ai^e into VectorConfig
  BOOST_FOREACH(const NamedMatrix& jA, As_)
    x.insert(jA.first,jA.second^E);
	return x;
}

/* ************************************************************************* */
void GaussianFactor::transposeMultiplyAdd(double alpha, const Vector& e,
		VectorConfig& x) const {
	Vector E = alpha * model_->whiten(e);
	// Just iterate over all A matrices and insert Ai^e into VectorConfig
	BOOST_FOREACH(const NamedMatrix& jA, As_)
		gtsam::transposeMultiplyAdd(1.0, jA.second, E, x[jA.first]);
}

/* ************************************************************************* */  
pair<Matrix,Vector> GaussianFactor::matrix(const Ordering& ordering, bool weight) const {

  // rtodo: this is called in eliminate, potential function to optimize?
	// get pointers to the matrices
	vector<const Matrix *> matrices;
	BOOST_FOREACH(const Symbol& j, ordering) {
		const Matrix& Aj = get_A(j);
		matrices.push_back(&Aj);
	}

	// assemble
	Matrix A = collect(matrices);
	Vector b(b_);

	// divide in sigma so error is indeed 0.5*|Ax-b|
	if (weight) model_->WhitenSystem(A,b);
	return make_pair(A, b);
}

/* ************************************************************************* */
Matrix GaussianFactor::matrix_augmented(const Ordering& ordering, bool weight) const {
	// get pointers to the matrices
	vector<const Matrix *> matrices;
	BOOST_FOREACH(const Symbol& j, ordering) {
		const Matrix& Aj = get_A(j);
		matrices.push_back(&Aj);
	}

	// load b into a matrix
	size_t rows = b_.size();
	Matrix B_mat(rows, 1);
	memcpy(B_mat.data().begin(), b_.data().begin(), rows*sizeof(double));
	matrices.push_back(&B_mat);

	// divide in sigma so error is indeed 0.5*|Ax-b|
	Matrix Ab = collect(matrices);
	if (weight) model_->WhitenInPlace(Ab);

	return Ab;
}

/* ************************************************************************* */
boost::tuple<list<int>, list<int>, list<double> >
GaussianFactor::sparse(const Dimensions& columnIndices) const {

	// declare return values
	list<int> I,J;
	list<double> S;

	// iterate over all matrices in the factor
	BOOST_FOREACH(const NamedMatrix& jA, As_) {
		// find first column index for this key
		int column_start = columnIndices.at(jA.first);
		for (size_t i = 0; i < jA.second.size1(); i++) {
			double sigma_i = model_->sigma(i);
			for (size_t j = 0; j < jA.second.size2(); j++)
				if (jA.second(i, j) != 0.0) {
					I.push_back(i + 1);
					J.push_back(j + column_start);
					S.push_back(jA.second(i, j) / sigma_i);
				}
		}
	}

	// return the result
	return boost::tuple<list<int>, list<int>, list<double> >(I,J,S);
}

/* ************************************************************************* */
void GaussianFactor::append_factor(GaussianFactor::shared_ptr f, size_t m, size_t pos) {

	// iterate over all matrices from the factor f
	BOOST_FOREACH(const NamedMatrix& p, f->As_) {
		const Symbol& key = p.first;
		const Matrix& Aj = p.second;

		// find the corresponding matrix among As
		iterator mine = As_.find(key);
		const bool exists = mine != As_.end();

		// find rows and columns
		const size_t n = Aj.size2();

		// use existing or create new matrix
		if (exists)
		  copy(Aj.data().begin(), Aj.data().end(), (mine->second).data().begin()+pos*n);
		else {
			Matrix Z = zeros(m, n);
			copy(Aj.data().begin(), Aj.data().end(), Z.data().begin()+pos*n);
			insert(key, Z);
		}

	} // FOREACH
}

/* ************************************************************************* */
/* Note, in place !!!!
 * Do incomplete QR factorization for the first n columns
 * We will do QR on all matrices and on RHS
 * Then take first n rows and make a GaussianConditional,
 * and last rows to make a new joint linear factor on separator
 */
/* ************************************************************************* */

#include <boost/numeric/ublas/triangular.hpp>
#include <boost/numeric/ublas/io.hpp>
#include <boost/numeric/ublas/matrix_proxy.hpp>

pair<GaussianBayesNet, GaussianFactor::shared_ptr>
GaussianFactor::eliminateMatrix(Matrix& Ab, SharedDiagonal model,
		        const Ordering& frontals, const Ordering& separators,
		        const Dimensions& dimensions) {
	bool verbose = false;

	// Use in-place QR on dense Ab appropriate to NoiseModel
	if (verbose) model->print("Before QR");
	SharedDiagonal noiseModel = model->QR(Ab);
	if (verbose) model->print("After QR");

	// get dimensions of the eliminated variable
	// TODO: this is another map find that should be avoided !
	size_t n1 = dimensions.at(frontals.front()), n = Ab.size2() - 1;

	// Get alias to augmented RHS d
	ublas::matrix_column<Matrix> d(Ab,n);

	// extract the conditionals
	GaussianBayesNet bn;
	size_t n0 = 0;
	Ordering::const_iterator itFrontal1 = frontals.begin(), itFrontal2;
	for(; itFrontal1!=frontals.end(); itFrontal1++) {
		n1 = n0 + dimensions.at(*itFrontal1);
		// create base conditional Gaussian
		GaussianConditional::shared_ptr conditional(new GaussianConditional(*itFrontal1,
				sub(d,  n0, n1),                   // form d vector
				sub(Ab, n0, n1, n0, n1),           // form R matrix
				sub(noiseModel->sigmas(),n0,n1))); // get standard deviations

		// add parents to the conditional
		itFrontal2 = itFrontal1;
		itFrontal2 ++;
		size_t j = n1;
		for (; itFrontal2!=frontals.end(); itFrontal2++) {
			size_t dim = dimensions.at(*itFrontal2);
			conditional->add(*itFrontal2, sub(Ab, n0, n1, j, j+dim));
			j+=dim;
		}
		BOOST_FOREACH(const Symbol& cur_key, separators) {
			size_t dim = dimensions.at(cur_key);
			conditional->add(cur_key, sub(Ab, n0, n1, j, j+dim));
			j+=dim;
		}
		n0 = n1;
		bn.push_back(conditional);
	}

	// if m<n1, this factor cannot be eliminated
	size_t maxRank = noiseModel->dim();
	if (maxRank<n1) {
		cout << "Perhaps your factor graph is singular." << endl;
		cout << "Here are the keys involved in the factor now being eliminated:" << endl;
		separators.print("Keys");
		cout << "The first key, '" << (string)frontals.front() << "', corresponds to the variable being eliminated" << endl;
		throw(domain_error("GaussianFactor::eliminate: fewer constraints than unknowns"));
	}

	// extract the new factor
	GaussianFactor::shared_ptr factor(new GaussianFactor);
	size_t j = n1;
	BOOST_FOREACH(const Symbol& cur_key, separators) {
		size_t dim = dimensions.at(cur_key); // TODO avoid find !
		factor->insert(cur_key, sub(Ab, n1, maxRank, j, j+dim)); // TODO: handle zeros properly
		j+=dim;
	}

	// Set sigmas
	// set the right model here
	if (noiseModel->isConstrained())
		factor->model_ = noiseModel::Constrained::MixedSigmas(sub(noiseModel->sigmas(),n1,maxRank));
	else
		factor->model_ = noiseModel::Diagonal::Sigmas(sub(noiseModel->sigmas(),n1,maxRank));

	// extract ds vector for the new b
	factor->set_b(sub(d, n1, maxRank));

	return make_pair(bn, factor);

}

/* ************************************************************************* */
pair<GaussianConditional::shared_ptr, GaussianFactor::shared_ptr>
GaussianFactor::eliminateMatrix(Matrix& Ab, SharedDiagonal model,
		        const Symbol& frontal, const Ordering& separator,
		        const Dimensions& dimensions) {
	Ordering frontals; frontals += frontal;
	pair<GaussianBayesNet, shared_ptr> ret =
			eliminateMatrix(Ab, model, frontals, separator, dimensions);
	return make_pair(*ret.first.begin(), ret.second);
}
/* ************************************************************************* */
pair<GaussianConditional::shared_ptr, GaussianFactor::shared_ptr>
GaussianFactor::eliminate(const Symbol& key) const
{
	// if this factor does not involve key, we exit with empty CG and LF
	const_iterator it = As_.find(key);
	if (it==As_.end()) {
		// Conditional Gaussian is just a parent-less node with P(x)=1
		GaussianFactor::shared_ptr lf(new GaussianFactor);
		GaussianConditional::shared_ptr cg(new GaussianConditional(key));
		return make_pair(cg,lf);
	}

	// create an internal ordering that eliminates key first
	Ordering ordering;
	ordering += key;
	BOOST_FOREACH(const Symbol& k, keys())
		if (k != key) ordering += k;

	// extract [A b] from the combined linear factor (ensure that x is leading)
	Matrix Ab = matrix_augmented(ordering,false);

	// TODO: this is where to split
	ordering.pop_front();
	return eliminateMatrix(Ab, model_, key, ordering, dimensions());
}

/* ************************************************************************* */
namespace gtsam {

	string symbol(char c, int index) {
		stringstream ss;
		ss << c << index;
		return ss.str();
	}

}
/* ************************************************************************* */
