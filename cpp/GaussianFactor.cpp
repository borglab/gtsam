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

// MACRO to loop. Ugly with pointer, but best I could in short time
#define FOREACH_PAIR(KEY,VAL,COL) const_iterator it = COL.begin(); \
	const Symbol* KEY = it == COL.end() ? NULL : &(it->first); \
	const Matrix* VAL = it == COL.end() ? NULL : &(it->second); \
	for (; it != COL.end(); it++, KEY=&(it->first), VAL=&(it->second))

/* ************************************************************************* */
GaussianFactor::GaussianFactor(const boost::shared_ptr<GaussianConditional>& cg) :
	b_(cg->get_d()) {
	As_.insert(make_pair(cg->key(), cg->get_R()));
	std::map<Symbol, Matrix>::const_iterator it = cg->parentsBegin();
	for (; it != cg->parentsEnd(); it++) {
		const Symbol& j = it->first;
		const Matrix& Aj = it->second;
		As_.insert(make_pair(j, Aj));
	}
	// set sigmas from precisions
	size_t n = b_.size();
	model_ = noiseModel::Diagonal::Sigmas(cg->get_sigmas());
}

/* ************************************************************************* */
GaussianFactor::GaussianFactor(const vector<shared_ptr> & factors)
{
	bool verbose = false;
	if (verbose) cout << "GaussianFactor::GaussianFactor (factors)" << endl;

	// Create RHS and sigmas of right size by adding together row counts
  size_t m = 0;
  BOOST_FOREACH(shared_ptr factor, factors) m += factor->numberOfRows();
  b_ = Vector(m);
  Vector sigmas(m);

  size_t pos = 0; // save last position inserted into the new rhs vector

  // iterate over all factors
  BOOST_FOREACH(shared_ptr factor, factors){
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

    pos += mf;
  }
	if (verbose) cout << "GaussianFactor::GaussianFactor done" << endl;
	model_ = noiseModel::Diagonal::Sigmas(sigmas);
}

/* ************************************************************************* */
void GaussianFactor::print(const string& s) const {
  cout << s << endl;
  if (empty()) cout << " empty" << endl; 
  else {
  	FOREACH_PAIR(j,Aj,As_)
  		gtsam::print(*Aj, "A["+(string)*j+"]=\n");
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

  for(; it1 != As_.end(); it1++, it2++) {
    const Symbol& j1 = it1->first, j2 = it2->first;
    const Matrix A1 = it1->second, A2 = it2->second;
    if (j1 != j2) return false;
    if (!equal_with_abs_tol(A1,A2,tol))
      return false;
  }

  if( !(::equal_with_abs_tol(b_, (lf->b_),tol)) )
    return false;

  return model_->equals(*(lf->model_),tol);
}

/* ************************************************************************* */
Vector GaussianFactor::unweighted_error(const VectorConfig& c) const {
  Vector e = -b_;
  if (empty()) return e;
	FOREACH_PAIR(j,Aj,As_)
    e += (*Aj * c[*j]);
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
	FOREACH_PAIR(j,Aj,As_)
    result.insert(make_pair(*j,Aj->size2()));
  return result;
}

/* ************************************************************************* */
void GaussianFactor::tally_separator(const Symbol& key, set<Symbol>& separator) const {
  if(involves(key)) {
    FOREACH_PAIR(j,A,As_)
      if(*j != key) separator.insert(*j);
  }
}

/* ************************************************************************* */
Vector GaussianFactor::operator*(const VectorConfig& x) const {
	Vector Ax = zero(b_.size());
  if (empty()) return Ax;

  // Just iterate over all A matrices and multiply in correct config part
  FOREACH_PAIR(j, Aj, As_)
    Ax += (*Aj * x[*j]);

  return model_->whiten(Ax);
}

/* ************************************************************************* */
VectorConfig GaussianFactor::operator^(const Vector& e) const {
  Vector E = model_->whiten(e);
	VectorConfig x;
  // Just iterate over all A matrices and insert Ai^e into VectorConfig
  FOREACH_PAIR(j, Aj, As_)
    x.insert(*j,(*Aj)^E);
	return x;
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
	Matrix B_mat(numberOfRows(), 1);
	for (int i=0; i<b_.size(); ++i)
		B_mat(i,0) = b_(i);
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
	FOREACH_PAIR( key, Aj, As_) {
		// find first column index for this key
		// TODO: check if end() and throw exception if not found
		Dimensions::const_iterator it = columnIndices.find(*key);
		int column_start = it->second;
		for (size_t i = 0; i < Aj->size1(); i++) {
			double sigma_i = model_->sigma(i);
			for (size_t j = 0; j < Aj->size2(); j++)
				if ((*Aj)(i, j) != 0.0) {
					I.push_back(i + 1);
					J.push_back(j + column_start);
					S.push_back((*Aj)(i, j) / sigma_i);
				}
		}
	}

	// return the result
	return boost::tuple<list<int>, list<int>, list<double> >(I,J,S);
}

/* ************************************************************************* */
void GaussianFactor::append_factor(GaussianFactor::shared_ptr f, size_t m, size_t pos) {

	// iterate over all matrices from the factor f
	FOREACH_PAIR( key, A, f->As_) {

		// find the corresponding matrix among As
		iterator mine = As_.find(*key);
		const bool exists = mine != As_.end();

		// find rows and columns
		const size_t n = A->size2();

		// use existing or create new matrix
		if (exists)
		  copy(A->data().begin(), A->data().end(), (mine->second).data().begin()+pos*n);
		else {
			Matrix Z = zeros(m, n);
			copy(A->data().begin(), A->data().end(), Z.data().begin()+pos*n);
			insert(*key, Z);
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

	// extract A, b from the combined linear factor (ensure that x is leading)
	// TODO: get Ab as augmented matrix
	// Matrix Ab = matrix_augmented(ordering,false);
	Matrix A; Vector b;
	boost::tie(A, b) = matrix(ordering, false);
	size_t n = A.size2();

	// Do in-place QR to get R, d of the augmented system
	std::list<boost::tuple<Vector, double, double> > solution =
							weighted_eliminate(A, b, model_->sigmas());

	// TODO, fix using NoiseModel, the read out Ab
	// model->QR(Ab)

	// get dimensions of the eliminated variable
	// TODO: this is another map find that should be avoided !
	size_t n1 = getDim(key);

	// if m<n1, this factor cannot be eliminated
	size_t maxRank = solution.size();
	if (maxRank<n1)
		throw(domain_error("GaussianFactor::eliminate: fewer constraints than unknowns"));

	// unpack the solutions
	Matrix R(maxRank, n);
	Vector r, d(maxRank), newSigmas(maxRank); double di, sigma;
	Matrix::iterator2 Rit = R.begin2();
	size_t i = 0;
	BOOST_FOREACH(boost::tie(r, di, sigma), solution) {
		copy(r.begin(), r.end(), Rit); // copy r vector
		d(i) = di;                     // copy in rhs
		newSigmas(i) = sigma;          // copy in new sigmas
		Rit += n; i += 1;
	}

	// create base conditional Gaussian
	GaussianConditional::shared_ptr conditional(new GaussianConditional(key,
			sub(d, 0, n1),            // form d vector
			sub(R, 0, n1, 0, n1),     // form R matrix
			sub(newSigmas, 0, n1)));  // get standard deviations

	// extract the block matrices for parents in both CG and LF
	GaussianFactor::shared_ptr factor(new GaussianFactor);
	size_t j = n1;
	BOOST_FOREACH(Symbol& cur_key, ordering)
		if (cur_key!=key) {
			size_t dim = getDim(cur_key);
			conditional->add(cur_key, sub(R, 0, n1, j, j+dim));
			factor->insert(cur_key, sub(R, n1, maxRank, j, j+dim));
			j+=dim;
		}

	// Set sigmas
	factor->model_ = noiseModel::Diagonal::Sigmas(sub(newSigmas,n1,maxRank));

	// extract ds vector for the new b
	factor->set_b(sub(d, n1, maxRank));

	return make_pair(conditional, factor);
}

/* ************************************************************************* */
// Creates a factor on step-size, given initial estimate and direction d, e.g.
// Factor |A1*x+A2*y-b|/sigma -> |A1*(x0+alpha*dx)+A2*(y0+alpha*dy)-b|/sigma
//                            -> |(A1*dx+A2*dy)*alpha-(b-A1*x0-A2*y0)|/sigma
/* ************************************************************************* */
GaussianFactor::shared_ptr GaussianFactor::alphaFactor(const Symbol& key, const VectorConfig& x,
		const VectorConfig& d) const {

	// Calculate A matrix
	size_t m = b_.size();
	Vector A = zero(m);
  FOREACH_PAIR(j, Aj, As_)
  	A += *Aj * d[*j];

  // calculate the value of the factor for RHS
	Vector b = - unweighted_error(x);

	// construct factor
	shared_ptr factor(new GaussianFactor(key,Matrix_(A),b,model_->sigmas()));
	return factor;
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
