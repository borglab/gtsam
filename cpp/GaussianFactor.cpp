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

// trick from some reading group
#define FOREACH_PAIR( KEY, VAL, COL) BOOST_FOREACH (boost::tie(KEY,VAL),COL) 

using namespace gtsam;

typedef pair<const string, Matrix>& mypair;

/* ************************************************************************* */
GaussianFactor::GaussianFactor(const boost::shared_ptr<GaussianConditional>& cg) :
	b_(cg->get_d()) {
	As_.insert(make_pair(cg->key(), cg->get_R()));
	std::map<std::string, Matrix>::const_iterator it = cg->parentsBegin();
	for (; it != cg->parentsEnd(); it++) {
		const std::string& j = it->first;
		const Matrix& Aj = it->second;
		As_.insert(make_pair(j, Aj));
	}
	// set sigmas from precisions
	size_t n = b_.size();
	sigmas_ = cg->get_sigmas();
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
  sigmas_ = Vector(m);

  size_t pos = 0; // save last position inserted into the new rhs vector

  // iterate over all factors
  BOOST_FOREACH(shared_ptr factor, factors){
  	if (verbose) factor->print();
    // number of rows for factor f
    const size_t mf = factor->numberOfRows();

    // copy the rhs vector from factor to b
    const Vector bf = factor->get_b();
    for (size_t i=0; i<mf; i++) b_(pos+i) = bf(i);

    // copy the sigmas_
    for (size_t i=0; i<mf; i++) sigmas_(pos+i) = factor->sigmas_(i);

    // update the matrices
    append_factor(factor,m,pos);

    pos += mf;
  }
	if (verbose) cout << "GaussianFactor::GaussianFactor done" << endl;
}

/* ************************************************************************* */
void GaussianFactor::print(const string& s) const {
  cout << s << endl;
  if (empty()) cout << " empty" << endl; 
  else {
    string j; Matrix A;
    FOREACH_PAIR(j,A,As_) gtsam::print(A, "A["+j+"]=\n");
    gtsam::print(b_,"b=");
    gtsam::print(sigmas_, "sigmas = ");
  }
}

/* ************************************************************************* */
size_t GaussianFactor::getDim(const std::string& key) const {
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
    const string& j1 = it1->first, j2 = it2->first;
    const Matrix A1 = it1->second, A2 = it2->second;
    if (j1 != j2) return false;
    if (!equal_with_abs_tol(A1,A2,tol))
      return false;
  }

  if( !(::equal_with_abs_tol(b_, (lf->b_),tol)) )
    return false;

  if( !(::equal_with_abs_tol(sigmas_, (lf->sigmas_),tol)) )
      return false;

  return true;
}

/* ************************************************************************* */
// we might have multiple As, so iterate and subtract from b
double GaussianFactor::error(const VectorConfig& c) const {
  if (empty()) return 0;
  Vector e = b_;
  string j; Matrix Aj;
  FOREACH_PAIR(j, Aj, As_)
    e -= Vector(Aj * c[j]);
  Vector weighted = ediv(e,sigmas_);
  return 0.5 * inner_prod(weighted,weighted);
}

/* ************************************************************************* */
list<string> GaussianFactor::keys() const {
	list<string> result;
  string j; Matrix A;
  FOREACH_PAIR(j,A,As_)
    result.push_back(j);
  return result;
}

/* ************************************************************************* */
Dimensions GaussianFactor::dimensions() const {
  Dimensions result;
  string j; Matrix A;
  FOREACH_PAIR(j,A,As_)
    result.insert(make_pair(j,A.size2()));
  return result;
}

/* ************************************************************************* */
void GaussianFactor::tally_separator(const string& key, set<string>& separator) const {
  if(involves(key)) {
    string j; Matrix A;
    FOREACH_PAIR(j,A,As_)
      if(j != key) separator.insert(j);
  }
}

/* ************************************************************************* */  
pair<Matrix,Vector> GaussianFactor::matrix(const Ordering& ordering, bool weight) const {

	// get pointers to the matrices
	vector<const Matrix *> matrices;
	BOOST_FOREACH(string j, ordering) {
		const Matrix& Aj = get_A(j);
		matrices.push_back(&Aj);
	}

	// assemble
	Matrix A = collect(matrices);
	Vector b(b_);

	// divide in sigma so error is indeed 0.5*|Ax-b|
	if (weight) {
		Vector t = ediv(ones(sigmas_.size()),sigmas_);
		A = vector_scale(A, t);
		for (int i=0; i<b_.size(); ++i)
			b(i) *= t(i);
	}
	return make_pair(A, b);
}

/* ************************************************************************* */
Matrix GaussianFactor::matrix_augmented(const Ordering& ordering) const {
	// get pointers to the matrices
	vector<const Matrix *> matrices;
	BOOST_FOREACH(string j, ordering) {
		const Matrix& Aj = get_A(j);
		matrices.push_back(&Aj);
	}

	// load b into a matrix
	Matrix B_mat(numberOfRows(), 1);
	for (int i=0; i<b_.size(); ++i)
		B_mat(i,0) = b_(i);
	matrices.push_back(&B_mat);

	return collect(matrices);
}

/* ************************************************************************* */
boost::tuple<list<int>, list<int>, list<double> >
GaussianFactor::sparse(const Ordering& ordering, const Dimensions& variables) const {

	// declare return values
	list<int> I,J;
	list<double> S;

	// loop over all variables in correct order
	size_t column_start = 1;
	BOOST_FOREACH(string key, ordering) {
		try {
			const Matrix& Aj = get_A(key);
			for (size_t i = 0; i < Aj.size1(); i++) {
				double sigma_i = sigmas_(i);
				for (size_t j = 0; j < Aj.size2(); j++)
					if (Aj(i, j) != 0.0) {
						I.push_back(i + 1);
						J.push_back(j + column_start);
						S.push_back(Aj(i, j) / sigma_i);
					}
			}
		} catch (std::invalid_argument& exception) {
			// it's ok to not have a key in the ordering
		}
		// find dimension for this key
		Dimensions::const_iterator it = variables.find(key);
		// TODO: check if end() and throw exception if not found
		int dim = it->second;
		// advance column index to next block by adding dim(key)
		column_start += dim;
	}

	// return the result
	return boost::tuple<list<int>, list<int>, list<double> >(I,J,S);
}

/* ************************************************************************* */
void GaussianFactor::append_factor(GaussianFactor::shared_ptr f, const size_t m,
		const size_t pos) {
	bool verbose = false;
	if (verbose) cout << "GaussianFactor::append_factor" << endl;

	// iterate over all matrices from the factor f
	GaussianFactor::const_iterator it = f->begin();
	for (; it != f->end(); it++) {
		string j = it->first;
		Matrix A = it->second;

		// find rows and columns
		const size_t mrhs = A.size1(), n = A.size2();

		// find the corresponding matrix among As
		const_iterator mine = As_.find(j);
		const bool exists = mine != As_.end();

		// create the matrix or use existing
		Matrix Anew = exists ? mine->second : zeros(m, n);

		// copy the values in the existing matrix
		for (size_t i = 0; i < mrhs; i++)
			for (size_t j = 0; j < n; j++)
				Anew(pos + i, j) = A(i, j);

		// insert the matrix into the factor
		if (exists) As_.erase(j);
		insert(j, Anew);
	}
	if (verbose) cout << "GaussianFactor::append_factor done" << endl;

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
GaussianFactor::eliminate(const string& key) const
{
	bool verbose = false;
	if (verbose) cout << "GaussianFactor::eliminate(" << key << ")" << endl;

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
	BOOST_FOREACH(string k, keys())
		if (k != key) ordering += k;

	// extract A, b from the combined linear factor (ensure that x is leading)
	Matrix A; Vector b;
	boost::tie(A, b) = matrix(ordering, false);
	size_t n = A.size2();

	// Do in-place QR to get R, d of the augmented system
	if (verbose) ::print(A,"A");
	if (verbose) ::print(b,"b = ");
	if (verbose) ::print(sigmas_,"sigmas = ");
	std::list<boost::tuple<Vector, double, double> > solution =
							weighted_eliminate(A, b, sigmas_);

	// get dimensions of the eliminated variable
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
	BOOST_FOREACH(string cur_key, ordering)
		if (cur_key!=key) {
			size_t dim = getDim(cur_key);
			conditional->add(cur_key, sub(R, 0, n1, j, j+dim));
			factor->insert(cur_key, sub(R, n1, maxRank, j, j+dim));
			j+=dim;
		}

	// Set sigmas
	factor->sigmas_ = sub(newSigmas,n1,maxRank);

	// extract ds vector for the new b
	factor->set_b(sub(d, n1, maxRank));
	if (verbose) conditional->print("Conditional");
	if (verbose) factor->print("Factor");

	return make_pair(conditional, factor);
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
