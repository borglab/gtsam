/**
 * @file    LinearFactor.cpp
 * @brief   Linear Factor....A Gaussian
 * @brief   linearFactor
 * @author  Christian Potthast
 */

#include <boost/foreach.hpp>
#include <boost/tuple/tuple.hpp>
#include <boost/assign/list_inserter.hpp> // for 'insert()'
#include <boost/assign/std/list.hpp> // for operator += in Ordering

#include "Matrix.h"
#include "Ordering.h"
#include "ConditionalGaussian.h"
#include "LinearFactor.h"

using namespace std;
using namespace boost::assign;
namespace ublas = boost::numeric::ublas;

// trick from some reading group
#define FOREACH_PAIR( KEY, VAL, COL) BOOST_FOREACH (boost::tie(KEY,VAL),COL) 

using namespace gtsam;

typedef pair<const string, Matrix>& mypair;

/* ************************************************************************* */
LinearFactor::LinearFactor(const boost::shared_ptr<ConditionalGaussian>& cg) :
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
	sigmas_ = ediv(ones(n),cg->get_precisions());
	for(int j=0;j<n;j++) sigmas_(j)=sqrt(sigmas_(j));
}

/* ************************************************************************* */
LinearFactor::LinearFactor(const vector<shared_ptr> & factors)
{
	bool verbose = false;
	if (verbose) cout << "LinearFactor::LinearFactor (factors)" << endl;

	// Create RHS and precision vector of the right size by adding together row counts
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

    // copy the precision vector from factor to sigmas_
    for (size_t i=0; i<mf; i++) sigmas_(pos+i) = factor->sigmas_(i);

    // update the matrices
    append_factor(factor,m,pos);

    pos += mf;
  }
	if (verbose) cout << "LinearFactor::LinearFactor done" << endl;
}

/* ************************************************************************* */
void LinearFactor::print(const string& s) const {
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
size_t LinearFactor::getDim(const std::string& key) const {
	const_iterator it = As_.find(key);
	if (it != As_.end())
		return it->second.size2();
	else
		return 0;
}

/* ************************************************************************* */
// Check if two linear factors are equal
bool LinearFactor::equals(const Factor<VectorConfig>& f, double tol) const {
    
  const LinearFactor* lf = dynamic_cast<const LinearFactor*>(&f);
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
double LinearFactor::error(const VectorConfig& c) const {
  if (empty()) return 0;
  Vector e = b_;
  string j; Matrix Aj;
  FOREACH_PAIR(j, Aj, As_)
    e -= Vector(Aj * c[j]);
  Vector weighted = ediv(e,sigmas_);
  return 0.5 * inner_prod(weighted,weighted);
}

/* ************************************************************************* */
list<string> LinearFactor::keys() const {
	list<string> result;
  string j; Matrix A;
  FOREACH_PAIR(j,A,As_)
    result.push_back(j);
  return result;
}

/* ************************************************************************* */
VariableSet LinearFactor::variables() const {
  VariableSet result;
  string j; Matrix A;
  FOREACH_PAIR(j,A,As_) {
    Variable v(j,A.size2());
    result.insert(v);
  }
  return result;
}

/* ************************************************************************* */
void LinearFactor::tally_separator(const string& key, set<string>& separator) const {
  if(involves(key)) {
    string j; Matrix A;
    FOREACH_PAIR(j,A,As_)
      if(j != key) separator.insert(j);
  }
}

/* ************************************************************************* */  
pair<Matrix,Vector> LinearFactor::matrix(const Ordering& ordering) const {

  // get pointers to the matrices
  vector<const Matrix *> matrices;
  BOOST_FOREACH(string j, ordering) {
    const Matrix& Aj = get_A(j);
    matrices.push_back(&Aj);
  }

  // divide in sigma so error is indeed 0.5*|Ax-b|
  Matrix t = diag(ediv(ones(sigmas_.size()),sigmas_));
  Matrix A = t*collect(matrices);
  return make_pair(A, t*b_);
}

/* ************************************************************************* */
void LinearFactor::append_factor(LinearFactor::shared_ptr f, const size_t m,
		const size_t pos) {
	bool verbose = false;
	if (verbose) cout << "LinearFactor::append_factor" << endl;

	// iterate over all matrices from the factor f
	LinearFactor::const_iterator it = f->begin();
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
	if (verbose) cout << "LinearFactor::append_factor done" << endl;

}

/* ************************************************************************* */
/* Note, in place !!!!
 * Do incomplete QR factorization for the first n columns
 * We will do QR on all matrices and on RHS
 * Then take first n rows and make a ConditionalGaussian,
 * and last rows to make a new joint linear factor on separator
 */
/* ************************************************************************* */
pair<ConditionalGaussian::shared_ptr, LinearFactor::shared_ptr>
LinearFactor::eliminate(const string& key)
{
	bool verbose = false;
	if (verbose) cout << "LinearFactor::eliminate(" << key << ")" << endl;

	// if this factor does not involve key, we exit with empty CG and LF
	iterator it = As_.find(key);
	if (it==As_.end()) {
		// Conditional Gaussian is just a parent-less node with P(x)=1
		LinearFactor::shared_ptr lf(new LinearFactor);
		ConditionalGaussian::shared_ptr cg(new ConditionalGaussian(key));
		return make_pair(cg,lf);
	}
	if (verbose) cout <<  "<<<<<<<<<<<< 1" << endl;

	// create an internal ordering that eliminates key first
	Ordering ordering;
	ordering += key;
	BOOST_FOREACH(string k, keys())
		if (k != key) ordering += k;
	if (verbose) cout <<  "<<<<<<<<<<<< 2" << endl;

	// extract A, b from the combined linear factor (ensure that x is leading)
	Matrix A; Vector b;
	boost::tie(A, b) = matrix(ordering);
	size_t m = A.size1(); size_t n = A.size2();
	if (verbose) cout <<  "<<<<<<<<<<<< 3" << endl;

	// get dimensions of the eliminated variable
	size_t n1 = getDim(key);

	// if m<n1, this factor cannot be eliminated
	size_t maxRank = min(m,n);
	if (maxRank<n1)
		throw(domain_error("LinearFactor::eliminate: fewer constraints than unknowns"));

	// QR performed using an augmented matrix Rd =[A b]
	// TODO: We should get rid of this copy
	Matrix Rd(m, n+1);
	// copy in A
	for (int i=0; i<m; ++i)
		for (int j=0; j<n; ++j)
			Rd(i,j) = A(i,j);
	// copy in b
	for (int i=0; i<m; ++i)
		Rd(i,n) = b(i);
	if (verbose) cout <<  "<<<<<<<<<<<< 4" << endl;

	// Do in-place QR to get R, d of the augmented system
	if (verbose) ::print(Rd,"Rd before");
	householder(Rd, maxRank);
	if (verbose) ::print(Rd,"Rd after");
	if (verbose) cout <<  "<<<<<<<<<<<< 5" << endl;

	// R as calculated by householder has inverse sigma on diagonal
	// Use them to normalize R to unit-upper-triangular matrix
	Vector sigmas(m); // standard deviations
	Vector tau(n1);   // precisions for conditional
	if (verbose) cout << n1 << " " << n  << " " << m << endl;
	for (int i=0; i<maxRank; ++i) {
		double Rii = Rd(i,i);
		// detect rank < maxRank
		if (fabs(Rii)<1e-8) { maxRank=i; break;}
		sigmas(i) = 1.0/Rii;
		if (i<n1) tau(i) = Rii*Rii;
		for (int j=0; j<=n; ++j)
			Rd(i,j) = Rd(i,j)*sigmas(i);
	}
	if (verbose) cout <<  "<<<<<<<<<<<< 6" << endl;

	// extract RHS
	Vector d(m);
	for (int i=0; i<m; ++i)
		d(i) = Rd(i,n);
	if (verbose) cout <<  "<<<<<<<<<<<< 7" << endl;

	// create base conditional Gaussian
	ConditionalGaussian::shared_ptr cg(new ConditionalGaussian(key,
			sub(d, 0, n1),         // form d vector
			sub(Rd, 0, n1, 0, n1), // form R matrix
			sub(tau, 0, n1)));     // get precisions
	if (verbose) cout <<  "<<<<<<<<<<<< 8" << endl;

	// extract the block matrices for parents in both CG and LF
	LinearFactor::shared_ptr lf(new LinearFactor);
	size_t j = n1;
	BOOST_FOREACH(string cur_key, ordering)
		if (cur_key!=key) {
			size_t dim = getDim(cur_key);
			cg->add(cur_key, sub(Rd, 0, n1, j, j+dim));
			lf->insert(cur_key, sub(Rd, n1, maxRank, j, j+dim));
			j+=dim;
		}
	if (verbose) cout <<  "<<<<<<<<<<<< 9" << endl;

	// Set sigmas
	lf->sigmas_ = sub(sigmas,n1,maxRank);
	if (verbose) cout <<  "<<<<<<<<<<<< 10" << endl;

	// extract ds vector for the new b
	lf->set_b(sub(d, n1, maxRank));
	if (verbose) lf->print("lf");
	if (verbose) cout <<  "<<<<<<<<<<<< 11" << endl;

	return make_pair(cg, lf);
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
