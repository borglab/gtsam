/**
 * @file    LinearFactor.cpp
 * @brief   Linear Factor....A Gaussian
 * @brief   linearFactor
 * @author  Christian Potthast
 */

#include <boost/foreach.hpp>
#include <boost/tuple/tuple.hpp>
#include "LinearFactor.h"

using namespace std;
namespace ublas = boost::numeric::ublas;

// trick from some reading group
#define FOREACH_PAIR( KEY, VAL, COL) BOOST_FOREACH (boost::tie(KEY,VAL),COL) 

using namespace std;
using namespace gtsam;

typedef pair<const string, Matrix>& mypair;

/* ************************************************************************* */
LinearFactor::LinearFactor(const vector<shared_ptr> & factors)
{
  // Create RHS vector of the right size by adding together row counts
  size_t m = 0;
  BOOST_FOREACH(shared_ptr factor, factors) m += factor->numberOfRows();
  b = Vector(m);

  size_t pos = 0; // save last position inserted into the new rhs vector

  // iterate over all factors
  BOOST_FOREACH(shared_ptr factor, factors){
    // number of rows for factor f
    const size_t mf = factor->numberOfRows();

    // copy the rhs vector from factor to b
    const Vector bf = factor->get_b();
    for (size_t i=0; i<mf; i++) b(pos+i) = bf(i);

    // update the matrices
    append_factor(factor,m,pos);

    pos += mf;
  }
}

/* ************************************************************************* */
void LinearFactor::print(const string& s) const {
  cout << s << endl;
  if (empty()) cout << " empty" << endl; 
  else {
    string j; Matrix A;
    FOREACH_PAIR(j,A,As) gtsam::print(A, "A["+j+"]=\n");
    gtsam::print(b,"b=");
  }
}

/* ************************************************************************* */
// Check if two linear factors are equal
bool LinearFactor::equals(const Factor<VectorConfig>& f, double tol) const {
    
  const LinearFactor* lf = dynamic_cast<const LinearFactor*>(&f);
  if (lf == NULL) return false;

  if (empty()) return (lf->empty());

  const_iterator it1 = As.begin(), it2 = lf->As.begin();
  if(As.size() != lf->As.size()) return false;

  for(; it1 != As.end(); it1++, it2++) {
    const string& j1 = it1->first, j2 = it2->first;
    const Matrix A1 = it1->second, A2 = it2->second;
    if (j1 != j2) return false;
    if (!equal_with_abs_tol(A1,A2,tol))
      return false;
  }

  if( !(::equal_with_abs_tol(b, (lf->b),tol)) )
    return false;

  return true;
}

/* ************************************************************************* */
// we might have multiple As, so iterate and subtract from b
double LinearFactor::error(const VectorConfig& c) const {
  if (empty()) return 0;
  Vector e = b;
  string j; Matrix Aj;
  FOREACH_PAIR(j, Aj, As)
    e -= Vector(Aj * c[j]);
  return 0.5 * inner_prod(trans(e),e);
}

/* ************************************************************************* */
list<string> LinearFactor::keys() const {
	list<string> result;
  string j; Matrix A;
  FOREACH_PAIR(j,A,As)
    result.push_back(j);
  return result;
}

/* ************************************************************************* */
VariableSet LinearFactor::variables() const {
  VariableSet result;
  string j; Matrix A;
  FOREACH_PAIR(j,A,As) {
    Variable v(j,A.size2());
    result.insert(v);
  }
  return result;
}

/* ************************************************************************* */
void LinearFactor::tally_separator(const string& key, set<string>& separator) const {
  if(involves(key)) {
    string j; Matrix A;
    FOREACH_PAIR(j,A,As)
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

  return make_pair( collect(matrices), b);
}

/* ************************************************************************* */
void LinearFactor::append_factor(LinearFactor::shared_ptr f, const size_t m,
		const size_t pos) {
	// iterate over all matrices from the factor f
	LinearFactor::const_iterator it = f->begin();
	for (; it != f->end(); it++) {
		string j = it->first;
		Matrix A = it->second;

		// find rows and columns
		const size_t mrhs = A.size1(), n = A.size2();

		// find the corresponding matrix among As
		const_iterator mine = As.find(j);
		const bool exists = mine != As.end();

		// create the matrix or use existing
		Matrix Anew = exists ? mine->second : zeros(m, n);

		// copy the values in the existing matrix
		for (size_t i = 0; i < mrhs; i++)
			for (size_t j = 0; j < n; j++)
				Anew(pos + i, j) = A(i, j);

		// insert the matrix into the factor
		if (exists) As.erase(j);
		insert(j, Anew);
	}
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
  // start empty remaining factor to be returned
  boost::shared_ptr<LinearFactor> lf(new LinearFactor);

  // find the matrix associated with key
  iterator it = As.find(key);

  // if this factor does not involve key, we exit with empty CG and LF
  if (it==As.end()) {
    // Conditional Gaussian is just a parent-less node with P(x)=1
    ConditionalGaussian::shared_ptr cg(new ConditionalGaussian);
    return make_pair(cg,lf);
  }

  // get the matrix reference associated with key
  const Matrix& R = it->second;
  const size_t m = R.size1(), n = R.size2();

  // if m<n, this factor cannot be eliminated
  if (m<n)
    throw(domain_error("LinearFactor::eliminate: fewer constraints than unknowns"));
  
  // we will apply n Householder reflections to zero out R below diagonal
  for(size_t j=0; j < n; j++){
    // below, the indices r,c always refer to original A

    // copy column from matrix to xjm, i.e. x(j:m) = R(j:m,j)
    Vector xjm(m-j);
    for(size_t r = j ; r < m; r++)
      xjm(r-j) = R(r,j);
        
    // calculate the Householder vector v
    double beta; Vector vjm;
    boost::tie(beta,vjm) = house(xjm);

    // update all matrices
    BOOST_FOREACH(mypair jA,As) {
      // update A matrix using reflection as in householder_
      Matrix& A = jA.second;
      householder_update(A, j, beta, vjm);
    }

    // update RHS, b -= (beta * inner_prod(v,b)) * v;
    double inner = 0;
    for(size_t r = j ; r < m; r++) 
      inner += vjm(r-j) * b(r);
    for(size_t r = j ; r < m; r++) 
      b(r) -= beta*inner*vjm(r-j);
  } // column j
  
  // create ConditionalGaussian with first n rows
  ConditionalGaussian::shared_ptr cg (new ConditionalGaussian(::sub(b,0,n), sub(R,0,n,0,n)) );
  
  // create linear factor with remaining rows
  lf->set_b(::sub(b,n,m));

  // for every separator variable
  string j; Matrix A;
  FOREACH_PAIR(j,A,As) {
    if (j != key) {
      const size_t nj = A.size2();   // get dimension of variable
      cg->add(j, sub(A,0,n,0,nj));   // add a parent to conditional Gaussian
      lf->insert(j,sub(A,n,m,0,nj)); // insert into linear factor
    }
  }
  return make_pair(cg,lf);
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
