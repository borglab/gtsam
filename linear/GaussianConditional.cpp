/**
 * @file   GaussianConditional.cpp
 * @brief  Conditional Gaussian Base class
 * @author Christian Potthast
 */

#include <string.h>
#include <boost/numeric/ublas/vector.hpp>
#include <boost/numeric/ublas/operation.hpp>
#include <boost/format.hpp>
#include <boost/lambda/bind.hpp>

#include <gtsam/linear/GaussianConditional.h>
#include <gtsam/base/Matrix-inl.h>

using namespace std;
namespace ublas = boost::numeric::ublas;

namespace gtsam {

/* ************************************************************************* */
GaussianConditional::GaussianConditional() : rsd_(matrix_) {}

/* ************************************************************************* */
GaussianConditional::GaussianConditional(Index key) : Conditional(key), rsd_(matrix_) {}

/* ************************************************************************* */
GaussianConditional::GaussianConditional(Index key,const Vector& d, const Matrix& R, const Vector& sigmas) :
	    Conditional(key), rsd_(matrix_), sigmas_(sigmas) {
  assert(R.size1() <= R.size2());
  size_t dims[] = { R.size2(), 1 };
  rsd_.copyStructureFrom(rsd_type(matrix_, dims, dims+2, d.size()));
  ublas::noalias(rsd_(0)) = ublas::triangular_adaptor<const Matrix, ublas::upper>(R);
  ublas::noalias(get_d_()) = d;
}

/* ************************************************************************* */
GaussianConditional::GaussianConditional(Index key, const Vector& d, const Matrix& R,
    Index name1, const Matrix& S, const Vector& sigmas) :
    Conditional(key,name1), rsd_(matrix_), sigmas_(sigmas) {
  assert(R.size1() <= R.size2());
  size_t dims[] = { R.size2(), S.size2(), 1 };
  rsd_.copyStructureFrom(rsd_type(matrix_, dims, dims+3, d.size()));
  ublas::noalias(rsd_(0)) = ublas::triangular_adaptor<const Matrix, ublas::upper>(R);
  ublas::noalias(rsd_(1)) = S;
  ublas::noalias(get_d_()) = d;
}

/* ************************************************************************* */
GaussianConditional::GaussianConditional(Index key, const Vector& d, const Matrix& R,
		Index name1, const Matrix& S, Index name2, const Matrix& T, const Vector& sigmas) :
		Conditional(key,name1,name2), rsd_(matrix_), sigmas_(sigmas) {
  assert(R.size1() <= R.size2());
  size_t dims[] = { R.size2(), S.size2(), T.size2(), 1 };
  rsd_.copyStructureFrom(rsd_type(matrix_, dims, dims+4, d.size()));
  ublas::noalias(rsd_(0)) = ublas::triangular_adaptor<const Matrix, ublas::upper>(R);
  ublas::noalias(rsd_(1)) = S;
  ublas::noalias(rsd_(2)) = T;
  ublas::noalias(get_d_()) = d;
}

/* ************************************************************************* */
GaussianConditional::GaussianConditional(Index key, const Vector& d, const Matrix& R, const list<pair<Index, Matrix> >& parents, const Vector& sigmas) :
    rsd_(matrix_), sigmas_(sigmas) {
  assert(R.size1() <= R.size2());
  Conditional::nrFrontals_ = 1;
  Conditional::factor_.keys_.resize(1+parents.size());
  size_t dims[1+parents.size()+1];
  dims[0] = R.size2();
  Conditional::factor_.keys_[0] = key;
  size_t j=1;
  for(std::list<std::pair<Index, Matrix> >::const_iterator parent=parents.begin(); parent!=parents.end(); ++parent) {
    Conditional::factor_.keys_[j] = parent->first;
    dims[j] = parent->second.size2();
    ++ j;
  }
  dims[j] = 1;
  rsd_.copyStructureFrom(rsd_type(matrix_, dims, dims+1+parents.size()+1, d.size()));
  ublas::noalias(rsd_(0)) = ublas::triangular_adaptor<const Matrix, ublas::upper>(R);
  j = 1;
  for(std::list<std::pair<Index, Matrix> >::const_iterator parent=parents.begin(); parent!=parents.end(); ++parent) {
    ublas::noalias(rsd_(j)) = parent->second;
    ++ j;
  }
  ublas::noalias(get_d_()) = d;
}

/* ************************************************************************* */
void GaussianConditional::print(const string &s) const
{
  cout << s << ": density on " << key() << endl;
  gtsam::print(get_R(),"R");
  for(const_iterator it = beginParents() ; it != endParents() ; it++ ) {
    gtsam::print(get_S(it), (boost::format("A[%1%]")%(*it)).str());
  }
  gtsam::print(get_d(),"d");
  gtsam::print(sigmas_,"sigmas");
}    

/* ************************************************************************* */
bool GaussianConditional::equals(const GaussianConditional &c, double tol) const {
	// check if the size of the parents_ map is the same
	if (parents().size() != c.parents().size()) return false;

	// check if R_ and d_ are linear independent
	for (size_t i=0; i<rsd_.size1(); i++) {
		list<Vector> rows1; rows1.push_back(row_(get_R(), i));
		list<Vector> rows2; rows2.push_back(row_(c.get_R(), i));

		// check if the matrices are the same
		// iterate over the parents_ map
		for (const_iterator it = beginParents(); it != endParents(); ++it) {
		  const_iterator it2 = c.beginParents() + (it-beginParents());
		  if(*it != *(it2))
		    return false;
		  rows1.push_back(row_(get_S(it), i));
		  rows2.push_back(row_(c.get_S(it2), i));
		}

		Vector row1 = concatVectors(rows1);
		Vector row2 = concatVectors(rows2);
		if (!linear_dependent(row1, row2, tol)) return false;
	}

	// check if sigmas are equal
	if (!(equal_with_abs_tol(sigmas_, c.sigmas_, tol))) return false;

	return true;
}

///* ************************************************************************* */
//void GaussianConditional::permuteWithInverse(const Permutation& inversePermutation) {
//  Conditional::permuteWithInverse(inversePermutation);
//  BOOST_FOREACH(Parents::value_type& parent, parents_) {
//    parent.first = inversePermutation[parent.first];
//  }
//#ifndef NDEBUG
//  const_iterator parent = parents_.begin();
//  Conditional::const_iterator baseParent = Conditional::parents_.begin();
//  while(parent != parents_.end())
//    assert((parent++)->first == *(baseParent++));
//  assert(baseParent == Conditional::parents_.end());
//#endif
//}
//
///* ************************************************************************* */
//bool GaussianConditional::permuteSeparatorWithInverse(const Permutation& inversePermutation) {
//  bool separatorChanged = Conditional::permuteSeparatorWithInverse(inversePermutation);
//  BOOST_FOREACH(Parents::value_type& parent, parents_) {
//    parent.first = inversePermutation[parent.first];
//  }
//#ifndef NDEBUG
//  const_iterator parent = parents_.begin();
//  Conditional::const_iterator baseParent = Conditional::parents_.begin();
//  while(parent != parents_.end())
//    assert((parent++)->first == *(baseParent++));
//  assert(baseParent == Conditional::parents_.end());
//#endif
//  return separatorChanged;
//}

/* ************************************************************************* */
Vector GaussianConditional::solve(const VectorValues& x) const {
  static const bool debug = false;
  if(debug) print("Solving conditional ");
	Vector rhs(get_d());
	for (const_iterator parent = beginParents(); parent != endParents(); ++parent) {
    ublas::axpy_prod(-get_S(parent), x[*parent], rhs, false);
//		multiplyAdd(-1.0, get_S(parent), x[*parent], rhs);
	}
	if(debug) gtsam::print(get_R(), "Calling backSubstituteUpper on ");
	if(debug) gtsam::print(rhs, "rhs: ");
	if(debug) {
	  Vector soln = backSubstituteUpper(get_R(), rhs, false);
	  gtsam::print(soln, "back-substitution solution: ");
	  return soln;
	} else
	  return backSubstituteUpper(get_R(), rhs, false);
}

/* ************************************************************************* */
Vector GaussianConditional::solve(const Permuted<VectorValues>& x) const {
  Vector rhs(get_d());
  for (const_iterator parent = beginParents(); parent != endParents(); ++parent) {
    ublas::axpy_prod(-get_S(parent), x[*parent], rhs, false);
//    multiplyAdd(-1.0, get_S(parent), x[*parent], rhs);
  }
  return backSubstituteUpper(get_R(), rhs, false);
}

}

