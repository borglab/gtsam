/**
 * @file   ConditionalGaussian.cpp
 * @brief  Conditional Gaussian Base class
 * @author Christian Potthast
 */


#include <string.h>
#include <boost/numeric/ublas/vector.hpp>
#include "ConditionalGaussian.h"

using namespace std;
using namespace gtsam;

/* ************************************************************************* */
ConditionalGaussian::ConditionalGaussian(Vector d,Matrix R) : R_(R),d_(d)
{
}

/* ************************************************************************* */
ConditionalGaussian::ConditionalGaussian(Vector d,
					 Matrix R,
					 const string& name1,
					 Matrix S)
  : R_(R),d_(d)
{
  parents_.insert(make_pair(name1, S));
}

/* ************************************************************************* */
ConditionalGaussian::ConditionalGaussian(Vector d,
					 Matrix R,
					 const string& name1,
					 Matrix S,
					 const string& name2,
					 Matrix T)
  : R_(R),d_(d)
{
  parents_.insert(make_pair(name1, S));
  parents_.insert(make_pair(name2, T));
}

/* ************************************************************************* */
ConditionalGaussian::ConditionalGaussian(const Vector& d,
    		const Matrix& R,
    		const map<string, Matrix>& parents)
 : R_(R), d_(d), parents_(parents)
 {
 }

/* ************************************************************************* */
void ConditionalGaussian::print(const string &s) const
{
  cout << s << ":" << endl;
  gtsam::print(R_,"R");
  for(Parents::const_iterator it = parents_.begin() ; it != parents_.end() ; it++ ) {
    const string&   j = it->first;
    const Matrix& Aj = it->second;
    gtsam::print(Aj, "A["+j+"]");
  }
  gtsam::print(d_,"d");
}    

/* ************************************************************************* */
bool ConditionalGaussian::equals(const ConditionalGaussian &cg, double tol) const {
	Parents::const_iterator it = parents_.begin();

	// check if the size of the parents_ map is the same
	if (parents_.size() != cg.parents_.size()) return false;

	// check if R_ is equal
	if (!(equal_with_abs_tol(R_, cg.R_, tol))) return false;

	// check if d_ is equal
	if (!(::equal_with_abs_tol(d_, cg.d_, tol))) return false;

	// check if the matrices are the same
	// iterate over the parents_ map
	for (it = parents_.begin(); it != parents_.end(); it++) {
		Parents::const_iterator it2 = cg.parents_.find(it->first.c_str());
		if (it2 != cg.parents_.end()) {
			if (!(equal_with_abs_tol(it->second, it2->second, tol))) return false;
		} else
			return false;
	}
	return true;
}

/* ************************************************************************* */
list<string> ConditionalGaussian::parents() {
	list<string> result;
	for (Parents::const_iterator it = parents_.begin(); it != parents_.end(); it++)
		result.push_back(it->first);
}

/* ************************************************************************* */
Vector ConditionalGaussian::solve(const VectorConfig& x) const {
	Vector rhs = d_;
	for (Parents::const_iterator it = parents_.begin(); it
			!= parents_.end(); it++) {
		const string& j = it->first;
		const Matrix& Aj = it->second;
		rhs -= Aj * x[j];
	}
	Vector result = backsubstitution(R_, rhs);
	return result;
}

/* ************************************************************************* */
