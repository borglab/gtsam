/**
 * @file   GaussianConditional.cpp
 * @brief  Conditional Gaussian Base class
 * @author Christian Potthast
 */

#include <string.h>
#include <boost/numeric/ublas/vector.hpp>
#include "Ordering.h"
#include "GaussianConditional.h"

using namespace std;
using namespace gtsam;

/* ************************************************************************* */
GaussianConditional::GaussianConditional(const string& key,Vector d, Matrix R, Vector sigmas) :
	Conditional (key), R_(R),sigmas_(sigmas),d_(d)
{
}

/* ************************************************************************* */
GaussianConditional::GaussianConditional(const string& key, Vector d, Matrix R,
		const string& name1, Matrix S, Vector sigmas) :
	Conditional (key), R_(R), sigmas_(sigmas), d_(d) {
	parents_.insert(make_pair(name1, S));
}

/* ************************************************************************* */
GaussianConditional::GaussianConditional(const string& key, Vector d, Matrix R,
		const string& name1, Matrix S, const string& name2, Matrix T, Vector sigmas) :
	Conditional (key), R_(R),sigmas_(sigmas), d_(d) {
	parents_.insert(make_pair(name1, S));
	parents_.insert(make_pair(name2, T));
}

/* ************************************************************************* */
GaussianConditional::GaussianConditional(const string& key,
		const Vector& d, const Matrix& R, const map<string, Matrix>& parents, Vector sigmas) :
	Conditional (key), R_(R),sigmas_(sigmas), d_(d), parents_(parents) {
}

/* ************************************************************************* */
void GaussianConditional::print(const string &s) const
{
  cout << s << ": density on " << key_ << endl;
  gtsam::print(R_,"R");
  for(Parents::const_iterator it = parents_.begin() ; it != parents_.end() ; it++ ) {
    const string&   j = it->first;
    const Matrix& Aj = it->second;
    gtsam::print(Aj, "A["+j+"]");
  }
  gtsam::print(d_,"d");
  gtsam::print(sigmas_,"sigmas");
}    

/* ************************************************************************* */
bool GaussianConditional::equals(const Conditional &c, double tol) const {
	if (!Conditional::equals(c)) return false;
	const GaussianConditional* p = dynamic_cast<const GaussianConditional*> (&c);
	if (p == NULL) return false;
	Parents::const_iterator it = parents_.begin();

	// check if the size of the parents_ map is the same
	if (parents_.size() != p->parents_.size()) return false;

	// check if R_ is equal
	if (!(equal_with_abs_tol(R_, p->R_, tol))) return false;

	// check if d_ is equal
	if (!(::equal_with_abs_tol(d_, p->d_, tol))) return false;

	// check if sigmas are equal
	if (!(::equal_with_abs_tol(sigmas_, p->sigmas_, tol))) return false;

	// check if the matrices are the same
	// iterate over the parents_ map
	for (it = parents_.begin(); it != parents_.end(); it++) {
		Parents::const_iterator it2 = p->parents_.find(it->first.c_str());
		if (it2 != p->parents_.end()) {
			if (!(equal_with_abs_tol(it->second, it2->second, tol))) return false;
		} else
			return false;
	}
	return true;
}

/* ************************************************************************* */
list<string> GaussianConditional::parents() const {
	list<string> result;
	for (Parents::const_iterator it = parents_.begin(); it != parents_.end(); it++)
		result.push_back(it->first);
	return result;
}

/* ************************************************************************* */
Vector GaussianConditional::solve(const VectorConfig& x) const {
	Vector rhs = d_;
	for (Parents::const_iterator it = parents_.begin(); it
			!= parents_.end(); it++) {
		const string& j = it->first;
		const Matrix& Aj = it->second;
		rhs -= Aj * x[j];
	}
	Vector result = backSubstituteUpper(R_, rhs, true);
	return result;
}

/* ************************************************************************* */
