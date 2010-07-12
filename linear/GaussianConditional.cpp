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
GaussianConditional::GaussianConditional(const Symbol& key,Vector d, Matrix R, Vector sigmas) :
	Conditional (key), R_(R),d_(d),sigmas_(sigmas)
{
}

/* ************************************************************************* */
GaussianConditional::GaussianConditional(const Symbol& key, Vector d, Matrix R,
		const Symbol& name1, Matrix S, Vector sigmas) :
	Conditional (key), R_(R), d_(d), sigmas_(sigmas) {
	parents_.insert(make_pair(name1, S));
}

/* ************************************************************************* */
GaussianConditional::GaussianConditional(const Symbol& key, Vector d, Matrix R,
		const Symbol& name1, Matrix S, const Symbol& name2, Matrix T, Vector sigmas) :
	Conditional (key), R_(R), d_(d),sigmas_(sigmas) {
	parents_.insert(make_pair(name1, S));
	parents_.insert(make_pair(name2, T));
}

/* ************************************************************************* */
GaussianConditional::GaussianConditional(const Symbol& key,
		const Vector& d, const Matrix& R, const SymbolMap<Matrix>& parents, Vector sigmas) :
	Conditional (key), R_(R), parents_(parents), d_(d),sigmas_(sigmas) {
}

/* ************************************************************************* */
void GaussianConditional::print(const string &s) const
{
  cout << s << ": density on " << (string)key_ << endl;
  gtsam::print(R_,"R");
  for(Parents::const_iterator it = parents_.begin() ; it != parents_.end() ; it++ ) {
    const Symbol& j = it->first;
    const Matrix& Aj = it->second;
    gtsam::print(Aj, "A["+(string)j+"]");
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

	// check if R_ and d_ are linear independent
	for (size_t i=0; i<d_.size(); i++) {
		list<Vector> rows1; rows1.push_back(row_(R_, i));
		list<Vector> rows2; rows2.push_back(row_(p->R_, i));

		// check if the matrices are the same
		// iterate over the parents_ map
		for (it = parents_.begin(); it != parents_.end(); it++) {
			Parents::const_iterator it2 = p->parents_.find(it->first);
			if (it2 != p->parents_.end()) {
				rows1.push_back(row_(it->second, i));
				rows2.push_back(row_(it2->second,i));
			} else
				return false;
		}

		Vector row1 = concatVectors(rows1);
		Vector row2 = concatVectors(rows2);
		if (!linear_dependent(row1, row2, tol)) return false;
	}

	// check if sigmas are equal
	if (!(::equal_with_abs_tol(sigmas_, p->sigmas_, tol))) return false;

	return true;
}

/* ************************************************************************* */
list<Symbol> GaussianConditional::parents() const {
	list<Symbol> result;
	for (Parents::const_iterator it = parents_.begin(); it != parents_.end(); it++)
		result.push_back(it->first);
	return result;
}

/* ************************************************************************* */
Vector GaussianConditional::solve(const VectorConfig& x) const {
	Vector rhs = d_;
	for (Parents::const_iterator it = parents_.begin(); it!= parents_.end(); it++) {
		const Symbol& j = it->first;
		const Matrix& Aj = it->second;
		multiplyAdd(-1.0,Aj,x[j],rhs);
	}
	return backSubstituteUpper(R_, rhs, false);
}

/* ************************************************************************* */
