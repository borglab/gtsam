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
void ConditionalGaussian::print(const string &s) const
{
  cout << s << ":" << endl;
  gtsam::print(R_,"R");
  for(map<string, Matrix>::const_iterator it = parents_.begin() ; it != parents_.end() ; it++ ) {
    const string&   j = it->first;
    const Matrix& Aj = it->second;
    gtsam::print(Aj, "A["+j+"]");
  }
  gtsam::print(d_,"d");
}    

/* ************************************************************************* */
Vector ConditionalGaussian::solve(const FGConfig& x) const {
	Vector rhs = d_;
	for (map<string, Matrix>::const_iterator it = parents_.begin(); it
			!= parents_.end(); it++) {
		const string& j = it->first;
		const Matrix& Aj = it->second;
		rhs -= Aj * x[j];
	}
	Vector result = backsubstitution(R_, rhs);
	return result;
}    

/* ************************************************************************* */
bool ConditionalGaussian::equals(const ConditionalGaussian &cg) const
{
  map<string, Matrix>::const_iterator it = parents_.begin();

  // check if the size of the parents_ map is the same
  if( parents_.size() != cg.parents_.size() ) goto fail;

  // check if R_ is equal
  if( !(equal_with_abs_tol(R_, cg.R_, 0.0001) ) ) goto fail;

  // check if d_ is equal
  if( !(::equal_with_abs_tol(d_, cg.d_, 0.0001) ) ) goto fail;

  // check if the matrices are the same
  // iterate over the parents_ map
  for(it = parents_.begin(); it != parents_.end(); it++){
    map<string, Matrix>::const_iterator it2 = cg.parents_.find(it->first.c_str());
    if( it2 != cg.parents_.end() ){
      if( !(equal_with_abs_tol(it->second, it2->second, 0.0001)) ) goto fail;
    }else{
      goto fail;
    }
  } 
  return true;

 fail:
  (*this).print();
  cg.print();
  return false;

}

/* ************************************************************************* */
