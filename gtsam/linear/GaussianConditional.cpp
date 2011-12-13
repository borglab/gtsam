/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file   GaussianConditional.cpp
 * @brief  Conditional Gaussian Base class
 * @author Christian Potthast
 */

#include <string.h>
#include <boost/format.hpp>
#include <boost/lambda/bind.hpp>

#include <gtsam/linear/GaussianConditional.h>
#include <gtsam/linear/GaussianFactor.h>
#include <gtsam/linear/JacobianFactor.h>

using namespace std;

namespace gtsam {

/* ************************************************************************* */
// Helper function used only in this file - extracts vectors with variable indices
// in the first and last iterators, and concatenates them in that order into the
// output.
template<typename ITERATOR>
static Vector extractVectorValuesSlices(const VectorValues& values, ITERATOR first, ITERATOR last) {
  // Find total dimensionality
  int dim = 0;
  for(ITERATOR j = first; j != last; ++j)
    dim += values.dim(*j);

  // Copy vectors
  Vector ret(dim);
  int varStart = 0;
  for(ITERATOR j = first; j != last; ++j) {
    ret.segment(varStart, values.dim(*j)) = values[*j];
    varStart += values.dim(*j);
  }
  return ret;
}

/* ************************************************************************* */
// Helper function used only in this file - writes to the variables in values
// with indices iterated over by first and last, interpreting vector as the
// concatenated vectors to write.
template<class VECTOR, typename ITERATOR>
static void writeVectorValuesSlices(const VECTOR& vector, VectorValues& values, ITERATOR first, ITERATOR last) {
  // Copy vectors
  int varStart = 0;
  for(ITERATOR j = first; j != last; ++j) {
    values[*j] = vector.segment(varStart, values.dim(*j));
    varStart += values.dim(*j);
  }
  assert(varStart = vector.rows());
}

/* ************************************************************************* */
GaussianConditional::GaussianConditional() : rsd_(matrix_) {}

/* ************************************************************************* */
GaussianConditional::GaussianConditional(Index key) : IndexConditional(key), rsd_(matrix_) {}

/* ************************************************************************* */
GaussianConditional::GaussianConditional(Index key,const Vector& d, const Matrix& R, const Vector& sigmas) :
	    IndexConditional(key), rsd_(matrix_), sigmas_(sigmas) {
  assert(R.rows() <= R.cols());
  size_t dims[] = { R.cols(), 1 };
  rsd_.copyStructureFrom(rsd_type(matrix_, dims, dims+2, d.size()));
  rsd_(0) = R.triangularView<Eigen::Upper>();
  get_d_() = d;
}

/* ************************************************************************* */
GaussianConditional::GaussianConditional(Index key, const Vector& d, const Matrix& R,
    Index name1, const Matrix& S, const Vector& sigmas) :
    IndexConditional(key,name1), rsd_(matrix_), sigmas_(sigmas) {
  assert(R.rows() <= R.cols());
  size_t dims[] = { R.cols(), S.cols(), 1 };
  rsd_.copyStructureFrom(rsd_type(matrix_, dims, dims+3, d.size()));
  rsd_(0) = R.triangularView<Eigen::Upper>();
  rsd_(1) = S;
  get_d_() = d;
}

/* ************************************************************************* */
GaussianConditional::GaussianConditional(Index key, const Vector& d, const Matrix& R,
		Index name1, const Matrix& S, Index name2, const Matrix& T, const Vector& sigmas) :
		IndexConditional(key,name1,name2), rsd_(matrix_), sigmas_(sigmas) {
  assert(R.rows() <= R.cols());
  size_t dims[] = { R.cols(), S.cols(), T.cols(), 1 };
  rsd_.copyStructureFrom(rsd_type(matrix_, dims, dims+4, d.size()));
  rsd_(0) = R.triangularView<Eigen::Upper>();
  rsd_(1) = S;
  rsd_(2) = T;
  get_d_() = d;
}

/* ************************************************************************* */
	GaussianConditional::GaussianConditional(Index key, const Vector& d,
			const Matrix& R, const list<pair<Index, Matrix> >& parents, const Vector& sigmas) :
		IndexConditional(key, GetKeys(parents.size(), parents.begin(), parents.end())), rsd_(matrix_), sigmas_(sigmas) {
  assert(R.rows() <= R.cols());
  size_t dims[1+parents.size()+1];
  dims[0] = R.cols();
  size_t j=1;
  std::list<std::pair<Index, Matrix> >::const_iterator parent=parents.begin();
  for(; parent!=parents.end(); ++parent,++j)
    dims[j] = parent->second.cols();
  dims[j] = 1;
  rsd_.copyStructureFrom(rsd_type(matrix_, dims, dims+1+parents.size()+1, d.size()));
  rsd_(0) = R.triangularView<Eigen::Upper>();
  j = 1;
  for(std::list<std::pair<Index, Matrix> >::const_iterator parent=parents.begin(); parent!=parents.end(); ++parent) {
    rsd_(j).noalias() = parent->second;
    ++ j;
  }
  get_d_() = d;
}

/* ************************************************************************* */
GaussianConditional::GaussianConditional(const std::list<std::pair<Index, Matrix> >& terms,
    const size_t nrFrontals, const Vector& d, const Vector& sigmas) :
    IndexConditional(GetKeys(terms.size(), terms.begin(), terms.end()), nrFrontals),
    rsd_(matrix_), sigmas_(sigmas) {
  size_t dims[terms.size()+1];
  size_t j=0;
  typedef pair<Index, Matrix> Index_Matrix;
  BOOST_FOREACH(const Index_Matrix& term, terms) {
    dims[j] = term.second.cols();
    ++ j;
  }
  dims[j] = 1;
  rsd_.copyStructureFrom(rsd_type(matrix_, dims, dims+terms.size()+1, d.size()));
  j=0;
  BOOST_FOREACH(const Index_Matrix& term, terms) {
    rsd_(j) = term.second;
    ++ j;
  }
  get_d_() = d;
}

/* ************************************************************************* */
void GaussianConditional::print(const string &s) const
{
  cout << s << ": density on ";
  for(const_iterator it = beginFrontals(); it != endFrontals(); ++it) {
  	cout << (boost::format("[%1%]")%(*it)).str() << " ";
  }
  cout << endl;
  gtsam::print(Matrix(get_R()),"R");
  for(const_iterator it = beginParents() ; it != endParents() ; ++it ) {
    gtsam::print(Matrix(get_S(it)), (boost::format("A[%1%]")%(*it)).str());
  }
  gtsam::print(Vector(get_d()),"d");
  gtsam::print(sigmas_,"sigmas");
  cout << "Permutation: " << permutation_.indices().transpose() << endl;
}    

/* ************************************************************************* */
bool GaussianConditional::equals(const GaussianConditional &c, double tol) const {
	// check if the size of the parents_ map is the same
	if (parents().size() != c.parents().size())
		return false;

	// check if R_ and d_ are linear independent
	for (size_t i=0; i<rsd_.rows(); i++) {
		list<Vector> rows1; rows1.push_back(Vector(get_R().row(i)));
		list<Vector> rows2; rows2.push_back(Vector(c.get_R().row(i)));

		// check if the matrices are the same
		// iterate over the parents_ map
		for (const_iterator it = beginParents(); it != endParents(); ++it) {
		  const_iterator it2 = c.beginParents() + (it-beginParents());
		  if(*it != *(it2))
		    return false;
		  rows1.push_back(row(get_S(it), i));
		  rows2.push_back(row(c.get_S(it2), i));
		}

		Vector row1 = concatVectors(rows1);
		Vector row2 = concatVectors(rows2);
		if (!linear_dependent(row1, row2, tol))
			return false;
	}

	// check if sigmas are equal
	if (!(equal_with_abs_tol(sigmas_, c.sigmas_, tol))) return false;

	return true;
}

/* ************************************************************************* */
JacobianFactor::shared_ptr GaussianConditional::toFactor() const {
  return JacobianFactor::shared_ptr(new JacobianFactor(*this));
}

/* ************************************************************************* */
void GaussianConditional::rhs(VectorValues& x) const {
  writeVectorValuesSlices(get_d(), x, beginFrontals(), endFrontals());
}

/* ************************************************************************* */
void GaussianConditional::rhs(Permuted<VectorValues>& x) const {
  // Copy the rhs into x, accounting for the permutation
  Vector d = get_d();
  size_t rhsPosition = 0;  // We walk through the rhs by variable
  for(const_iterator j = beginFrontals(); j != endFrontals(); ++j) {
    // Get the segment of the rhs for this variable
    x[*j] = d.segment(rhsPosition, this->dim(j));
    // Increment the position
    rhsPosition += this->dim(j);
  }
}

/* ************************************************************************* */
void GaussianConditional::solveInPlace(VectorValues& x) const {
  static const bool debug = false;
  if(debug) print("Solving conditional in place");
  Vector rhs = extractVectorValuesSlices(x, beginFrontals(), endFrontals());
	for (const_iterator parent = beginParents(); parent != endParents(); ++parent) {
		rhs += -get_S(parent) * x[*parent];
	}
	Vector soln = permutation_.transpose() * get_R().triangularView<Eigen::Upper>().solve(rhs);
	if(debug) {
		gtsam::print(Matrix(get_R()), "Calling backSubstituteUpper on ");
		gtsam::print(rhs, "rhs: ");
	  gtsam::print(soln, "full back-substitution solution: ");
	}
	writeVectorValuesSlices(soln, x, beginFrontals(), endFrontals());
}

/* ************************************************************************* */
void GaussianConditional::solveInPlace(Permuted<VectorValues>& x) const {
  static const bool debug = false;
  if(debug) print("Solving conditional in place (permuted)");
	// Extract RHS from values - inlined from VectorValues
	size_t s = 0;
	for (const_iterator it=beginFrontals(); it!=endFrontals(); ++it)
		s += x[*it].size();
	Vector rhs(s); size_t start = 0;
	for (const_iterator it=beginFrontals(); it!=endFrontals(); ++it) {
		SubVector v = x[*it];
		const size_t d = v.size();
		rhs.segment(start, d) = v;
		start += d;
	}

	// apply parents to rhs
	for (const_iterator parent = beginParents(); parent != endParents(); ++parent) {
		rhs += -get_S(parent) * x[*parent];
	}

	// solve system - backsubstitution
	Vector soln = permutation_.transpose() * get_R().triangularView<Eigen::Upper>().solve(rhs);

	// apply solution: inlined manually due to permutation
  size_t solnStart = 0;
  for (const_iterator frontal = beginFrontals(); frontal != endFrontals(); ++frontal) {
  	const size_t d = this->dim(frontal);
  	x[*frontal] = soln.segment(solnStart, d);
  	solnStart += d;
  }
}

/* ************************************************************************* */
VectorValues GaussianConditional::solve(const VectorValues& x) const {
	VectorValues result = x;
	solveInPlace(result);
	return result;
}

/* ************************************************************************* */
void GaussianConditional::solveTransposeInPlace(VectorValues& gy) const {
	Vector frontalVec = extractVectorValuesSlices(gy, beginFrontals(), endFrontals());
	// TODO: verify permutation
	frontalVec = permutation_ * gtsam::backSubstituteUpper(frontalVec,Matrix(get_R()));
	GaussianConditional::const_iterator it;
	for (it = beginParents(); it!= endParents(); it++) {
		const Index i = *it;
		transposeMultiplyAdd(-1.0,get_S(it),frontalVec,gy[i]);
	}
	writeVectorValuesSlices(frontalVec, gy, beginFrontals(), endFrontals());
}

/* ************************************************************************* */
void GaussianConditional::scaleFrontalsBySigma(VectorValues& gy) const {
	Vector frontalVec = extractVectorValuesSlices(gy, beginFrontals(), endFrontals());
	frontalVec = emul(frontalVec, get_sigmas());
	writeVectorValuesSlices(frontalVec, gy, beginFrontals(), endFrontals());
}

}

