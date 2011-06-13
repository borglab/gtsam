/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file VectorValues.cpp
 * @brief Implementations for VectorValues
 * @author Richard Roberts
 * @author Alex Cunningham
 */

#include <gtsam/linear/VectorValues.h>

using namespace std;
using namespace gtsam;

/* ************************************************************************* */
VectorValues::VectorValues(Index nVars, size_t varDim) : varStarts_(nVars+1) {
	varStarts_[0] = 0;
	size_t varStart = 0;
	for(Index j=1; j<=nVars; ++j)
		varStarts_[j] = (varStart += varDim);
	values_.resize(varStarts_.back());
}

/* ************************************************************************* */
VectorValues::VectorValues(const std::vector<size_t>& dimensions, const Vector& values) :
        		values_(values), varStarts_(dimensions.size()+1) {
	varStarts_[0] = 0;
	size_t varStart = 0;
	Index var = 0;
	BOOST_FOREACH(size_t dim, dimensions) {
		varStarts_[++var] = (varStart += dim);
	}
	assert(varStarts_.back() == (size_t) values.size());
}

/* ************************************************************************* */
VectorValues::VectorValues(const std::vector<size_t>& dimensions, const double* values) :
						varStarts_(dimensions.size()+1) {
	varStarts_[0] = 0;
	size_t varStart = 0;
	Index var = 0;
	BOOST_FOREACH(size_t dim, dimensions) {
		varStarts_[++var] = (varStart += dim);
	}
	values_ = Vector_(varStart, values);
}

/* ************************************************************************* */
VectorValues VectorValues::SameStructure(const VectorValues& otherValues) {
	VectorValues ret;
	ret.varStarts_ = otherValues.varStarts_;
	ret.values_.resize(ret.varStarts_.back(), false);
	return ret;
}

/* ************************************************************************* */
VectorValues::mapped_type VectorValues::operator[](Index variable) {
	checkVariable(variable);
	const size_t start = varStarts_[variable], n = varStarts_[variable+1] - start;
	return values_.segment(start, n);
}

/* ************************************************************************* */
VectorValues::const_mapped_type VectorValues::operator[](Index variable) const {
	checkVariable(variable);
	const size_t start = varStarts_[variable], n = varStarts_[variable+1] - start;
	return values_.segment(start, n);
}

/* ************************************************************************* */
Index VectorValues::push_back_preallocated(const Vector& vector) {
	Index var = varStarts_.size()-1;
	varStarts_.push_back(varStarts_.back()+vector.size());
	this->operator[](var) = vector;  // This will assert that values_ has enough allocated space.
	return var;
}

/* ************************************************************************* */
void VectorValues::print(const std::string& str) const {
	std::cout << str << ": " << varStarts_.size()-1 << " elements\n";
	for(Index var=0; var<size(); ++var) {
		std::cout << "  " << var << ": \n" << operator[](var) << "\n";
	}
	std::cout.flush();
}

/* ************************************************************************* */
bool VectorValues::equals(const VectorValues& x, double tol) const {
	return varStarts_ == x.varStarts_ && equal_with_abs_tol(values_, x.values_, tol);
}

/* ************************************************************************* */
VectorValues VectorValues::operator+(const VectorValues& c) const {
	assert(varStarts_ == c.varStarts_);
	VectorValues result;
	result.varStarts_ = varStarts_;
	result.values_ = values_.head(varStarts_.back()) + c.values_.head(varStarts_.back());
	return result;
}

/* ************************************************************************* */
void VectorValues::operator+=(const VectorValues& c) {
	assert(varStarts_ == c.varStarts_);
	this->values_ += c.values_.head(varStarts_.back());
}

/* ************************************************************************* */
VectorValues VectorValues::zero(const VectorValues& x) {
	VectorValues cloned(x);
	cloned.makeZero();
	return cloned;
}

/* ************************************************************************* */
size_t VectorValues::dim(Index variable) const {
  checkVariable(variable);
  const size_t start = varStarts_[variable], n = varStarts_[variable+1] - start;
  return n;
}

/* ************************************************************************* */


