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

#include <gtsam/base/FastVector.h>
#include <gtsam/inference/Permutation.h>
#include <gtsam/linear/VectorValues.h>

using namespace std;

namespace gtsam {

/* ************************************************************************* */
VectorValues::VectorValues(const VectorValues& other) {
  *this = other;
}

/* ************************************************************************* */
VectorValues& VectorValues::operator=(const VectorValues& rhs) {
  if(this != &rhs) {
    resizeLike(rhs);        // Copy structure
    values_ = rhs.values_;  // Copy values
  }
  return *this;
}

/* ************************************************************************* */
VectorValues VectorValues::Zero(const VectorValues& x) {
	VectorValues cloned(SameStructure(x));
	cloned.setZero();
	return cloned;
}

/* ************************************************************************* */
vector<size_t> VectorValues::dims() const {
  std::vector<size_t> result(this->size());
	for(Index j = 0; j < this->size(); ++j)
		result[j] = this->dim(j);
	return result;
}

/* ************************************************************************* */
void VectorValues::insert(Index j, const Vector& value) {
  // Make sure j does not already exist
  if(exists(j))
    throw invalid_argument("VectorValues: requested variable index to insert already exists.");

  // Get vector of dimensions
  FastVector<size_t> dimensions(size());
  for(size_t k=0; k<maps_.size(); ++k)
    dimensions[k] = maps_[k].rows();

  // If this adds variables at the end, insert zero-length entries up to j
  if(j >= size())
    dimensions.insert(dimensions.end(), j+1-size(), 0);

  // Set correct dimension for j
  dimensions[j] = value.rows();

  // Make a copy to make assignment easier
  VectorValues original(*this);

  // Resize to accomodate new variable
  resize(dimensions);

  // Copy original variables
  for(Index k = 0; k < original.size(); ++k)
    if(k != j && exists(k))
      operator[](k) = original[k];

  // Copy new variable
  operator[](j) = value;
}

/* ************************************************************************* */
void VectorValues::print(const std::string& str, const IndexFormatter& formatter) const {
	std::cout << str << ": " << size() << " elements\n";
	for (Index var = 0; var < size(); ++var)
		std::cout << "  " << formatter(var) << ": \n" << operator[](var) << "\n";
	std::cout.flush();
}

/* ************************************************************************* */
bool VectorValues::equals(const VectorValues& x, double tol) const {
	return hasSameStructure(x) && equal_with_abs_tol(values_, x.values_, tol);
}

/* ************************************************************************* */
void VectorValues::resize(Index nVars, size_t varDim) {
  maps_.clear();
  maps_.reserve(nVars);
  values_.resize(nVars * varDim);
	int varStart = 0;
	for (Index j = 0; j < nVars; ++j) {
		maps_.push_back(values_.segment(varStart, varDim));
		varStart += varDim;
	}
}

/* ************************************************************************* */
void VectorValues::resizeLike(const VectorValues& other) {
  values_.resize(other.dim());
  // Create SubVectors referencing our values_ vector
  maps_.clear();
  maps_.reserve(other.size());
  int varStart = 0;
  BOOST_FOREACH(const SubVector& value, other) {
    maps_.push_back(values_.segment(varStart, value.rows()));
    varStart += value.rows();
  }
}

/* ************************************************************************* */
VectorValues VectorValues::SameStructure(const VectorValues& other) {
  VectorValues ret;
  ret.resizeLike(other);
  return ret;
}

/* ************************************************************************* */
VectorValues VectorValues::Zero(Index nVars, size_t varDim) {
  VectorValues ret(nVars, varDim);
  ret.setZero();
  return ret;
}

/* ************************************************************************* */
void VectorValues::setZero() {
  values_.setZero();
}

/* ************************************************************************* */
bool VectorValues::hasSameStructure(const VectorValues& other) const {
  if(this->size() != other.size())
    return false;
  for(size_t j=0; j<size(); ++j)
    if(this->dim(j) != other.dim(j))
      return false;
  return true;
}

/* ************************************************************************* */
VectorValues VectorValues::operator+(const VectorValues& c) const {
	assert(this->hasSameStructure(c));
	VectorValues result(SameStructure(c));
	result.values_ = this->values_ + c.values_;
	return result;
}

/* ************************************************************************* */
VectorValues VectorValues::operator-(const VectorValues& c) const {
  assert(this->hasSameStructure(c));
  VectorValues result(SameStructure(c));
  result.values_ = this->values_ - c.values_;
  return result;
}

/* ************************************************************************* */
void VectorValues::operator+=(const VectorValues& c) {
	assert(this->hasSameStructure(c));
	this->values_ += c.values_;
}

/* ************************************************************************* */
VectorValues VectorValues::permute(const Permutation& permutation) const {
	// Create result and allocate space
	VectorValues lhs;
	lhs.values_.resize(this->dim());
	lhs.maps_.reserve(this->size());

	// Copy values from this VectorValues to the permuted VectorValues
	size_t lhsPos = 0;
	for(size_t i = 0; i < this->size(); ++i) {
		// Map the next LHS subvector to the next slice of the LHS vector
		lhs.maps_.push_back(SubVector(lhs.values_, lhsPos, this->at(permutation[i]).size()));
		// Copy the data from the RHS subvector to the LHS subvector
		lhs.maps_[i] = this->at(permutation[i]);
		// Increment lhs position
		lhsPos += lhs.maps_[i].size();
	}

	return lhs;
}

/* ************************************************************************* */
void VectorValues::swap(VectorValues& other) {
	this->values_.swap(other.values_);
	this->maps_.swap(other.maps_);
}

}