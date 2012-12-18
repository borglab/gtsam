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
VectorValues VectorValues::Zero(const VectorValues& x) {
  VectorValues result;
  result.values_.resize(x.size());
  for(size_t j=0; j<x.size(); ++j)
    result.values_[j] = Vector::Zero(x.dim(j));
  return result;
}

/* ************************************************************************* */
vector<size_t> VectorValues::dims() const {
  vector<size_t> result(this->size());
  for(Index j = 0; j < this->size(); ++j)
    result[j] = this->dim(j);
  return result;
}

/* ************************************************************************* */
void VectorValues::insert(Index j, const Vector& value) {
  // Make sure j does not already exist
  if(exists(j))
    throw invalid_argument("VectorValues: requested variable index to insert already exists.");

  // If this adds variables at the end, insert zero-length entries up to j
  if(j >= size())
    values_.resize(j+1);

  // Assign value
  values_[j] = value;
}

/* ************************************************************************* */
void VectorValues::print(const std::string& str, const IndexFormatter& formatter) const {
  std::cout << str << ": " << size() << " elements\n";
  for (Index var = 0; var < size(); ++var)
    std::cout << "  " << formatter(var) << ": \n" << (*this)[var] << "\n";
  std::cout.flush();
}

/* ************************************************************************* */
bool VectorValues::equals(const VectorValues& x, double tol) const {
  if(this->size() != x.size())
    return false;
  for(size_t j=0; j<size(); ++j)
    if(!equal_with_abs_tol(values_[j], x.values_[j], tol))
      return false;
  return true;
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
    // Directly accessing maps instead of using VV::dim in case some values are empty
    if(this->maps_[j].rows() != other.maps_[j].rows())
      return false;
  return true;
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

/* ************************************************************************* */
double VectorValues::dot(const VectorValues& V) const {

}

/* ************************************************************************* */
double VectorValues::norm() const {

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
Vector VectorValues::vector(const std::vector<Index>& indices) const {
  if (indices.empty())
    return Vector();

  // find dimensions
  size_t d = 0;
  BOOST_FOREACH(const Index& idx, indices)
    d += dim(idx);

  // copy out values
  Vector result(d);
  size_t curHead = 0;
  BOOST_FOREACH(const Index& j, indices) {
    const SubVector& vj = at(j);
    size_t dj = (size_t) vj.rows();
    result.segment(curHead, dj) = vj;
    curHead += dj;
  }
  return result;
}

/* ************************************************************************* */

} // \namespace gtsam
