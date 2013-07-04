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

#include <gtsam/linear/VectorValuesUnordered.h>

#include <boost/range/combine.hpp>
#include <boost/range/numeric.hpp>
#include <boost/range/adaptor/map.hpp>
#include <boost/range/adaptor/transformed.hpp>

using namespace std;

namespace gtsam {

/* ************************************************************************* */
void VectorValuesUnordered::print(const std::string& str, const KeyFormatter& formatter) const {
  std::cout << str << ": " << size() << " elements\n";
  BOOST_FOREACH(const value_type& key_value, *this)
    std::cout << "  " << formatter(key_value.first) << ": \n" << key_value.second.transpose() << "\n";
  std::cout.flush();
}

/* ************************************************************************* */
bool VectorValuesUnordered::equals(const VectorValuesUnordered& x, double tol) const {
  if(this->size() != x.size())
    return false;
  typedef boost::tuple<value_type, value_type> ValuePair;
  BOOST_FOREACH(const ValuePair& values, boost::combine(*this, x)) {
    if(values.get<0>.first != values.get<1>.first ||
      !equal_with_abs_tol(values.get<0>.second, values.get<1>.second, tol))
      return false;
  }
  return true;
}

/* ************************************************************************* */
const Vector VectorValuesUnordered::asVector() const
{
  using boost::adaptors::map_values;
  using boost::adaptors::transformed;

  // Count dimensions
  const DenseIndex totalDim = boost::accumulate(*this | map_values | transformed(&Vector::size), 0);

  // Copy vectors
  Vector result;
  DenseIndex pos = 0;
  BOOST_FOREACH(const Vector& v, *this | map_values) {
    result.segment(pos, v.size()) = v;
    pos += v.size();
  }

  return result;
}

/* ************************************************************************* */
const Vector VectorValuesUnordered::vector(const std::vector<Key>& keys) const
{
  // Count dimensions and collect pointers to avoid double lookups
  DenseIndex totalDim = 0;
  std::vector<const Vector*> items(keys.size());
  for(size_t i = 0; i < keys.size(); ++i) {
    items[i] = &at(i);
    totalDim += items[i]->size();
  }

  // Copy vectors
  Vector result(totalDim);
  DenseIndex pos = 0;
  BOOST_FOREACH(const Vector *v, items) {
    result.segment(pos, v->size()) = *v;
    pos += v->size();
  }

  return result;
}

/* ************************************************************************* */
void VectorValuesUnordered::swap(VectorValuesUnordered& other) {
  this->values_.swap(other.values_);
}

/* ************************************************************************* */
double VectorValuesUnordered::dot(const VectorValuesUnordered& v) const
{
  if(this->size() != v.size())
    throw invalid_argument("VectorValues::dot called with a VectorValues of different structure");
  double result = 0.0;
  typedef boost::tuple<value_type, value_type> ValuePair;
  using boost::adaptors::map_values;
  BOOST_FOREACH(const ValuePair& values, boost::combine(*this, v)) {
    assert_throw(values.get<0>().first == values.get<1>.first,
      std::invalid_argument("VectorValues::dot called with a VectorValues of different structure"));
    assert_throw(values.get<0>().second.size() == values.get<1>().second.size(),
      std::invalid_argument("VectorValues::dot called with a VectorValues of different structure"));
    result += values.get<0>().second.dot(values.get<1>().second);
  }
  return result;
}

/* ************************************************************************* */
double VectorValuesUnordered::norm() const {
  return std::sqrt(this->squaredNorm());
}

/* ************************************************************************* */
double VectorValuesUnordered::squaredNorm() const {
  double sumSquares = 0.0;
  using boost::adaptors::map_values;
  BOOST_FOREACH(const Vector& v, *this | map_values)
    sumSquares += v.squaredNorm();
  return sumSquares;
}

/* ************************************************************************* */
VectorValuesUnordered VectorValuesUnordered::operator+(const VectorValuesUnordered& c) const {
  VectorValuesUnordered result = SameStructure(*this);
  if(this->size() != c.size())
    throw invalid_argument("VectorValues::operator+ called with different vector sizes");
  for(Index j = 0; j < this->size(); ++j)
    // Directly accessing maps instead of using VV::dim in case some values are empty
    if(this->values_[j].size() == c.values_[j].size())
      result.values_[j] = this->values_[j] + c.values_[j];
    else
      throw invalid_argument("VectorValues::operator- called with different vector sizes");
  return result;
}

/* ************************************************************************* */
VectorValuesUnordered VectorValuesUnordered::operator-(const VectorValuesUnordered& c) const {
  VectorValuesUnordered result = SameStructure(*this);
  if(this->size() != c.size())
    throw invalid_argument("VectorValues::operator- called with different vector sizes");
  for(Index j = 0; j < this->size(); ++j)
    // Directly accessing maps instead of using VV::dim in case some values are empty
    if(this->values_[j].size() == c.values_[j].size())
      result.values_[j] = this->values_[j] - c.values_[j];
    else
      throw invalid_argument("VectorValues::operator- called with different vector sizes");
  return result;
}

/* ************************************************************************* */
void VectorValuesUnordered::operator+=(const VectorValuesUnordered& c) {
  if(this->size() != c.size())
    throw invalid_argument("VectorValues::operator+= called with different vector sizes");
  for(Index j = 0; j < this->size(); ++j)
    // Directly accessing maps instead of using VV::dim in case some values are empty
    if(this->values_[j].size() == c.values_[j].size())
      this->values_[j] += c.values_[j];
    else
      throw invalid_argument("VectorValues::operator+= called with different vector sizes");
}

/* ************************************************************************* */

} // \namespace gtsam
