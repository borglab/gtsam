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

#include <boost/bind.hpp>
#include <boost/range/combine.hpp>
#include <boost/range/numeric.hpp>
#include <boost/range/adaptor/transformed.hpp>
#include <boost/range/adaptor/map.hpp>

using namespace std;

namespace gtsam {

  using boost::combine;
  using boost::adaptors::transformed;
  using boost::adaptors::map_values;
  using boost::accumulate;

  /* ************************************************************************* */
  VectorValues::VectorValues(const VectorValues& first, const VectorValues& second)
  {
    // Merge using predicate for comparing first of pair
    merge(first.begin(), first.end(), second.begin(), second.end(), inserter(values_, values_.end()),
      boost::bind(&less<Key>::operator(), less<Key>(), boost::bind(&KeyValuePair::first, _1), boost::bind(&KeyValuePair::first, _2)));
    if(size() != first.size() + second.size())
      throw invalid_argument("Requested to merge two VectorValues that have one or more variables in common.");
  }

  /* ************************************************************************* */
  VectorValues::VectorValues(const Vector& x, const Dims& dims) {
    typedef pair<Key, size_t> Pair;
    size_t j = 0;
    for (const Pair& v : dims) {
      Key key;
      size_t n;
      boost::tie(key, n) = v;
      values_.insert(make_pair(key, x.segment(j, n)));
      j += n;
    }
  }

  /* ************************************************************************* */
  VectorValues::VectorValues(const Vector& x, const Scatter& scatter) {
    size_t j = 0;
    for (const SlotEntry& v : scatter) {
      values_.insert(make_pair(v.key, x.segment(j, v.dimension)));
      j += v.dimension;
    }
  }

  /* ************************************************************************* */
  VectorValues VectorValues::Zero(const VectorValues& other)
  {
    VectorValues result;
    for(const KeyValuePair& v: other)
      result.values_.insert(make_pair(v.first, Vector::Zero(v.second.size())));
    return result;
  }

  /* ************************************************************************* */
  VectorValues::iterator VectorValues::insert(const std::pair<Key, Vector>& key_value) {
    // Note that here we accept a pair with a reference to the Vector, but the Vector is copied as
    // it is inserted into the values_ map.
    std::pair<iterator, bool> result = values_.insert(key_value);
    if(!result.second)
      throw std::invalid_argument(
      "Requested to insert variable '" + DefaultKeyFormatter(key_value.first)
      + "' already in this VectorValues.");
    return result.first;
  }

  /* ************************************************************************* */
  void VectorValues::update(const VectorValues& values)
  {
    iterator hint = begin();
    for(const KeyValuePair& key_value: values)
    {
      // Use this trick to find the value using a hint, since we are inserting from another sorted map
      size_t oldSize = values_.size();
      hint = values_.insert(hint, key_value);
      if(values_.size() > oldSize) {
        values_.unsafe_erase(hint);
        throw out_of_range("Requested to update a VectorValues with another VectorValues that contains keys not present in the first.");
      } else {
        hint->second = key_value.second;
      }
    }
  }

  /* ************************************************************************* */
  void VectorValues::insert(const VectorValues& values)
  {
    size_t originalSize = size();
    values_.insert(values.begin(), values.end());
    if(size() != originalSize + values.size())
      throw invalid_argument("Requested to insert a VectorValues into another VectorValues that already contains one or more of its keys.");
  }

  /* ************************************************************************* */
  void VectorValues::setZero()
  {
    for(Vector& v: values_ | map_values)
      v.setZero();
  }

  /* ************************************************************************* */
  void VectorValues::print(const string& str, const KeyFormatter& formatter) const {
    cout << str << ": " << size() << " elements\n";
    for(const value_type& key_value: *this)
      cout << "  " << formatter(key_value.first) << ": " << key_value.second.transpose() << "\n";
    cout.flush();
  }

  /* ************************************************************************* */
  bool VectorValues::equals(const VectorValues& x, double tol) const {
    if(this->size() != x.size())
      return false;
    typedef boost::tuple<value_type, value_type> ValuePair;
    for(const ValuePair& values: boost::combine(*this, x)) {
      if(values.get<0>().first != values.get<1>().first ||
        !equal_with_abs_tol(values.get<0>().second, values.get<1>().second, tol))
        return false;
    }
    return true;
  }

  /* ************************************************************************* */
  Vector VectorValues::vector() const
  {
    // Count dimensions
    DenseIndex totalDim = 0;
    for(const Vector& v: *this | map_values)
      totalDim += v.size();

    // Copy vectors
    Vector result(totalDim);
    DenseIndex pos = 0;
    for(const Vector& v: *this | map_values) {
      result.segment(pos, v.size()) = v;
      pos += v.size();
    }

    return result;
  }

  /* ************************************************************************* */
  Vector VectorValues::vector(const FastVector<Key>& keys) const
  {
    // Count dimensions and collect pointers to avoid double lookups
    DenseIndex totalDim = 0;
    FastVector<const Vector*> items(keys.size());
    for(size_t i = 0; i < keys.size(); ++i) {
      items[i] = &at(keys[i]);
      totalDim += items[i]->size();
    }

    // Copy vectors
    Vector result(totalDim);
    DenseIndex pos = 0;
    for(const Vector *v: items) {
      result.segment(pos, v->size()) = *v;
      pos += v->size();
    }

    return result;
  }

  /* ************************************************************************* */
  Vector VectorValues::vector(const Dims& keys) const
  {
    // Count dimensions
    DenseIndex totalDim = 0;
    for(size_t dim: keys | map_values)
      totalDim += dim;
    Vector result(totalDim);
    size_t j = 0;
    for(const Dims::value_type& it: keys) {
      result.segment(j,it.second) = at(it.first);
      j += it.second;
    }
    return result;
  }

  /* ************************************************************************* */
  void VectorValues::swap(VectorValues& other) {
    this->values_.swap(other.values_);
  }

  /* ************************************************************************* */
  namespace internal
  {
    bool structureCompareOp(const boost::tuple<VectorValues::value_type,
      VectorValues::value_type>& vv)
    {
      return vv.get<0>().first == vv.get<1>().first
        && vv.get<0>().second.size() == vv.get<1>().second.size();
    }
  }

  /* ************************************************************************* */
  bool VectorValues::hasSameStructure(const VectorValues other) const
  {
    return accumulate(combine(*this, other)
      | transformed(internal::structureCompareOp), true, logical_and<bool>());
  }

  /* ************************************************************************* */
  double VectorValues::dot(const VectorValues& v) const
  {
    if(this->size() != v.size())
      throw invalid_argument("VectorValues::dot called with a VectorValues of different structure");
    double result = 0.0;
    typedef boost::tuple<value_type, value_type> ValuePair;
    using boost::adaptors::map_values;
    for(const ValuePair& values: boost::combine(*this, v)) {
      assert_throw(values.get<0>().first == values.get<1>().first,
        invalid_argument("VectorValues::dot called with a VectorValues of different structure"));
      assert_throw(values.get<0>().second.size() == values.get<1>().second.size(),
        invalid_argument("VectorValues::dot called with a VectorValues of different structure"));
      result += values.get<0>().second.dot(values.get<1>().second);
    }
    return result;
  }

  /* ************************************************************************* */
  double VectorValues::norm() const {
    return std::sqrt(this->squaredNorm());
  }

  /* ************************************************************************* */
  double VectorValues::squaredNorm() const {
    double sumSquares = 0.0;
    using boost::adaptors::map_values;
    for(const Vector& v: *this | map_values)
      sumSquares += v.squaredNorm();
    return sumSquares;
  }

  /* ************************************************************************* */
  VectorValues VectorValues::operator+(const VectorValues& c) const
  {
    if(this->size() != c.size())
      throw invalid_argument("VectorValues::operator+ called with different vector sizes");
    assert_throw(hasSameStructure(c),
      invalid_argument("VectorValues::operator+ called with different vector sizes"));

    VectorValues result;
    // The result.end() hint here should result in constant-time inserts
    for(const_iterator j1 = begin(), j2 = c.begin(); j1 != end(); ++j1, ++j2)
      result.values_.insert(result.end(), make_pair(j1->first, j1->second + j2->second));

    return result;
  }

  /* ************************************************************************* */
  VectorValues VectorValues::add(const VectorValues& c) const
  {
    return *this + c;
  }

  /* ************************************************************************* */
  VectorValues& VectorValues::operator+=(const VectorValues& c)
  {
    if(this->size() != c.size())
      throw invalid_argument("VectorValues::operator+= called with different vector sizes");
    assert_throw(hasSameStructure(c),
      invalid_argument("VectorValues::operator+= called with different vector sizes"));

    iterator j1 = begin();
    const_iterator j2 = c.begin();
    // The result.end() hint here should result in constant-time inserts
    for(; j1 != end(); ++j1, ++j2)
      j1->second += j2->second;

    return *this;
  }

  /* ************************************************************************* */
  VectorValues& VectorValues::addInPlace(const VectorValues& c)
  {
    return *this += c;
  }

  /* ************************************************************************* */
  VectorValues& VectorValues::addInPlace_(const VectorValues& c)
  {
    for(const_iterator j2 = c.begin(); j2 != c.end(); ++j2) {
      pair<VectorValues::iterator, bool> xi = tryInsert(j2->first, Vector());
      if(xi.second)
        xi.first->second = j2->second;
      else
        xi.first->second += j2->second;
    }
    return *this;
  }

  /* ************************************************************************* */
  VectorValues VectorValues::operator-(const VectorValues& c) const
  {
    if(this->size() != c.size())
      throw invalid_argument("VectorValues::operator- called with different vector sizes");
    assert_throw(hasSameStructure(c),
      invalid_argument("VectorValues::operator- called with different vector sizes"));

    VectorValues result;
    // The result.end() hint here should result in constant-time inserts
    for(const_iterator j1 = begin(), j2 = c.begin(); j1 != end(); ++j1, ++j2)
      result.values_.insert(result.end(), make_pair(j1->first, j1->second - j2->second));

    return result;
  }

  /* ************************************************************************* */
  VectorValues VectorValues::subtract(const VectorValues& c) const
  {
    return *this - c;
  }

  /* ************************************************************************* */
  VectorValues operator*(const double a, const VectorValues &v)
  {
    VectorValues result;
    for(const VectorValues::KeyValuePair& key_v: v)
      result.values_.insert(result.values_.end(), make_pair(key_v.first, a * key_v.second));
    return result;
  }

  /* ************************************************************************* */
  VectorValues VectorValues::scale(const double a) const
  {
    return a * *this;
  }

  /* ************************************************************************* */
  VectorValues& VectorValues::operator*=(double alpha)
  {
    for(Vector& v: *this | map_values)
      v *= alpha;
    return *this;
  }

  /* ************************************************************************* */
  VectorValues& VectorValues::scaleInPlace(double alpha)
  {
    return *this *= alpha;
  }

  /* ************************************************************************* */

} // \namespace gtsam
