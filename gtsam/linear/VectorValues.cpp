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

#include <boost/bind/bind.hpp>
#include <boost/range/numeric.hpp>

using namespace std;

namespace gtsam {

  /* ************************************************************************ */
  VectorValues::VectorValues(const VectorValues& first, const VectorValues& second)
  {
    // Merge using predicate for comparing first of pair
    merge(first.begin(), first.end(), second.begin(), second.end(), inserter(values_, values_.end()),
      std::bind(&less<Key>::operator(), less<Key>(), std::bind(&KeyValuePair::first, std::placeholders::_1),
          std::bind(&KeyValuePair::first, std::placeholders::_2)));
    if(size() != first.size() + second.size())
      throw invalid_argument("Requested to merge two VectorValues that have one or more variables in common.");
  }

  /* ************************************************************************ */
  VectorValues::VectorValues(const Vector& x, const Dims& dims) {
    size_t j = 0;
    for (const auto& [key,n] : dims)  {
#ifdef TBB_GREATER_EQUAL_2020
      values_.emplace(key, x.segment(j, n));
#else
      values_.insert({key, x.segment(j, n)});
#endif
      j += n;
    }
  }

  /* ************************************************************************ */
  VectorValues::VectorValues(const Vector& x, const Scatter& scatter) {
    size_t j = 0;
    for (const SlotEntry& v : scatter) {
#ifdef TBB_GREATER_EQUAL_2020
      values_.emplace(v.key, x.segment(j, v.dimension));
#else
      values_.insert({v.key, x.segment(j, v.dimension)});
#endif
      j += v.dimension;
    }
  }

  /* ************************************************************************ */
  VectorValues VectorValues::Zero(const VectorValues& other)
  {
    VectorValues result;
    for(const auto& [key,value]: other)
#ifdef TBB_GREATER_EQUAL_2020
      result.values_.emplace(key, Vector::Zero(value.size()));
#else
      result.values_.insert({key, Vector::Zero(value.size())});
#endif
    return result;
  }

  /* ************************************************************************ */
  VectorValues::iterator VectorValues::insert(const std::pair<Key, Vector>& key_value) {
    std::pair<iterator, bool> result = values_.insert(key_value);
    if(!result.second)
      throw std::invalid_argument(
      "Requested to insert variable '" + DefaultKeyFormatter(key_value.first)
      + "' already in this VectorValues.");
    return result.first;
  }

  /* ************************************************************************ */
  VectorValues& VectorValues::update(const VectorValues& values) {
    iterator hint = begin();
    for (const auto& [key,value] : values) {
      // Use this trick to find the value using a hint, since we are inserting
      // from another sorted map
      size_t oldSize = values_.size();
      hint = values_.insert(hint, {key, value});
      if (values_.size() > oldSize) {
        values_.unsafe_erase(hint);
        throw out_of_range(
            "Requested to update a VectorValues with another VectorValues that "
            "contains keys not present in the first.");
      } else {
        hint->second = value;
      }
    }
    return *this;
  }

  /* ************************************************************************ */
  VectorValues& VectorValues::insert(const VectorValues& values) {
    size_t originalSize = size();
    values_.insert(values.begin(), values.end());
    if (size() != originalSize + values.size())
      throw invalid_argument(
          "Requested to insert a VectorValues into another VectorValues that "
          "already contains one or more of its keys.");
    return *this;
  }

  /* ************************************************************************ */
  void VectorValues::setZero()
  {
    for(auto& [key, value] : *this) {
      value.setZero();
    }
  }

  /* ************************************************************************ */
  GTSAM_EXPORT ostream& operator<<(ostream& os, const VectorValues& v) {
    // Change print depending on whether we are using TBB
#ifdef GTSAM_USE_TBB
    map<Key, Vector> sorted;
    for (const auto& [key,value] : v) {
      sorted.emplace(key, value);
    }
    for (const auto& [key,value] : sorted)
#else
    for (const auto& [key,value] : v)
#endif
    {
      os << "  " << StreamedKey(key) << ": " << value.transpose() << "\n";
    }
    return os;
  }

  /* ************************************************************************ */
  void VectorValues::print(const string& str,
                           const KeyFormatter& formatter) const {
    cout << str << ": " << size() << " elements\n";
    cout << key_formatter(formatter) << *this;
    cout.flush();
}

  /* ************************************************************************ */
  bool VectorValues::equals(const VectorValues& x, double tol) const {
    if(this->size() != x.size())
      return false;
    auto this_it = this->begin();
    auto x_it = x.begin();
    for(; this_it != this->end(); ++this_it, ++x_it) {
      if(this_it->first != x_it->first || 
          !equal_with_abs_tol(this_it->second, x_it->second, tol))
        return false;
    }
    return true;
  }

  /* ************************************************************************ */
  Vector VectorValues::vector() const {
    // Count dimensions
    DenseIndex totalDim = 0;
    for (const auto& [key, value] : *this)
      totalDim += value.size();

    // Copy vectors
    Vector result(totalDim);
    DenseIndex pos = 0;
    for (const auto& [key, value] : *this) {
      result.segment(pos, value.size()) = value;
      pos += value.size();
    }

    return result;
  }

  /* ************************************************************************ */
  Vector VectorValues::vector(const Dims& keys) const
  {
    // Count dimensions
    DenseIndex totalDim = 0;
    for (const auto& [key, dim] : keys)
      totalDim += dim;
    Vector result(totalDim);
    size_t j = 0;
    for(const Dims::value_type& it: keys) {
      result.segment(j,it.second) = at(it.first);
      j += it.second;
    }
    return result;
  }

  /* ************************************************************************ */
  void VectorValues::swap(VectorValues& other) {
    this->values_.swap(other.values_);
  }

  /* ************************************************************************ */
  namespace internal
  {
    bool structureCompareOp(const VectorValues::value_type& a, const VectorValues::value_type& b)
    {
      return a.first == b.first && a.second.size() == b.second.size();
    }
  }

  /* ************************************************************************ */
  bool VectorValues::hasSameStructure(const VectorValues other) const
  {
    // compare the "other" container with this one, using the structureCompareOp
    // and then return true if all elements are compared as equal
    return std::equal(this->begin(), this->end(), other.begin(), other.end(),
      internal::structureCompareOp);
  }

  /* ************************************************************************ */
  double VectorValues::dot(const VectorValues& v) const
  {
    if(this->size() != v.size())
      throw invalid_argument("VectorValues::dot called with a VectorValues of different structure");
    double result = 0.0;
    auto this_it = this->begin();
    auto v_it = v.begin();
    for(; this_it != this->end(); ++this_it, ++v_it) {
      assert_throw(this_it->first == v_it->first, 
          invalid_argument("VectorValues::dot called with a VectorValues of different structure"));
      assert_throw(this_it->second.size() == v_it->second.size(), 
          invalid_argument("VectorValues::dot called with a VectorValues of different structure"));
      result += this_it->second.dot(v_it->second);
    }
    return result;
  }

  /* ************************************************************************ */
  double VectorValues::norm() const {
    return std::sqrt(this->squaredNorm());
  }

  /* ************************************************************************ */
  double VectorValues::squaredNorm() const {
    double sumSquares = 0.0;
    for(const auto& [key, value]: *this) {
      sumSquares += value.squaredNorm();
    }
    return sumSquares;
  }

  /* ************************************************************************ */
  VectorValues VectorValues::operator+(const VectorValues& c) const
  {
    if(this->size() != c.size())
      throw invalid_argument("VectorValues::operator+ called with different vector sizes");
    assert_throw(hasSameStructure(c),
      invalid_argument("VectorValues::operator+ called with different vector sizes"));

    VectorValues result;
    // The result.end() hint here should result in constant-time inserts
    for(const_iterator j1 = begin(), j2 = c.begin(); j1 != end(); ++j1, ++j2)
#ifdef TBB_GREATER_EQUAL_2020
      result.values_.emplace(j1->first, j1->second + j2->second);
#else
      result.values_.insert({j1->first, j1->second + j2->second});
#endif

    return result;
  }

  /* ************************************************************************ */
  VectorValues VectorValues::add(const VectorValues& c) const
  {
    return *this + c;
  }

  /* ************************************************************************ */
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

  /* ************************************************************************ */
  VectorValues& VectorValues::addInPlace(const VectorValues& c)
  {
    return *this += c;
  }

  /* ************************************************************************ */
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

  /* ************************************************************************ */
  VectorValues VectorValues::operator-(const VectorValues& c) const
  {
    if(this->size() != c.size())
      throw invalid_argument("VectorValues::operator- called with different vector sizes");
    assert_throw(hasSameStructure(c),
      invalid_argument("VectorValues::operator- called with different vector sizes"));

    VectorValues result;
    // The result.end() hint here should result in constant-time inserts
    for(const_iterator j1 = begin(), j2 = c.begin(); j1 != end(); ++j1, ++j2)
#ifdef TBB_GREATER_EQUAL_2020
      result.values_.emplace(j1->first, j1->second - j2->second);
#else
      result.values_.insert({j1->first, j1->second - j2->second});
#endif

    return result;
  }

  /* ************************************************************************ */
  VectorValues VectorValues::subtract(const VectorValues& c) const
  {
    return *this - c;
  }

  /* ************************************************************************ */
  VectorValues operator*(const double a, const VectorValues &v)
  {
    VectorValues result;
    for(const VectorValues::KeyValuePair& key_v: v)
#ifdef TBB_GREATER_EQUAL_2020
      result.values_.emplace(key_v.first, a * key_v.second);
#else
      result.values_.insert({key_v.first, a * key_v.second});
#endif
    return result;
  }

  /* ************************************************************************ */
  VectorValues VectorValues::scale(const double a) const
  {
    return a * *this;
  }

  /* ************************************************************************ */
  VectorValues& VectorValues::operator*=(double alpha)
  {
    for (auto& [key, value]: *this) {
      value *= alpha;
    }
    return *this;
  }

  /* ************************************************************************ */
  VectorValues& VectorValues::scaleInPlace(double alpha)
  {
    return *this *= alpha;
  }

  /* ************************************************************************ */
  string VectorValues::html(const KeyFormatter& keyFormatter) const {
    stringstream ss;

    // Print out preamble.
    ss << "<div>\n<table class='VectorValues'>\n  <thead>\n";

    // Print out header row.
    ss << "    <tr><th>Variable</th><th>value</th></tr>\n";

    // Finish header and start body.
    ss << "  </thead>\n  <tbody>\n";

    // Print out all rows.
#ifdef GTSAM_USE_TBB
    // TBB uses un-ordered map, so inefficiently order them:
    std::map<Key, Vector> ordered;
    for (const auto& kv : *this) ordered.emplace(kv);
    for (const auto& kv : ordered) {
#else
    for (const auto& kv : *this) {
#endif
      ss << "    <tr>";
      ss << "<th>" << keyFormatter(kv.first) << "</th><td>"
         << kv.second.transpose() << "</td>";
      ss << "</tr>\n";
    }
    ss << "  </tbody>\n</table>\n</div>";
    return ss.str();
  }

  /* ************************************************************************ */

} // \namespace gtsam
