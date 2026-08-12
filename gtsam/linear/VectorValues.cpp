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

#include <functional>
#include <utility>

// assert_throw needs a semicolon in Release mode.
#if defined(__clang__)
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wextra-semi-stmt"
#endif

namespace gtsam {

  /* ************************************************************************ */
  VectorValues::VectorValues(const VectorValues& first, const VectorValues& second)
  {
    // Merge using predicate for comparing first of pair
    merge(first.begin(), first.end(), second.begin(), second.end(), inserter(values_, values_.end()),
      std::bind(&std::less<Key>::operator(), std::less<Key>(), std::bind(&KeyValuePair::first, std::placeholders::_1),
          std::bind(&KeyValuePair::first, std::placeholders::_2)));
    if(size() != first.size() + second.size())
      throw std::invalid_argument("Requested to merge two VectorValues that have one or more variables in common.");
  }

  /* ************************************************************************ */
  VectorValues::VectorValues(const Vector& x, const Dims& dims) {
    size_t j = 0;
    for (const auto& [key, n] : dims) {
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
  std::map<Key, const Vector&> VectorValues::sorted() const {
    std::map<Key, const Vector&> ordered;
    for (const auto& kv : *this) ordered.emplace(kv);
    return ordered;
  }

  /* ************************************************************************ */
  VectorValues VectorValues::Zero(const VectorValues& other)
  {
    VectorValues result;
    for (const auto& [key, value] : other)
#ifdef TBB_GREATER_EQUAL_2020
      result.values_.emplace(key, Vector::Zero(value.size()));
#else
      result.values_.insert({key, Vector::Zero(value.size())});
#endif
    return result;
  }

  /* ************************************************************************ */
  VectorValues::iterator VectorValues::insert(const std::pair<Key, Vector>& key_value) {
    const std::pair<iterator, bool> result = values_.insert(key_value);
    if(!result.second)
      throw std::invalid_argument(
      "Requested to insert variable '" + DefaultKeyFormatter(key_value.first)
      + "' already in this VectorValues.");
    return result.first;
  }

  /* ************************************************************************ */
  VectorValues& VectorValues::update(const VectorValues& values) {
    iterator hint = begin();
    for (const auto& [key, value] : values) {
      // Use this trick to find the value using a hint, since we are inserting
      // from another sorted map
      size_t oldSize = values_.size();
      hint = values_.insert(hint, {key, value});
      if (values_.size() > oldSize) {
        values_.unsafe_erase(hint);
        throw std::out_of_range(
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
      throw std::invalid_argument(
          "Requested to insert a VectorValues into another VectorValues that "
          "already contains one or more of its keys.");
    return *this;
  }

  /* ************************************************************************ */
  VectorValues& VectorValues::insert(const Vector& values,
                                     const KeyVector& keys, const Dims& dims) {
    DenseIndex offset = 0;
    for (Key key : keys) {
      const size_t dim = dims.at(key);
      insert(key, values.segment(offset, dim));
      offset += dim;
    }
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
  GTSAM_EXPORT std::ostream& operator<<(std::ostream& os, const VectorValues& v) {
    // Always print in key-sorted order for deterministic output.
    for (const auto& [key, value] : v.sorted()) {
      os << "  " << StreamedKey(key) << ": " << value.transpose() << "\n";
    }
    return os;
  }

  /* ************************************************************************ */
  void VectorValues::print(const std::string& str,
                           const KeyFormatter& formatter) const {
    std::cout << str << ": " << size() << " elements\n";
    std::cout << key_formatter(formatter) << *this;
    std::cout.flush();
}

  /* ************************************************************************ */
  bool VectorValues::equals(const VectorValues& x, double tol) const {
    if (this->size() != x.size()) return false;

    // Compare in key-sorted order so equality is independent of
    // the underlying (possibly unordered) container iteration order.
    const auto thisOrdered = this->sorted();
    const auto xOrdered = x.sorted();

    auto it1 = thisOrdered.begin();
    auto it2 = xOrdered.begin();
    for (; it1 != thisOrdered.end(); ++it1, ++it2) {
      if (it1->first != it2->first ||
          !equal_with_abs_tol(it1->second, it2->second, tol))
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
    // Always order by key so the concatenated vector is deterministic
    // even if the underlying container is unordered.
    for (const auto& [key, value] : sorted()) {
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
    if (this->size() != other.size()) return false;

    // Compare in key-sorted order so structure comparison is
    // independent of the underlying container iteration order.
    const auto thisOrdered = this->sorted();
    const auto otherOrdered = other.sorted();

    auto it1 = thisOrdered.begin();
    auto it2 = otherOrdered.begin();
    for (; it1 != thisOrdered.end(); ++it1, ++it2) {
      if (!internal::structureCompareOp(*it1, *it2)) return false;
    }
    return true;
  }

  /* ************************************************************************ */
  double VectorValues::dot(const VectorValues& v) const
  {
    if (this->size() != v.size())
      throw std::invalid_argument(
          "VectorValues::dot called with a VectorValues of different "
          "structure");

    double result = 0.0;
    for (const auto& [key, value] : *this) {
      const auto it = v.find(key);
      assert_throw(it != v.end(), std::invalid_argument(
                                      "VectorValues::dot called with a "
                                      "VectorValues of different structure"));
      assert_throw(
          value.size() == it->second.size(),
          std::invalid_argument("VectorValues::dot called with a VectorValues "
                                "of different structure"));
      result += value.dot(it->second);
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
    if (this->size() != c.size())
      throw std::invalid_argument(
          "VectorValues::operator+ called with different vector sizes");

    VectorValues result;
    for (const auto& [key, value] : *this) {
      const auto it = c.find(key);
      assert_throw(
          it != c.end(),
          std::invalid_argument(
              "VectorValues::operator+ called with different vector sizes"));
      assert_throw(
          value.size() == it->second.size(),
          std::invalid_argument(
              "VectorValues::operator+ called with different vector sizes"));
#ifdef TBB_GREATER_EQUAL_2020
      result.values_.emplace(key, value + it->second);
#else
      result.values_.insert({key, value + it->second});
#endif
    }

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
    if (this->size() != c.size())
      throw std::invalid_argument(
          "VectorValues::operator+= called with different vector sizes");

    for (auto& [key, value] : *this) {
      const auto it = c.find(key);
      assert_throw(
          it != c.end(),
          std::invalid_argument(
              "VectorValues::operator+= called with different vector sizes"));
      assert_throw(
          value.size() == it->second.size(),
          std::invalid_argument(
              "VectorValues::operator+= called with different vector sizes"));
      value += it->second;
    }

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
      const auto& [it, success] = tryInsert(j2->first, Vector());
      if(success)
        it->second = j2->second;
      else
        it->second += j2->second;
    }
    return *this;
  }

  /* ************************************************************************ */
  VectorValues VectorValues::operator-(const VectorValues& c) const
  {
    if (this->size() != c.size())
      throw std::invalid_argument(
          "VectorValues::operator- called with different vector sizes");

    VectorValues result;
    for (const auto& [key, value] : *this) {
      const auto it = c.find(key);
      assert_throw(
          it != c.end(),
          std::invalid_argument(
              "VectorValues::operator- called with different vector sizes"));
      assert_throw(
          value.size() == it->second.size(),
          std::invalid_argument(
              "VectorValues::operator- called with different vector sizes"));
#ifdef TBB_GREATER_EQUAL_2020
      result.values_.emplace(key, value - it->second);
#else
      result.values_.insert({key, value - it->second});
#endif
    }

    return result;
  }

  /* ************************************************************************ */
  VectorValues VectorValues::subtract(const VectorValues& c) const
  {
    return *this - c;
  }

  /* ************************************************************************ */
  VectorValues operator*(const double a, const VectorValues& c) {
    VectorValues result;
    for (const auto& [key, value] : c)
#ifdef TBB_GREATER_EQUAL_2020
      result.values_.emplace(key, a * value);
#else
      result.values_.insert({key, a * value});
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
  std::string VectorValues::html(const KeyFormatter& keyFormatter) const {
    std::stringstream ss;

    // Print out preamble.
    ss << "<div>\n<table class='VectorValues'>\n  <thead>\n";

    // Print out header row.
    ss << "    <tr><th>Variable</th><th>value</th></tr>\n";

    // Finish header and start body.
    ss << "  </thead>\n  <tbody>\n";

    // Print out all rows.
    for (const auto& kv : sorted()) {
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

#if defined(__clang__)
#pragma clang diagnostic pop
#endif
