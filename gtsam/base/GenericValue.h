/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/*
 * @file GenericValue.h
 * @brief Wraps any type T so it can play as a Value
 * @date October, 2014
 * @author Michael Bosse, Abel Gawel, Renaud Dube
 * based on DerivedValue.h by Duy Nguyen Ta
 */

#pragma once

#include <gtsam/base/Value.h>

namespace gtsam {

// To play as a GenericValue, we need the following traits
namespace traits {

// trait to wrap the default equals of types
template<typename T>
bool equals(const T& a, const T& b, double tol) {
  return a.equals(b, tol);
}

// trait to wrap the default print of types
template<typename T>
void print(const T& obj, const std::string& str) {
  obj.print(str);
}

}

/**
 * Wraps any type T so it can play as a Value
 */
template<class T>
class GenericValue: public Value {

public:

  typedef T type;

protected:

  T value_; ///< The wrapped value

public:

  /// Construct from value
  GenericValue(const T& value) :
      value_(value) {
  }

  /// Return a constant value
  const T& value() const {
    return value_;
  }

  /// Return the value
  T& value() {
    return value_;
  }

  /// Destructor
  virtual ~GenericValue() {
  }

  /// equals implementing generic Value interface
  virtual bool equals_(const Value& p, double tol = 1e-9) const {
    // Cast the base class Value pointer to a templated generic class pointer
    const GenericValue& genericValue2 = static_cast<const GenericValue&>(p);
    // Return the result of using the equals traits for the derived class
    return traits::equals<T>(this->value_, genericValue2.value_, tol);
  }

  /// non virtual equals function, uses traits
  bool equals(const GenericValue &other, double tol = 1e-9) const {
    return traits::equals<T>(this->value(), other.value(), tol);
  }

  /// Virtual print function, uses traits
  virtual void print(const std::string& str) const {
    traits::print<T>(value_, str);
  }

  // Serialization below:

  friend class boost::serialization::access;
  template<class ARCHIVE>
  void serialize(ARCHIVE & ar, const unsigned int version) {
    ar & BOOST_SERIALIZATION_BASE_OBJECT_NVP(Value);
    ar & BOOST_SERIALIZATION_NVP(value_);
  }

protected:

  // Assignment operator for this class not needed since GenericValue<T> is an abstract class

};

// define Value::cast here since now GenericValue has been declared
template<typename ValueType>
const ValueType& Value::cast() const {
  return dynamic_cast<const GenericValue<ValueType>&>(*this).value();
}

} /* namespace gtsam */
