/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/*
 * @file GenericValue.h
 * @date Jan 26, 2012
 * @author Michael Bosse, Abel Gawel, Renaud Dube
 * based on DrivedValue.h by Duy Nguyen Ta
 */

#pragma once

#include <gtsam/base/Value.h>

namespace gtsam {

namespace traits {

// trait to wrap the default equals of types
template<typename T>
  bool equals(const T& a, const T& b, double tol) {
    return a.equals(b,tol);
  }

template<typename T>
void print(const T& obj, const std::string& str) {
  obj.print(str);
}

}

template<class T>
class GenericValue : public Value {
public:
  typedef T type;
protected:
  T value_;

public:
  GenericValue(const T& value) : value_(value) {}

  const T& value() const { return value_; }
  T& value() { return value_; }

  virtual ~GenericValue() {}

  /// equals implementing generic Value interface
  virtual bool equals_(const Value& p, double tol = 1e-9) const {
    // Cast the base class Value pointer to a templated generic class pointer
    const GenericValue& genericValue2 = static_cast<const GenericValue&>(p);

    // Return the result of using the equals traits for the derived class
    return traits::equals<T>(this->value_, genericValue2.value_, tol);
  }

  // non virtual equals function
  bool equals(const GenericValue &other, double tol = 1e-9) const {
    return traits::equals<T>(this->value(),other.value(),tol);
  }

  virtual void print(const std::string& str) const {
    traits::print<T>(value_,str);
  }

protected:
  /// Assignment operator for this class not needed since GenricValue<T> is an abstract class

};

// define Value::cast here since now GenericValue has been declared
template<typename ValueType>
 const ValueType& Value::cast() const {
   return dynamic_cast<const GenericValue<ValueType>&>(*this).value();
 }


} /* namespace gtsam */
