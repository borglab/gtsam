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

#include <gtsam/base/Manifold.h>
#include <gtsam/base/types.h>
#include <gtsam/base/Value.h>

#include <boost/pool/pool_alloc.hpp>

#include <cmath>
#include <iostream>
#include <typeinfo> // operator typeid

#ifdef _WIN32
#define GENERICVALUE_VISIBILITY
#else
// This will trigger a LNKxxxx on MSVC, so disable for MSVC build
// Please refer to https://github.com/borglab/gtsam/blob/develop/Using-GTSAM-EXPORT.md
#define GENERICVALUE_VISIBILITY GTSAM_EXPORT
#endif

namespace gtsam {

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
  // Only needed for serialization.
  GenericValue(){}

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
  ~GenericValue() override {
  }

  /// equals implementing generic Value interface
  bool equals_(const Value& p, double tol = 1e-9) const override {
    // Cast the base class Value pointer to a templated generic class pointer
    const GenericValue& genericValue2 = static_cast<const GenericValue&>(p);
    // Return the result of using the equals traits for the derived class
    return traits<T>::Equals(this->value_, genericValue2.value_, tol);
  }

  /// non virtual equals function, uses traits
  bool equals(const GenericValue &other, double tol = 1e-9) const {
    return traits<T>::Equals(this->value(), other.value(), tol);
  }

  /// Virtual print function, uses traits
  void print(const std::string& str) const override {
    std::cout << "(" << demangle(typeid(T).name()) << ")\n";
    traits<T>::Print(value_, str);
  }

    /**
     * Create a duplicate object returned as a pointer to the generic Value interface.
     */
    Value* clone_() const override {
      GenericValue* ptr = new GenericValue(*this); // calls copy constructor to fill in
      return ptr;
    }

    /**
     * Destroy and deallocate this object, only if it was originally allocated using clone_().
     */
    void deallocate_() const override {
      delete this;
    }

    /**
     * Clone this value (normal clone on the heap, delete with 'delete' operator)
     */
    std::shared_ptr<Value> clone() const override {
		return std::allocate_shared<GenericValue>(Eigen::aligned_allocator<GenericValue>(), *this);
    }

    /// Generic Value interface version of retract
    Value* retract_(const Vector& delta) const override {
      // Call retract on the derived class using the retract trait function
      const T retractResult = traits<T>::Retract(GenericValue<T>::value(), delta);

      Value* resultAsValue = new GenericValue(retractResult);

      // Return the pointer to the Value base class
      return resultAsValue;
    }

    /// Generic Value interface version of localCoordinates
    Vector localCoordinates_(const Value& value2) const override {
      // Cast the base class Value pointer to a templated generic class pointer
      const GenericValue<T>& genericValue2 =
          static_cast<const GenericValue<T>&>(value2);

      // Return the result of calling localCoordinates trait on the derived class
      return traits<T>::Local(GenericValue<T>::value(), genericValue2.value());
    }

    /// Non-virtual version of retract
    GenericValue retract(const Vector& delta) const {
      return GenericValue(traits<T>::Retract(GenericValue<T>::value(), delta));
    }

    /// Non-virtual version of localCoordinates
    Vector localCoordinates(const GenericValue& value2) const {
      return localCoordinates_(value2);
    }

    /// Return run-time dimensionality
    size_t dim() const override {
      return traits<T>::GetDimension(value_);
    }

    /// Assignment operator
    Value& operator=(const Value& rhs) override {
      // Cast the base class Value pointer to a derived class pointer
      const GenericValue& derivedRhs = static_cast<const GenericValue&>(rhs);

      // Do the assignment and return the result
      *this = GenericValue(derivedRhs); // calls copy constructor
      return *this;
    }

  protected:

    /// Assignment operator, protected because only the Value or DERIVED
    /// assignment operators should be used.
    GenericValue<T>& operator=(const GenericValue<T>& rhs) {
      Value::operator=(static_cast<Value const&>(rhs));
      value_ = rhs.value_;
      return *this;
    }

  private:

#ifdef GTSAM_ENABLE_BOOST_SERIALIZATION
    /** Serialization function */
    friend class boost::serialization::access;
    template<class ARCHIVE>
    void serialize(ARCHIVE & ar, const unsigned int /*version*/) {
      ar & boost::serialization::make_nvp("GenericValue",
              boost::serialization::base_object<Value>(*this));
      ar & boost::serialization::make_nvp("value", value_);
	}
#endif


  // Alignment, see https://eigen.tuxfamily.org/dox/group__TopicStructHavingEigenMembers.html
  enum { NeedsToAlign = (sizeof(T) % 16) == 0 };
public:
  GTSAM_MAKE_ALIGNED_OPERATOR_NEW_IF(NeedsToAlign)
};

/// use this macro instead of BOOST_CLASS_EXPORT for GenericValues
#define GTSAM_VALUE_EXPORT(Type) BOOST_CLASS_EXPORT(gtsam::GenericValue<Type>)

// traits
template <typename ValueType>
struct traits<GenericValue<ValueType> >
    : public Testable<GenericValue<ValueType> > {};

// define Value::cast here since now GenericValue has been declared
template<typename ValueType>
const ValueType& Value::cast() const {
  return dynamic_cast<const GenericValue<ValueType>&>(*this).value();
}

/** Functional constructor of GenericValue<T> so T can be automatically deduced
  */
template<class T>
GenericValue<T> genericValue(const T& v) {
  return GenericValue<T>(v);
}


} /* namespace gtsam */
