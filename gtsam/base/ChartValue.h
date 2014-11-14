/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/*
 * @file ChartValue.h
 * @brief
 * @date October, 2014
 * @author Michael Bosse, Abel Gawel, Renaud Dube
 * based on DerivedValue.h by Duy Nguyen Ta
 */

#pragma once

#include <gtsam/base/GenericValue.h>
#include <gtsam/base/Manifold.h>
#include <boost/make_shared.hpp>

//////////////////
// The following includes windows.h in some MSVC versions, so we undef min, max, and ERROR
#include <boost/pool/singleton_pool.hpp>

#ifdef min
#undef min
#endif

#ifdef max
#undef max
#endif

#ifdef ERROR
#undef ERROR
#endif
//////////////////

namespace gtsam {

/**
 * ChartValue is derived from GenericValue<T> and Chart so that
 * Chart can be zero sized (as in DefaultChart<T>)
 * if the Chart is a member variable then it won't ever be zero sized.
 */
template<class T, class Chart_ = DefaultChart<T> >
class ChartValue: public GenericValue<T>, public Chart_ {

  BOOST_CONCEPT_ASSERT((ChartConcept<Chart_>));

public:

  typedef T type;
  typedef Chart_ Chart;

public:

  /// Default Constructor. TODO might not make sense for some types
  ChartValue() :
      GenericValue<T>(T()) {
  }

  /// Construct froma  value
  ChartValue(const T& value) :
      GenericValue<T>(value) {
  }

  /// Construct from a value and initialize the chart
  template<typename C>
  ChartValue(const T& value, C chart_initializer) :
      GenericValue<T>(value), Chart(chart_initializer) {
  }

  /// Destructor
  virtual ~ChartValue() {
  }

  /**
   * Create a duplicate object returned as a pointer to the generic Value interface.
   * For the sake of performance, this function use singleton pool allocator instead of the normal heap allocator.
   * The result must be deleted with Value::deallocate_, not with the 'delete' operator.
   */
  virtual Value* clone_() const {
    void *place = boost::singleton_pool<PoolTag, sizeof(ChartValue)>::malloc();
    ChartValue* ptr = new (place) ChartValue(*this); // calls copy constructor to fill in
    return ptr;
  }

  /**
   * Destroy and deallocate this object, only if it was originally allocated using clone_().
   */
  virtual void deallocate_() const {
    this->~ChartValue(); // Virtual destructor cleans up the derived object
    boost::singleton_pool<PoolTag, sizeof(ChartValue)>::free((void*) this); // Release memory from pool
  }

  /**
   * Clone this value (normal clone on the heap, delete with 'delete' operator)
   */
  virtual boost::shared_ptr<Value> clone() const {
    return boost::make_shared<ChartValue>(*this);
  }

  /// Chart Value interface version of retract
  virtual Value* retract_(const Vector& delta) const {
    // Call retract on the derived class using the retract trait function
    const T retractResult = Chart::retract(GenericValue<T>::value(), delta);

    // Create a Value pointer copy of the result
    void* resultAsValuePlace =
        boost::singleton_pool<PoolTag, sizeof(ChartValue)>::malloc();
    Value* resultAsValue = new (resultAsValuePlace) ChartValue(retractResult,
        static_cast<const Chart&>(*this));

    // Return the pointer to the Value base class
    return resultAsValue;
  }

  /// Generic Value interface version of localCoordinates
  virtual Vector localCoordinates_(const Value& value2) const {
    // Cast the base class Value pointer to a templated generic class pointer
    const GenericValue<T>& genericValue2 =
        static_cast<const GenericValue<T>&>(value2);

    // Return the result of calling localCoordinates trait on the derived class
    return Chart::local(GenericValue<T>::value(), genericValue2.value());
  }

  /// Non-virtual version of retract
  ChartValue retract(const Vector& delta) const {
    return ChartValue(Chart::retract(GenericValue<T>::value(), delta),
        static_cast<const Chart&>(*this));
  }

  /// Non-virtual version of localCoordinates
  Vector localCoordinates(const ChartValue& value2) const {
    return localCoordinates_(value2);
  }

  /// Return run-time dimensionality
  virtual size_t dim() const {
    // need functional form here since the dimension may be dynamic
    return Chart::getDimension(GenericValue<T>::value());
  }

  /// Assignment operator
  virtual Value& operator=(const Value& rhs) {
    // Cast the base class Value pointer to a derived class pointer
    const ChartValue& derivedRhs = static_cast<const ChartValue&>(rhs);

    // Do the assignment and return the result
    *this = ChartValue(derivedRhs); // calls copy constructor
    return *this;
  }

protected:

  // implicit assignment operator for (const ChartValue& rhs) works fine here
  /// Assignment operator, protected because only the Value or DERIVED
  /// assignment operators should be used.
  //  DerivedValue<DERIVED>& operator=(const DerivedValue<DERIVED>& rhs) {
  //    // Nothing to do, do not call base class assignment operator
  //    return *this;
  //  }

private:

  /// Fake Tag struct for singleton pool allocator. In fact, it is never used!
  struct PoolTag {
  };

private:

  /** Serialization function */
  friend class boost::serialization::access;
  template<class ARCHIVE>
  void serialize(ARCHIVE & ar, const unsigned int version) {
    // ar & boost::serialization::make_nvp("value",);
    // todo: implement a serialization for charts
    ar
        & boost::serialization::make_nvp("GenericValue",
            boost::serialization::base_object<GenericValue<T> >(*this));
  }

};

// Define
namespace traits {

/// The dimension of a ChartValue is the dimension of the chart
template<typename T, typename Chart>
struct dimension<ChartValue<T, Chart> > : public dimension<Chart> {
  // TODO Frank thinks dimension is a property of type, chart should conform
};

} // \ traits

/// Get the chart from a Value
template<typename Chart>
const Chart& Value::getChart() const {
  return dynamic_cast<const Chart&>(*this);
}

/// Convenience function that can be used to make an expression to convert a value to a chart
template<typename T>
ChartValue<T> convertToChartValue(const T& value,
    boost::optional<
        Eigen::Matrix<double, traits::dimension<T>::value,
            traits::dimension<T>::value>&> H = boost::none) {
  if (H) {
    *H = Eigen::Matrix<double, traits::dimension<T>::value,
        traits::dimension<T>::value>::Identity();
  }
  return ChartValue<T>(value);
}

} /* namespace gtsam */
