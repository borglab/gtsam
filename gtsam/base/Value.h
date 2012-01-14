/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file Value.h
 * @brief The interface class for any variable that can be optimized or used in a factor.
 * @author Richard Roberts
 * @date Jan 14, 2012
 */

#pragma once

#include <memory>

#include <gtsam/base/Vector.h>

namespace gtsam {

  /**
   * This is the interface class for any value that may be used as a variable
   * assignment in a factor graph, and which you must derive to create new
   * variable types to use with gtsam.  Examples of built-in classes
   * implementing this are mainly in geometry, including Rot3, Pose2, etc.
   *
   * This interface specifies pure virtual retract_(), localCoordinates_() and
   * equals_() functions that work with pointers and references to this interface
   * class, i.e. the base class.  These functions allow containers, such as
   * Values can operate generically on Value objects, retracting or computing
   * local coordinates for many Value objects of different types.
   *
   * When you implement retract_(), localCoordinates_(), and equals_(), we
   * suggest first implementing versions of these functions that work directly
   * with derived objects, then using the provided helper functions to
   * implement the generic Value versions.  This makes your implementation
   * easier, and also improves performance in situations where the derived type
   * is in fact known, such as in most implementations of \c evaluateError() in
   * classes derived from NonlinearFactor.
   *
   * Using the above practice, here is an example of implementing a typical
   * class derived from Value:
   * \code
     class Rot3 : public Value {
     public:
       // Constructor, there is never a need to call the Value base class constructor.
       Rot3() { ... }

       // Print for unit tests and debugging (virtual, implements Value::print())
       virtual void print(const std::string& str = "") const;

       // Equals working directly with Rot3 objects (non-virtual, non-overriding!)
       bool equals(const Rot3& other, double tol = 1e-9) const;

       // Tangent space dimensionality (virtual, implements Value::dim())
       virtual size_t dim() const {
         return 3;
       }

       // retract working directly with Rot3 objects (non-virtual, non-overriding!)
       Rot3 retract(const Vector& delta) const {
         // Math to implement a 3D rotation retraction e.g. exponential map
         return Rot3(result);
       }

       // localCoordinates working directly with Rot3 objects (non-virtual, non-overriding!)
       Vector localCoordinates(const Rot3& r2) const {
         // Math to implement 3D rotation localCoordinates, e.g. logarithm map
         return Vector(result);
       }

       // Equals implementing the generic Value interface (virtual, implements Value::equals_())
       virtual bool equals_(const Value& other, double tol = 1e-9) const {
         // Call our provided helper function to call your Rot3-specific
         // equals with appropriate casting.
         return CallDerivedEquals(this, other, tol);
       }

       // retract implementing the generic Value interface (virtual, implements Value::retract_())
       virtual std::auto_ptr<Value> retract_(const Vector& delta) const {
         // Call our provided helper function to call your Rot3-specific
         // retract and do the appropriate casting and allocation.
         return CallDerivedRetract(this, delta);
       }

       // localCoordinates implementing the generic Value interface (virtual, implements Value::localCoordinates_())
       virtual Vector localCoordinates_(const Value& value) const {
         // Call our provided helper function to call your Rot3-specific
         // localCoordinates and do the appropriate casting.
         return CallDerivedLocalCoordinates(this, value);
       }
     };
     \endcode
   */
  class Value {
  public:

    /** Print this value, for debugging and unit tests */
    virtual void print(const std::string& str = "") const = 0;

    /** Compare this Value with another for equality. */
    virtual bool equals_(const Value& other, double tol = 1e-9) const = 0;

    /** Return the dimensionality of the tangent space of this value.  This is
     * the dimensionality of \c delta passed into retract() and of the vector
     * returned by localCoordinates().
     * @return The dimensionality of the tangent space
     */
    virtual size_t dim() const = 0;

    /** Increment the value, by mapping from the vector delta in the tangent
     * space of the current value back to the manifold to produce a new,
     * incremented value.
     * @param delta The delta vector in the tangent space of this value, by
     * which to increment this value.
     */
    virtual std::auto_ptr<Value> retract_(const Vector& delta) const = 0;

    /** Compute the coordinates in the tangent space of this value that
     * retract() would map to \c value.
     * @param value The value whose coordinates should be determined in the
     * tangent space of the value on which this function is called.
     * @return The coordinates of \c value in the tangent space of \c this.
     */
    virtual Vector localCoordinates_(const Value& value) const = 0;

    /** Virutal destructor */
    virtual ~Value() {}

  protected:
    /** This is a convenience function to make it easy for you to implement the
     * generic Value inferface, see the example at the top of the Value
     * documentation.
     * @param value1 The object on which to call equals, stored as a derived class pointer
     * @param value2 The argument to pass to the derived equals function, strored as a Value reference
     * @return The result of equals of the derived class
     */
    template<class Derived>
    static bool CallDerivedEquals(const Derived* value1, const Value& value2, double tol) {
      // Cast the base class Value pointer to a derived class pointer
      const Derived& derivedValue2 = dynamic_cast<const Derived&>(value2);

      // Return the result of calling equals on the derived class
      return value1->equals(derivedValue2, tol);
    }

    /** This is a convenience function to make it easy for you to implement the
     * generic Value inferface, see the example at the top of the Value
     * documentation.
     * @param derived A pointer to the derived class on which to call retract
     * @param delta The delta vector to pass to the derived retract
     * @return The result of retract on the derived class, stored as a Value pointer
     */
    template<class Derived>
    static std::auto_ptr<Value> CallDerivedRetract(const Derived* derived, const Vector& delta) {
      // Call retract on the derived class
      const Derived retractResult = derived->retract(delta);

      // Create a Value pointer copy of the result
      std::auto_ptr<Value> resultAsValue(new Derived(retractResult));

      // Return the pointer to the Value base class
      return resultAsValue;
    }

    /** This is a convenience function to make it easy for you to implement the
     * generic Value inferface, see the example at the top of the Value
     * documentation.
     * @param value1 The object on which to call localCoordinates, stored as a derived class pointer
     * @param value2 The argument to pass to the derived localCoordinates function, stored as a Value reference
     * @return The result of localCoordinates of the derived class
     */
    template<class Derived>
    static Vector CallDerivedLocalCoordinates(const Derived* value1, const Value& value2) {
      // Cast the base class Value pointer to a derived class pointer
      const Derived& derivedValue2 = dynamic_cast<const Derived&>(value2);

      // Return the result of calling localCoordinates on the derived class
      return value1->localCoordinates(derivedValue2);
    }
  };

} /* namespace gtsam */
