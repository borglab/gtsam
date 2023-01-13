/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file Value.h
 * @brief The base class for any variable that can be optimized or used in a factor.
 * @author Richard Roberts
 * @date Jan 14, 2012
 */

#pragma once

#include <gtsam/config.h>      // Configuration from CMake

#include <gtsam/base/Vector.h>
#include <boost/make_shared.hpp>
#include <boost/serialization/nvp.hpp>
#include <boost/serialization/assume_abstract.hpp>
#include <memory>

namespace gtsam {

  /**
   * This is the base class for any type to be stored in Values.
   * Note: As of GTSAM 4.0, Value types should no longer derive from Value or
   * DerivedValue. Use type traits instead.
   * See https://bitbucket.org/gtborg/gtsam/wiki/Migrating%20from%20GTSAM%203.X%20to%20GTSAM%204.0#markdown-header-custom-value-types
   * for current usage and migration details.
   */
  class GTSAM_EXPORT Value {
  public:

    /** Clone this value in a special memory pool, must be deleted with Value::deallocate_, *not* with the 'delete' operator. */
    virtual Value* clone_() const = 0;

    /** Deallocate a raw pointer of this value */
    virtual void deallocate_() const = 0;

    /** Clone this value (normal clone on the heap, delete with 'delete' operator) */
    virtual boost::shared_ptr<Value> clone() const = 0;

    /** Compare this Value with another for equality. */
    virtual bool equals_(const Value& other, double tol = 1e-9) const = 0;

    /** Print this value, for debugging and unit tests */
    virtual void print(const std::string& str = "") const = 0;

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
    virtual Value* retract_(const Vector& delta) const = 0;

    /** Compute the coordinates in the tangent space of this value that
     * retract() would map to \c value.
     * @param value The value whose coordinates should be determined in the
     * tangent space of the value on which this function is called.
     * @return The coordinates of \c value in the tangent space of \c this.
     */
    virtual Vector localCoordinates_(const Value& value) const = 0;

    /** Assignment operator */
    virtual Value& operator=(const Value& /*rhs*/) {
      //needs a empty definition so recursion in implicit derived assignment operators work
     return *this;
    }

    /** Cast to known ValueType */
    template<typename ValueType>
    const ValueType& cast() const;

    /** Virutal destructor */
    virtual ~Value() {}

  private:
    /** Empty serialization function.
     *
     * There are two important things that users need to do to serialize derived objects in Values successfully:
     * (Those derived objects are stored in Values as pointer to this abstract base class Value)
     *
     *     1. All DERIVED classes derived from Value must put the following line in their serialization function:
     *       \code
                ar & boost::serialization::make_nvp("DERIVED", boost::serialization::base_object<Value>(*this));
            \endcode
     *       or, alternatively
     *       \code
                ar & BOOST_SERIALIZATION_BASE_OBJECT_NVP(Value);
            \endcode
     *       See: http://www.boost.org/doc/libs/release/libs/serialization/doc/serialization.html#runtimecasting
     *
     *     2. The source module that includes archive class headers to serialize objects of derived classes
     *      (boost/archive/text_oarchive.h, for example) must *export* all derived classes, using either
     *      BOOST_CLASS_EXPORT or BOOST_CLASS_EXPORT_GUID macros:
                 \code
                BOOST_CLASS_EXPORT(DERIVED_CLASS_1)
                BOOST_CLASS_EXPORT_GUID(DERIVED_CLASS_2, "DERIVED_CLASS_2_ID_STRING")
                 \endcode
     *       See:   http://www.boost.org/doc/libs/release/libs/serialization/doc/serialization.html#derivedpointers
     *             http://www.boost.org/doc/libs/release/libs/serialization/doc/serialization.html#export
     *             http://www.boost.org/doc/libs/release/libs/serialization/doc/serialization.html#instantiation\
     *             http://www.boost.org/doc/libs/release/libs/serialization/doc/special.html#export
     *             http://www.boost.org/doc/libs/release/libs/serialization/doc/traits.html#export
     *       The last two links explain why these export lines have to be in the same source module that includes
     *       any of the archive class headers.
     * */
    friend class boost::serialization::access;
    template<class ARCHIVE>
    void serialize(ARCHIVE & /*ar*/, const unsigned int /*version*/) {
    }

  };

} /* namespace gtsam */

BOOST_SERIALIZATION_ASSUME_ABSTRACT(gtsam::Value)
