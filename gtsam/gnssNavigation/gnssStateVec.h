/**
 * @file   gnssStateVec.h
 * @brief  gnss state vector ---> {delta pos, clock bias, residual zenith trop}
 * @author Ryan Watson
 */

// \callgraph

#pragma once

#include <gtsam/config.h>
#include <gtsam/base/VectorSpace.h>
#include <gtsam/base/Vector.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/dllexport.h>
#include <boost/serialization/nvp.hpp>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Point4.h>

namespace gtsam {

#ifdef GTSAM_TYPEDEF_POINTS_TO_VECTORS

/// As of GTSAM 4, in order to make GTSAM more lean,
/// it is now possible to just typedef gnssStateVec to Vector5
typedef Vector5 gnssStateVec;

#else

/**
 * A vector of size 5 with multiple methods needed for GTSAM
 * @addtogroup geometry
 * \nosubgrouping
 */
class GTSAM_EXPORT gnssStateVec : public Vector5  {

private:
double x_, y_, z_, cb_, tz_;

public:

enum { dimension = 5 };

/// @name Standard Constructors
/// @{

gnssStateVec() :
        x_(0), y_(0), z_(0), cb_(0), tz_(0) {
}

/** constructor */
gnssStateVec(double x, double y, double z, double cb, double tz) :
        x_(x), y_(y), z_(z), cb_(cb), tz_(tz) {
}

/// construct from 3D vector
explicit gnssStateVec(const Vector5& v) :
        x_(v(0)), y_(v(1)), z_(v(2)), cb_(v(3)), tz_(v(4)) {
}

using Vector5::Vector5;

/// @}
/// @name Testable
/// @{

/** print with optional string */
void print(const std::string& s = "") const;

/** equals with an tolerance */
bool equals(const gnssStateVec& p, double tol = 1e-9) const;

/// @}
/// @name Group
/// @{

/// identity for group operation
inline static gnssStateVec identity() {
        return gnssStateVec(0.0, 0.0, 0.0, 0.0, 0.0);
}

/// @}
/// @name Vector Space
/// @{

/** distance between two points */
double distance(const gnssStateVec& p2, OptionalJacobian<1, 5> H1 = boost::none,
                OptionalJacobian<1, 5> H2 = boost::none) const;

/** Distance of the point from the origin, with Jacobian */
double norm(OptionalJacobian<1,5> H = boost::none) const;


/** dot product @return this * q*/
double dot(const gnssStateVec &q, OptionalJacobian<1, 5> H_p = boost::none,     //
           OptionalJacobian<1, 5> H_q = boost::none) const;

/// @}
/// @name Standard Interface
/// @{

/// return as Vector5
const Vector5& vector() const {
        return *this;
}

/// get x
inline double x() const {
        return (*this)[0];
}

/// get y
inline double y() const {
        return (*this)[1];
}

/// get z
inline double z() const {
        return (*this)[2];
}

/// get cb
inline double cb() const {
        return (*this)[3];
}
/// @}

// get tz
inline double tz() const {
        return (*this)[4];
}
/// @}

/// Output stream operator
GTSAM_EXPORT friend std::ostream &operator<<(std::ostream &os, const gnssStateVec& p);

#ifdef GTSAM_ALLOW_DEPRECATED_SINCE_V4
/// @name Deprecated
/// @{
gnssStateVec inverse() const {
        return -(*this);
}
gnssStateVec compose(const gnssStateVec& q) const {
        return (*this)+q;
}
gnssStateVec between(const gnssStateVec& q) const {
        return q-(*this);
}
Vector5 localCoordinates(const gnssStateVec& q) const {
        return between(q);
}
gnssStateVec retract(const Vector5& v) const {
        return compose(gnssStateVec(v));
}
static Vector5 Logmap(const gnssStateVec& p) {
        return p;
}
static gnssStateVec Expmap(const Vector5& v) {
        return gnssStateVec(v);
}
inline double dist(const gnssStateVec& q) const {
        return (q - *this).norm();
}
gnssStateVec add(const gnssStateVec& q, OptionalJacobian<5, 5> H1 = boost::none,
                 OptionalJacobian<5, 5> H2 = boost::none) const;
gnssStateVec sub(const gnssStateVec& q, OptionalJacobian<5, 5> H1 = boost::none,
                 OptionalJacobian<5, 5> H2 = boost::none) const;
/// @}
#endif

private:

/** Serialization function */
friend class boost::serialization::access;
template<class ARCHIVE>
void serialize(ARCHIVE & ar, const unsigned int /*version*/) {
        ar & BOOST_SERIALIZATION_BASE_OBJECT_NVP(Vector5);
}
};

template<>
struct traits<gnssStateVec> : public internal::VectorSpace<gnssStateVec> {};

template<>
struct traits<const gnssStateVec> : public internal::VectorSpace<gnssStateVec> {};

#endif // GTSAM_TYPEDEF_POINTS_TO_VECTORS

// Convenience typedef
typedef std::pair<gnssStateVec, gnssStateVec> Point5Pair;
std::ostream &operator<<(std::ostream &os, const gtsam::Point5Pair &p);

/// distance between two points
double distance5(const gnssStateVec& p1, const gnssStateVec& q,
                 OptionalJacobian<1, 5> H1 = boost::none,
                 OptionalJacobian<1, 5> H2 = boost::none);


/// Distance of the point from the origin, with Jacobian
double norm5(const gnssStateVec& p, OptionalJacobian<1, 5> H = boost::none);

/// dot product
double dot(const gnssStateVec& p, const gnssStateVec& q,
           OptionalJacobian<1, 5> H_p = boost::none,
           OptionalJacobian<1, 5> H_q = boost::none);

template <typename A1, typename A2>
struct Range;

template <>
struct Range<gnssStateVec, gnssStateVec> {
        typedef double result_type;
        double operator()(const gnssStateVec& p, const gnssStateVec& q,
                          OptionalJacobian<1, 5> H1 = boost::none,
                          OptionalJacobian<1, 5> H2 = boost::none) {
                return distance5(p, q, H1, H2);
        }
};
}  // namespace gtsam
