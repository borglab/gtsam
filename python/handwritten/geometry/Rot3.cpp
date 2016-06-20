/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @brief wraps Rot3 class to python
 * @author Andrew Melim
 * @author Ellon Paiva Mendes (LAAS-CNRS)
 **/

#include <boost/python.hpp>

#define NO_IMPORT_ARRAY
#include <numpy_eigen/NumpyEigenConverter.hpp>

#include "gtsam/geometry/Rot3.h"

using namespace boost::python;
using namespace gtsam;

static Rot3 Quaternion_0(const Vector4& q)
{
    return Rot3::Quaternion(q[0],q[1],q[2],q[3]);
}

static Rot3 Quaternion_1(double w, double x, double y, double z)
{
    return Rot3::Quaternion(w,x,y,z);
}

// Prototypes used to perform overloading
// See: http://www.boost.org/doc/libs/1_59_0/libs/python/doc/tutorial/doc/html/python/functions.html
gtsam::Rot3  (*AxisAngle_0)(const gtsam::Point3&, double) = &Rot3::AxisAngle;
gtsam::Rot3  (*AxisAngle_1)(const gtsam::Unit3&, double) = &Rot3::AxisAngle;
gtsam::Rot3  (*Rodrigues_0)(const Vector3&) = &Rot3::Rodrigues;
gtsam::Rot3  (*Rodrigues_1)(double, double, double) = &Rot3::Rodrigues;
gtsam::Rot3  (*RzRyRx_0)(double, double, double) = &Rot3::RzRyRx;
gtsam::Rot3  (*RzRyRx_1)(const Vector&) = &Rot3::RzRyRx;
Vector (Rot3::*quaternion_0)() const = &Rot3::quaternion;

BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(print_overloads, Rot3::print, 0, 1)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(equals_overloads, Rot3::equals, 1, 2)

void exportRot3(){

  class_<Rot3>("Rot3")
    .def(init<Point3,Point3,Point3>())
    .def(init<double,double,double,double,double,double,double,double,double>())
    .def(init<double,double,double,double>())
    .def(init<const Quaternion &>())
    .def(init<const Matrix3 &>())
    .def(init<const Matrix &>())
    .def("Quaternion", Quaternion_0, arg("q"), "Creates a Rot3 from an array [w,x,y,z] representing a quaternion")
    .def("Quaternion", Quaternion_1, (arg("w"),arg("x"),arg("y"),arg("z")) )
    .staticmethod("Quaternion")
    .def("Expmap", &Rot3::Expmap)
    .staticmethod("Expmap")
    .def("ExpmapDerivative", &Rot3::ExpmapDerivative)
    .staticmethod("ExpmapDerivative")
    .def("Logmap", &Rot3::Logmap)
    .staticmethod("Logmap")
    .def("LogmapDerivative", &Rot3::LogmapDerivative)
    .staticmethod("LogmapDerivative")
    .def("AxisAngle", AxisAngle_0)
    .def("AxisAngle", AxisAngle_1)
    .staticmethod("AxisAngle")
    .def("Rodrigues", Rodrigues_0)
    .def("Rodrigues", Rodrigues_1)
    .staticmethod("Rodrigues")
    .def("Rx", &Rot3::Rx)
    .staticmethod("Rx")
    .def("Ry", &Rot3::Ry)
    .staticmethod("Ry")
    .def("Rz", &Rot3::Rz)
    .staticmethod("Rz")
    .def("RzRyRx", RzRyRx_0, (arg("x"),arg("y"),arg("z")), "Rotations around Z, Y, then X axes as in http://en.wikipedia.org/wiki/Rotation_matrix, counterclockwise when looking from unchanging axis" )
    .def("RzRyRx", RzRyRx_1, arg("xyz"), "Rotations around Z, Y, then X axes as in http://en.wikipedia.org/wiki/Rotation_matrix, counterclockwise when looking from unchanging axis" )
    .staticmethod("RzRyRx")
    .def("Ypr", &Rot3::Ypr)
    .staticmethod("Ypr")
    .def("identity", &Rot3::identity)
    .staticmethod("identity")
    .def("AdjointMap", &Rot3::AdjointMap)
    .def("column", &Rot3::column)
    .def("conjugate", &Rot3::conjugate)
    .def("equals", &Rot3::equals, equals_overloads(args("q","tol")))
#ifndef GTSAM_USE_QUATERNIONS
    .def("localCayley", &Rot3::localCayley)
    .def("retractCayley", &Rot3::retractCayley)
#endif
    .def("matrix", &Rot3::matrix)
    .def("print", &Rot3::print, print_overloads(args("s")))
    .def("r1", &Rot3::r1)
    .def("r2", &Rot3::r2)
    .def("r3", &Rot3::r3)
    .def("rpy", &Rot3::rpy)
    .def("slerp", &Rot3::slerp)
    .def("transpose", &Rot3::transpose)
    .def("xyz", &Rot3::xyz)
    .def("quaternion", quaternion_0)
    .def(self * self)
    .def(self * other<Point3>())
    .def(self * other<Unit3>())
    .def(self_ns::str(self)) // __str__
    .def(repr(self))         // __repr__
  ;

}
