/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file geometry_python.cpp
 * @brief wraps geometry classes into the geometry submodule of gtsam
 * @author Ellon Paiva Mendes (LAAS-CNRS)
 **/

 /** TODOs Summary:
  *
  */

#include <boost/python.hpp>

#include <numpy_eigen/NumpyEigenConverter.hpp>

#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/Pose3.h>

using namespace boost::python;
using namespace gtsam;

// Prototypes used to perform overloading
// See: http://www.boost.org/doc/libs/1_59_0/libs/python/doc/tutorial/doc/html/python/functions.html
// Rot3
gtsam::Rot3  (*AxisAngle_0)(const Vector3&, double) = &Rot3::AxisAngle;
gtsam::Rot3  (*AxisAngle_1)(const gtsam::Point3&, double) = &Rot3::AxisAngle;
gtsam::Rot3  (*Rodrigues_0)(const Vector3&) = &Rot3::Rodrigues;
gtsam::Rot3  (*Rodrigues_1)(double, double, double) = &Rot3::Rodrigues;
gtsam::Rot3  (*RzRyRx_0)(double, double, double) = &Rot3::RzRyRx;
gtsam::Rot3  (*RzRyRx_1)(const Vector&) = &Rot3::RzRyRx;
// Pose3
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(Pose3_equals_overloads_0, equals, 1, 2)
bool  (Pose3::*equals_0)(const gtsam::Pose3&, double) const = &Pose3::equals;


BOOST_PYTHON_MODULE(libgeometry_python)
{

class_<Point3>("Point3")
  .def(init<>())
  .def(init<double,double,double>())
  .def(init<Vector>())
  .def("identity", &Point3::identity)
  .staticmethod("identity")
  .def("add", &Point3::add)
  .def("cross", &Point3::cross)
  .def("dist", &Point3::dist)
  .def("distance", &Point3::distance)
  .def("dot", &Point3::dot)
  .def("equals", &Point3::equals)
  .def("norm", &Point3::norm)
  .def("normalize", &Point3::normalize)
  .def("print", &Point3::print)
  .def("sub", &Point3::sub)
  .def("vector", &Point3::vector)
  .def("x", &Point3::x)
  .def("y", &Point3::y)
  .def("z", &Point3::z)
  .def(self * other<double>())
  .def(other<double>() * self)
  .def(self + self)
  .def(-self)
  .def(self - self)
  .def(self / other<double>())
  .def(self_ns::str(self))
  .def(repr(self))
  .def(self == self)
;

class_<Rot3>("Rot3")
  .def(init<>())
  .def(init<Point3,Point3,Point3>())
  .def(init<double,double,double,double,double,double,double,double,double>())
  .def(init<Matrix3>())
  .def(init<Matrix>())
  .def("AxisAngle", AxisAngle_0)
  .def("AxisAngle", AxisAngle_1)
  .staticmethod("AxisAngle")
  .def("Expmap", &Rot3::Expmap)
  .staticmethod("Expmap")
  .def("ExpmapDerivative", &Rot3::ExpmapDerivative)
  .staticmethod("ExpmapDerivative")
  .def("Logmap", &Rot3::Logmap)
  .staticmethod("Logmap")
  .def("LogmapDerivative", &Rot3::LogmapDerivative)
  .staticmethod("LogmapDerivative")
  .def("Rodrigues", Rodrigues_0)
  .def("Rodrigues", Rodrigues_1)
  .staticmethod("Rodrigues")
  .def("Rx", &Rot3::Rx)
  .staticmethod("Rx")
  .def("Ry", &Rot3::Ry)
  .staticmethod("Ry")
  .def("Rz", &Rot3::Rz)
  .staticmethod("Rz")
  .def("RzRyRx", RzRyRx_0)
  .def("RzRyRx", RzRyRx_1)
  .staticmethod("RzRyRx")
  .def("identity", &Rot3::identity)
  .staticmethod("identity")
  .def("AdjointMap", &Rot3::AdjointMap)
  .def("column", &Rot3::column)
  .def("conjugate", &Rot3::conjugate)
  .def("equals", &Rot3::equals)
  .def("localCayley", &Rot3::localCayley)
  .def("matrix", &Rot3::matrix)
  .def("print", &Rot3::print)
  .def("r1", &Rot3::r1)
  .def("r2", &Rot3::r2)
  .def("r3", &Rot3::r3)
  .def("retractCayley", &Rot3::retractCayley)
  .def("rpy", &Rot3::rpy)
  .def("slerp", &Rot3::slerp)
  .def("transpose", &Rot3::transpose)
  .def("xyz", &Rot3::xyz)
  .def(self * self)
  .def(self * other<Point3>())
  .def(self * other<Unit3>())
  .def(self_ns::str(self))
  .def(repr(self))
;

class_<Pose3>("Pose3")
  .def(init<>())
  .def(init<Pose3>())
  .def(init<Rot3,Point3>())
  .def(init<Matrix>())
  .def("identity", &Pose3::identity)
  .staticmethod("identity")
  .def("bearing", &Pose3::bearing)
  .def("equals", equals_0, Pose3_equals_overloads_0())
  .def("matrix", &Pose3::matrix)
  .def("print", &Pose3::print)
  .def("transform_from", &Pose3::transform_from)
  .def("x", &Pose3::x)
  .def("y", &Pose3::y)
  .def("z", &Pose3::z)
  .def(self * self)
  .def(self * other<Point3>())
  .def(self_ns::str(self))
  .def(repr(self))
;

}