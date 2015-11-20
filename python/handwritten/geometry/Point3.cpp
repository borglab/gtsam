/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @brief wraps Point3 class to python
 * @author Andrew Melim
 * @author Ellon Paiva Mendes (LAAS-CNRS)
 **/

#include <boost/python.hpp>

#define NO_IMPORT_ARRAY
#include <numpy_eigen/NumpyEigenConverter.hpp>

#include "gtsam/geometry/Point3.h"

using namespace boost::python;
using namespace gtsam;

BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(print_overloads, Point3::print, 0, 1)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(equals_overloads, Point3::equals, 1, 2)

void exportPoint3(){

class_<Point3>("Point3")
  .def(init<>())
  .def(init<double,double,double>())
  .def(init<const Vector3 &>())
  .def("identity", &Point3::identity)
  .staticmethod("identity")
  .def("add", &Point3::add)
  .def("cross", &Point3::cross)
  .def("dist", &Point3::dist)
  .def("distance", &Point3::distance)
  .def("dot", &Point3::dot)
  .def("equals", &Point3::equals, equals_overloads(args("q","tol")))
  .def("norm", &Point3::norm)
  .def("normalize", &Point3::normalize)
  .def("print", &Point3::print, print_overloads(args("s")))
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

}