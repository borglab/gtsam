/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @brief wraps Point2 class to python
 * @author Andrew Melim
 * @author Ellon Paiva Mendes (LAAS-CNRS)
 **/

#include <boost/python.hpp>

#define NO_IMPORT_ARRAY
#include <numpy_eigen/NumpyEigenConverter.hpp>

#include "gtsam/geometry/Point2.h"

using namespace boost::python;
using namespace gtsam;

BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(print_overloads, Point2::print, 0, 1)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(equals_overloads, Point2::equals, 1, 2)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(compose_overloads, Point2::compose, 1, 3)

void exportPoint2(){

  class_<Point2>("Point2", init<>())
    .def(init<double, double>())
    .def(init<const Vector2 &>())
    .def("identity", &Point2::identity)
    .def("dist", &Point2::dist)
    .def("distance", &Point2::distance)
    .def("equals", &Point2::equals, equals_overloads(args("q","tol")))
    .def("norm", &Point2::norm)
    .def("print", &Point2::print, print_overloads(args("s")))
    .def("unit", &Point2::unit)
    .def("vector", &Point2::vector)
    .def("x", &Point2::x)
    .def("y", &Point2::y)
    .def(self * other<double>()) // __mult__
    .def(other<double>() * self) // __mult__
    .def(self + self)
    .def(-self)
    .def(self - self)
    .def(self / other<double>())
    .def(self_ns::str(self))
    .def(repr(self))
    .def(self == self)
  ;

}