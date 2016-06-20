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

#ifndef GTSAM_TYPEDEF_POINTS_TO_VECTORS
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(print_overloads, Point3::print, 0, 1)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(equals_overloads, Point3::equals, 1, 2)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(norm_overloads, Point3::norm, 0, 1)
#endif

void exportPoint3(){

#ifndef GTSAM_TYPEDEF_POINTS_TO_VECTORS
class_<Point3>("Point3")
  .def(init<>())
  .def(init<double,double,double>())
  .def(init<const Vector3 &>())
  .def("vector", &Point3::vector, return_value_policy<copy_const_reference>())
  .def("x", &Point3::x)
  .def("y", &Point3::y)
  .def("z", &Point3::z)
  .def("print", &Point3::print, print_overloads(args("s")))
  .def("equals", &Point3::equals, equals_overloads(args("q","tol")))
  .def("distance", &Point3::distance)
  .def("cross", &Point3::cross)
  .def("dot", &Point3::dot)
  .def("norm", &Point3::norm, norm_overloads(args("OptionalJacobian<1,3>")))
  .def("normalized", &Point3::normalized)
  .def("identity", &Point3::identity)
  .staticmethod("identity")
  .def(self * other<double>())
  .def(other<double>() * self)
  .def(self + self)
  .def(-self)
  .def(self - self)
  .def(self / other<double>())
  .def(self_ns::str(self))
  .def(repr(self))
  .def(self == self);
#endif

class_<Point3Pair>("Point3Pair", init<Point3, Point3>())
    .def_readwrite("first", &Point3Pair::first)
    .def_readwrite("second", &Point3Pair::second);
}
