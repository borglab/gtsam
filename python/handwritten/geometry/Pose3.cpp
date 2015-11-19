/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @brief wraps Pose3 class to python
 * @author Andrew Melim
 * @author Ellon Paiva Mendes (LAAS-CNRS)
 **/

#include <boost/python.hpp>
#include "gtsam/geometry/Pose3.h"

#include "gtsam/geometry/Pose2.h"
#include "gtsam/geometry/Point3.h"
#include "gtsam/geometry/Rot3.h"

using namespace boost::python;
using namespace gtsam;

BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(print_overloads, Pose3::print, 0, 1)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(equals_overloads, Pose3::equals, 1, 2)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(transform_to_overloads, Pose3::transform_to, 1, 3)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(transform_from_overloads, Pose3::transform_from, 1, 3)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(translation_overloads, Pose3::translation, 0, 1)
// BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(bearing_overloads, Pose3::bearing, 1, 3)
// BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(range_overloads, Pose3::range, 1, 3)
// BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(compose_overloads, Pose3::compose, 1, 3)
// BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(between_overloads, Pose3::between, 1, 3)

void exportPose3(){

  Point3 (Pose3::*transform_to1)(const Point3&,  OptionalJacobian< 3, 6 >,  OptionalJacobian< 3, 3 > ) const
    = &Pose3::transform_to;
  Pose3 (Pose3::*transform_to2)(const Pose3&) const
    = &Pose3::transform_to;
  
  class_<Pose3>("Pose3")
    .def(init<>())
    .def(init<Pose3>())
    .def(init<Rot3,Point3>())
    .def(init<Rot3,Vector3>())
    .def(init<Pose2>())
    .def(init<Matrix>())
    .def("print", &Pose3::print, print_overloads(args("s")))
    .def("equals", &Pose3::equals, equals_overloads(args("pose","tol")))
    .def("identity", &Pose3::identity)
    .staticmethod("identity")
    .def("bearing", &Pose3::bearing)
    .def("matrix", &Pose3::matrix)
    .def("transform_from", &Pose3::transform_from, 
      transform_from_overloads(args("point", "H1", "H2")))
    .def("transform_to", transform_to1, 
      transform_to_overloads(args("point", "H1", "H2")))
    .def("transform_to", transform_to2)
    .def("x", &Pose3::x)
    .def("y", &Pose3::y)
    .def("z", &Pose3::z)
    .def("translation", &Pose3::translation,
      translation_overloads()[return_value_policy<copy_const_reference>()])
    .def("rotation", &Pose3::rotation, return_value_policy<copy_const_reference>())
    .def(self * self)            // __mult__
    .def(self * other<Point3>()) // __mult__
    .def(self_ns::str(self))     // __str__
    .def(repr(self))             // __repr__
  ;
}