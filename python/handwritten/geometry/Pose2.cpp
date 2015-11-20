/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @brief wraps Pose2 class to python
 * @author Andrew Melim
 * @author Ellon Paiva Mendes (LAAS-CNRS)
 **/

#include <boost/python.hpp>

#define NO_IMPORT_ARRAY
#include <numpy_eigen/NumpyEigenConverter.hpp>

#include "gtsam/geometry/Pose2.h"

using namespace boost::python;
using namespace gtsam;

BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(print_overloads, Pose2::print, 0, 1)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(equals_overloads, Pose2::equals, 1, 2)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(compose_overloads, Pose2::compose, 1, 3)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(between_overloads, Pose2::between, 1, 3)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(transform_to_overloads, Pose2::transform_to, 1, 3)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(transform_from_overloads, Pose2::transform_from, 1, 3)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(bearing_overloads, Pose2::bearing, 1, 3)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(range_overloads, Pose2::range, 1, 3)

// Manually wrap 

void exportPose2(){

  // double (Pose2::*range1)(const Pose2&, boost::optional<Matrix&>, boost::optional<Matrix&>) const
  //   = &Pose2::range;
  // double (Pose2::*range2)(const Point2&, boost::optional<Matrix&>, boost::optional<Matrix&>) const
  //   = &Pose2::range;

  // Rot2 (Pose2::*bearing1)(const Pose2&, boost::optional<Matrix&>, boost::optional<Matrix&>) const
  //   = &Pose2::bearing;
  // Rot2 (Pose2::*bearing2)(const Point2&, boost::optional<Matrix&>, boost::optional<Matrix&>) const
  //   = &Pose2::bearing;

  class_<Pose2>("Pose2", init<>())
    .def(init<Pose2>())
    .def(init<double, double, double>())
    .def(init<double, Point2>())
    .def("print", &Pose2::print, print_overloads(args("s")))

    .def("equals", &Pose2::equals, equals_overloads(args("pose","tol")))
    // .def("inverse", &Pose2::inverse)
    // .def("compose", &Pose2::compose, compose_overloads(args("p2", "H1", "H2")))
    // .def("between", &Pose2::between, between_overloads(args("p2", "H1", "H2")))
    // .def("dim", &Pose2::dim)
    // .def("retract", &Pose2::retract)

    .def("transform_to", &Pose2::transform_to, 
      transform_to_overloads(args("point", "H1", "H2")))
    .def("transform_from", &Pose2::transform_from, 
      transform_to_overloads(args("point", "H1", "H2")))

    .def("x", &Pose2::x)
    .def("y", &Pose2::y)
    .def("theta", &Pose2::theta)
    // See documentation on call policy for more information
    // https://wiki.python.org/moin/boost.python/CallPolicy
    .def("t", &Pose2::t, return_value_policy<copy_const_reference>())
    .def("r", &Pose2::r, return_value_policy<copy_const_reference>())
    .def("translation", &Pose2::translation, return_value_policy<copy_const_reference>())
    .def("rotation", &Pose2::rotation, return_value_policy<copy_const_reference>())

    // .def("bearing", bearing1, bearing_overloads())
    // .def("bearing", bearing2, bearing_overloads())

    // Function overload example
    // .def("range", range1, range_overloads())
    // .def("range", range2, range_overloads())


    .def("Expmap", &Pose2::Expmap)
    .staticmethod("Expmap")

    .def(self * self) // __mult__
  ;

}