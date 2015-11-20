/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @brief wraps Rot2 class to python
 * @author Andrew Melim
 * @author Ellon Paiva Mendes (LAAS-CNRS)
 **/

#include <boost/python.hpp>

#define NO_IMPORT_ARRAY
#include <numpy_eigen/NumpyEigenConverter.hpp>

#include "gtsam/geometry/Rot2.h"

using namespace boost::python;
using namespace gtsam;

BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(print_overloads, Rot2::print, 0, 1)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(equals_overloads, Rot2::equals, 1, 2)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(compose_overloads, Rot2::compose, 1, 3)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(relativeBearing_overloads, Rot2::relativeBearing, 1, 3)

void exportRot2(){

  class_<Rot2>("Rot2", init<>())
    .def(init<double>())
    .def("Expmap", &Rot2::Expmap)
    .staticmethod("Expmap")
    .def("Logmap", &Rot2::Logmap)
    .staticmethod("Logmap")
    .def("atan2", &Rot2::atan2)
    .staticmethod("atan2")
    .def("fromAngle", &Rot2::fromAngle)
    .staticmethod("fromAngle")
    .def("fromCosSin", &Rot2::fromCosSin)
    .staticmethod("fromCosSin")
    .def("fromDegrees", &Rot2::fromDegrees)
    .staticmethod("fromDegrees")
    .def("identity", &Rot2::identity)
    .staticmethod("identity")
    .def("relativeBearing", &Rot2::relativeBearing)
    .staticmethod("relativeBearing")
    .def("c", &Rot2::c)
    .def("degrees", &Rot2::degrees)
    .def("equals", &Rot2::equals, equals_overloads(args("q","tol")))
    .def("matrix", &Rot2::matrix)
    .def("print", &Rot2::print, print_overloads(args("s")))
    .def("rotate", &Rot2::rotate)
    .def("s", &Rot2::s)
    .def("theta", &Rot2::theta)
    .def("unrotate", &Rot2::unrotate)
    .def(self * self) // __mult__
  ;

}