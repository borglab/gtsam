#include <boost/python.hpp>
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

    .def("fromAngle", &Rot2::fromAngle)
    .staticmethod("fromAngle")

    .def("fromDegrees", &Rot2::fromDegrees)
    .staticmethod("fromDegrees")

    .def("fromCosSin", &Rot2::fromCosSin)
    .staticmethod("fromCosSin")

    .def("atan2", &Rot2::atan2)
    .staticmethod("atan2")

    .def("print", &Rot2::print, print_overloads(args("s")))
    .def("equals", &Rot2::equals, equals_overloads(args("q","tol")))
    .def("inverse", &Rot2::inverse)
    .def("compose", &Rot2::compose, compose_overloads(args("q", "H1", "H2")))
    .def("between", &Rot2::between)
    .def("dim", &Rot2::dim)
    .def("retract", &Rot2::retract)

    .def("Expmap", &Rot2::Expmap)
    .staticmethod("Expmap")

    .def("theta", &Rot2::theta)
    .def("degrees", &Rot2::degrees)
    .def("c", &Rot2::c)
    .def("s", &Rot2::s)

    .def(self * self) // __mult__
  ;

}