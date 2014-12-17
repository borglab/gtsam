#include <boost/python.hpp>
#include "gtsam/geometry/Point2.h"

using namespace boost::python;
using namespace gtsam;

BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(print_overloads, Point2::print, 0, 1)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(equals_overloads, Point2::equals, 1, 2)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(compose_overloads, Point2::compose, 1, 3)

void exportPoint2(){

  class_<Point2>("Point2", init<>())
    .def(init<double, double>())
    .def("print", &Point2::print, print_overloads(args("s")))
    .def("equals", &Point2::equals, equals_overloads(args("q","tol")))
    .def("inverse", &Point2::inverse)
    .def("compose", &Point2::compose, compose_overloads(args("q", "H1", "H2")))
    .def("between", &Point2::between)
    .def("dim", &Point2::dim)
    .def("retract", &Point2::retract)
    .def("x", &Point2::x)
    .def("y", &Point2::y)
  ;

}