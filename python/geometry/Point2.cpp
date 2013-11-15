#include <boost/python.hpp>
#include "gtsam/geometry/Point2.h"

using namespace boost::python;
using namespace gtsam;

BOOST_PYTHON_FUNCTION_OVERLOADS(equals_overloads, &Point2::equals, 1, 2)

void exportPoint2(){

  class_<Point2>("Point2", init<>())
    .def(init<double, double>())
    .def("print", &Point2::print)
    .def("equals", &Point2::equals)
    .def("inverse", &Point2::inverse)
    .def("compose", &Point2::compose)
    .def("between", &Point2::between)
    .def("dim", &Point2::dim)
    .def("retract", &Point2::retract)
    .def("x", &Point2::x)
    .def("y", &Point2::y)
  ;
}