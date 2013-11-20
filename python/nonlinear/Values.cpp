#include <boost/python.hpp>
#include <gtsam/nonlinear/Values.h>

using namespace boost::python;
using namespace gtsam;

void exportValues(){

  const Value& (Values::*at1)(Key) const = &Values::at;
  bool (Values::*exists1)(Key) const = &Values::exists;
  void (Values::*insert1)(Key, const Value&) = &Values::insert;

  class_<Values>("Values", init<>())
  .def(init<Values>())
  .def("at", at1, return_value_policy<copy_const_reference>())
  .def("exists", exists1)
  .def("insert", insert1, return_value_policy<copy_const_reference>())
  ;
}