 /* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @brief wraps Symbol class to python
 * @author Ellon Paiva Mendes (LAAS-CNRS)
 **/

#include <boost/python.hpp>
#include <boost/make_shared.hpp>

#define NO_IMPORT_ARRAY
#include <numpy_eigen/NumpyEigenConverter.hpp>

#include <sstream> // for stringstream

#include "gtsam/inference/Symbol.h"

using namespace boost::python;
using namespace gtsam;

BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(print_overloads, Symbol::print, 0, 1)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(equals_overloads, Symbol::equals, 1, 2)

// Helper function to allow building a symbol from a python string and a index.
static boost::shared_ptr<Symbol> makeSymbol(const std::string &str, size_t j)
{
  if(str.size() > 1)
  	throw std::runtime_error("string argument must have one character only");

  return boost::make_shared<Symbol>(str.at(0),j);
}

// Helper function to print the symbol as "char-and-index" in python
std::string selfToString(const Symbol & self)
{
	return (std::string)self;
}

// Helper function to convert a Symbol to int using int() cast in python
size_t selfToKey(const Symbol & self)
{
	return self.key();
}

// Helper function to recover symbol's unsigned char as string
std::string chrFromSelf(const Symbol & self)
{
	std::stringstream ss;
	ss << self.chr();
	return ss.str();
}

void exportSymbol(){

class_<Symbol, boost::shared_ptr<Symbol> >("Symbol")
  .def(init<>())
  .def(init<const Symbol &>())
  .def("__init__", make_constructor(makeSymbol))
  .def(init<Key>())
  .def("print", &Symbol::print, print_overloads(args("s")))
  .def("equals", &Symbol::equals, equals_overloads(args("q","tol")))
  .def("key", &Symbol::key)
  .def("index", &Symbol::index)
  .def(self < self)
  .def(self == self)
  .def(self == other<Key>())
  .def(self != self)
  .def(self != other<Key>())
  .def("__repr__", &selfToString)
  .def("__int__", &selfToKey)
  .def("chr", &chrFromSelf)
;

}