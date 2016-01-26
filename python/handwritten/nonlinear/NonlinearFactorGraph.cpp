/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @brief exports NonlinearFactorGraph class to python
 * @author Andrew Melim 
 * @author Ellon Paiva Mendes (LAAS-CNRS)
 **/

#include <boost/python.hpp>

#define NO_IMPORT_ARRAY
#include <numpy_eigen/NumpyEigenConverter.hpp>

#include "gtsam/nonlinear/NonlinearFactorGraph.h"
#include "gtsam/nonlinear/NonlinearFactor.h"

using namespace boost::python;
using namespace gtsam;

BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(print_overloads, NonlinearFactorGraph::print, 0, 1);

void exportNonlinearFactorGraph(){

  typedef NonlinearFactorGraph::sharedFactor sharedFactor;

  void (NonlinearFactorGraph::*push_back1)(const sharedFactor&) = &NonlinearFactorGraph::push_back;
  void (NonlinearFactorGraph::*add1)(const sharedFactor&) = &NonlinearFactorGraph::add;

  class_<NonlinearFactorGraph>("NonlinearFactorGraph", init<>())
    .def("size",&NonlinearFactorGraph::size)  
    .def("push_back", push_back1)
    .def("add", add1)
    .def("resize", &NonlinearFactorGraph::resize)
    .def("empty", &NonlinearFactorGraph::empty)
    .def("print", &NonlinearFactorGraph::print, print_overloads(args("s")))
  ;

}
