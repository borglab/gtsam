/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @brief exports virtual class NonlinearFactor to python
 * @author Ellon Paiva Mendes (LAAS-CNRS)
 **/

#include <boost/python.hpp>

#define NO_IMPORT_ARRAY
#include <numpy_eigen/NumpyEigenConverter.hpp>

#include "gtsam/nonlinear/NonlinearFactor.h"

using namespace boost::python;
using namespace gtsam;

// Wrap around pure virtual class NonlinearFactor.
// All pure virtual methods should be wrapped. Non-pure may be wrapped if we want to mimic the 
// overloading through inheritance in Python.
// See: http://www.boost.org/doc/libs/1_59_0/libs/python/doc/tutorial/doc/html/python/exposing.html#python.class_virtual_functions
struct NonlinearFactorCallback : NonlinearFactor, wrapper<NonlinearFactor>
{
  double error (const Values & values) const {
    return this->get_override("error")(values);
  }
  size_t dim () const {
    return this->get_override("dim")();
  }
  boost::shared_ptr<GaussianFactor> linearize(const Values & values) const {
    return this->get_override("linearize")(values);
  }
};

void exportNonlinearFactor(){

  class_<NonlinearFactorCallback,boost::noncopyable>("NonlinearFactor")
  ;

}