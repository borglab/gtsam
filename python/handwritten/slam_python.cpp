/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file slam_python.cpp
 * @brief wraps slam classes into the slam submodule of gtsam python
 * @author Ellon Paiva Mendes (LAAS-CNRS)
 **/

 /** TODOs Summary:
  *
  */

#include <boost/python.hpp>

#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/linear/NoiseModel.h>

using namespace boost::python;
using namespace gtsam;

// Prototypes used to perform overloading
// See: http://www.boost.org/doc/libs/1_59_0/libs/python/doc/tutorial/doc/html/python/functions.html
// *NONE*

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

// Macro used to define templated factors
#define BETWEENFACTOR(VALUE) \
  class_< BetweenFactor<VALUE>, bases<NonlinearFactor>, boost::shared_ptr< BetweenFactor<VALUE> > >("BetweenFactor"#VALUE) \
  .def(init<Key,Key,VALUE,noiseModel::Base::shared_ptr>()) \
  .def("measured", &BetweenFactor<VALUE>::measured, return_internal_reference<>()) \
;

#define PRIORFACTOR(VALUE) \
  class_< PriorFactor<VALUE>, bases<NonlinearFactor>, boost::shared_ptr< PriorFactor<VALUE> > >("PriorFactor"#VALUE) \
  .def(init<Key,VALUE,noiseModel::Base::shared_ptr>()) \
  .def("prior", &PriorFactor<VALUE>::prior, return_internal_reference<>()) \
;

BOOST_PYTHON_MODULE(libslam_python)
{

  class_<NonlinearFactorCallback,boost::noncopyable>("NonlinearFactor")
  ;

  BETWEENFACTOR(Point3)

  BETWEENFACTOR(Rot3)

  BETWEENFACTOR(Pose3)

  PRIORFACTOR(Point3)

  PRIORFACTOR(Rot3)

  PRIORFACTOR(Pose3)

}