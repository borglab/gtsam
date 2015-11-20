#include <boost/python.hpp>

#define NO_IMPORT_ARRAY
#include <numpy_eigen/NumpyEigenConverter.hpp>

#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>

using namespace boost::python;
using namespace gtsam;

void exportLevenbergMarquardtOptimizer(){
  class_<LevenbergMarquardtOptimizer>("LevenbergMarquardtOptimizer",
    init<const NonlinearFactorGraph&, const Values&, const LevenbergMarquardtParams&>())
  .def("optimize", &LevenbergMarquardtOptimizer::optimize, return_value_policy<copy_const_reference>())
  ;
}