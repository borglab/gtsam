#include <boost/python.hpp>

#define NO_IMPORT_ARRAY
#include <numpy_eigen/NumpyEigenConverter.hpp>

#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>

using namespace boost::python;
using namespace gtsam;

void exportLevenbergMarquardtOptimizer(){
  class_<LevenbergMarquardtParams>("LevenbergMarquardtParams", init<>())
  .def("setDiagonalDamping", &LevenbergMarquardtParams::setDiagonalDamping)
  .def("setlambdaFactor", &LevenbergMarquardtParams::setlambdaFactor)
  .def("setlambdaInitial", &LevenbergMarquardtParams::setlambdaInitial)
  .def("setlambdaLowerBound", &LevenbergMarquardtParams::setlambdaLowerBound)
  .def("setlambdaUpperBound", &LevenbergMarquardtParams::setlambdaUpperBound)
  .def("setLogFile", &LevenbergMarquardtParams::setLogFile)
  .def("setUseFixedLambdaFactor", &LevenbergMarquardtParams::setUseFixedLambdaFactor)
  .def("setVerbosityLM", &LevenbergMarquardtParams::setVerbosityLM)
  ;

  class_<LevenbergMarquardtOptimizer>("LevenbergMarquardtOptimizer",
    init<const NonlinearFactorGraph&, const Values&, const LevenbergMarquardtParams&>())
  .def("optimize", &LevenbergMarquardtOptimizer::optimize, return_value_policy<copy_const_reference>())
  ;
}
