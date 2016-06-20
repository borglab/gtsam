#include <boost/python.hpp>

#define NO_IMPORT_ARRAY
#include <numpy_eigen/NumpyEigenConverter.hpp>

#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>

using namespace boost::python;
using namespace gtsam;

void exportLevenbergMarquardtOptimizer() {
  class_<GaussNewtonParams>("GaussNewtonParams", init<>())
      .def_readwrite("maxIterations", &GaussNewtonParams::maxIterations)
      .def_readwrite("relativeErrorTol", &GaussNewtonParams::relativeErrorTol);

  class_<GaussNewtonOptimizer, boost::noncopyable>(
      "GaussNewtonOptimizer",
      init<const NonlinearFactorGraph&, const Values&, const GaussNewtonParams&>())
      .def("optimize", &GaussNewtonOptimizer::optimize,
           return_value_policy<copy_const_reference>());

  class_<LevenbergMarquardtParams>("LevenbergMarquardtParams", init<>())
      .def("setDiagonalDamping", &LevenbergMarquardtParams::setDiagonalDamping)
      .def("setlambdaFactor", &LevenbergMarquardtParams::setlambdaFactor)
      .def("setlambdaInitial", &LevenbergMarquardtParams::setlambdaInitial)
      .def("setlambdaLowerBound", &LevenbergMarquardtParams::setlambdaLowerBound)
      .def("setlambdaUpperBound", &LevenbergMarquardtParams::setlambdaUpperBound)
      .def("setLogFile", &LevenbergMarquardtParams::setLogFile)
      .def("setUseFixedLambdaFactor", &LevenbergMarquardtParams::setUseFixedLambdaFactor)
      .def("setVerbosityLM", &LevenbergMarquardtParams::setVerbosityLM);

  class_<LevenbergMarquardtOptimizer, boost::noncopyable>(
      "LevenbergMarquardtOptimizer",
      init<const NonlinearFactorGraph&, const Values&, const LevenbergMarquardtParams&>())
      .def("optimize", &LevenbergMarquardtOptimizer::optimize,
           return_value_policy<copy_const_reference>());

  class_<Marginals>("Marginals", init<const NonlinearFactorGraph&, const Values&>())
      .def("marginalCovariance", &Marginals::marginalCovariance);
}
